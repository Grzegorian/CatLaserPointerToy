import cv2
import numpy as np
from ultralytics import YOLO

# Configuration
LASER_AREA_COLOR = (0, 255, 0)  # Green
LASER_POINT_COLOR = (0, 0, 255)  # Red
CAT_POINT_COLOR = (0, 255, 0)  # Green
LINE_COLOR = (255, 0, 0)  # Blue

# Global variables
polygon_points = []
temp_points = []
calibration_complete = False
current_frame = None


class LaserController:
    """Simulates laser movement without hardware"""

    def __init__(self):
        self.x = 0.5  # Normalized position (0-1)
        self.y = 0.5

    def set_position(self, x, y):
        """Set normalized laser position (0-1)"""
        self.x = np.clip(x, 0, 1)
        self.y = np.clip(y, 0, 1)

    def get_position(self):
        """Get normalized laser position"""
        return (self.x, self.y)


def draw_calibration_frame():
    """Draw the current calibration frame with points"""
    frame_copy = current_frame.copy()
    for i, point in enumerate(temp_points):
        cv2.circle(frame_copy, point, 5, LASER_AREA_COLOR, -1)
        if i > 0:
            cv2.line(frame_copy, temp_points[i - 1], point, LASER_AREA_COLOR, 2)
    cv2.imshow('Define Laser Area', frame_copy)


def mouse_callback(event, x, y, flags, param):
    """Handle mouse clicks for calibration"""
    global polygon_points, temp_points, calibration_complete, current_frame
    if event == cv2.EVENT_LBUTTONDOWN:
        temp_points.append((x, y))
        draw_calibration_frame()


def point_in_polygon(point, polygon):
    """Check if point is inside polygon"""
    return cv2.pointPolygonTest(np.array(polygon), point, False) >= 0


def constrain_to_polygon(point, polygon):
    """Constrain point to polygon area"""
    if point_in_polygon(point, polygon):
        return point

    closest_point = None
    min_dist = float('inf')

    for i in range(len(polygon)):
        start = polygon[i]
        end = polygon[(i + 1) % len(polygon)]

        # Line segment parameter calculation
        t = ((point[0] - start[0]) * (end[0] - start[0]) +
             ((point[1] - start[1]) * (end[1] - start[1])))
        t /= ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
        t = max(0, min(1, t))

        # Projection point
        proj = (
            int(start[0] + t * (end[0] - start[0])),
            int(start[1] + t * (end[1] - start[1]))
        )

        dist = np.sqrt((point[0] - proj[0]) ** 2 + (point[1] - proj[1]) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_point = proj

    return closest_point


def escape_strategy(cat_pos, laser_pos, speed=15):
    """Calculate new laser position to escape from cat"""
    dx = laser_pos[0] - cat_pos[0]
    dy = laser_pos[1] - cat_pos[1]
    distance = np.sqrt(dx ** 2 + dy ** 2)
    if distance > 0:
        dx = dx / distance * speed
        dy = dy / distance * speed
    new_x = laser_pos[0] + dx
    new_y = laser_pos[1] + dy
    return (new_x, new_y)


def calibrate_area(cap):
    """Calibrate the play area by having user define a polygon"""
    global polygon_points, temp_points, calibration_complete, current_frame

    cv2.namedWindow('Define Laser Area')
    cv2.setMouseCallback('Define Laser Area', mouse_callback)

    while not calibration_complete:
        ret, current_frame = cap.read()
        if not ret:
            break

        draw_calibration_frame()

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # Space to confirm
            if len(temp_points) > 2:
                polygon_points = temp_points.copy()
                calibration_complete = True
        elif key == ord('c'):  # Clear points
            temp_points = []
        elif key == ord('q'):  # Quit
            break

    cv2.destroyWindow('Define Laser Area')
    return polygon_points if calibration_complete else None


def main():
    global polygon_points, temp_points, calibration_complete, current_frame

    # Initialize
    polygon_points = []
    temp_points = []
    calibration_complete = False
    current_frame = None

    model = YOLO('yolov8n.pt')
    cap = cv2.VideoCapture(0)
    laser = LaserController()

    # Calibration
    while True:
        polygon = calibrate_area(cap)
        if polygon is None:
            print("Calibration canceled")
            return

        # Verify area
        ret, frame = cap.read()
        if ret:
            pts = np.array(polygon, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, LASER_AREA_COLOR, 2)
            cv2.putText(frame, "Enter - confirm, R - redraw, Q - quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow('Verify Area', frame)

            key = cv2.waitKey(0)
            if key == 13:  # Enter
                break
            elif key == ord('r'):
                polygon_points = []
                temp_points = []
                calibration_complete = False
                continue
            else:
                return

    # Main loop
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Draw polygon
            pts = np.array(polygon, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, LASER_AREA_COLOR, 2)

            # Detect cats
            results = model(frame, classes=15, verbose=False)

            # Find largest cat
            largest_cat = None
            max_area = 0
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        largest_cat = ((x1 + x2) // 2, (y1 + y2) // 2)

            # Update laser position
            if largest_cat:
                laser_pos = (int(laser.get_position()[0] * frame.shape[1]),
                             int(laser.get_position()[1] * frame.shape[0]))
                new_pos = escape_strategy(largest_cat, laser_pos)
                constrained_pos = constrain_to_polygon(new_pos, polygon)

                # Update simulated laser
                laser.set_position(constrained_pos[0] / frame.shape[1],
                                   constrained_pos[1] / frame.shape[0])

                # Visualization
                cv2.circle(frame, constrained_pos, 10, LASER_POINT_COLOR, -1)
                cv2.circle(frame, largest_cat, 10, CAT_POINT_COLOR, -1)
                cv2.line(frame, largest_cat, constrained_pos, LINE_COLOR, 2)

            cv2.imshow('Cat Laser Chase', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()