import cv2
from config import LASER_AREA_COLOR

def draw_calibration_points(frame, points):
    for i, point in enumerate(points):
        cv2.circle(frame, point, 8, LASER_AREA_COLOR, -1)
        cv2.putText(frame, str(i+1), (point[0]+15, point[1]+15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if i > 0:
            cv2.line(frame, points[i-1], point, LASER_AREA_COLOR, 2)
    if len(points) > 2:
        cv2.line(frame, points[-1], points[0], LASER_AREA_COLOR, 2)