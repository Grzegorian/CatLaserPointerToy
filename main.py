import cv2
import numpy as np
from ultralytics import YOLO
import math

# Konfiguracja
LASER_AREA_COLOR = (0, 255, 0)  # Zielony dla obszaru
LASER_POINT_COLOR = (0, 0, 255)  # Czerwony dla lasera
CAT_POINT_COLOR = (0, 255, 0)  # Zielony dla kota
LINE_COLOR = (255, 0, 0)  # Niebieski dla linii
BASE_SPEED = 5  # Bazowa prędkość
MAX_SPEED = 30  # Maksymalna prędkość ucieczki
MIN_DISTANCE = 80  # Pożądana minimalna odległość
ESCAPE_START_DISTANCE = 150  # Odległość rozpoczęcia ucieczki
CENTER_RETURN_SPEED = 3  # Szybkość powrotu do środka


class LaserController:
    def __init__(self):
        self.x = 0.5
        self.y = 0.5
        self.trapped_counter = 0

    def set_position(self, x, y):
        self.x = np.clip(x, 0, 1)
        self.y = np.clip(y, 0, 1)

    def get_position(self):
        return (int(self.x * 1000), int(self.y * 1000))


def initialize_camera():
    for i in range(3):  # Sprawdź pierwsze 3 urządzenia
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"Znaleziono kamerę na indeksie {i}")
            return cap
        cap.release()
    return None


def calculate_smart_escape(cat_pos, laser_pos, polygon_points):
    cat_pos = np.array(cat_pos)
    laser_pos = np.array(laser_pos)

    distance = np.linalg.norm(cat_pos - laser_pos)

    if distance > ESCAPE_START_DISTANCE:
        speed = BASE_SPEED
    else:
        speed = min(MAX_SPEED, BASE_SPEED + (ESCAPE_START_DISTANCE - distance) / 5)

    if distance > 0:
        escape_vector = (laser_pos - cat_pos) / distance * speed
    else:
        escape_vector = np.zeros(2)

    corner_penalty = np.zeros(2)
    for point in polygon_points:
        corner = np.array(point)
        dist_to_corner = np.linalg.norm(laser_pos - corner)
        if dist_to_corner < 100:
            corner_penalty += (laser_pos - corner) * (100 - dist_to_corner) / 500

    final_vector = escape_vector + corner_penalty

    if np.linalg.norm(final_vector) > MAX_SPEED:
        final_vector = final_vector / np.linalg.norm(final_vector) * MAX_SPEED

    return final_vector


def constrain_to_polygon(point, polygon):
    if len(polygon) < 3:
        return point

    point = np.array(point)
    polygon_np = np.array(polygon, dtype=np.int32)

    inside = cv2.pointPolygonTest(polygon_np, (int(point[0]), int(point[1])), False) >= 0

    if inside:
        return (int(point[0]), int(point[1]))

    closest_point = None
    min_dist = float('inf')

    for i in range(len(polygon)):
        start = np.array(polygon[i])
        end = np.array(polygon[(i + 1) % len(polygon)])

        edge_vec = end - start
        point_vec = point - start
        edge_len_sq = np.dot(edge_vec, edge_vec)

        if edge_len_sq == 0:
            continue

        t = np.dot(point_vec, edge_vec) / edge_len_sq
        t = max(0, min(1, t))
        projection = start + t * edge_vec

        dist = np.linalg.norm(point - projection)
        if dist < min_dist:
            min_dist = dist
            closest_point = projection

    if closest_point is not None:
        return (int(closest_point[0]), int(closest_point[1]))

    return (int(point[0]), int(point[1]))


def is_trapped(pos, polygon_points, threshold=30):
    pos = np.array(pos)
    for point in polygon_points:
        if np.linalg.norm(pos - point) < threshold:
            return True
    return False


def move_toward_center(current_pos, center, frame_size, speed=CENTER_RETURN_SPEED):
    current_pos = np.array(current_pos)
    center = np.array(center)

    direction = center - current_pos
    distance = np.linalg.norm(direction)

    if distance > 0:
        direction = direction / distance
        move_distance = min(speed, distance)
        new_pos = current_pos + direction * move_distance
        return (int(new_pos[0]), int(new_pos[1]))

    return (int(current_pos[0]), int(current_pos[1]))


def main():
    cap = initialize_camera()
    if cap is None:
        print("Błąd: Nie można znaleźć kamery! Sprawdź połączenie.")
        return

    try:
        model = YOLO('yolov8n.pt')
    except Exception as e:
        print(f"Błąd ładowania modelu YOLO: {e}")
        cap.release()
        return

    laser = LaserController()
    polygon_points = []
    temp_points = []
    calibration_complete = False
    mouse_cat_pos = None
    left_button_down = False

    def mouse_callback(event, x, y, flags, param):
        nonlocal temp_points, mouse_cat_pos, left_button_down

        if event == cv2.EVENT_LBUTTONDOWN:
            if not calibration_complete:
                temp_points.append((x, y))
            left_button_down = True
            mouse_cat_pos = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            left_button_down = False
            mouse_cat_pos = None

        elif event == cv2.EVENT_MOUSEMOVE and left_button_down:
            mouse_cat_pos = (x, y)

    cv2.namedWindow('Cat Laser Chase')
    cv2.setMouseCallback('Cat Laser Chase', mouse_callback)

    # Kalibracja obszaru
    while not calibration_complete:
        ret, frame = cap.read()
        if not ret:
            print("Błąd: Nie można odczytać klatki z kamery")
            break

        for i, point in enumerate(temp_points):
            cv2.circle(frame, point, 5, LASER_AREA_COLOR, -1)
            if i > 0:
                cv2.line(frame, temp_points[i - 1], point, LASER_AREA_COLOR, 2)

        cv2.putText(frame, "Kliknij punkty obszaru, spacja - zatwierdź", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow('Cat Laser Chase', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if len(temp_points) > 2:
                polygon_points = temp_points.copy()
                calibration_complete = True
                print(f"Zatwierdzono obszar z {len(polygon_points)} punktami")
        elif key == ord('c'):
            temp_points = []
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return

    # Główna pętla programu
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Błąd: Nie można odczytać klatki")
            break

        frame_height, frame_width = frame.shape[:2]

        # Narysuj obszar gry
        if len(polygon_points) > 2:
            cv2.polylines(frame, [np.array(polygon_points, dtype=np.int32)], True, LASER_AREA_COLOR, 2)

        # Wykrywanie kota
        largest_cat = None
        if mouse_cat_pos:
            largest_cat = mouse_cat_pos
            cv2.putText(frame, "Tryb: Symulacja mysza", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            results = model(frame, classes=15, verbose=False)
            max_area = 0
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    if area > max_area:
                        max_area = area
                        largest_cat = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv2.putText(frame, "Tryb: Auto-detekcja", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Aktualizacja pozycji lasera
        if largest_cat:
            laser_pos = laser.get_position()
            scaled_pos = (
                int(laser_pos[0] * frame_width / 1000),
                int(laser_pos[1] * frame_height / 1000)
            )

            escape_vector = calculate_smart_escape(largest_cat, scaled_pos, polygon_points)
            new_pos = np.array(scaled_pos) + escape_vector

            constrained_pos = constrain_to_polygon(new_pos, polygon_points)

            if is_trapped(constrained_pos, polygon_points):
                center = np.mean(polygon_points, axis=0)
                constrained_pos = move_toward_center(constrained_pos, center, (frame_width, frame_height))

            norm_x = constrained_pos[0] / frame_width
            norm_y = constrained_pos[1] / frame_height
            laser.set_position(norm_x, norm_y)

            # Wizualizacja
            cv2.circle(frame, tuple(map(int, constrained_pos)), 10, LASER_POINT_COLOR, -1)
            cv2.circle(frame, largest_cat, 10, CAT_POINT_COLOR, -1)
            cv2.line(frame, largest_cat, tuple(map(int, constrained_pos)), LINE_COLOR, 2)

            distance = np.linalg.norm(np.array(constrained_pos) - np.array(largest_cat))
            speed = np.linalg.norm(escape_vector)
            cv2.putText(frame, f"Odleglosc: {int(distance)}px", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Predkosc: {speed:.1f}px/klatke", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Cat Laser Chase', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            calibration_complete = False
            temp_points = []
            polygon_points = []
            print("Resetowano kalibrację")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()