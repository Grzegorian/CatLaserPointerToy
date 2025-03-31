import cv2
import numpy as np
from ultralytics import YOLO
import time

from config import *
from controllers import LaserController, CatTracker, AIController
from utils import draw_calibration_points, constrain_to_polygon


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Błąd kamery!")
        return

    try:
        model = YOLO('yolov8n.pt')
    except Exception as e:
        print(f"Błąd YOLO: {e}")
        cap.release()
        return

    laser = LaserController()
    cat_tracker = CatTracker()
    ai_controller = AIController()
    polygon_points = []
    temp_points = []
    calibration_complete = False

    def mouse_callback(event, x, y, flags, param):
        nonlocal temp_points
        if event == cv2.EVENT_LBUTTONDOWN and not calibration_complete:
            temp_points.append((x, y))

    cv2.namedWindow('Cat Laser Chase', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Cat Laser Chase', mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Błąd odczytu klatki!")
            break

        frame = cv2.resize(frame, (1280, 720))
        largest_cat = None  # Inicjalizacja zmiennej przed użyciem

        if not calibration_complete:
            draw_calibration_points(frame, temp_points)
            cv2.putText(frame, f"Punkty: {len(temp_points)}/{MIN_CALIBRATION_POINTS} | Spacja - zatwierdź",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            if len(polygon_points) > 2:
                # Rysuj półprzezroczysty obszar gry
                overlay = frame.copy()
                cv2.fillPoly(overlay, [np.array(polygon_points)], (0, 255, 0))
                cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
                cv2.polylines(frame, [np.array(polygon_points)], True, LASER_AREA_COLOR, 2)

            # Detekcja kota
            results = model(frame, classes=15, verbose=False)
            max_area = 0
            largest_cat = None

            for result in results:
                for box in result.boxes:
                    # Dodatkowe sprawdzenie pewności detekcji
                    if box.conf < 0.5:  # Pomijaj detekcje z pewnością <50%
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)

                    # Dodatkowy warunek minimalnej powierzchni
                    if area > max_area and area > MIN_CAT_AREA:
                        max_area = area
                        largest_cat = ((x1 + x2) // 2, (y1 + y2) // 2)

            if largest_cat:
                x, y = largest_cat
                if not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
                    print(f"Nieprawidłowe współrzędne kota: {largest_cat}")
                    largest_cat = None

            if largest_cat:  # Tylko jeśli kot został wykryty
                cat_tracker.update(largest_cat)
                laser_pos = laser.get_smoothed_position(frame.shape[1], frame.shape[0])

                # Aktualizacja strategii AI
                ai_controller.update_strategy(largest_cat, laser_pos, cat_tracker)

                # Oblicz nową pozycję lasera
                target_pos = ai_controller.get_target_position(
                    largest_cat, laser_pos, polygon_points, cat_tracker
                )
                constrained_pos = constrain_to_polygon(target_pos, polygon_points)

                # Uaktualnij pozycję lasera
                if laser.set_position(
                        constrained_pos[0] / frame.shape[1],
                        constrained_pos[1] / frame.shape[0],
                        frame.shape[1],
                        frame.shape[0]
                ):
                    # Wizualizacja
                    current_laser_pos = laser.get_smoothed_position(frame.shape[1], frame.shape[0])
                    cv2.circle(frame, current_laser_pos, 15, LASER_POINT_COLOR, -1)
                    cv2.circle(frame, largest_cat, 15, CAT_POINT_COLOR, -1)
                    cv2.line(frame, largest_cat, current_laser_pos, LINE_COLOR, 2)

                    # Rysuj ścieżkę kota
                    for i in range(1, len(cat_tracker.position_history)):
                        if cat_tracker.position_history[i - 1] and cat_tracker.position_history[i]:
                            cv2.line(frame,
                                     cat_tracker.position_history[i - 1],
                                     cat_tracker.position_history[i],
                                     HISTORY_COLOR, 2)
            else:
                cv2.putText(frame, "Nie wykryto kota", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('Cat Laser Chase', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and not calibration_complete and len(temp_points) >= MIN_CALIBRATION_POINTS:
            calibration_complete = True
            polygon_points = temp_points.copy()
            print(f"Zatwierdzono obszar gry z {len(polygon_points)} punktami")
        elif key == ord('r'):
            calibration_complete = False
            temp_points = []
            print("Resetowano kalibrację")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()