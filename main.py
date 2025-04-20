import cv2
import logging
import numpy as np
from ultralytics import YOLO
import time
from config import *
from controllers import LaserController, CatTracker, AIController, ServoController
from utils import draw_calibration_points, constrain_to_polygon



def main():
    logging.basicConfig(level=logging.DEBUG)


    # Inicjalizacja servo
    servo_control = ServoController()  # Dostosuj port do swojego systemu
    servo_control.straighten_out()  # Ustaw w pozycji neutralnej
    servo_control.move_servos(400,980)
    servo_control.move_servos(600,960)
    servo_control.move_servos(400,980)
    servo_control.move_servos(600,960)
    servo_control.move_servos(400,980)
    servo_control.move_servos(600,960)



    # Inicjalizacja kamery
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    if not cap.isOpened():
        print("Błąd kamery!")
        return

    # Inicjalizacja modelu YOLO
    try:
        model = YOLO('yolov8n.pt')
    except Exception as e:
        print(f"Błąd YOLO: {e}")
        cap.release()
        return

    # Inicjalizacja kontrolerów
    laser = LaserController()
    cat_tracker = CatTracker()
    ai_controller = AIController()

    # Zmienne stanu
    polygon_points = []
    temp_points = []
    calibration_complete = False
    simulation_mode = False
    simulated_cat_pos = None

    def mouse_callback(event, x, y, flags, param):
        nonlocal temp_points, simulated_cat_pos, simulation_mode

        if event == cv2.EVENT_LBUTTONDOWN:
            if not calibration_complete:
                temp_points.append((x, y))
            elif simulation_mode:
                simulated_cat_pos = (x, y)
                print(f"Symulowana pozycja kota: {simulated_cat_pos}")

    # Konfiguracja okna
    cv2.namedWindow('Cat Laser Chase', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Cat Laser Chase', mouse_callback)

    # Główna pętla
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (1920, 1080))
        largest_cat = None



        # Tryb kalibracji
        if not calibration_complete:
            draw_calibration_points(frame, temp_points)
            cv2.putText(frame, f"Kliknij {MIN_CALIBRATION_POINTS - len(temp_points)} punktow | Spacja - zatwierdz",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Tryb główny
        else:
            # Rysuj obszar gry
            if len(polygon_points) > 2:
                overlay = frame.copy()
                cv2.fillPoly(overlay, [np.array(polygon_points)], (0, 255, 0, 50))
                cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
                cv2.polylines(frame, [np.array(polygon_points)], True, LASER_AREA_COLOR, 2)

            # Tryb symulacji
            if simulation_mode:
                if simulated_cat_pos:
                    largest_cat = simulated_cat_pos
                    radius = int(SIMULATION_RADIUS * (1 + 0.1 * np.sin(time.time() * 5)))
                    cv2.circle(frame, simulated_cat_pos, radius, SIMULATION_COLOR, -1)
                    cv2.putText(frame, "SYMULACJA", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, SIMULATION_COLOR, 2)
                    cat_tracker.update(simulated_cat_pos)

            # Tryb normalny-detekcja kota
            else:
                results = model(frame, classes=15, verbose=False)
                max_area = 0
                for result in results:
                    for box in result.boxes:
                        if box.conf < 0.5:  # Filtruj słabe detekcje
                            continue
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        area = (x2 - x1) * (y2 - y1)
                        if area > max_area and area > MIN_CAT_AREA:
                            max_area = area
                            largest_cat = ((x1 + x2) // 2, (y1 + y2) // 2)

            # Śledzenie i sterowanie laserem
            if largest_cat:
                if not simulation_mode:
                    cat_tracker.update(largest_cat)
                    cv2.circle(frame, largest_cat, 15, CAT_POINT_COLOR, -1)
                    servo1, servo2 = laser.update_servo_position(*constrained_pos, frame.shape[1], frame.shape[0])
                    servo_control.move_servos(servo1, servo2)

                laser_pos = laser.get_smoothed_position(frame.shape[1], frame.shape[0])

                # Oblicz pozycję z uwzględnieniem strategii i dystansu
                target_pos = ai_controller.get_target_position(
                    largest_cat, laser_pos, polygon_points, cat_tracker
                )
                constrained_pos = constrain_to_polygon(target_pos, polygon_points)

                # Oblicz i wyświetl dystans
                distance = np.linalg.norm(np.array(largest_cat) - np.array(constrained_pos))
                cv2.circle(frame, largest_cat, MIN_DISTANCE,
                           (0, 0, 255) if distance < MIN_DISTANCE else SAFE_DISTANCE_COLOR,
                           1 if distance > MIN_DISTANCE else 2)
                cv2.putText(frame, f"Dystans: {int(distance)}px",
                            (largest_cat[0] + MIN_DISTANCE + 10, largest_cat[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255) if distance < MIN_DISTANCE else SAFE_DISTANCE_COLOR,
                            1)

                # Aktualizuj pozycję lasera
                if laser.set_position(
                        constrained_pos[0] / frame.shape[1],
                        constrained_pos[1] / frame.shape[0],
                        frame.shape[1],
                        frame.shape[0]
                ):
                    current_laser_pos = laser.get_smoothed_position(frame.shape[1], frame.shape[0])
                    cv2.circle(frame, current_laser_pos, 15, LASER_POINT_COLOR, -1)
                    cv2.line(frame, largest_cat, current_laser_pos,
                             (0, 0, 255) if distance < MIN_DISTANCE else LINE_COLOR,
                             2)

        # Informacje o trybie
        mode_text = "[S] Tryb: " + ("SYMULACJA" if simulation_mode else "NORMALNY")
        cv2.putText(frame, mode_text, (frame.shape[1] - 300, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, MODE_TEXT_COLOR, 2)

        cv2.imshow('Cat Laser Chase', frame)

        # Obsługa klawiszy
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and not calibration_complete and len(temp_points) >= MIN_CALIBRATION_POINTS:
            calibration_complete = True
            polygon_points = temp_points.copy()
            print(f"Zatwierdzono obszar z {len(polygon_points)} punktami")
        elif key == ord('s'):
            simulation_mode = not simulation_mode
            print(f"Tryb symulacji: {'ON' if simulation_mode else 'OFF'}")
        elif key == ord('r'):
            calibration_complete = False
            temp_points = []
            simulation_mode = False
            simulated_cat_pos = None
            print("Resetowano kalibrację")
        elif key == ord('q'):
            break

            # W głównej pętli while, w sekcji obsługi klawiszy:
        elif key == ord('i'):  # Strzałka w górę
            servo_control.move_relative(0, -5)
        elif key == ord('k'):  # Strzałka w dół
            servo_control.move_relative(0, 5)
        elif key == ord('j'):  # Strzałka w lewo
            servo_control.move_relative(-5, 0)
        elif key == ord('l'):  # Strzałka w prawo
            servo_control.move_relative(5, 0)
        elif key == ord('c'):  # Centrowanie
            servo_control.center_servos()


    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()