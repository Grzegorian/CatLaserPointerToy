import serial
import time
from threading import Lock


class ServoController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.serial = None
        self.lock = Lock()
        self.port = port
        self.baudrate = baudrate
        self.connect()

        # Zakresy ruchu serw (w stopniach)
        self.servo1_range = (0, 180)
        self.servo2_range = (0, 180)

        # Aktualne pozycje
        self.servo1_pos = 90
        self.servo2_pos = 90

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Czas na inicjalizację
            print(f"Połączono z ESP32 na {self.port}")
        except Exception as e:
            print(f"Błąd połączenia: {e}")
            self.serial = None

    def send_command(self, cmd):
        with self.lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.write(f"{cmd}\n".encode('utf-8'))
                    return True
                except Exception as e:
                    print(f"Błąd wysyłania: {e}")
                    self.serial.close()
                    self.serial = None
                    return False
            return False

    def move_servos(self, servo1, servo2):
        """Przesuwa serwa do zadanych pozycji (0-180)"""
        if not (self.servo1_range[0] <= servo1 <= self.servo1_range[1] and
                self.servo2_range[0] <= servo2 <= self.servo2_range[1]):
            return False

        self.servo1_pos = servo1
        self.servo2_pos = servo2
        return self.send_command(f"MOVE {servo1} {servo2}")

    def move_relative(self, delta1, delta2):
        """Przesuwa serwa względem aktualnej pozycji"""
        new_pos1 = self.servo1_pos + delta1
        new_pos2 = self.servo2_pos + delta2
        return self.move_servos(
            max(self.servo1_range[0], min(self.servo1_range[1], new_pos1)),
            max(self.servo2_range[0], min(self.servo2_range[1], new_pos2))
        )

    def center_servos(self):
        """Centruje serwomechanizmy"""
        return self.move_servos(90, 90)

    def __del__(self):
        if self.serial and self.serial.is_open:
            self.serial.close()