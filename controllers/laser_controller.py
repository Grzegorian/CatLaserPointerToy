import numpy as np
import time
from collections import deque
from config import REACTION_DELAY


class LaserController:
    def __init__(self):
        self.x = 0.5
        self.y = 0.5
        self.last_move_time = 0
        self.smooth_moves = deque(maxlen=5)

    def set_position(self, x, y, frame_width, frame_height):
        current_time = time.time()
        if current_time - self.last_move_time < REACTION_DELAY:
            return False

        self.x = np.clip(x, 0, 1)
        self.y = np.clip(y, 0, 1)
        self.last_move_time = current_time
        self.smooth_moves.append((self.x, self.y))
        return True

    def get_smoothed_position(self, frame_width, frame_height):
        if not self.smooth_moves:
            return self.get_position(frame_width, frame_height)

        avg_x = sum(pos[0] for pos in self.smooth_moves) / len(self.smooth_moves)
        avg_y = sum(pos[1] for pos in self.smooth_moves) / len(self.smooth_moves)
        return (int(avg_x * frame_width), int(avg_y * frame_height))

    def get_position(self, frame_width, frame_height):
        return (int(self.x * frame_width), int(self.y * frame_height))

    def update_servo_position(self, x, y, frame_width, frame_height):
        """Konwertuje pozycję XY na kąty serw"""
        servo1 = int(180 * x / frame_width)
        servo2 = int(180 * (1 - y / frame_height))  # Odwróć oś Y
        return servo1, servo2