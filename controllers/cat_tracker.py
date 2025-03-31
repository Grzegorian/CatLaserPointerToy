import time
from collections import deque
from config import HISTORY_LENGTH


class CatTracker:
    def __init__(self):
        self.position_history = deque(maxlen=HISTORY_LENGTH)
        self.velocity_history = deque(maxlen=HISTORY_LENGTH)
        self.last_update_time = time.time()

    def update(self, cat_pos):
        if cat_pos is None:
            return

        if not isinstance(cat_pos, (tuple, list)) or len(cat_pos) != 2:
            print(f"Nieprawidłowa pozycja kota: {cat_pos}")
            return

        current_time = time.time()

        if self.position_history:
            last_pos = self.position_history[-1]
            dt = current_time - self.last_update_time
            if dt > 0:
                try:
                    velocity = (
                        (cat_pos[0] - last_pos[0]) / dt,
                        (cat_pos[1] - last_pos[1]) / dt
                    )
                    self.velocity_history.append(velocity)
                except ZeroDivisionError:
                    print("Ostrzeżenie: dt równe zero")

        self.position_history.append(cat_pos)
        self.last_update_time = current_time

    def predict_next_position(self):
        if not self.velocity_history:
            return None

        avg_vx = sum(v[0] for v in self.velocity_history) / len(self.velocity_history)
        avg_vy = sum(v[1] for v in self.velocity_history) / len(self.velocity_history)

        dt = time.time() - self.last_update_time
        last_pos = self.position_history[-1]
        return (last_pos[0] + avg_vx * dt, last_pos[1] + avg_vy * dt)

    def get_movement_pattern(self):
        if len(self.position_history) < 2:
            return "unknown"

        start_pos = self.position_history[0]
        end_pos = self.position_history[-1]
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]

        if abs(dx) > abs(dy):
            return "horizontal" if dx > 0 else "horizontal_reverse"
        return "vertical" if dy > 0 else "vertical_reverse"