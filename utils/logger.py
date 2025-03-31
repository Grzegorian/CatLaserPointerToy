import time
from config import DEBUG_MODE, LOG_LEVEL, DEBUG_COLORS


class GameLogger:
    def __init__(self):
        self.start_time = time.time()

    def log(self, message, level='INFO'):
        levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR']
        if levels.index(level) >= levels.index(LOG_LEVEL):
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            print(f"[{timestamp}] [{level}] {message}")

    def debug_frame(self, frame, message, level='INFO'):
        if DEBUG_MODE and frame is not None:
            color = DEBUG_COLORS.get(level, (255, 255, 255))
            cv2.putText(frame, message, (10, frame.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)