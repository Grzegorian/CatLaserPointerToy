import numpy as np
import random
import time
from config import BASE_SPEED


class AIController:
    def __init__(self):
        self.strategy = "explore"
        self.last_strategy_change = time.time()

    def update_strategy(self, cat_pos, laser_pos, cat_tracker):
        current_time = time.time()
        if current_time - self.last_strategy_change < 2.0:
            return self.strategy

        distance = np.linalg.norm(np.array(cat_pos) - np.array(laser_pos))

        if distance < 150:
            new_strategy = "escape"
        elif distance < 300:
            new_strategy = "confuse"
        else:
            new_strategy = "explore"

        if new_strategy != self.strategy:
            self.strategy = new_strategy
            self.last_strategy_change = current_time

        return self.strategy

    def get_target_position(self, cat_pos, laser_pos, polygon_points, cat_tracker):
        if self.strategy == "escape":
            return self._escape_strategy(cat_pos, laser_pos, polygon_points)
        elif self.strategy == "confuse":
            return self._confuse_strategy(laser_pos)
        return self._explore_strategy(laser_pos, cat_tracker)

    def _escape_strategy(self, cat_pos, laser_pos, polygon_points):
        farthest_point = max(polygon_points,
                             key=lambda p: np.linalg.norm(np.array(p) - np.array(cat_pos)))
        direction = np.array(farthest_point) - np.array(laser_pos)
        if np.linalg.norm(direction) > 0:
            direction = direction / np.linalg.norm(direction) * BASE_SPEED * 1.5
        return (laser_pos[0] + direction[0], laser_pos[1] + direction[1])

    def _confuse_strategy(self, laser_pos):
        angle = random.uniform(0, 2 * np.pi)
        distance = random.uniform(BASE_SPEED * 0.5, BASE_SPEED * 1.5)
        return (
            laser_pos[0] + np.cos(angle) * distance,
            laser_pos[1] + np.sin(angle) * distance
        )

    def _explore_strategy(self, laser_pos, cat_tracker):
        if cat_tracker.position_history and len(cat_tracker.position_history) > 5:
            avoid_pos = cat_tracker.position_history[-1]
            direction = np.array(laser_pos) - np.array(avoid_pos)
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction) * BASE_SPEED
        else:
            angle = random.uniform(0, 2 * np.pi)
            direction = np.array([
                np.cos(angle) * BASE_SPEED * 0.5,
                np.sin(angle) * BASE_SPEED * 0.5
            ])
        return (laser_pos[0] + direction[0], laser_pos[1] + direction[1])