import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random


class LaserEnv(gym.Env):
    """ Środowisko RL dla lasera uciekającego przed kotem """

    def __init__(self, width=640, height=480):
        super(LaserEnv, self).__init__()
        self.width = width
        self.height = height
        self.laser_pos = np.array([width // 2, height // 2])
        self.cat_pos = np.array([width // 4, height // 4])
        self.action_space = gym.spaces.Discrete(4)  # Góra, dół, lewo, prawo
        self.observation_space = gym.spaces.Box(low=0, high=max(width, height), shape=(4,), dtype=np.float32)

    def step(self, action):
        """ Wykonuje akcję w środowisku """
        if action == 0:  # Góra
            self.laser_pos[1] -= 10
        elif action == 1:  # Dół
            self.laser_pos[1] += 10
        elif action == 2:  # Lewo
            self.laser_pos[0] -= 10
        elif action == 3:  # Prawo
            self.laser_pos[0] += 10

        # Kot goni laser (prostym ruchem)
        dx, dy = self.laser_pos - self.cat_pos
        self.cat_pos[0] += np.sign(dx) * 5
        self.cat_pos[1] += np.sign(dy) * 5

        # Obliczenie nagrody
        distance = np.linalg.norm(self.laser_pos - self.cat_pos)
        reward = distance / 100  # Większa odległość = większa nagroda
        done = distance < 20  # Jeśli kot dogoni laser, kończymy epizod

        return self._get_state(), reward, done, {}

    def reset(self):
        """ Resetuje środowisko """
        self.laser_pos = np.array([self.width // 2, self.height // 2])
        self.cat_pos = np.array([self.width // 4, self.height // 4])
        return self._get_state()

    def _get_state(self):
        """ Zwraca stan środowiska (pozycje lasera i kota) """
        return np.array([self.laser_pos[0], self.laser_pos[1], self.cat_pos[0], self.cat_pos[1]], dtype=np.float32)
