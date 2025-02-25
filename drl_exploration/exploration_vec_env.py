import gymnasium as gym
import numpy as np
from gymnasium import spaces
from .ros_interface import ROSInterface
import time


class ExplorationVecEnv(gym.Env):
    def __init__(self, namespace: str, ros_interface: ROSInterface):
        super().__init__()
        self.namespace = namespace
        self.ros_int = ros_interface

        self.observation_space = spaces.Box(low=0, high=1, shape=(4,), dtype=np.float32)
        self.action_space = spaces.Discrete(2)  # Example: 2 possible actions


    # def wait_for_callback(self, callbacks):
    #     while any(getattr(self.ros_int, cb)() is None for cb in callbacks):
    #         time.sleep(0.2)
    #         pass


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        obs = np.random.rand(4)  # Random initial state
        info = {}
        return obs, info

    def step(self, action):
        # print("step for environment with namespace:", self.namespace)
        obs = np.random.rand(4)  # New state
        reward = np.random.rand()  # Random reward
        done = np.random.rand() > 0.95  # Random episode termination
        info = {}
        return obs, reward, done, False, info
