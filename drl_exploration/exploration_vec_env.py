import gymnasium as gym
import numpy as np
from gymnasium import spaces
from .ros_interface import ROSInterface
import time
import rclpy

class ExplorationVecEnv(gym.Env):
    def __init__(self, namespace: str, ros_interface: ROSInterface):
        super().__init__()
        self.namespace = namespace
        self.ros_int = ros_interface

        print(f"{namespace} env initialized")
        self.map_area = None

        self._init_obs_space()
        self._init_action_space()



    def wait_for_msgs(self, msgs):
        while any(getattr(self.ros_int, msg) is None for msg in msgs):
            rclpy.spin_once(self.ros_int, timeout_sec=0.001)
            # print(f"[{self.namespace}] Waiting for {msgs}...")
            time.sleep(1)
            pass


    def get_map_area(self):
        rclpy.spin_once(self.ros_int, timeout_sec=0.001)
        return self.ros_int.get_map_area()

    def _init_obs_space(self):
        self.wait_for_msgs(['map_msg'])
        while True:
            self.map_area = self.get_map_area()
            # if self.namespace == 'env_0':
            #     print(f"I am {self.namespace}, my map area is {self.map_area}")


        self.observation_space = spaces.Dict({
            'scan': spaces.Box(low=0, high=1, shape=(360,), dtype=np.float32),
            'robot_pose': spaces.Box(low=0, high=1, shape=(3,), dtype=np.float32),
            'map': spaces.Box(low=0, high=1, shape=(self.map_area,), dtype=np.float32),
        })

    def _init_action_space(self):
        self.action_space = spaces.Discrete(2)  # Example: 2 possible actions





    def _get_obs(self):
        pass

    def _get_reward(self):
        pass

    def _get_done(self):
        pass

    def _get_truncated(self):
        return False

    def _get_info(self):
        return {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def step(self, action):
        obs = self._get_obs()
        reward = self._get_reward()
        done = self._get_done()
        truncated = self._get_truncated()
        info = self._get_info()

        return obs, reward, done, truncated, info
