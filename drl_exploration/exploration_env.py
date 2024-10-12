# my_environment.py
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time

from .ROS_interface import ROSInterface
from .simulation_reset import SimulationReset   

DISTANCE_THRESHOLD = 0.01
ACTION_PERIOD = 1.0
KNOWN_MAP_PERCENTAGE_GOAL = 0.75
MAX_LINEAR_VELOCITY = 0.22
MAX_ANGULAR_VELOCITY = 2.84

class ExplorationEnv(gym.Env):
    def __init__(self, ros_interface: ROSInterface, sim_reset: SimulationReset):
        super(ExplorationEnv, self).__init__()


        self.ros_int = ros_interface
        self.sim_reset = sim_reset

        self.once = True
        self.reset_finished = False

        self.linear_max = MAX_LINEAR_VELOCITY
        self.angular_max = MAX_ANGULAR_VELOCITY
        self._init_action_space()
        self._init_observation_space()


    def _init_action_space(self):
        # linear velocity along x, angular velocity along z
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)

    def _init_observation_space(self):
        self.wait_for_callbacks()
        map_height = self.ros_int.get_map_info().height
        map_width = self.ros_int.get_map_info().width

        self.observation_space = spaces.Dict({
            'scan': spaces.Box(low=0, high=1, shape=(360,), dtype=np.float32),
            'robot_pose': spaces.Box(low=0, high=1, shape=(3,), dtype=np.float32),
            'cell_values' : spaces.Box(low=0, high=1, shape=(map_height * map_width, ), dtype=np.float32),
            'cell_positions' : spaces.Box(low=0, high=1, shape=(map_height * map_width, 2), dtype=np.float32) 
        })


    def wait_for_callbacks(self):
        while   self.ros_int.get_scan_msg() is None or \
                self.ros_int.get_map_msg() is None or \
                self.ros_int.get_tf_odom2foot() is None or \
                self.ros_int.get_tf_map2odom() is None or \
                self.ros_int.get_gazebo_clock_msg() is None:

            # if self.ros_int.get_scan_msg() is None:
            #     self.ros_int.get_logger().info("[Exploration_env] Waiting for scan messages...")
            # if self.ros_int.get_map_msg() is None:
            #     self.ros_int.get_logger().info("[Exploration_env] Waiting for map messages...")
            # if self.ros_int.get_tf_odom2foot() is None:
            #     self.ros_int.get_logger().info("[Exploration_env] Waiting for tf odom2foot messages...")
            # if self.ros_int.get_tf_map2odom() is None:
            #     self.ros_int.get_logger().info("[Exploration_env] Waiting for tf map2odom messages...")
            # if self.ros_int.get_gazebo_clock_msg() is None:
            #     self.ros_int.get_logger().info("[Exploration_env] Waiting for gazebo clock messages...")

            time.sleep(0.2)
            pass

    def correct_obs_size(self, array, name):
        if name == "cell_values":
            expected_shape = self.observation_space["cell_values"].shape
        elif name == "cell_positions":
            expected_shape = self.observation_space["cell_positions"].shape
        
        if array.shape[0] > expected_shape[0]:
            return array[:expected_shape[0]]
        elif array.shape[0] < expected_shape[0]:
            if len(array.shape) == 1:
                return np.pad(array, (0, expected_shape[0] - array.shape[0]), 'constant')
            else:
                return np.pad(array, ((0, expected_shape[0] - array.shape[0]), (0, 0)), 'constant')
        else:
            return array
        
    def _get_obs(self):
        self.wait_for_callbacks()
        scan_msg = self.ros_int.get_scan_msg()
        map_msg = self.ros_int.get_map_msg()
        robot_position = np.array(self.ros_int.get_robot_pos_wrt_map())
        robot_angle = self.ros_int.get_robot_angle()
        cell_positions = np.array(self.ros_int.get_cells_position())


        ranges = np.array(scan_msg.ranges)
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max

        angle_min = -np.pi
        angle_max = np.pi

        position_min = np.array([0, 0])
        position_max = np.array([map_msg.info.width * map_msg.info.resolution, map_msg.info.height * map_msg.info.resolution])

        obs_scan = (ranges - range_min) / (range_max - range_min)

        # values already between 0 and 100
        cell_values = np.array(map_msg.data)
        cell_values[cell_values == -1] = 0
        cell_values = cell_values / 100
        
        robot_position = (robot_position - position_min) / (position_max - position_min)
        robot_angle = (robot_angle - angle_min) / (angle_max - angle_min)
        robot_pose = np.concatenate([robot_position, [robot_angle]])

        cell_positions = (cell_positions - position_min) / (position_max - position_min)

        # Clip values between 0 and 1 to make sure they are in the correct range
        obs_scan = np.clip(obs_scan, 0, 1)
        robot_pose = np.clip(robot_pose, 0, 1)
        cell_values = np.clip(cell_values, 0, 1)
        cell_values = self.correct_obs_size(cell_values, "cell_values")
        cell_positions = np.clip(cell_positions, 0, 1)
        cell_positions = self.correct_obs_size(cell_positions, "cell_positions")

        return {"scan": obs_scan,
                "cell_values": cell_values.astype(np.float32),
                "cell_positions": cell_positions.astype(np.float32),
                "robot_pose": robot_pose.astype(np.float32),}

    def get_known_map_percentage(self):
        return self.ros_int.count_known_cells() / len(self.ros_int.get_map_data())

    def _get_info(self):
        return {}

    def _get_reward(self, observation):
        reward = self.ros_int.count_known_cells() * 1e-3
        return reward - 10.0 if min(observation["scan"]) < DISTANCE_THRESHOLD else reward

    def _get_done(self):
        if self.get_known_map_percentage() > KNOWN_MAP_PERCENTAGE_GOAL:
            # self.ros_int.get_logger().info("[Exploration_env] Map Completed!!")
            return True
        else:
            return False

    def _get_truncated(self, observation):
        if min(observation["scan"]) < DISTANCE_THRESHOLD:
            # self.ros_int.get_logger().info("[Exploration_env] Collision detected")
            return True
        else:    
            return False

    def wait_gazebo_time(self, period):
        start_time = self.ros_int.get_life_time() 
        print_once = True
        while self.ros_int.get_life_time() - start_time < ACTION_PERIOD:
            if print_once:
                # self.ros_int.get_logger().info("[Exploration_env] Performing action...")
                print_once = False


    def scaled_action(self, action):
        linear = action[0] * self.linear_max
        angular = action[1] * self.angular_max
        return [linear, angular]

    def step(self, action):
        # self.ros_int.get_logger().info(f"[Exploration_env] New Action: {action}")
        self.ros_int.cmd_vel_publish(self.scaled_action(action))

        self.wait_gazebo_time(1.0)
        
        done = self._get_done()
        observation = self._get_obs()
        truncated = self._get_truncated(observation)
        reward = self._get_reward(observation)
        info = self._get_info()


        return observation, reward, done, truncated, info


    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.sim_reset.reset_simulation()
        # while not self.ros_int.reset_done():
        #     pass
        return self._get_obs(), self._get_info()

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    def test(self):
        if self.once == True:
            self.reset()
            self.once = False

        
