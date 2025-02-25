import rclpy
import threading
import os
import yaml
from rclpy.executors import SingleThreadedExecutor
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

from .exploration_vec_env import ExplorationVecEnv
from .ros_interface import ROSInterface


def load_yaml(file_path):
    """
    Load a YAML file and return its content as a dictionary.

    :param file_path: Path to the YAML file.
    :return: Parsed YAML content as a dictionary.
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def start_ros_interface(ros_int):
    """Start a ROS interface in its own executor (single-threaded)."""
    executor = SingleThreadedExecutor()
    executor.add_node(ros_int)
    executor.spin()  # Spin the executor to handle ROS communications

def create_env(index):
    """Create and return an environment along with its corresponding ROS interface."""
    rclpy.init()  # Initialize rclpy for this process
    namespace = f'env_{index}'
    
    # Create the ROS interface for this environment
    ros_int = ROSInterface(namespace)
    
    # Create a separate thread to handle the ROS interface
    ros_thread = threading.Thread(target=start_ros_interface, args=(ros_int,))
    ros_thread.start() 
    
    # Create and return the environment
    env = ExplorationVecEnv(namespace, ros_interface=ros_int)
    return env

def main():

    yaml_file_path = os.path.join(
        os.getcwd(),
        'src',
        'tb3_multi_env_spawner',
        'config',
        'launch_params.yaml'
    )

    # Load launch parameters from YAML
    params = load_yaml(yaml_file_path)

    # Extract parameters from the YAML file
    num_envs = params['env']['num_envs']
    
    # Create a SubprocVecEnv with multiple environments running in parallel
    env = SubprocVecEnv([lambda i=i: create_env(i) for i in range(num_envs)])
    
    # Initialize the PPO model with the parallel environments
    model = PPO("MlpPolicy", env, verbose=1)
    
    # Train the model for the specified number of timesteps
    model.learn(total_timesteps=10000)

    # Shutdown rclpy after training
    rclpy.shutdown()

if __name__ == "__main__":
    main()
