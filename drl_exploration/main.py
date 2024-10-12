import os

from .agent import Agent
from .ROS_interface import ROSInterface
from .simulation_reset import SimulationReset
import rclpy
from rclpy.executors import SingleThreadedExecutor
import threading

from ament_index_python.packages import get_package_share_directory

package_name = 'drl_exploration'
package_share_directory = get_package_share_directory(package_name)
workspace_path = os.path.abspath(os.path.join(package_share_directory, '../../../..'))

# Define paths for saving models and logs
models_directory = os.path.join(workspace_path, "src", package_name, 'models')
logs_directory = os.path.join(workspace_path, "src", package_name, 'logs')


MODE = "train"

def main():

    rclpy.init()

    ros_int_executor = SingleThreadedExecutor()

    ROS_inter = ROSInterface()
    sim_reset = SimulationReset(ROS_inter)

    ros_int_executor.add_node(ROS_inter)
    ros_int_executor_thread = threading.Thread(target=ros_int_executor.spin, daemon=True)
    print("Starting ROS interface executor thread...")
    ros_int_executor_thread.start()

    # Agent initialization should be done after ROS interface is ready, since it needs messages...
    agent = Agent(ROS_inter, sim_reset, models_directory, logs_directory)
    try: 
        while rclpy.ok():
            match MODE:
                case "train":
                    agent.train()
                case "eval":
                    agent.eval()
                case "test":
                    agent.test()
                case "check":
                    agent.check()
    except KeyboardInterrupt:
        pass

    # Cleanup: shutdown executor and ROS2
    ros_int_executor.shutdown()
    rclpy.shutdown()

    # Wait for the executor thread to finish
    ros_int_executor_thread.join()

    


if __name__ == '__main__': 
    main()
