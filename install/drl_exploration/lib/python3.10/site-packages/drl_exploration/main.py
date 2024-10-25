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
models_directory = os.path.join(workspace_path, "src", package_name, 'rl_models')
logs_directory = os.path.join(workspace_path, "src", package_name, 'rl_logs')


MODE = "train"

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
class Prova(Node):
    def __init__(self, env_ID):
        super().__init__('prova')
        self.env_port = 11345 + env_ID
        self.pub = self.create_publisher(Twist, f'env_{env_ID}/cmd_vel', 10)

    def start(self):
        self.timer = self.create_timer(1, self.run)

    def run(self):
        os.environ['GAZEBO_MASTER_URI'] = f"http://localhost:{self.env_port}"
        msg = Twist()
        if self.env_port == 11345:
            print("SONO 11355")
            msg.angular.z = 0.5
        elif self.env_port == 11346:
            print("SONO 11356")
            msg.angular.z = -0.5

        msg.linear.x = 0.5
        self.pub.publish(msg)


def main():

    # rclpy.init()

    # ros_int_executor = SingleThreadedExecutor()

    # ROS_inter = ROSInterface()
    # sim_reset = SimulationReset(ROS_inter)

    # ros_int_executor.add_node(ROS_inter)
    # ros_int_executor_thread = threading.Thread(target=ros_int_executor.spin, daemon=True)
    # print("Starting ROS interface executor thread...")
    # ros_int_executor_thread.start()

    # # Agent initialization should be done after ROS interface is ready, since it needs messages...
    # agent = Agent(ROS_inter, sim_reset, models_directory, logs_directory)
    # try: 
    #     while rclpy.ok():
    #         match MODE:
    #             case "train":
    #                 agent.train()
    #             case "eval":
    #                 agent.eval()
    #             case "test":
    #                 agent.test()
    #             case "check":
    #                 agent.check()
    # except KeyboardInterrupt:
    #     pass

    # # Cleanup: shutdown executor and ROS2
    # ros_int_executor.shutdown()
    # rclpy.shutdown()

    # # Wait for the executor thread to finish
    # ros_int_executor_thread.join()


    rclpy.init()

    nodes = [Prova(i) for i in range(2)]

    [node.start() for node in nodes]
  
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(nodes[0])
    executor.add_node(nodes[1])

    try:
        # Spin both nodes concurrently
        executor.spin()
    finally:
        # Clean up on shutdown
        [node.destroy_node() for node in nodes]
        rclpy.shutdown()


if __name__ == '__main__': 
    main()
