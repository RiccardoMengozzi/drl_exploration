from rclpy.node import Node
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

class ROSInterface(Node):
    def __init__(self, namespace: str):
        super().__init__(namespace=namespace, node_name='ros_interface')
        self.namespace = namespace

        self.map_msg = None

        self.create_subscription(OccupancyGrid, f'/{self.namespace}/map', self._map_callback, 10)


    def _map_callback(self, msg):
        if self.namespace == "env_0": self.get_logger().info(f"[{self.namespace}] Received map message")
        self.map_msg = OccupancyGrid()
        self.map_msg = msg

    def get_map_area(self):
        return self.map_msg.info.width * self.map_msg.info.height