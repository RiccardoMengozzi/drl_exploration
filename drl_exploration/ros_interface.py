from rclpy.node import Node

class ROSInterface(Node):
    def __init__(self, namespace: str):
        super().__init__(namespace=namespace, node_name='ros_interface')
        self.namespace = namespace
