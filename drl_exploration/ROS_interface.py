
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage



class ROSInterface(Node):
    def __init__(self):
        super().__init__('ros_interface')
        self._scan_msg = None
        self._gazebo_clock_msg = None
        self._map_msg = None
        self._tf_msg = None
        self._tf_odom2foot = None
        self._tf_map2odom = None
        self._robot_angle = None
        self._robot_position = None
        clock_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )


        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.gazreb_clock_sub = self.create_subscription(Clock, '/clock', self.gazebo_clock_callback, clock_qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sim_reset_client = self.create_client(Empty, '/reset_simulation')

    ###### SCAN TOPIC ######

    def scan_callback(self, msg):
        self._scan_msg = msg

    def get_scan_msg(self):
        return self._scan_msg
    
    ###### GAZEBO CLOCK TOPIC ######

    def gazebo_clock_callback(self, msg):
        self._gazebo_clock_msg = msg

    def get_gazebo_clock_msg(self):
        return self._gazebo_clock_msg
    
    def get_life_time(self):
        return self._gazebo_clock_msg.clock.sec + self._gazebo_clock_msg.clock.nanosec * 1e-9
    
    ###### MAP TOPIC ######
    
    def map_callback(self, msg):
        self._map_msg = msg

    def get_map_msg(self):
        return self._map_msg
    
    def get_map_info(self):
        return self._map_msg.info
    
    def get_map_data(self):
        return self._map_msg.data
    
    def get_cells_position(self):
        cell_positions = []
        height = self.get_map_info().height
        width = self.get_map_info().width
        resolution = self.get_map_info().resolution

        for y in range(height):
            for x in range(width):
                # Calculate the real-world position of the cell
                real_x = x * resolution 
                real_y = y * resolution
                
                # Store the position and occupancy value
                cell_positions.append((real_x, real_y))
        
        return cell_positions
    
    def count_known_cells(self):
        return sum([1 for cell in self.get_map_data() if cell != -1])
    

    ###### TF TOPIC ######

    def tf_callback(self, msg):
        self._tf_msg = msg
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self._tf_map2odom = transform.transform
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                self._tf_odom2foot = transform.transform


    def get_robot_pose(self):
        if self._tf_odom2foot is not None and self._tf_map2odom is not None:


            p_o2f = np.array([
                self._tf_odom2foot.translation.x,
                self._tf_odom2foot.translation.y,
                self._tf_odom2foot.translation.z
            ])

            q_o2f = np.array([
                self._tf_odom2foot.rotation.x,
                self._tf_odom2foot.rotation.y,
                self._tf_odom2foot.rotation.z,
                self._tf_odom2foot.rotation.w
            ])

            p_m2o = np.array([
                self._tf_map2odom.translation.x,
                self._tf_map2odom.translation.y,
                self._tf_map2odom.translation.z
            ])

            q_m2o = np.array([
                self._tf_map2odom.rotation.x,
                self._tf_map2odom.rotation.y,
                self._tf_map2odom.rotation.z,
                self._tf_map2odom.rotation.w
            ])

            def quaternion_to_rotation_matrix(q):
                x = q[0]
                y = q[1]
                z = q[2]
                w = q[3]

                return np.array([
                    [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)], \
                    [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)], \
                    [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)] \
                ])
            
            def quaternion_to_euler(q):
                x = q[0]
                y = q[1]
                z = q[2]
                w = q[3]

                t0 = +2.0 * (w * x + y * z)
                t1 = +1.0 - 2.0 * (x * x + y * y)
                roll_x = np.arctan2(t0, t1)

                t2 = +2.0 * (w * y - z * x)
                t2 = +1.0 if t2 > +1.0 else t2
                t2 = -1.0 if t2 < -1.0 else t2
                pitch_y = np.arcsin(t2)

                t3 = +2.0 * (w * z + x * y)
                t4 = +1.0 - 2.0 * (y * y + z * z)
                yaw_z = np.arctan2(t3, t4)

                return roll_x, pitch_y, yaw_z
            
            def quaternion_multiply(q1, q2):
                w1, x1, y1, z1 = q1
                w2, x2, y2, z2 = q2
                return np.array([
                    w1*w2 - x1*x2 - y1*y2 - z1*z2,
                    w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2
                ])

                
            self._robot_position = (p_m2o + np.dot(quaternion_to_rotation_matrix(q_m2o), np.append(p_o2f, 1)[:3]))[:2]
            self._robot_angle = quaternion_to_euler(quaternion_multiply(q_m2o, q_o2f))[2]
            return self._robot_position, self._robot_angle
    
    def get_robot_pos_wrt_map(self):
        return self.get_robot_pose()[0] - [self.get_map_info().origin.position.x, self.get_map_info().origin.position.y]
        
    def get_robot_angle(self):
        return self.get_robot_pose()[1]
        
     
    def get_tf_msg(self):
        return self._tf_msg
    
    def get_tf_odom2foot(self):
        return self._tf_odom2foot
    
    def get_tf_map2odom(self):
        return self._tf_map2odom

    def cmd_vel_publish(self, action):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = action[0].astype(float)
        cmd_vel_msg.angular.z = action[1].astype(float)
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def reset_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info('Simulation reset successfully')
            else:
                self.get_logger().error('Failed to reset Simulation')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def reset_simulation(self):
        print("Resetting simulation")
        future = self.sim_reset_client.call_async(Empty.Request())
        future.add_done_callback(self.reset_callback)
        return future

