from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import math
# from teleop import TeleopNode

class PersonFollowerNode(Node):
    dist = 0

    def __init__(self):
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.yaw = 0
        self.angle = 0
        self.turn_angle = 0

    def process_odom(self, msg):
        _, _, self.yaw = euler_from_quaternion(msg.pose.pose.orientation)

        
        pass
    
    def process_laserscan(self, msg):
        print("whole thing", msg)
        # print("ranges", msg.ranges)
        distance = max(msg.ranges)
        self.angle = msg.ranges.index(distance)
        if self.angle < 180:
            self.turn_angle = 0.2 
        else:
            self.turn_angle = -0.2
        
    def run_loop(self):
        # needs work
        velocity = Twist()
        velocity.angular.z = self.turn_angle 

            
        
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = PersonFollowerNode()     # Create our Node
    # teleopNode = TeleopNode()           # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw