import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math 

class DriveSquare(Node):
    start_turn = False
    start_straight = True
    first = True
    initial_position = 0
    initial_orientation = 0

    def __init__(self):
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_odom(self, msg):
        _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation)
        if self.first:
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = yaw
            self.first = False
            
        if self.start_straight and math.dist([self.initial_position.x, self.initial_position.y], [msg.pose.pose.position.x, msg.pose.pose.position.y]) > 1:
            self.set_to_turn()
        if self.start_turn and abs(yaw - self.initial_orientation) > math.pi / 2:
            self.set_to_straight()

    def set_to_turn(self):
        self.start_straight = False
        self.start_turn = True
        self.first = True    

    def set_to_straight(self):
        self.start_straight = True
        self.start_turn = False
        self.first = True

    def run_loop(self):
        velocity = Twist()
        if self.start_straight:        
            velocity.linear.x = 0.1
            velocity.angular.z = 0.0
        if self.start_turn:        
            velocity.linear.x = 0.0
            velocity.angular.z = 0.1
        self.pub.publish(velocity)



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

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = DriveSquare()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
