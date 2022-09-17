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
        # self.sub = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.angle = 0
        self.turn_speed = 0
        self.straight_speed = 0
        self.turn = True
        self.straight = True


    def process_laserscan(self, msg):
        # print("whole thing", msg)
        distance = min(msg.ranges)
        print(f'Distance: {distance}')
        self.angle = msg.ranges.index(distance)
        if self.angle > 180:
            self.angle = self.angle - 360
        print(f'Angle to object: {self.angle}')
        # turn proportionally to the distance away from it
        self.turn_speed = self.angle / 100
        self.straight_speed = distance / 5
        print(f'Turning angle: {self.turn_speed}')

        if np.allclose(self.angle, 0, atol=1):
            self.turn_speed = 0
            self.turn = False 
            print("Stopping turning")
        else:
            self.turn = True

        if np.allclose(distance, 0, atol=0.3):
            self.straight = False
            print("Stopping straight")
        else:
            self.straight = True
        
    def run_loop(self):
        velocity = Twist()
        if self.turn:
            velocity.angular.z = float(self.turn_speed)
        else:
            velocity.angular.z = 0.0
        if self.straight:
            velocity.linear.x = float(self.straight_speed)
        else:
            velocity.linear.x = 0.0
        self.pub.publish(velocity)

        
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = PersonFollowerNode()     # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

