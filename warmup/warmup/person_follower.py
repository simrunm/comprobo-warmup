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
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.ranges = []
        self.angle = 0
        self.turn_speed = 0
        self.straight_speed = 0
        self.turn = True
        self.straight = True
        self.first = True

    def process_laserscan(self, msg):
        self.ranges = msg.ranges
        # print(f'Ranges list: {self.ranges}')
        distance = min(msg.ranges)
        # print(f'Distance: {distance}')
        self.angle = msg.ranges.index(distance)
        if self.angle > 180:
            self.angle = self.angle - 360
        # print(f'Angle to object: {self.angle}')

        # turn proportionally to the distance away from it
        self.turn_speed = self.angle / 100
        self.straight_speed = distance / 6
        # print(f'Turning angle: {self.turn_speed}')

        if np.allclose(self.angle, 0, atol=1):
            self.turn_speed = 0
            self.turn = False 
        else:
            self.turn = True

        if np.allclose(distance, 0, atol=0.3):
            self.straight = False
        else:
            self.straight = True

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom";
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace";
        marker.id = 0

        marker.type = Marker.CYLINDER;
        marker.action = Marker.ADD;
        marker.pose.position.x = -2.0
        marker.pose.position.y = -3.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.vis_pub.publish( marker );
    def run_loop(self):
        self.publish_marker()
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


def polar_to_cartesian(r, theta):
    x = r * math.cos(theta * math.pi / 180)
    y = r * math.sin(theta * math.pi / 180)
    return x,y
        
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = PersonFollowerNode()     # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()

