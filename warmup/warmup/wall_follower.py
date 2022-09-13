from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# from teleop import TeleopNode

class WallFollowerNode(Node):
    dist = 0

    def __init__(self):
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.run_loop)

    def process_laserscan(self, msg):
        # print("0: ", msg.ranges[0])
        # print("10: ", msg.ranges[10])
        # print("20: ", msg.ranges[20])
        # print("30: ", msg.ranges[30])
        # print("45: ", msg.ranges[45])
        self.dist = msg.ranges[55]
        print("55: ", self.dist)
        # print("70: ", msg.ranges[70])
        # print("85: ", msg.ranges[85])
        # print("120: ", msg.ranges[120])
        # print("140: ", msg.ranges[140])
        # print("180: ", msg.ranges[180])
        # print("270: ", msg.ranges[270])

    def run_loop(self):
        # needs work
        my_twist_velocity = Twist()
        my_twist_velocity.linear.x = 0.2
        if self.dist > 0.8:
            print("adjusting to left")
            my_twist_velocity.angular.z = 0.1
            self.pub.publish(my_twist_velocity)
            sleep(0.5)
            my_twist_velocity.angular.z = 0.0
            self.pub.publish(my_twist_velocity)
        if self.dist < 0.6:
            print("adjusting to right")
            my_twist_velocity.angular.z = -0.1
            self.pub.publish(my_twist_velocity)
            sleep(0.5)
            my_twist_velocity.angular.z = 0.0
            self.pub.publish(my_twist_velocity)
        self.pub.publish(my_twist_velocity)


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallFollowerNode()     # Create our Node
    # teleopNode = TeleopNode()           # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
