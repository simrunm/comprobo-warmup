import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios
import time

class TeleopNode(Node):
    key = ''
    settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('receive_message_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.run_loop)

    def process_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def run_loop(self):
        # Use w and s to move forwards and backwards
        # a and d to rotate left and right
        while self.key != '\x03':
            my_twist_velocity = Twist()
            self.process_key()
            print("key: ", self.key)
            if self.key == "w":
                print('moving foward')
                my_twist_velocity.linear.x = 2.0
            elif self.key == "s":
                print('moving backwards')
                my_twist_velocity.linear.x = -2.0
            elif self.key == "a":
                print('moving left')
                my_twist_velocity.angular.z = 1.0
            elif self.key == "d":
                print('moving right')
                my_twist_velocity.angular.z = -1.0
            elif self.key == " ":
                print('stopping')
                my_twist_velocity.linear.x = 0.0
                my_twist_velocity.angular.z = 0.0
            self.pub.publish(my_twist_velocity)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = TeleopNode()           # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
