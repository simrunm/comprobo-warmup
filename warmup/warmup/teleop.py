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
        while self.key != '\x03':
            self.process_key()
            print("key: ", self.key)
            my_twist_velocity = Twist()
            if self.key == "\033[A":
                print('moving foward')
                my_twist_velocity.linear.x = 1.0
            elif self.key == "\033[B":
                print('moving backwards')
                my_twist_velocity.linear.x = -1.0
            elif self.key == "\033[D":
                print('moving left')
                my_twist_velocity.linear.y = -1.0
            elif self.key == "\033[C":
                print('moving left')
                my_twist_velocity.linear.y = 1.0
            self.pub.publish(my_twist_velocity)
            time.sleep(2)
            my_twist_velocity.linear.x = 0.0
            my_twist_velocity.linear.y = 0.0
            self.pub.publish(my_twist_velocity)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = TeleopNode()           # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
