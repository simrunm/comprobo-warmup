import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class DriveSquare(Node):
    start_turn = False
    # start_straight = True

    def __init__(self):
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(Bump, 'bump', self.process_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.run_loop)

    def process_odom(self, msg):
        print("msg: ", msg)
        print("matrix: ", msg.pose.covariance)


    def run_loop(self):
        straight_velocity = Twist()
        straight_velocity.linear.x = 1.0
        # self.pub.publish(straight_velocity)


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = DriveSquare()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
