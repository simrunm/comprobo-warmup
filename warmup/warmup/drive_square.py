import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        self.timer = self.create_timer(1, self.run_loop)


    def process_odom(self, msg):
        if self.first:
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = msg.pose.pose.orientation
            print("Initial orientation",self.initial_orientation)
            # print("INitial position",self.initial_position)
            self.first = False
        # print("oreintation: ", msg.pose.pose.orientation)
        # print("position: ", msg.pose.pose.position)
        if self.start_straight and abs(msg.pose.pose.position.x - self.initial_position.x) > 0.2:
            self.start_straight = False
            self.start_turn = True
            self.first = True

        if self.start_turn and abs(msg.pose.pose.orientation.z - self.initial_orientation.z) > 0.28:
            self.start_straight = True
            self.start_turn = False
            self.first = True
            

        


    def run_loop(self):
        velocity = Twist()
        if self.start_straight:        
            velocity.linear.x = 1.0
            velocity.angular.z = 0.0
        if self.start_turn:        
            velocity.linear.x = 0.0
            velocity.angular.z = 1.0
        
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = DriveSquare()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
