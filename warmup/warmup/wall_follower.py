from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallFollowerNode(Node):
    front_dist = 0
    back_dist = 0

    def __init__(self):
        super().__init__('receive_message_node')
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1, self.run_loop)

    def process_laserscan(self, msg):
        self.front_dist = msg.ranges[60]
        self.back_dist = msg.ranges[120]
        print("front: ", self.front_dist, "back: ", self.back_dist)

    def run_loop(self):
        my_twist_velocity = Twist()
        my_twist_velocity.linear.x = 0.2
        dist = self.front_dist - self.back_dist
        if dist > 0.1:
            print("adjusting to left")
            my_twist_velocity.angular.z = 0.1
            self.pub.publish(my_twist_velocity)
        if dist < -0.1:
            print("adjusting to right")
            my_twist_velocity.angular.z = -0.1
            self.pub.publish(my_twist_velocity)
        self.pub.publish(my_twist_velocity)
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom";
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w=1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.points=[]
        marker.points.append(Point(x=-7.0, y=2.0, z=0.0))
        marker.points.append(Point(x=-3.0, y=2.0, z=0.0))
        marker.points.append(Point(x=1.0, y=2.5, z=0.0))
        marker.points.append(Point(x=5.0, y=4.0, z=0.0))
        marker.points.append(Point(x=8.5, y=4.8, z=0.0))
        marker.points.append(Point(x=14.0, y=4.0, z=0.0))
        self.vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallFollowerNode()     # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
