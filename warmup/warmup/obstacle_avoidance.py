import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import math

class Vector2D:
    def __init__(self, x, y):
        self.x, self.y = x, y
    
    def add(self, vector):
        return Vector2D(self.x + vector.x, self.y + vector.y)

class AvoidObstacle(Node):
    goal_x = 5
    goal_y = 0
    goal_vector = Vector2D(0, 0)
    goal_vector_weight_x = 0.5
    goal_vector_weight_y = 3
    direction = Vector2D(0, 0)
    yaw = 0

    def __init__(self):
        super().__init__('receive_message_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.process_odom, 10)
        self.laserscan_sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)

    def process_odom(self, msg):
        _, _, self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        goal_vector_x = - (self.goal_y - msg.pose.pose.position.y)
        goal_vector_y = self.goal_x - msg.pose.pose.position.x
        self.goal_vector = Vector2D(goal_vector_x*self.goal_vector_weight_x, goal_vector_y*self.goal_vector_weight_y)

    def process_laserscan(self, msg):
        distance = msg.ranges
        obstacle_vectors = []
        for i in range(len(distance)):
            if distance[i] != math.inf and (i < 90 or i > 270) and distance[i] < 3:
                obstacle_vector = self.get_vector_from_scandata(i, 3-distance[i])
                obstacle_vectors.append(obstacle_vector)
        
        averaged_obstacle_vector = Vector2D(0, 0)
        if (len(obstacle_vectors) != 0):
            averaged_obstacle_vector = self.average_vectors(obstacle_vectors)
        self.direction = averaged_obstacle_vector.add(self.goal_vector)

    def get_vector_from_scandata(self, angle, distance):
        rad_angle = angle * (math.pi/180)
        if angle <= 90:
            x = distance * math.sin(rad_angle)
            y = - distance * math.cos(rad_angle)
        elif angle > 90 and angle <= 180:
            x = distance * math.sin(math.pi - rad_angle)
            y = distance * math.cos(math.pi - rad_angle)
        elif angle > 180 and angle <= 270:
            x = - distance * math.sin(rad_angle - math.pi)
            y = distance * math.cos(rad_angle - math.pi)
        else:
            x = - distance * math.sin(2*math.pi - rad_angle)
            y = - distance * math.cos(2*math.pi - rad_angle)
        return Vector2D(x, y)

    def average_vectors(self, vectors):
        x = 0
        y = 0
        for vector in vectors:
            x += vector.x
            y += vector.y
        return Vector2D(x/len(vectors), y/len(vectors))

    def run_loop(self):
        velocity = Twist()
        angle = math.atan2(self.direction.y, self.direction.x)
        if (math.dist([self.goal_vector.x, self.goal_vector.y], [0, 0]) > 0.5):
            velocity.linear.x = 0.1
            if (self.direction.x < 0 and abs(self.direction.x + self.yaw) < 1):
                velocity.angular.z = 0.1    # turn left
            elif (self.direction.x > 0 and abs(self.direction.x - self.yaw) < 1):
                velocity.angular.z = - 0.1  # turn right
        self.pub.publish(velocity)
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
        marker.points.append(Point(x=4.0, y=-1.0, z=0.0))
        marker.points.append(Point(x=4.0, y=-2.0, z=0.0))
        self.vis_pub.publish(marker)

        marker2 = Marker()
        marker2.header.frame_id = "odom";
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.id = 2
        marker2.type = marker.LINE_STRIP
        marker2.action = marker.ADD
        marker2.scale.x = 0.5
        marker2.color.a = 1.0
        marker2.color.b = 1.0
        marker2.pose.orientation.w=1.0
        marker2.pose.position.x = 0.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0
        marker2.points=[]
        marker2.points.append(Point(x=2.0, y=0.0, z=0.0))
        marker2.points.append(Point(x=2.0, y=1.5, z=0.0))
        self.vis_pub.publish(marker2)

        marker3 = Marker()
        marker3.header.frame_id = "odom";
        marker3.header.stamp = self.get_clock().now().to_msg()
        marker3.ns = "goal";
        marker3.id = 3
        marker3.type = Marker.SPHERE;
        marker3.action = Marker.ADD;
        marker3.pose.position.x = 5.0
        marker3.pose.position.y = 0.0
        marker3.pose.position.z = 0.0
        marker3.pose.orientation.x = 0.0;
        marker3.pose.orientation.y = 0.0;
        marker3.pose.orientation.z = 0.0;
        marker3.pose.orientation.w = 1.0;
        marker3.scale.x = 0.3
        marker3.scale.y = 0.3
        marker3.scale.z = 0.0
        marker3.color.a = 1.0; # Don't forget to set the alpha!
        marker3.color.r = 0.0
        marker3.color.g = 1.0
        marker3.color.b = 0.0
        self.vis_pub.publish(marker3);

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
    node = AvoidObstacle()        # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
