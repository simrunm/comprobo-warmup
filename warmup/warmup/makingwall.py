#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class SimpleVisualizationPublisher(Node):
    def __init__(self):
        super().__init__('test_vis')
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.publish_marker)
    
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
        # markerArray = MarkerArray()
        # for i in range(len(marker.points)):
        #     markerArray.markers.append(marker)

        self.vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    simple_visualization_publisher = SimpleVisualizationPublisher()

    rclpy.spin(simple_visualization_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_visualization_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()