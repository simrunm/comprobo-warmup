
import rclpy
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import time
import numpy as np

class StateMachine:
    """Class for controlling state transitions"""
    def __init__(self):
        self.current_state = 'find_wall'
        self.found_object = False
        self.aligned = False
        self.checked = False
        self.is_wall = False
        self.wall_followed = False


    def find_state(self):
        """Find the current state"""
        if self.current_state == 'find_wall' and self.found_object:
            self.current_state = 'align'
        
        elif self.current_state == 'align' and self.aligned:
            self.current_state = 'check_wall'
        
        elif self.current_state == 'check_wall' and self.checked:
            if self.is_wall:
                self.current_state = 'follow_wall'
            else:
                self.current_state = 'find_wall'
        
        elif self.current_state == 'follow_wall' and self.wall_followed:
            self.current_state = 'find_wall'



class WallBehaviourNode(Node):
    """ROS node for finding and following wall using finite state machine"""
    DEFAULT_SPEED = 0.2

    def __init__(self):
        self.state_machine = StateMachine()
        super().__init__('wall_behaviour')
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_laserscan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vis_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1, self.run_loop)
        self.ranges = []
        self.turn_speed = 0
        self.straight_speed = 0
        self.turn = True
        self.straight  = True
        self.front_dist = 0
        self.back_dist = 0
        self.first = True
        self.start_time = 0

    def process_laserscan(self, msg):
        # Distances needed for wall following at 60 and 120 degrees
        self.front_dist = msg.ranges[60]
        self.back_dist = msg.ranges[120]
        self.ranges = msg.ranges     

    
    def find_wall(self):
        """
        Finding the angle and distance of the nearest object
        and setting the velocity based on that
        """

        # Find nearest object and record distance and angle
        if min(self.ranges) == math.inf:
            print("No object visible")
        distance = min(self.ranges)
        angle = self.ranges.index(distance)
        if angle > 180:
            angle = angle - 360

        # turn proportionally to the distance away from it
        self.turn_speed = angle / 100
        self.straight_speed = distance / 6

        # Set the speed back to 0 if NEATO successfully approaches object
        if np.allclose(angle, 0, atol=1) and np.allclose(distance, 0, atol=0.75):
            self.turn_speed = 0.0
            self.straight_speed = 0.0
            self.state_machine.found_object = True
  

    def align(self):
        """Align the NEATO so it is on the left side of the wall"""
        self.turn_speed = -0.2
        if np.allclose(self.ranges.index(min(self.ranges)), 90, atol=5): # first part is just the angle
            self.turn_speed = 0.0
            self.state_machine.aligned = True



    def check_wall(self):
        """Check that the wall is more than 5 meters"""
        
        # Record time when robot first starts driving
        self.straight_speed = self.DEFAULT_SPEED
        if self.first:
            self.start_time = time.time()
            self.first = False

        # check that three seconds have passed and there is still a wall
        if (time.time() - self.start_time) > 3 and self.start_time != 0:
            self.state_machine.checked = True
            if self.ranges[90] < 5:
                self.state_machine.is_wall = True      


    def wall_follow(self):
        """Follow wall"""

        dist = self.front_dist - self.back_dist

        # Adjust velocity based on how close the NEATO is to the wall
        if dist > 0.1:
            print("adjusting to left")
            self.turn_speed = 0.1
        if dist < -0.1:
            print("adjusting to right")
            self.turn_speed = -0.1



    def run_loop(self):
        """Main loop for running all behaviours"""

        velocity = Twist()

        # Execute behaviour based on  state
        self.state_machine.find_state()
        if self.state_machine.current_state == 'find_wall':
            self.find_wall()

        elif self.state_machine.current_state == 'align':
            self.align()

        elif self.state_machine.current_state == 'check_wall':
            self.check_wall()

        elif self.state_machine.current_state == 'follow_wall':
            self.wall_follow()

        else:
            print("Unknown state")

        # Publish velocity to NEATO
        velocity.angular.z = float(self.turn_speed)
        velocity.linear.x = float(self.straight_speed)
        self.pub.publish(velocity)


        

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallBehaviourNode()     # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()



