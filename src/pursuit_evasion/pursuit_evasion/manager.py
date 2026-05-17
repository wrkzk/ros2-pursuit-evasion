import rclpy
import json
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from functools import partial

class Manager(Node):

    def __init__(self):
        super().__init__('manager')

        # Data structure keeping track of robot states
        self.states = {}

        # Publisher info
        self.publisher_ = self.create_publisher(String, 'sim_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Node parameters
        self.declare_parameter('active_robots', ['pursuer'])        
        self.active_robots = self.get_parameter('active_robots').value

        self.odom_subs = []
        for robot in self.active_robots:

            self.states[robot] = { 'x': 0.0, 'y': 0.0, 'yaw': 0.0 }
            
            self.odom_subs.append(
                self.create_subscription(
                    Pose,
                    f'/{robot}/pose',
                    partial(self.listener_callback, extra_arg=robot),
                    10
                )
            )
        
    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.states)
        self.publisher_.publish(msg)

    def listener_callback(self, msg, extra_arg):

        # Update the current x and y position for the current robot
        self.states[extra_arg]['x'] = round(msg.position.x, 4)
        self.states[extra_arg]['y'] = round(msg.position.y, 4)

        # Use the incoming quaternion data to calculate the heading angle inthe xy plane
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        self.states[extra_arg]['yaw'] = round(math.atan2(siny_cosp, cosy_cosp), 4)
        
def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
