import rclpy
import json

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

            self.states[robot] = {}
            
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
        self.states[extra_arg]['x'] = round(msg.position.x, 4)
        self.states[extra_arg]['y'] = round(msg.position.y, 4)
        
def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
