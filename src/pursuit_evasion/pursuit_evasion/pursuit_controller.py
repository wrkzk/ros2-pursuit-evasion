import rclpy
import json
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PursuitController(Node):

    def __init__(self):
        super().__init__('pursuit_controller')

        # Argument for which robot this controller controls
        self.declare_parameter('robot', 'pursuer')
        self.target_robot = self.get_parameter('robot').value

        # Publisher that sends velocity commands to the robot's cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, f'/{self.target_robot}/cmd_vel', 10)

        # Subscriber that listens to /sim_state and runs the control loop on each update
        self.create_subscription(
            String,
            f'/sim_state',
            self.control_loop,
            10
        )
            
    def control_loop(self, msg):
        data = json.loads(msg.data)

        # self.get_logger().info(msg.data)

        try:
            dx = data["evader"]["x"] - data["pursuer"]["x"]
            dy = data["evader"]["y"] - data["pursuer"]["y"]

            theta = math.atan2(dy, dx)

            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = theta - data["pursuer"]["yaw"]

            self.publisher_.publish(cmd_vel)

        except KeyError:
            pass
        
def main(args=None):
    rclpy.init(args=args)
    pursuit_controller = PursuitController()
    rclpy.spin(pursuit_controller)
    pursuit_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
