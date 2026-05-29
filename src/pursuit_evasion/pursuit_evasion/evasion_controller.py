import rclpy
import json
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class EvasionController(Node):

    def __init__(self):
        super().__init__('evasion_controller')

        # Argument for which robot this controller controls
        self.declare_parameter('robot', 'evader')
        self.robot = self.get_parameter('robot').value

        # Publisher that sends velocity commands to the robot's cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)

        # Subscriber that listens to /sim_state and runs the control loop on each update
        self.create_subscription(
            String,
            f'/sim_state',
            self.control_loop,
            10
        )
            
    def control_loop(self, msg):
        data = json.loads(msg.data)

        # Calculate repulsive forces from pursuing robots
        F_x = 0.0
        F_y = 0.0
        
        for robot in data.keys():
            if robot == self.robot:
                continue

            pursuer_dx = data[robot]['x'] - data[self.robot]['x']
            pursuer_dy = data[robot]['y'] - data[self.robot]['y']
            distance = math.sqrt((pursuer_dx ** 2) + (pursuer_dy ** 2))

            if distance > 0:
                magnitude = 2.0 * (1.0 / distance)
                F_x -= magnitude * (pursuer_dx / distance)
                F_y -= magnitude * (pursuer_dy / distance)

        # self.get_logger().info(f'F_x: {F_x}, F_y: {F_y}')

        F_magnitude = math.sqrt(F_x ** 2 + F_y ** 2)
        desired_heading = math.atan2(F_y, F_x)

        heading_error = desired_heading - data[self.robot]['yaw']
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate linear and angular velocity
        max_turn_vel = 1.0
        max_linear_vel = 1.0
        
        kw = 2.0
        w = kw * heading_error

        kv = 5.0
        v = kv * F_magnitude

        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w

        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    evasion_controller = EvasionController()
    rclpy.spin(evasion_controller)
    evasion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
