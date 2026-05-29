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

        # Calculate repulsive forces from team member robots
        total_repulse_x = 0.0
        total_repulse_y = 0.0
        
        for robot in data.keys():

            if robot == 'evader_1' or robot == self.robot:
                continue

            teammate_dx = data[robot]['x'] - data[self.robot]['x']
            teammate_dy = data[robot]['y'] - data[self.robot]['y']
            distance = math.sqrt(teammate_dx ** 2 + teammate_dy ** 2)

            if distance > 0:
                magnitude = 100.0 * (1.0 / distance) * (1.0 / (distance**2))
                total_repulse_x -= magnitude * (teammate_dx / distance)
                total_repulse_y -= magnitude * (teammate_dy / distance)

        F_magnitude = math.sqrt(total_repulse_x ** 2 + total_repulse_y ** 2)
        desired_heading = math.atan2(total_repulse_y, total_repulse_x)

        heading_error = desired_heading - data[self.robot]['yaw']
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate linear and angular velocity
        max_turn_vel = 1.0
        max_linear_vel = 1.0
        
        kw = 4.0
        w = kw * heading_error

        kv = 0.5
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
