import rclpy
import json
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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

        # Subscriber that listens to /<robot>/scan and updates latest scan var
        self.latest_scan = None
        self.create_subscription(
            LaserScan,
            f'/{self.robot}/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        self.latest_scan = msg
            
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

        # Forces from static obstacles
        if self.latest_scan:
            robot_yaw = data[self.robot]['yaw']

            step_size = 5
            for i in range(0, len(self.latest_scan.ranges), step_size):
                distance = self.latest_scan.ranges[i]

                if 0.1 < distance < 2.0:
                    self.get_logger().info('Obstacle Detected', throttle_duration_sec=1.0)

                    local_angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
                    global_angle = robot_yaw + local_angle
                    magnitude = 1.0 * (1.0 / (distance ** 2))
                    F_x -= magnitude * math.cos(global_angle)
                    F_y -= magnitude * math.sin(global_angle)

        F_magnitude = math.sqrt(F_x ** 2 + F_y ** 2)
        desired_heading = math.atan2(F_y, F_x)

        heading_error = desired_heading - data[self.robot]['yaw']
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate linear and angular velocity
        max_turn_vel = 2.0
        max_linear_vel = 1.0
        
        kw = 2.0
        w = kw * heading_error

        kv = 5.0
        v = kv * F_magnitude

        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(v, max_linear_vel), 0.0)
        cmd_vel.angular.z = max(min(w, max_turn_vel), -max_turn_vel)

        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    evasion_controller = EvasionController()
    rclpy.spin(evasion_controller)
    evasion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
