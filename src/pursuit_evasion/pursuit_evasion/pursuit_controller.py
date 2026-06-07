import rclpy
import json
import math

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

class PursuitController(Node):

    def __init__(self):
        super().__init__('pursuit_controller')

        # Argument for which robot this controller controls
        self.declare_parameter('robot', 'pursuer')
        self.declare_parameter('lookahead', 3.0)
        self.robot = self.get_parameter('robot').value
        self.lookahead_d = self.get_parameter('lookahead').value

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
        # self.get_logger().info(msg.data)

        # Tuning parameters
        k_att = 1.0
        radius = 1.5

        # Calculate attractive forces
        r_pos = data["evader_1"]
        lookahead = [math.cos(r_pos["yaw"]) * self.lookahead_d, math.sin(r_pos["yaw"]) * self.lookahead_d]
        attractive_x = k_att * (r_pos["x"] + lookahead[0] - data[self.robot]["x"])
        attractive_y = k_att * (r_pos["y"] + lookahead[1] - data[self.robot]["y"])

        # Calculate repulsive forces from team member robots
        total_repulse_x = 0.0
        total_repulse_y = 0.0

        F_x = 0.0
        F_y = 0.0
        
        for robot in data.keys():
            if robot == 'evader_1' or robot == self.robot:
                continue

            teammate_dx = data[robot]['x'] - data[self.robot]['x']
            teammate_dy = data[robot]['y'] - data[self.robot]['y']
            distance = math.sqrt(teammate_dx ** 2 + teammate_dy ** 2)

            if distance > 0:
                magnitude = 7.0 / distance
                total_repulse_x -= magnitude * (teammate_dx / distance)
                total_repulse_y -= magnitude * (teammate_dy / distance)

        # Forces from static obstacles
        if self.latest_scan:
            robot_yaw = data[self.robot]['yaw']
            robot_x = data[self.robot]['x']
            robot_y = data[self.robot]['y']

            evader_x = data['evader_1']['x']
            evader_y = data['evader_1']['y']

            step_size = 5
            for i in range(0, len(self.latest_scan.ranges), step_size):
                distance = self.latest_scan.ranges[i]

                if 0.1 < distance < 1.0:
                    local_angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
                    global_angle = robot_yaw + local_angle

                    # Compare the object detected by lidar to the known evader pose
                    # If these are within a certain threshold, then ignore
                    estimated_x = robot_x + distance * math.cos(global_angle)
                    estimated_y = robot_y + distance * math.sin(global_angle)
                    offset = math.sqrt((estimated_x - evader_x) ** 2 + (estimated_y - evader_y) ** 2)

                    if offset < 0.4:
                        continue

                    magnitude = 1.0 * (1.0 / (distance ** 2))
                    F_x -= magnitude * math.cos(global_angle)
                    F_y -= magnitude * math.sin(global_angle)

        # Total forces
        F_x += attractive_x + total_repulse_x
        F_y += attractive_y + total_repulse_y

        F_magnitude = math.sqrt(F_x ** 2 + F_y ** 2)
        desired_heading = math.atan2(F_y, F_x)

        heading_error = desired_heading - data[self.robot]['yaw']
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate linear and angular velocity
        max_turn_vel = 2.0
        max_linear_vel = 1.0
        
        kw = 2.0
        w = kw * heading_error

        kv = 0.5
        v = kv * F_magnitude

        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(v, max_linear_vel), 0.0)
        cmd_vel.angular.z = max(min(w, max_turn_vel), -max_turn_vel)

        self.publisher_.publish(cmd_vel)

    def publish_debug_dot(self, x, y, z):
        marker = Marker()
        
        # Must match the fixed frame of your world/robot
        marker.header.frame_id = self.robot  
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "robot_targets"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Dynamically assigned coordinates from the robot logic
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        
        # Scale (Size)
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # Color: Bright Green for target positions
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Give it a short lifetime so old dots disappear as the robot moves
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    pursuit_controller = PursuitController()
    rclpy.spin(pursuit_controller)
    pursuit_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
