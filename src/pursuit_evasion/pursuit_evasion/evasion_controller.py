# import rclpy
# import json
# import math

# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# class EvasionController(Node):

#     def __init__(self):
#         super().__init__('evasion_controller')

#         # Argument for which robot this controller controls
#         self.declare_parameter('robot', 'evader')
#         self.robot = self.get_parameter('robot').value

#         # Publisher that sends velocity commands to the robot's cmd_vel topic
#         self.publisher_ = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)

#         # Subscriber that listens to /sim_state and runs the control loop on each update
#         self.create_subscription(
#             String,
#             f'/sim_state',
#             self.control_loop,
#             10
#         )

#         # Subscriber that listens to /<robot>/scan and updates latest scan var
#         self.latest_scan = None
#         self.create_subscription(
#             LaserScan,
#             f'/{self.robot}/scan',
#             self.scan_callback,
#             10
#         )

#     def scan_callback(self, msg):
#         self.latest_scan = msg
            
#     def control_loop(self, msg):
#         data = json.loads(msg.data)

#         # Calculate repulsive forces from pursuing robots
#         F_x = 0.0
#         F_y = 0.0
        
#         for robot in data.keys():
#             if robot == self.robot:
#                 continue

#             pursuer_dx = data[robot]['x'] - data[self.robot]['x']
#             pursuer_dy = data[robot]['y'] - data[self.robot]['y']
#             distance = math.sqrt((pursuer_dx ** 2) + (pursuer_dy ** 2))

#             if distance > 0:
#                 if distance < 3.0:
#                     magnitude = 2.0 * (1.0 / distance ** 2)
#                 else:
#                     magnitude = 2.0 * (1.0 / distance)

#                 F_x -= magnitude * (pursuer_dx / distance)
#                 F_y -= magnitude * (pursuer_dy / distance)

#         # Forces from static obstacles
#         if self.latest_scan:
#             robot_yaw = data[self.robot]['yaw']

#             step_size = 5
#             for i in range(0, len(self.latest_scan.ranges), step_size):
#                 distance = self.latest_scan.ranges[i]

#                 if 0.1 < distance < 4.0:
#                     # self.get_logger().info('Obstacle Detected', throttle_duration_sec=1.0)

#                     local_angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
#                     global_angle = robot_yaw + local_angle
#                     magnitude = 1.0 * (1.0 / (distance ** 2))
#                     F_x -= magnitude * math.cos(global_angle)
#                     F_y -= magnitude * math.sin(global_angle)

#         F_magnitude = math.sqrt(F_x ** 2 + F_y ** 2)
#         desired_heading = math.atan2(F_y, F_x)
#         if desired_heading < 0:
#             desired_heading += 2*math.pi

#         heading_error = desired_heading - data[self.robot]['yaw']
#         heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

#         # Calculate linear and angular velocity
#         max_turn_vel = 2.0
#         max_linear_vel = 1.0
        
#         kw = 2.0
#         w = kw * heading_error

#         kv = 5.0
#         v = kv * F_magnitude

#         cmd_vel = Twist()
#         cmd_vel.linear.x = max(min(v, max_linear_vel), 0.0)
#         cmd_vel.angular.z = max(min(w, max_turn_vel), -max_turn_vel)

#         self.publisher_.publish(cmd_vel)

# def main(args=None):
#     rclpy.init(args=args)
#     evasion_controller = EvasionController()
#     rclpy.spin(evasion_controller)
#     evasion_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# Including corner  and gap seeking
# import rclpy
# import json
# import math
# import numpy as np

# from rclpy.node import Node
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan


# class EvasionController(Node):

#     def __init__(self):
#         super().__init__('evasion_controller')

#         self.declare_parameter('robot', 'evader')
#         self.robot = self.get_parameter('robot').value

#         self.publisher_ = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)

#         self.create_subscription(String, '/sim_state', self.control_loop, 10)

#         self.latest_scan = None
#         self.create_subscription(LaserScan, f'/{self.robot}/scan', self.scan_callback, 10)

#     def scan_callback(self, msg):
#         self.latest_scan = msg

#     def control_loop(self, msg):
#         data = json.loads(msg.data)

#         evader_x = data[self.robot]['x']
#         evader_y = data[self.robot]['y']
#         evader_yaw = data[self.robot]['yaw']
#         evader_pos = np.array([evader_x, evader_y])

#         pursuer_positions = [
#             np.array([data[r]['x'], data[r]['y']])
#             for r in data if r != self.robot
#         ]

#         F_x = 0.0
#         F_y = 0.0

#         # --- 1. Pursuer repulsion ---
#         for pos in pursuer_positions:
#             dx = pos[0] - evader_x
#             dy = pos[1] - evader_y
#             distance = math.sqrt(dx**2 + dy**2)
#             if distance > 0:
#                 magnitude = 2.0 / (distance**2) if distance < 3.0 else 2.0 / distance
#                 F_x -= magnitude * (dx / distance)
#                 F_y -= magnitude * (dy / distance)

#         # --- 2. Corner seeking: attract toward the arena corner
#         #        that is currently farthest from all pursuers ---
#         corners = [
#             np.array([-9.0, -9.0]), np.array([-9.0,  9.0]),
#             np.array([ 9.0, -9.0]), np.array([ 9.0,  9.0]),
#         ]
#         best_corner = max(
#             corners,
#             key=lambda c: min(np.linalg.norm(c - p) for p in pursuer_positions)
#         )
#         corner_dir = best_corner - evader_pos
#         corner_dist = np.linalg.norm(corner_dir)
#         if corner_dist > 0.5:
#             F_x += 0.8 * (corner_dir[0] / corner_dist)
#             F_y += 0.8 * (corner_dir[1] / corner_dist)

#         # --- 3. Gap seeking + obstacle repulsion from LiDAR ---
#         if self.latest_scan:
#             ranges = self.latest_scan.ranges
#             step = 5

#             # Gap seeking: find the widest arc of free space and bias toward it
#             best_gap_angle = None
#             best_gap_width = 0
#             in_gap = False
#             gap_start_i = 0

#             for i in range(0, len(ranges), step):
#                 r = ranges[i]
#                 free = r > 2.5 or math.isinf(r)
#                 if free and not in_gap:
#                     gap_start_i = i
#                     in_gap = True
#                 elif not free and in_gap:
#                     gap_width = i - gap_start_i
#                     if gap_width > best_gap_width:
#                         best_gap_width = gap_width
#                         mid_i = (gap_start_i + i) // 2
#                         best_gap_angle = (
#                             evader_yaw
#                             + self.latest_scan.angle_min
#                             + mid_i * self.latest_scan.angle_increment
#                         )
#                     in_gap = False

#             if best_gap_angle is not None and best_gap_width > 10:
#                 F_x += 1.0 * math.cos(best_gap_angle)
#                 F_y += 1.0 * math.sin(best_gap_angle)

#             # Obstacle repulsion: tightened to 1.5m so walls don't dominate
#             for i in range(0, len(ranges), step):
#                 distance = ranges[i]
#                 if 0.1 < distance < 1.5:
#                     local_angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
#                     global_angle = evader_yaw + local_angle
#                     magnitude = 1.0 / (distance**2)
#                     F_x -= magnitude * math.cos(global_angle)
#                     F_y -= magnitude * math.sin(global_angle)

#         # --- Convert to velocity command ---
#         F_magnitude = math.sqrt(F_x**2 + F_y**2)
#         desired_heading = math.atan2(F_y, F_x)

#         heading_error = desired_heading - evader_yaw
#         heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

#         max_turn_vel = 2.0
#         max_linear_vel = 1.0
#         kw = 2.0
#         kv = 5.0

#         cmd_vel = Twist()
#         cmd_vel.linear.x = max(min(kv * F_magnitude, max_linear_vel), 0.0)
#         cmd_vel.angular.z = max(min(kw * heading_error, max_turn_vel), -max_turn_vel)

#         self.publisher_.publish(cmd_vel)


# def main(args=None):
#     rclpy.init(args=args)
#     node = EvasionController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!!
# Tuned APF

import rclpy
import json
import math
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class EvasionController(Node):

    def __init__(self):
        super().__init__('evasion_controller')

        self.declare_parameter('robot', 'evader')
        self.robot = self.get_parameter('robot').value

        self.publisher_ = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)
        self.create_subscription(String, '/sim_state', self.control_loop, 10)

        self.latest_scan = None
        self.create_subscription(LaserScan, f'/{self.robot}/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def control_loop(self, msg):
        data = json.loads(msg.data)

        ex = data[self.robot]['x']
        ey = data[self.robot]['y']
        eyaw = data[self.robot]['yaw']
        evader_pos = np.array([ex, ey])

        pursuers = {r: np.array([data[r]['x'], data[r]['y']])
                    for r in data if r != self.robot}

        F_x = 0.0
        F_y = 0.0

        # --- 1. Find the candidate escape point ---
        # Sample 32 points on a circle of radius 3m around the evader.
        # Pick the one that maximizes min-distance to all pursuers,
        # subject to not being too close to a wall.
        best_score = -1.0
        best_target = None
        n_samples = 32
        sample_radius = 3.0

        for i in range(n_samples):
            angle = (2 * math.pi * i) / n_samples
            tx = ex + sample_radius * math.cos(angle)
            ty = ey + sample_radius * math.sin(angle)

            # Clip to arena
            r = math.sqrt(tx**2 + ty**2)
            if r > 10.5:
                tx = tx * 10.5 / r
                ty = ty * 10.5 / r

            # Score = min distance from this point to any pursuer
            min_d = min(np.linalg.norm(np.array([tx, ty]) - p)
                        for p in pursuers.values())

            # Penalty if the candidate is blocked by a LiDAR hit
            # (i.e. there's an obstacle between evader and candidate)
            blocked = False
            if self.latest_scan:
                # Check if the LiDAR scan shows an obstacle in this direction
                # within sample_radius
                scan_angle = math.atan2(ty - ey, tx - ex)
                # Normalize to scan range
                rel_angle = scan_angle - eyaw
                rel_angle = (rel_angle + math.pi) % (2 * math.pi) - math.pi
                angle_min = self.latest_scan.angle_min
                angle_max = self.latest_scan.angle_max
                if angle_min <= rel_angle <= angle_max:
                    idx = int((rel_angle - angle_min) /
                              self.latest_scan.angle_increment)
                    idx = max(0, min(len(self.latest_scan.ranges) - 1, idx))
                    scan_dist = self.latest_scan.ranges[idx]
                    if 0.1 < scan_dist < sample_radius - 0.3:
                        blocked = True

            if not blocked and min_d > best_score:
                best_score = min_d
                best_target = np.array([tx, ty])

        # Attract toward the best escape point
        if best_target is not None:
            direction = best_target - evader_pos
            dist = np.linalg.norm(direction)
            if dist > 0.1:
                F_x += 2.0 * (direction[0] / dist)
                F_y += 2.0 * (direction[1] / dist)

        # --- 2. Direct pursuer repulsion (close range panic) ---
        for pos in pursuers.values():
            dx = pos[0] - ex
            dy = pos[1] - ey
            distance = math.sqrt(dx**2 + dy**2)
            if 0 < distance < 3.0:
                magnitude = 2.0 / (distance**2)
                F_x -= magnitude * (dx / distance)
                F_y -= magnitude * (dy / distance)

        # --- 3. Obstacle repulsion (tight range, safety only) ---
        if self.latest_scan:
            step = 5
            for i in range(0, len(self.latest_scan.ranges), step):
                distance = self.latest_scan.ranges[i]
                if 0.1 < distance < 1.2:
                    local_angle = (self.latest_scan.angle_min
                                   + i * self.latest_scan.angle_increment)
                    global_angle = eyaw + local_angle
                    magnitude = 1.0 / (distance**2)
                    F_x -= magnitude * math.cos(global_angle)
                    F_y -= magnitude * math.sin(global_angle)

        # --- Convert to velocity command ---
        F_magnitude = math.sqrt(F_x**2 + F_y**2)
        desired_heading = math.atan2(F_y, F_x)

        heading_error = desired_heading - eyaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(5.0 * F_magnitude, 1.0), 0.0)
        cmd_vel.angular.z = max(min(2.0 * heading_error, 2.0), -2.0)

        self.publisher_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = EvasionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()