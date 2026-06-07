import rclpy
import json
import math
import numpy as np
from scipy.spatial import Voronoi

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PursuitController(Node):

    def __init__(self):
        super().__init__('pursuit_controller')

        self.declare_parameter('robot', 'pursuer_1')
        self.robot = self.get_parameter('robot').value

        self.publisher_ = self.create_publisher(Twist, f'/{self.robot}/cmd_vel', 10)

        self.create_subscription(String, '/sim_state', self.control_loop, 10)

        self.latest_scan = None
        self.create_subscription(
            LaserScan, f'/{self.robot}/scan', self.scan_callback, 10
        )

    def scan_callback(self, msg):
        self.latest_scan = msg

    def control_loop(self, msg):
        data = json.loads(msg.data)

        my_pos = np.array([data[self.robot]['x'], data[self.robot]['y']])
        my_yaw = data[self.robot]['yaw']

        evader_pos = np.array([data['evader_1']['x'], data['evader_1']['y']])

        # --- Build point list: evader first, then all pursuers ---
        # Index 0 = evader, indices 1..N = pursuers (in consistent order)
        pursuer_names = sorted([r for r in data if r.startswith('pursuer')])
        my_idx_in_pursuers = pursuer_names.index(self.robot)

        points = [evader_pos]
        for name in pursuer_names:
            points.append(np.array([data[name]['x'], data[name]['y']]))
        points = np.array(points)

        # --- Compute Voronoi ---
        # Add 4 corner mirror points so no ridges go to infinity
        arena = 13.0
        mirror_r = 14.0
        angles = [i * math.pi / 4 for i in range(8)]  # 0, 45, 90, 135, 180, 225, 270, 315
        mirrors = np.array([
            [mirror_r * math.cos(a), mirror_r * math.sin(a)]
            for a in angles
        ])
        all_points = np.vstack([points, mirrors])
        vor = Voronoi(all_points)

        # --- Find the Voronoi edge shared between this pursuer and the evader ---
        # In scipy Voronoi, point_region[i] gives the region index for point i.
        # ridge_points gives pairs of point indices that share a ridge.
        evader_idx = 0
        my_point_idx = 1 + my_idx_in_pursuers  # offset by 1 because evader is index 0

        midpoint = None
        for ridge_idx, (p1, p2) in enumerate(vor.ridge_points):
            if set([p1, p2]) == set([evader_idx, my_point_idx]):
                # This ridge is the shared edge between me and the evader
                v1_idx, v2_idx = vor.ridge_vertices[ridge_idx]

                if v1_idx == -1 or v2_idx == -1:
                    # Infinite ridge — fall back to pure pursuit
                    break

                v1 = vor.vertices[v1_idx]
                v2 = vor.vertices[v2_idx]
                midpoint = (v1 + v2) / 2.0
                break

        # --- Compute velocity command ---
        F_x = 0.0
        F_y = 0.0

        if midpoint is not None:
            # Theorem 2: head toward midpoint of shared Voronoi edge at full speed
            direction = midpoint - my_pos
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                # Unit vector toward midpoint — paper says move at max speed
                F_x = direction[0] / dist
                F_y = direction[1] / dist
        else:
            # Not a Voronoi neighbor of the evader — pure distance pursuit
            direction = evader_pos - my_pos
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                F_x = direction[0] / dist
                F_y = direction[1] / dist

        # --- Obstacle repulsion from LiDAR (keep from previous controller) ---
        if self.latest_scan:
            step_size = 5
            for i in range(0, len(self.latest_scan.ranges), step_size):
                distance = self.latest_scan.ranges[i]
                if 0.1 < distance < 1.0:
                    local_angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
                    global_angle = my_yaw + local_angle

                    # Ignore if this reading is the evader
                    est_x = my_pos[0] + distance * math.cos(global_angle)
                    est_y = my_pos[1] + distance * math.sin(global_angle)
                    if math.sqrt((est_x - evader_pos[0])**2 + (est_y - evader_pos[1])**2) < 0.4:
                        continue

                    magnitude = 1 / (distance ** 2)
                    F_x -= magnitude * math.cos(global_angle)
                    F_y -= magnitude * math.sin(global_angle)

        # --- Convert to Twist ---
        desired_heading = math.atan2(F_y, F_x)
        heading_error = desired_heading - my_yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        max_linear_vel = 1.0
        max_turn_vel = 2.0
        kw = 1.5

        # Paper assumes robots move at max speed toward target
        # Scale back linearly when turning hard
        turn_factor = max(0.0, 1.0 - abs(heading_error) / math.pi)
        v = max_linear_vel * turn_factor

        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = max(min(kw * heading_error, max_turn_vel), -max_turn_vel)

        self.publisher_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()