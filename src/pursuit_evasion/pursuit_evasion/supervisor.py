import rclpy
import json
import time
import subprocess
import numpy as np
import threading
import random

from rclpy.node import Node
from std_msgs.msg import String


WORLD_NAME   = 'pursuit_world'
CAPTURE_DIST = 0.9

START_POSES = {
    'pursuer_1': (-9.0, -9.0, 0.5),
    'pursuer_2': ( 9.0,  9.0, 0.5),
    'pursuer_3': ( 9.0, -9.0, 0.5),
    # 'pursuer_4': ( -9.0, 9.0, 0.5),
    'evader_1':  ( 1.0,  0.0, 0.5),
}


class SimSupervisor(Node):

    def __init__(self):
        super().__init__('sim_supervisor')

        self.episode_start = None
        self.episode_times = []
        self.resetting     = False

        self.declare_parameter('filename', "episode_times.csv")
        self.filename = self.get_parameter("filename").value

        self.create_subscription(String, '/sim_state', self.state_callback, 10)
        self.get_logger().info('Supervisor ready')

        not_allowed_x = [6, 3, -3.0, -6, -3.0, 3.0]
        not_allowed_y = [0, 5.196, 5.196, 0, -5.196, -5.196]
        allowed_dist = 1.5
        self.points = []

        for x in range(-7, 8, 1):
            for y in range(-7, 8, 1):

                flag = False
                x_dists = [(i - x) < allowed_dist for i in not_allowed_x]
                y_dists = [(i - y) < allowed_dist for i in not_allowed_y]

                for dist in zip(x_dists, y_dists):
                    if dist[0] and dist[1]:
                        flag = True

                if not flag:
                    self.points.append((x, y))


    def state_callback(self, msg):
        if self.resetting:
            return

        data = json.loads(msg.data)
        now = self.get_clock().now()

        if self.episode_start is None:
            self.episode_start = now
            self.get_logger().info('Episode started')
            return
        
        elapsed = (now - self.episode_start).nanoseconds / 1e9

        evader_pos = np.array([data['evader_1']['x'], data['evader_1']['y']])

        for name in [r for r in data if r.startswith('pursuer')]:
            pos = np.array([data[name]['x'], data[name]['y']])
            if np.linalg.norm(pos - evader_pos) < CAPTURE_DIST:
                self.episode_times.append(elapsed)
                self.get_logger().info(
                    f'[Episode {len(self.episode_times)}] '
                    f'Captured by {name} in {elapsed:.2f}s | '
                    f'Mean: {np.mean(self.episode_times):.2f}s | '
                    f'Best: {np.min(self.episode_times):.2f}s'
                )
                self.save_results()
                self.reset_episode()
                return

    def reset_episode(self):
        self.resetting     = True
        self.episode_start = None

        self.get_logger().info('Resetting...')

        threads = []
        for name, _ in START_POSES.items():
            point = random.choice(self.points)
            t = threading.Thread(target=self._set_pose, args = (name, point[0], point[1], 0.5))
            threads.append(t)
            t.start()
        
        for t in threads:
            t.join()

        time.sleep(0.5)
        self.resetting = False
        self.get_logger().info('Reset complete')

    def _set_pose(self, model_name, x, y, z):
        req = (
            f'name: "{model_name}" '
            f'position: {{x: {x} y: {y} z: {z}}} '
            f'orientation: {{x: 0 y: 0 z: 0 w: 1}}'
        )
        result = subprocess.run([
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', req
        ], capture_output=True, text=True)

        if result.returncode != 0:
            self.get_logger().warn(f'Failed to reset {model_name}: {result.stderr}')

    def save_results(self):
        with open(self.filename, 'w') as f:
            f.write('episode,time_s\n')
            for i, t in enumerate(self.episode_times):
                f.write(f'{i+1},{t:.4f}\n')

    # def make_maze(self):



def main(args=None):
    rclpy.init(args=args)
    node = SimSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

