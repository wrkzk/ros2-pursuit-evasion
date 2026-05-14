import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class Manager(Node):

    def __init__(self):
        super().__init__('manager')

        # Publisher info
        self.publisher_ = self.create_publisher(String, 'sim_state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Subscription info
        self.declare_parameter('active_robots', ['pursuer'])
        self.active_robots = self.get_parameter('active_robots').value

        self.odom_subs = []
        for robot in self.active_robots:
            self.odom_subs.append(
                self.create_subscription(
                    Odometry,
                    f'/{robot}/odom',
                    self.listener_callback,
                    10
                )
            )
        
    def timer_callback(self):
        msg = String()
        msg.data = 'robots: {}'
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
