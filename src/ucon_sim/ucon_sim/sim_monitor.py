import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class SimulationMonitor(Node):
    def __init__(self):
        super().__init__('ucon_sim_monitor')
        self.odom_count = 0
        self.cloud_count = 0
        self.first_position = None
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, 10)
        self.timer = self.create_timer(2.0, self.report)

    def on_odom(self, msg):
        self.odom_count += 1
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.first_position is None:
            self.first_position = position

    def on_cloud(self, _msg):
        self.cloud_count += 1

    def report(self):
        healthy = self.odom_count >= 20 and self.cloud_count >= 20
        state = 'PASS' if healthy else 'WAIT'
        self.get_logger().info(
            f'SIM_CHECK={state} odom={self.odom_count} cloud={self.cloud_count}')
        self.odom_count = 0
        self.cloud_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = SimulationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
