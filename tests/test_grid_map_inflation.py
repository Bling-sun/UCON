"""Exercise the real grid-map node in an unused ROS_DOMAIN_ID."""

import os
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class GridInflationTest(unittest.TestCase):
    def check_map(self, fading_time, hit_threshold, check_decay=False):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        executable = os.path.join(get_package_prefix('plan_env'), 'lib/plan_env/grid_map_node')
        process = subprocess.Popen([
            executable, '--ros-args',
            '-p', 'grid_map.local_update_range_x:=2.0',
            '-p', 'grid_map.local_update_range_y:=2.0',
            '-p', 'grid_map.local_update_range_z:=1.0',
            '-p', f'grid_map.fading_time:={fading_time}',
            '-p', f'grid_map.static_hit_thres:={hit_threshold}',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rclpy.init()
        node = rclpy.create_node('grid_inflation_test')
        messages = {'raw': [], 'inflated': []}
        subscriptions = [node.create_subscription(
            PointCloud2, topic,
            lambda m, key=key: messages[key].append(m.width * m.height), 10)
            for key, topic in [('raw', '/grid_map/occupancy'),
                               ('inflated', '/grid_map/occupancy_inflate')]]
        odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/static_map', 10)
        odom = Odometry()
        odom.header.frame_id = 'world'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.z = 0.4

        def pump(duration, publish_cloud=True):
            end = time.monotonic() + duration
            next_publish = 0
            while time.monotonic() < end:
                if time.monotonic() >= next_publish:
                    stamp = node.get_clock().now().to_msg()
                    odom.header.stamp = stamp
                    odom_pub.publish(odom)
                    if publish_cloud:
                        cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                            Header(stamp=stamp, frame_id='world'), [(1.05, 0.05, 0.25)]))
                    next_publish = time.monotonic() + 0.05
                rclpy.spin_once(node, timeout_sec=0.01)

        try:
            pump(3)
            self.assertIsNone(process.poll())
            self.assertTrue(any(messages['raw']), 'Obstacle never became occupied')
            self.assertTrue(any(messages['inflated']), 'Obstacle never became inflated')
            messages['raw'].clear()
            messages['inflated'].clear()
            pump(3)
            self.assertGreater(len(messages['raw']), 15)
            self.assertGreater(len(messages['inflated']), 15)
            self.assertTrue(all(messages['raw']), 'Stationary obstacle lost raw occupancy')
            self.assertTrue(all(messages['inflated']),
                            f'Inflated obstacle disappeared: {messages["inflated"]}')
            if check_decay:
                messages['raw'].clear()
                messages['inflated'].clear()
                pump(5, publish_cloud=False)
                self.assertEqual(messages['raw'][-1], 0, 'Old occupancy did not fade')
                self.assertEqual(messages['inflated'][-1], 0, 'Old inflation did not fade')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def test_stationary_obstacle_survives_hit_counter_reset(self):
        self.check_map(1000.0, 8)

    def test_unobserved_obstacle_and_inflation_fade_together(self):
        self.check_map(2.0, 1, check_decay=True)


if __name__ == '__main__':
    unittest.main()
