"""Run after sourcing install_ucon1/setup.bash, in an unused ROS_DOMAIN_ID.

Checks the real perception executable with synthetic odometry and point clouds.
"""

import math
import os
import subprocess
import time
import unittest

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class MappingFramesTest(unittest.TestCase):
    def test_world_points_are_pose_invariant_and_body_points_are_transformed(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'),
                        'Set an unused ROS_DOMAIN_ID to isolate this test.')
        executable = os.path.join(get_package_prefix('perception'),
                                  'lib/perception/mapping_node')
        process = subprocess.Popen([executable], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        rclpy.init()
        node = rclpy.create_node('mapping_frames_test')
        received = []
        subscription = node.create_subscription(PointCloud2, '/map_ros', received.append, 10)
        odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        points = np.array([[2.0 + i * 0.1, 1.0 + j * 0.1, 0.2]
                           for i in range(5) for j in range(5)], dtype=np.float32)
        try:
            deadline = time.monotonic() + 10
            while odom_pub.get_subscription_count() == 0 or cloud_pub.get_subscription_count() == 0:
                self.assertIsNone(process.poll(), 'Perception exited during startup')
                self.assertLess(time.monotonic(), deadline, 'Perception discovery timed out')
                rclpy.spin_once(node, timeout_sec=0.1)

            cases = [('world', (0.0, 0.0, 0.4), 0.0),
                     ('world', (4.0, -3.0, 0.4), math.pi / 2),
                     ('base_link', (4.0, -3.0, 0.4), math.pi / 2)]
            for frame, position, yaw in cases:
                with self.subTest(frame=frame, position=position, yaw=yaw):
                    odom = Odometry()
                    odom.header.frame_id = 'world'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
                    odom.pose.pose.orientation.z = math.sin(yaw / 2)
                    odom.pose.pose.orientation.w = math.cos(yaw / 2)
                    for _ in range(5):
                        odom.header.stamp = node.get_clock().now().to_msg()
                        odom_pub.publish(odom)
                        rclpy.spin_once(node, timeout_sec=0.05)
                    stamp = node.get_clock().now().to_msg()
                    cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), points)
                    deadline = time.monotonic() + 5
                    output = None
                    while output is None and time.monotonic() < deadline:
                        cloud_pub.publish(cloud)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        output = next((msg for msg in received if msg.header.stamp == stamp), None)
                    self.assertIsNotNone(output, 'No output preserving the input timestamp')
                    self.assertEqual(output.header.frame_id, 'world')
                    actual = point_cloud2.read_points_numpy(output, field_names=('x', 'y', 'z'))
                    expected = points.copy()
                    if frame == 'base_link':
                        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                             [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
                        expected = points @ rotation.T + position
                    self.assertEqual(len(actual), len(expected))
                    # Compare point sets independent of voxel-filter ordering.
                    distances = np.linalg.norm(actual[:, None, :] - expected[None, :, :], axis=2)
                    self.assertLess(float(distances.min(axis=1).max()), 1e-5)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == '__main__':
    unittest.main()
