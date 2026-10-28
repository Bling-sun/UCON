"""Verify full-height simulated obstacle perception in an unused ROS_DOMAIN_ID."""

import os
import signal
import subprocess
import time
import unittest

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from obj_state_msgs.msg import ObjectsStates


class PerceptionExtentTest(unittest.TestCase):
    def test_full_height_cloud_tracking_and_static_removal(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        process = subprocess.Popen(['ros2', 'launch', 'perception', 'sim_mapping.launch.py',
                                    'use_ground_truth:=false'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('perception_extent_test')
        clouds, boxes, static, states = [], [], [], []
        expected_centers = {}
        subscriptions = [
            node.create_subscription(PointCloud2, '/map_ros', clouds.append, 10),
            node.create_subscription(MarkerArray, '/box_edge', boxes.append, 10),
            node.create_subscription(PointCloud2, '/static_map', static.append, 10),
            node.create_subscription(ObjectsStates, '/states', states.append, 10),
        ]
        odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        points = np.array([(i * .15, j * .15, -.14 * 10 + k * .15)
                           for i in range(7) for j in range(7) for k in range(23)
                           if i in (0, 6) or j in (0, 6)], dtype=np.float32)
        try:
            end = time.monotonic() + 6
            start_stamp = node.get_clock().now().nanoseconds / 1e9
            next_frame, frame = 0, 0
            while time.monotonic() < end:
                if time.monotonic() >= next_frame:
                    stamp = node.get_clock().now().to_msg()
                    odom = Odometry()
                    odom.header = Header(stamp=stamp, frame_id='world')
                    odom.pose.pose.orientation.w = 1.0
                    odom.pose.pose.position.z = .4
                    odom_pub.publish(odom)
                    elapsed = stamp.sec + stamp.nanosec / 1e9 - start_stamp
                    moved = points + np.array([1 + elapsed * .8, .5, 0], dtype=np.float32)
                    expected_centers[(stamp.sec, stamp.nanosec)] = (
                        moved.min(axis=0) + moved.max(axis=0)) / 2
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=stamp, frame_id='world'), moved))
                    # Vary frame intervals to catch hard-coded 20 ms velocity estimates.
                    next_frame = time.monotonic() + (.03, .07, .05)[frame % 3]
                    frame += 1
                rclpy.spin_once(node, timeout_sec=.01)
            self.assertIsNone(process.poll())
            self.assertTrue(clouds)
            cloud = point_cloud2.read_points_numpy(clouds[-1], field_names=('x', 'y', 'z'))
            self.assertAlmostEqual(float(cloud[:, 2].min()), -1.4, places=4)
            self.assertAlmostEqual(float(cloud[:, 2].max()), 1.9, places=4)
            markers = [m for msg in boxes[-10:] for m in msg.markers if m.points]
            self.assertTrue(markers, 'No tracked obstacle box')
            for marker in markers:
                self.assertAlmostEqual(min(p.z for p in marker.points), -1.45, places=4)
                self.assertAlmostEqual(max(p.z for p in marker.points), 1.95, places=4)
                stamp_key = (marker.header.stamp.sec, marker.header.stamp.nanosec)
                self.assertIn(stamp_key, expected_centers, 'Box lost the sensor timestamp')
                bounds = np.array([(p.x, p.y, p.z) for p in marker.points])
                np.testing.assert_allclose((bounds.min(axis=0) + bounds.max(axis=0)) / 2,
                                           expected_centers[stamp_key], atol=.02)
            velocities = [s.velocity.x for msg in states[-10:] for s in msg.states]
            self.assertTrue(velocities)
            self.assertAlmostEqual(float(np.median(velocities)), .8, delta=.15)
            self.assertTrue(static)
            self.assertTrue(all(m.width * m.height == 0 for m in static[-10:]),
                            'Dynamic obstacle top/bottom leaked into the static map')
            count = len(boxes)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=node.get_clock().now().to_msg(), frame_id='world'), []))
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            self.assertTrue(any(m.action == 2 for msg in boxes[count:] for m in msg.markers),
                            'Old box was not explicitly deleted after the object disappeared')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    unittest.main()
