"""Check colored surfaces, stable updates, and occupancy removal."""

import os
import time
import unittest

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ucon_sim.static_map_display import StaticMapDisplay


class StaticDisplayTest(unittest.TestCase):
    def test_single_surface_colors_and_unchanged_frames(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        rclpy.init()
        display = StaticMapDisplay()
        probe = rclpy.create_node('static_display_test')
        messages = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = probe.create_subscription(PointCloud2, '/ucon_sim/static_display', messages.append, qos)
        map_pub = probe.create_publisher(PointCloud2, '/map_generator/global_cloud', 1)
        occupancy_pub = probe.create_publisher(PointCloud2, '/grid_map/occupancy_inflate', 1)

        def spin(duration=.3):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(display, timeout_sec=.01)
                rclpy.spin_once(probe, timeout_sec=.01)

        points = [(1.05, .05, .25), (1.05, .05, .4), (1.05, .2, .4),
                  (2.05, .05, .25)]
        world = Header(frame_id='world')
        cloud = point_cloud2.create_cloud_xyz32(world, points)
        occupied = point_cloud2.create_cloud_xyz32(world, points[:1])
        grey, teal = 0xB2B2CE, 0x79B7B1
        try:
            spin(.5)
            map_pub.publish(cloud)
            spin()
            self.assertTrue(messages)
            np.testing.assert_array_equal(point_cloud2.read_points(messages[-1])['rgb'], [grey] * 4)
            occupancy_pub.publish(occupied)
            spin()
            data = point_cloud2.read_points(messages[-1])
            np.testing.assert_array_equal(data['rgb'], [teal, teal, teal, grey])
            self.assertEqual(messages[-1].width, 4, 'Overlay duplicated the surface points')
            for axis, expected in zip(('x', 'y', 'z'), np.array(points).T):
                np.testing.assert_allclose(data[axis], expected)
            count = len(messages)
            for _ in range(5):
                map_pub.publish(cloud)
                occupancy_pub.publish(occupied)
                spin(.1)
            self.assertEqual(len(messages), count, 'Unchanged geometry was republished')
            occupancy_pub.publish(point_cloud2.create_cloud_xyz32(world, []))
            spin()
            np.testing.assert_array_equal(point_cloud2.read_points(messages[-1])['rgb'], [grey] * 4)
            late = []
            late_sub = probe.create_subscription(PointCloud2, '/ucon_sim/static_display', late.append, qos)
            spin()
            self.assertTrue(late, 'Late RViz subscriber missed the cached surface')
        finally:
            display.destroy_node()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
