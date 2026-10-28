"""Exercise simulator truth boxes and the independent static mapping input."""

import os
import signal
import subprocess
import time
import unittest

import numpy as np
import rclpy
from obj_state_msgs.msg import ObjectsStates, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class GroundTruthPerceptionTest(unittest.TestCase):
    def test_separate_overlapping_objects_exact_bounds_and_static_input(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        process = subprocess.Popen([
            'ros2', 'run', 'perception', 'mapping_node', '--ros-args',
            '-p', 'use_ground_truth:=true', '-p', 'visualization.box_padding:=0.05',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('ground_truth_perception_test')
        boxes, states, static = [], [], []
        subscriptions = [
            node.create_subscription(MarkerArray, '/box_edge', boxes.append, 10),
            node.create_subscription(ObjectsStates, '/states', states.append, 10),
            node.create_subscription(PointCloud2, '/static_map', static.append, 10),
        ]
        truth_pub = node.create_publisher(ObjectsStates, '/map_generator/obj_truth', 1)
        static_pub = node.create_publisher(PointCloud2, '/cloud_registered_static', 1)
        odom_pub = node.create_publisher(Odometry, '/Odometry', 1)

        def spin_until(condition):
            deadline = time.monotonic() + 8
            while not condition():
                self.assertIsNone(process.poll())
                self.assertLess(time.monotonic(), deadline, 'Timed out waiting for perception')
                rclpy.spin_once(node, timeout_sec=.02)

        try:
            spin_until(lambda: truth_pub.get_subscription_count() and
                       static_pub.get_subscription_count() and
                       odom_pub.get_subscription_count() and
                       all(s.get_publisher_count() for s in subscriptions))
            odom = Odometry()
            odom.header.frame_id = 'world'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.pose.position.z = .4
            # Deliver pose before truth; truth boxes must follow the drone's range.
            for _ in range(4):
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=.05)
            # Objects deliberately overlap: clustering must not merge their IDs.
            for displacement in (0.0, 1.7):
                truth = ObjectsStates()
                truth.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='world')
                for index in range(36):
                    state = State()
                    state.header = truth.header
                    state.position.x = 1.0 + index * .01 + displacement
                    state.position.y = .5
                    state.position.z = .25
                    state.size.x, state.size.y, state.size.z = .6, .4, 3.3
                    state.velocity.x = .8
                    truth.states.append(state)
                truth_pub.publish(truth)
                spin_until(lambda: any(m.header.stamp == truth.header.stamp
                                       for b in boxes for m in b.markers))
                spin_until(lambda: any(s.header.stamp == truth.header.stamp for s in states))
                current = [m for b in boxes for m in b.markers
                           if m.header.stamp == truth.header.stamp and m.action == Marker.ADD]
                self.assertEqual(len(current), 36)
                self.assertEqual({m.id for m in current}, set(range(36)))
                for marker in current:
                    state = truth.states[marker.id]
                    vertices = np.array([(p.x, p.y, p.z) for p in marker.points])
                    center = np.array([state.position.x, state.position.y, state.position.z])
                    half_size = np.array([.6, .4, 3.3]) / 2 + .05
                    np.testing.assert_allclose(vertices.min(axis=0), center - half_size, atol=1e-6)
                    np.testing.assert_allclose(vertices.max(axis=0), center + half_size, atol=1e-6)
                output = next(s for s in states if s.header.stamp == truth.header.stamp)
                self.assertEqual(len(output.states), 36)
                self.assertTrue(all(abs(s.velocity.x - .8) < 1e-8 for s in output.states))

            # Static points inside a truth box must survive: source separation,
            # not bounding-box subtraction, determines the static map.
            cloud = point_cloud2.create_cloud_xyz32(truth.header, [(2.8, .5, .2), (8., 2., 1.)])
            static_pub.publish(cloud)
            spin_until(lambda: any(c.header.stamp == cloud.header.stamp for c in static))
            self.assertEqual(bytes(static[-1].data), bytes(cloud.data))

            # Move the drone away: unchanged obstacles must disappear, then
            # reappear with their original IDs when the drone returns.
            odom.pose.pose.position.x = 20.0
            for _ in range(4):
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=.05)
            truth.header.stamp = node.get_clock().now().to_msg()
            truth_pub.publish(truth)
            spin_until(lambda: any(s.header.stamp == truth.header.stamp for s in states))
            self.assertFalse(states[-1].states)
            spin_until(lambda: any(m.header.stamp == truth.header.stamp
                                   for b in boxes for m in b.markers))
            self.assertEqual({m.id for b in boxes for m in b.markers
                              if m.header.stamp == truth.header.stamp and m.action == Marker.DELETE},
                             set(range(36)))

            odom.pose.pose.position.x = 0.0
            for _ in range(4):
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=.05)
            # Boundary, outside, and vertically outside cases (3-D range).
            truth.states[0].position.x = 4.79  # surface at 4.49: inside
            truth.states[0].position.y = 0.0
            truth.states[1].position.x = 4.81  # surface at 4.51: outside
            truth.states[1].position.y = 0.0
            truth.states[2].position.z = 10.0
            truth.header.stamp = node.get_clock().now().to_msg()
            truth_pub.publish(truth)
            spin_until(lambda: any(m.header.stamp == truth.header.stamp
                                   for b in boxes for m in b.markers))
            visible = {m.id for b in boxes for m in b.markers
                       if m.header.stamp == truth.header.stamp and m.action == Marker.ADD}
            self.assertEqual(visible, set(range(36)) - {1, 2})

            empty = ObjectsStates()
            empty.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='world')
            truth_pub.publish(empty)
            spin_until(lambda: any(m.header.stamp == empty.header.stamp
                                   for b in boxes for m in b.markers))
            deleted = {m.id for b in boxes for m in b.markers
                       if m.header.stamp == empty.header.stamp and m.action == Marker.DELETE}
            self.assertEqual(deleted, visible)
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
