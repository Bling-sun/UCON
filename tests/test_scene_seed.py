"""Verify that a fixed seed reproduces the static scene in fresh processes."""

import os
import signal
import subprocess
import time
import unittest

import rclpy
from sensor_msgs.msg import PointCloud2


class SceneSeedTest(unittest.TestCase):
    def test_same_seed_repeats_and_different_seed_changes_map(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        rclpy.init()
        node = rclpy.create_node('scene_seed_test')
        try:
            snapshots = []
            for index, seed in enumerate((3728542744, 3728542744, 3728542745)):
                messages = []
                topic = f'/scene_seed_test/map_{index}'
                subscription = node.create_subscription(PointCloud2, topic, messages.append, 10)
                process = subprocess.Popen([
                    'ros2', 'run', 'map_generator', 'random_forest', '--ros-args',
                    '-p', f'map.seed:={seed}', '-p', 'map.obs_num:=60',
                    '-p', 'map.moving_obs_num:=36', '-p', 'map.circle_num:=0',
                    '-r', f'/map_generator/global_cloud:={topic}',
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True)
                try:
                    deadline = time.monotonic() + 10
                    while not messages:
                        self.assertIsNone(process.poll())
                        self.assertLess(time.monotonic(), deadline)
                        rclpy.spin_once(node, timeout_sec=.02)
                    snapshots.append(bytes(messages[0].data))
                finally:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                    node.destroy_subscription(subscription)
            self.assertEqual(snapshots[0], snapshots[1])
            self.assertNotEqual(snapshots[0], snapshots[2])
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
