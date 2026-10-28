"""Launch the complete scene, check inputs, and send a navigation goal."""

import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from obj_state_msgs.msg import ObjectsStates
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import PolyTraj
from visualization_msgs.msg import MarkerArray


class SimulationSmokeTest(unittest.TestCase):
    def test_full_stack_and_navigation_goal(self):
        self.assertTrue(os.environ.get('ROS_DOMAIN_ID'))
        root = Path(__file__).resolve().parents[1]
        log_dir = root / 'log' / 'tests'
        log_dir.mkdir(parents=True, exist_ok=True)
        with (log_dir / 'simulation-smoke.log').open('w') as log:
            process = subprocess.Popen([
                str(root / 'run_simulation.sh'), 'rviz:=false',
                'static_obstacles:=60', 'dynamic_obstacles:=36',
                'map_seed:=3728542744', 'truth_range:=4.5',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('simulation_smoke_test')
            maps, static, truth, boxes, odom, trajectories = [], [], [], [], [], []
            subscriptions = [
                node.create_subscription(PointCloud2, '/map_generator/global_cloud', maps.append, 10),
                node.create_subscription(PointCloud2, '/static_map', static.append, 10),
                node.create_subscription(ObjectsStates, '/map_generator/obj_truth', truth.append, 10),
                node.create_subscription(MarkerArray, '/box_edge', boxes.append, 10),
                node.create_subscription(Odometry, '/Odometry', odom.append, 10),
                node.create_subscription(PolyTraj, '/planning/trajectory', trajectories.append, 10),
            ]
            goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

            def wait_for(condition, seconds=20):
                end = time.monotonic() + seconds
                while not condition():
                    self.assertIsNone(process.poll(), 'Simulation exited; see log/tests/simulation-smoke.log')
                    self.assertLess(time.monotonic(), end, 'Timed out; see log/tests/simulation-smoke.log')
                    rclpy.spin_once(node, timeout_sec=.02)

            try:
                wait_for(lambda: maps and static and truth and boxes and odom
                         and goal_pub.get_subscription_count() > 0)
                self.assertEqual(len(truth[-1].states), 36)
                self.assertGreater(static[-1].width, 0)
                self.assertEqual({info.node_name for info in
                                  node.get_subscriptions_info_by_topic('/move_base_simple/goal')},
                                 {'fapp_planner_node'})
                initial_map = bytes(maps[-1].data)
                points = point_cloud2.read_points_numpy(
                    maps[-1], field_names=('x', 'y', 'z'), skip_nans=True)
                # Pick a nearby goal with a clear straight corridor in this seed.
                goal_xy = None
                for angle in np.linspace(0, 2 * np.pi, 24, endpoint=False):
                    candidate = 1.2 * np.array([np.cos(angle), np.sin(angle)])
                    path = np.array([(candidate[0] * t, candidate[1] * t, .4)
                                     for t in np.linspace(0, 1, 12)])
                    if np.linalg.norm(points[:, None, :] - path[None, :, :], axis=2).min() > .65:
                        goal_xy = candidate
                        break
                self.assertIsNotNone(goal_xy, 'No nearby free test goal in the default seed')
                goal = PoseStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y = map(float, goal_xy)
                goal.pose.orientation.w = 1.0
                goal_pub.publish(goal)
                count = len(maps)
                wait_for(lambda: trajectories and len(maps) > count + 10)
                self.assertTrue(all(bytes(m.data) == initial_map for m in maps[count:]),
                                'Navigation goal inserted an obstacle')
                self.assertTrue(trajectories[-1].duration)
                wait_for(lambda: np.linalg.norm(np.array([
                    odom[-1].pose.pose.position.x, odom[-1].pose.pose.position.y])) > .1)
            finally:
                node.destroy_node()
                rclpy.shutdown()
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    unittest.main()
