#!/usr/bin/env python3
"""Record the real ROS 2/RViz simulation on a private display and ROS domain."""

import argparse
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=24)
    parser.add_argument('--domain', type=int, default=182)
    parser.add_argument('--output', default='assets/demo.gif')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    os.chdir(root)
    ffmpeg, xvfb = shutil.which('ffmpeg'), shutil.which('Xvfb')
    if not ffmpeg or not xvfb:
        parser.error('Install ffmpeg and xvfb, and source install_ucon1/local_setup.bash first.')
    if args.domain == int(os.environ.get('ROS_DOMAIN_ID', '0')):
        parser.error('Choose a ROS domain different from the interactive simulation.')
    if args.seconds < 8:
        parser.error('--seconds must be at least 8')

    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    work = root / 'log' / 'demo'
    work.mkdir(parents=True, exist_ok=True)
    display_id = next(i for i in range(90, 110)
                      if not Path(f'/tmp/.X11-unix/X{i}').exists()
                      and not Path(f'/tmp/.X{i}-lock').exists())
    os.environ.update(DISPLAY=f':{display_id}', ROS_DOMAIN_ID=str(args.domain),
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', LIBGL_ALWAYS_SOFTWARE='1',
                      QT_QPA_PLATFORM='xcb')
    os.environ.pop('ROS_LOCALHOST_ONLY', None)

    config = yaml.safe_load((root / 'src/perception/launch/rviz/ucon_ros2.rviz').read_text())
    config['Panels'] = []
    config['Window Geometry'] = {'Width': 1280, 'Height': 800, 'X': 0, 'Y': 0,
                                 'Hide Left Dock': True, 'Hide Right Dock': True}
    manager = config['Visualization Manager']
    manager['Views']['Current'].update(Distance=25.0, Pitch=1.25, Yaw=1.57)
    manager['Views']['Current']['Focal Point'] = {'X': 0, 'Y': 0, 'Z': .3}
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/Marker', 'Name': '4.5 m sensing range',
        'Enabled': True, 'Value': True,
        'Topic': {'Value': '/demo/sensing_range', 'Depth': 1,
                  'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile'},
    })
    # The actual flown path makes movement legible in a compact README image.
    for group in manager['Displays']:
        for display in group.get('Displays', []):
            if display.get('Name') == 'Actual Flight Path':
                display.update(Enabled=True, Value=True, Color='27; 133; 153')
    config_path = work / 'demo.rviz'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))

    processes = []
    logs = []
    node = None
    try:
        def start(command, name):
            log = (work / f'{name}.log').open('w')
            logs.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            processes.append(process)
            return process

        screen = start([xvfb, f':{display_id}', '-screen', '0', '1280x800x24',
                        '-nolisten', 'tcp', '-noreset', '-ac'], 'display')
        deadline = time.monotonic() + 10
        while not Path(f'/tmp/.X11-unix/X{display_id}').exists():
            if screen.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Virtual display failed; see log/demo/display.log')
            time.sleep(.1)
        start([str(root / 'run_simulation.sh'), 'rviz:=false'], 'simulation')
        start(['/opt/ros/jazzy/lib/rviz2/rviz2', '-d', str(config_path)], 'rviz')
        rclpy.init()
        node = rclpy.create_node('ucon_demo_director')
        odom, maps = [], []
        subscriptions = [
            node.create_subscription(Odometry, '/Odometry', odom.append, 10),
            node.create_subscription(PointCloud2, '/map_generator/global_cloud', maps.append, 1),
        ]
        goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        range_pub = node.create_publisher(Marker, '/demo/sensing_range', 1)
        deadline = time.monotonic() + 20
        while not odom or not maps or goal_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation startup timed out; see log/demo/')
            rclpy.spin_once(node, timeout_sec=.05)
        points = point_cloud2.read_points_numpy(maps[-1], field_names=('x', 'y', 'z'))
        planned_goals = []
        for desired in ((4., 0.), (4., 4.), (-4., 4.), (-4., -3.)):
            candidates = [(desired[0] + dx, desired[1] + dy, .4)
                          for dx in (-.6, 0., .6) for dy in (-.6, 0., .6)]
            clear = [p for p in candidates if np.linalg.norm(points - p, axis=1).min() > .8]
            if clear:
                planned_goals.append(min(clear, key=lambda p: math.hypot(p[0]-desired[0], p[1]-desired[1])))
        if not planned_goals:
            raise RuntimeError('No free demonstration goals in this scene')
        ready = time.monotonic() + 2
        while time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.05)
        video = work / 'demo.mkv'
        recorder = start([ffmpeg, '-y', '-f', 'x11grab', '-draw_mouse', '0',
                          '-framerate', '12', '-video_size', '1280x800',
                          '-i', f':{display_id}.0', '-t', str(args.seconds),
                          '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '18',
                          '-pix_fmt', 'yuv420p', str(video)], 'capture')
        started = time.monotonic()
        next_goal = started + 1
        goal_index = 0
        path = []
        while time.monotonic() - started < args.seconds + .5:
            rclpy.spin_once(node, timeout_sec=.03)
            p = odom[-1].pose.pose.position
            path.append((p.x, p.y))
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = node.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'demo_range', 0, Marker.LINE_STRIP
            marker.pose.orientation.w = 1.
            marker.scale.x = .035
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .55, .65, .65
            marker.points = [Point(x=p.x + 4.5*math.cos(t), y=p.y + 4.5*math.sin(t), z=p.z)
                             for t in np.linspace(0, 2*math.pi, 100)]
            range_pub.publish(marker)
            label = Marker()
            label.header = marker.header
            label.ns, label.id, label.type = 'demo_range', 1, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = p.x, p.y, p.z + .8
            label.pose.orientation.w = 1.
            label.scale.z = .5
            label.color.r, label.color.g, label.color.b, label.color.a = .65, .12, .15, 1.
            label.text = 'UAV'
            range_pub.publish(label)
            if goal_index < len(planned_goals) and time.monotonic() >= next_goal:
                goal = PoseStamped()
                goal.header = marker.header
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = planned_goals[goal_index]
                goal.pose.orientation.w = 1.
                goal_pub.publish(goal)
                goal_index += 1
                next_goal += args.seconds / len(planned_goals)
        recorder.wait(timeout=10)
        if recorder.returncode:
            raise RuntimeError('Recording failed; see log/demo/capture.log')
        distance = np.linalg.norm(np.diff(np.array(path), axis=0), axis=1).sum()
        if distance < 2:
            raise RuntimeError(f'Drone moved only {distance:.2f} m; inspect planner log before publishing demo')
        filters = (
            'fps=10,crop=1250:690:15:80,scale=800:442:flags=lanczos,'
            'pad=800:520:0:40:color=0xF6F5F4,'
            "drawtext=text='UCON | Dynamic obstacle avoidance':x=22:y=10:fontsize=20:fontcolor=0x23334A,"
            "drawtext=text='60 static / 36 moving | Ground-truth boxes within 4.5 m':x=22:y=496:fontsize=13:fontcolor=0x42546A,"
            'split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];'
            '[b][p]paletteuse=dither=none:diff_mode=rectangle'
        )
        subprocess.run([ffmpeg, '-y', '-i', str(video), '-filter_complex', filters,
                        '-loop', '0', str(output)], check=True, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        print(f'Recorded {args.seconds}s, flown path {distance:.1f} m, GIF {output.stat().st_size/1e6:.2f} MB: {output}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
