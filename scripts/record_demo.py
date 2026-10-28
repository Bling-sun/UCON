#!/usr/bin/env python3
"""Record the real ROS 2/RViz simulation on a private display and ROS domain."""

import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from obj_state_msgs.msg import ObjectsStates
from traj_utils.msg import PolyTraj
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
import yaml


def avoided_conflicts(telemetry):
    """Compare each replaced plan with the subsequently observed moving objects.

    This is an offline counterfactual, not a planner collision detector: retain
    the remaining old polynomial and check its distance to the recorded object
    AABBs, stopping at its end or the next goal. A conflict means less than
    0.25 m point-to-box clearance; the flown path is validated separately.
    """
    odometry = np.asarray(telemetry['odom'])
    events = {}
    for old, new in zip(telemetry['trajectories'], telemetry['trajectories'][1:]):
        start = new['stamp']
        end = min(old['stamp'] + sum(old['duration']),
                  min((t for t, _ in telemetry['goals'] if t > start), default=float('inf')))
        if any(old['stamp'] < t <= start + .1 for t, _ in telemetry['goals']):
            continue
        for stamp, objects in telemetry['truth']:
            if not start < stamp < end:
                continue
            local_t = stamp - old['stamp']
            for piece, duration in enumerate(old['duration']):
                if local_t <= duration:
                    break
                local_t -= duration
            predicted = np.array([np.polyval(np.asarray(old[key]).reshape(-1, 6)[piece], local_t)
                                  for key in ('coef_x', 'coef_y', 'coef_z')])
            objects = np.asarray(objects)
            actual = np.array([np.interp(stamp, odometry[:, 0], odometry[:, k]) for k in range(1, 4)])
            distance = np.linalg.norm(np.maximum(abs(objects[:, :3] - predicted) - objects[:, 3:6]/2, 0), axis=1)
            flown_distance = np.linalg.norm(np.maximum(abs(objects[:, :3] - actual) - objects[:, 3:6]/2, 0), axis=1)
            moving = np.linalg.norm(objects[:, 6:9], axis=1) > .05
            for index in np.where((distance < .25) & moving & (flown_distance >= .25))[0]:
                event = {'object_id': int(index),
                         'replan_s': start - telemetry['recording_start'],
                         'conflict_s': stamp - telemetry['recording_start'],
                         'previous_plan_clearance_m': float(distance[index]),
                         'flown_clearance_m': float(flown_distance[index])}
                if index not in events or distance[index] < events[index]['previous_plan_clearance_m']:
                    events[index] = event
    return sorted(events.values(), key=lambda event: event['replan_s'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=40, help='Maximum recording time; stop after all goals are reached')
    parser.add_argument('--domain', type=int, default=182)
    parser.add_argument('--output', default='assets/demo.gif')
    parser.add_argument('--speed', type=float, default=2.)
    parser.add_argument('--static-obstacles', type=int, default=110)
    parser.add_argument('--dynamic-obstacles', type=int, default=72)
    parser.add_argument('--map-seed', type=int, default=3728542750)
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

    if args.speed <= 0 or args.static_obstacles < 1 or args.dynamic_obstacles < 1:
        parser.error('Speed and obstacle count must be positive')

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
    manager['Views']['Current'].update(Distance=18.0, Pitch=1.40, Yaw=1.57)
    manager['Views']['Current']['Focal Point'] = {'X': .5, 'Y': .5, 'Z': .3}
    manager['Displays'].append({
        'Class': 'rviz_default_plugins/Marker', 'Name': 'UAV label',
        'Enabled': True, 'Value': True,
        'Topic': {'Value': '/demo/annotations', 'Depth': 10,
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
        start([str(root / 'run_simulation.sh'), 'rviz:=false',
               f'static_obstacles:={args.static_obstacles}',
               f'dynamic_obstacles:={args.dynamic_obstacles}',
               f'map_seed:={args.map_seed}'], 'simulation')
        start(['/opt/ros/jazzy/lib/rviz2/rviz2', '-d', str(config_path)], 'rviz')
        rclpy.init()
        node = rclpy.create_node('ucon_demo_director')
        odom, maps, truth, trajectories = [], [], [], []
        def stamp(message):
            return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

        def save_truth(message):
            truth.append([stamp(message), [[s.position.x, s.position.y, s.position.z,
                                           s.size.x, s.size.y, s.size.z,
                                           s.velocity.x, s.velocity.y, s.velocity.z]
                                          for s in message.states]])

        def save_trajectory(message):
            trajectories.append({'id': message.traj_id,
                                 'stamp': message.start_time.sec + message.start_time.nanosec / 1e9,
                                 'duration': list(message.duration),
                                 'coef_x': list(message.coef_x),
                                 'coef_y': list(message.coef_y),
                                 'coef_z': list(message.coef_z)})
        subscriptions = [
            node.create_subscription(Odometry, '/Odometry', odom.append, 100),
            node.create_subscription(ObjectsStates, '/map_generator/obj_truth', save_truth, 100),
            node.create_subscription(PolyTraj, '/planning/trajectory', save_trajectory, 100),
            node.create_subscription(PointCloud2, '/map_generator/global_cloud', maps.append, 1),
        ]
        goal_pub = node.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        annotation_pub = node.create_publisher(Marker, '/demo/annotations', 10)
        deadline = time.monotonic() + 20
        while not odom or not maps or not truth or goal_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation startup timed out; see log/demo/')
            rclpy.spin_once(node, timeout_sec=.05)
        points = point_cloud2.read_points_numpy(maps[-1], field_names=('x', 'y', 'z'))
        planned_goals = []
        for desired in ((5., 0.), (-3.4, 3.4), (4., -4.), (-4., -4.)):
            candidates = [(desired[0] + dx, desired[1] + dy, .4)
                          for dx in (-.6, 0., .6) for dy in (-.6, 0., .6)]
            clear = [p for p in candidates if np.linalg.norm(points - p, axis=1).min() > .8]
            if clear:
                planned_goals.append(min(clear, key=lambda p: math.hypot(p[0]-desired[0], p[1]-desired[1])))
        if len(planned_goals) != 4:
            raise RuntimeError('No free demonstration goals in this scene')
        ready = time.monotonic() + 2
        while time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.05)
        video = work / 'demo.mkv'
        recorder = start([ffmpeg, '-y', '-f', 'x11grab', '-draw_mouse', '0',
                          '-framerate', '15', '-video_size', '1280x800',
                          '-i', f':{display_id}.0', '-t', str(args.seconds),
                          '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '18',
                          '-pix_fmt', 'yuv420p', str(video)], 'capture')
        started = time.monotonic()
        recording_stamp = node.get_clock().now().nanoseconds / 1e9
        next_goal = started + 1
        goal_index = 0
        goals_sent, reached = [], []
        goal_active = False
        completed_at = None
        next_annotation = started
        while time.monotonic() - started < args.seconds + .5:
            rclpy.spin_once(node, timeout_sec=.03)
            p = odom[-1].pose.pose.position
            if time.monotonic() >= next_annotation:
                label = Marker()
                label.header.frame_id = 'world'
                label.header.stamp = node.get_clock().now().to_msg()
                label.ns, label.id, label.type = 'demo', 0, Marker.TEXT_VIEW_FACING
                label.pose.position.x, label.pose.position.y, label.pose.position.z = p.x, p.y, p.z + .8
                label.pose.orientation.w = 1.
                label.scale.z = .5
                label.color.r, label.color.g, label.color.b, label.color.a = .65, .12, .15, 1.
                label.text = 'UAV'
                annotation_pub.publish(label)
                next_annotation = time.monotonic() + .1
            if goal_index < len(planned_goals) and not goal_active and time.monotonic() >= next_goal:
                goal = PoseStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = planned_goals[goal_index]
                goal.pose.orientation.w = 1.
                goal_pub.publish(goal)
                goals_sent.append([stamp(goal), planned_goals[goal_index]])
                goal_active = True
            if goal_active and np.linalg.norm(np.array([p.x, p.y, p.z]) - planned_goals[goal_index]) < .35:
                reached.append([stamp(odom[-1]), goal_index])
                goal_index += 1
                goal_active = False
                next_goal = time.monotonic() + .25
                if goal_index == len(planned_goals):
                    completed_at = time.monotonic()
            if completed_at is not None and time.monotonic() - completed_at > .75:
                break
        recording_end = node.get_clock().now().nanoseconds / 1e9
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
        recorder.wait(timeout=10)
        if recorder.returncode not in (0, 255):
            raise RuntimeError('Recording failed; see log/demo/capture.log')
        odometry = [[stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                    for m in odom]
        telemetry = {'recording_start': recording_stamp, 'recording_end': recording_end,
                     'odom': odometry, 'truth': truth, 'trajectories': trajectories,
                     'goals': goals_sent, 'reached': reached}
        (work / 'telemetry.json').write_text(json.dumps(telemetry))
        points = np.asarray(points)
        np.save(work / 'static_points.npy', points)
        a = np.asarray([v for v in odometry if recording_stamp <= v[0] <= recording_end])
        distance = float(np.linalg.norm(np.diff(a[:, 1:], axis=0), axis=1).sum())
        closest, near_ids, samples = float('inf'), set(), []
        for t, objects in truth:
            if not a[0, 0] <= t <= a[-1, 0]:
                continue
            pos = np.array([np.interp(t, a[:, 0], a[:, k]) for k in range(1, 4)])
            objects = np.asarray(objects)
            if len(objects) != args.dynamic_obstacles:
                raise RuntimeError('Incomplete ground-truth frame')
            distances = np.linalg.norm(np.maximum(abs(objects[:, :3] - pos) - objects[:, 3:6]/2, 0), axis=1)
            closest = min(closest, float(distances.min()))
            near_ids.update(np.where((distances < 1.5) & (np.linalg.norm(objects[:, 6:9], axis=1) > .05))[0].tolist())
            samples.append(t)
        static_clearance = float(cKDTree(points).query(a[:, 1:])[0].min())
        simulation_log = (work / 'simulation.log').read_text()
        # A safety replan follows a prediction of conflict, not a physical collision.
        replans = len(re.findall(r'Plan success when detect collision', simulation_log))
        dynamic_warnings = len(re.findall(r'detect collision with objects', simulation_log))
        conflicts = avoided_conflicts(telemetry)
        summary = {'avoidance_comparison': 'remaining_previous_plan_until_next_goal',
                   'avoided_conflicts': conflicts, 'static_obstacles': args.static_obstacles, 'dynamic_obstacles': args.dynamic_obstacles,
                   'map_seed': args.map_seed, 'playback_speed': args.speed,
                   'recorded_seconds': recording_end - recording_stamp,
                   'goals_reached': len(reached), 'goals_total': len(planned_goals),
                   'path_m': distance, 'min_center_to_dynamic_aabb_m': closest,
                   'min_center_to_static_point_m': static_clearance,
                   'moving_objects_within_1_5m': sorted(near_ids),
                   'safety_replans': replans, 'dynamic_prediction_warnings': dynamic_warnings,
                   'trajectory_updates': len(trajectories), 'truth_samples': len(samples),
                   'max_truth_gap_s': float(np.max(np.diff(samples))) if len(samples) > 1 else None,
                   'max_odom_gap_s': float(np.max(np.diff(a[:, 0]))),
                   'validation': 'pending'}
        valid = (len(reached) == len(planned_goals) and distance > 15 and closest >= .25
                 and static_clearance >= .25 and len(near_ids) >= 3 and replans >= 3
                 and dynamic_warnings > 0 and len(conflicts) >= 2 and len(samples) > 1
                 and summary['max_truth_gap_s'] < .2 and summary['max_odom_gap_s'] < .1
                 and samples[0] - a[0, 0] < .2 and a[-1, 0] - samples[-1] < .2)
        summary['validation'] = 'passed' if valid else 'failed'
        (work / 'metrics.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)
        if not valid:
            raise RuntimeError('Demo did not pass arrival, multiple-conflict, clearance or telemetry checks; see log/demo/metrics.json')
        filters = (
            f'setpts=(PTS-STARTPTS)/{args.speed},fps=20,crop=1250:690:15:80,scale=800:442:flags=lanczos,'
            'pad=800:520:0:40:color=0xF6F5F4,'
            f"drawtext=text='UCON | Multiple moving obstacles | {args.speed:g}x':x=22:y=10:fontsize=20:fontcolor=0x23334A,"
            f"drawtext=text='{args.static_obstacles} static / {args.dynamic_obstacles} moving | Ground-truth boxes within 4.5 m':x=22:y=496:fontsize=13:fontcolor=0x42546A,"
            'split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];'
            '[b][p]paletteuse=dither=none:diff_mode=rectangle'
        )
        encoded = output.with_name(output.stem + '.pending.gif')
        try:
            subprocess.run([ffmpeg, '-y', '-i', str(video), '-filter_complex', filters,
                            '-loop', '0', str(encoded)], check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            encoded.replace(output)
        finally:
            encoded.unlink(missing_ok=True)
        summary['gif_sha256'] = hashlib.sha256(output.read_bytes()).hexdigest()
        output.with_suffix('.metrics.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(f'Recorded {recording_end - recording_stamp:.1f}s, flown path {distance:.1f} m, GIF {output.stat().st_size/1e6:.2f} MB: {output}')
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
