"""Simulated truth input or mixed-cloud perception for ROS 2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mapping = Node(
        package='perception',
        executable='mapping_node',
        name='mapping_node',
        output='screen',
        parameters=[{
            'use_ground_truth': ParameterValue(LaunchConfiguration('use_ground_truth'), value_type=bool),
            'truth_range': ParameterValue(LaunchConfiguration('truth_range'), value_type=float),
            'dbscan.core_pts': 3,
            'dbscan.tolerance': 0.2,
            'dbscan.min_cluster': 15,
            'dbscan.max_cluster': 5000,
            'detection.thresh_dist': 0.02,
            'detection.thresh_var': 0.8,
            # Keep the full simulated columns, including their below-zero base.
            'filtering.floor': -2.0,
            'filtering.ceiling': 5.0,
            'visualization.box_padding': 0.05,
            'lidar.range_x': 15.0,
            'lidar.range_y': 15.0,
            'lidar.range_z': 5.0,
            'odom_topic': '/Odometry',
            'lidar_topic': '/cloud_registered_body',
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_ground_truth', default_value='true'),
        DeclareLaunchArgument('truth_range', default_value='4.5'),
        mapping,
    ])
