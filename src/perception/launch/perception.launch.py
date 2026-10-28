from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[{
                'dbscan.core_pts': 3,
                'dbscan.tolerance': 0.20,
                'dbscan.min_cluster': 10,
                'dbscan.max_cluster': 400,
                'detection.thresh_dist': 0.05,
                'detection.thresh_var': 0.26,
                'filtering.floor': 0.04,
                'filtering.ceiling': 0.60,
                'lidar.range_x': 8.0,
                'lidar.range_y': 8.0,
                'lidar.range_z': 2.2,
                'odom_topic': '/Odometry',
                'lidar_topic': '/cloud_registered_body',
            }],
        ),
    ])
