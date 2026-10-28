from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ucon_sim', executable='kinematic_sim', output='screen'),
        Node(package='ucon_sim', executable='sim_monitor', output='screen'),
    ])
