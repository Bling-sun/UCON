from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('perception'), 'launch', 'sim_perception.launch.py'
        ])
    ), launch_arguments={'rviz': 'false'}.items())
    grid_map = Node(
        package='plan_env',
        executable='grid_map_node',
        output='screen',
        parameters=[{
            'grid_map.resolution': 0.1,
            'grid_map.local_update_range_x': 5.5,
            'grid_map.local_update_range_y': 5.5,
            'grid_map.local_update_range_z': 2.0,
            'grid_map.obstacles_inflation': 0.1,
            'grid_map.enable_virtual_wall': True,
            'grid_map.virtual_ceil': 5.0,
            'grid_map.virtual_ground': -0.1,
            'grid_map.fading_time': 1000.0,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('perception'), 'launch', 'rviz', 'ucon_ros2.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        perception_launch, grid_map, rviz])
