from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation_and_perception = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('perception'), 'launch', 'sim_perception.launch.py'
            ])), launch_arguments={'rviz': 'false'}.items())])

    planner = Node(
        package='fapp_planner',
        executable='fapp_planner_node',
        output='screen',
        parameters=[{
            'fsm.flight_type': ParameterValue(LaunchConfiguration('flight_type'), value_type=int),
            'fsm.waypoint_num': 1,
            'fsm.waypoint0.x': ParameterValue(LaunchConfiguration('target_x'), value_type=float),
            'fsm.waypoint0.y': ParameterValue(LaunchConfiguration('target_y'), value_type=float),
            'fsm.waypoint0.z': ParameterValue(LaunchConfiguration('target_z'), value_type=float),
            'fsm.manual_goal_z': ParameterValue(LaunchConfiguration('target_z'), value_type=float),
            'fsm.planning_horizon': 7.5,
            'fsm.thresh_replan_time': 1.0,
            'fsm.emergency_time': 1.0,
            'fsm.safety_replan_interval': ParameterValue(LaunchConfiguration('safety_replan_interval'), value_type=float),
            'fsm.object_clearance': ParameterValue(LaunchConfiguration('object_clearance'), value_type=float),
            'fsm.object_state_timeout': 0.5,
            'fsm.realworld_experiment': False,
            'fsm.fail_safe': True,
            'manager.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'optimization.max_vel': ParameterValue(LaunchConfiguration('max_vel'), value_type=float),
            'manager.max_acc': 10.0,
            'manager.planning_horizon': 7.5,
            'manager.drone_id': -1,
            'grid_map.resolution': 0.1,
            'grid_map.local_update_range_x': 5.5,
            'grid_map.local_update_range_y': 5.5,
            'grid_map.local_update_range_z': 2.0,
            'grid_map.obstacles_inflation': 0.1,
        }],
    )
    trajectory_server = Node(
        package='fapp_planner', executable='traj_server', output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('perception'), 'launch', 'rviz', 'ucon_ros2.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('max_vel', default_value='3.5'),
        DeclareLaunchArgument('safety_replan_interval', default_value='0.5'),
        DeclareLaunchArgument('object_clearance', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('flight_type', default_value='1'),
        DeclareLaunchArgument('target_x', default_value='3.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='0.4'),
        DeclareLaunchArgument('static_obstacles', default_value='60'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='36'),
        DeclareLaunchArgument('map_seed', default_value='3728542744'),
        DeclareLaunchArgument('use_ground_truth', default_value='true'),
        DeclareLaunchArgument('truth_range', default_value='4.5'),
        simulation_and_perception, planner, trajectory_server, rviz])
