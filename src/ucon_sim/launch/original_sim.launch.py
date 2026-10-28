from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    odometry = Node(
        package='poscmd_2_odom', executable='poscmd_2_odom', output='screen',
        parameters=[{
            'init_x': 0.0,
            'init_y': 0.0,
            'init_z': 0.4,
        }],
        remappings=[('command', '/position_cmd'), ('odometry', '/Odometry')],
    )
    map_generator = Node(
        package='map_generator', executable='random_forest', output='screen',
        parameters=[{
            'map.x_size': 30.0,
            'map.y_size': 30.0,
            'map.z_size': 3.0,
            'map.obs_num': ParameterValue(LaunchConfiguration('static_obstacles'), value_type=int),
            'map.moving_obs_num': ParameterValue(LaunchConfiguration('dynamic_obstacles'), value_type=int),
            'map.seed': ParameterValue(LaunchConfiguration('map_seed'), value_type=int),
            'map.resolution': 0.15,
            'map.circle_num': 0,
            'publish_rate': 20.0,
            'min_distance': 1.0,
            'obstacle_shape.lower_height': 0.5,
            'obstacle_shape.upper_height': 2.0,
        }],
    )
    local_sensing = Node(
        package='local_sensing_node', executable='pcl_render_node', output='screen',
        parameters=[{'sensing_horizon': 8.0, 'sensing_rate': 30.0, 'resolution': 0.1}],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('dynamic_map', '/map_generator/obj_cloud'),
            ('odometry', '/Odometry'),
            ('pcl_render_node/cloud', '/cloud_registered_body'),
            ('pcl_render_node/static_cloud', '/cloud_registered_static'),
        ],
    )
    static_display = Node(
        package='ucon_sim', executable='static_map_display', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('static_obstacles', default_value='60'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='36'),
        DeclareLaunchArgument('map_seed', default_value='3728542744'),
        odometry, map_generator, local_sensing, static_display,
    ])
