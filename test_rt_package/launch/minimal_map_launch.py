# minimal_map_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gebruik /clock (true/false)'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/wheeltec/wheeltec_ros2/src/test_rt_package/map/WHEELTEC.yaml',
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
    ])

