from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/wheeltec/wheeltec_ros2/src/test_rt_package/map/WHEELTEC.yaml',
            description='Pad naar map YAML'
        ),

        Node(
            package='test_rt_package',
            executable='static_map_publisher',
            name='static_map_publisher',
            output='screen',
            parameters=[{'map_yaml': map_yaml}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
    ])

