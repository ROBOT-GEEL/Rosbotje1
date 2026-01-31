import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    GroupAction, OpaqueFunction
)
from launch_ros.actions import LifecycleNode, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def include_lidar_launch(context, *args, **kwargs):
    # 1. Parameters ophalen
    yaml_path = LaunchConfiguration('lidar_type_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))
    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    print(f'Starten met lidar_type: {lidar_type}')

    actions = []

    # 2. Lidar Driver Configuratie
    if lidar_type.startswith('ls'):
        if lidar_type == 'lscx':
            template_yaml = Path(get_package_share_directory('lslidar_driver'), 'config', 'lslidar_cx.yaml')
            cx_cfg = yaml.safe_load(template_yaml.read_text())['cx']['lslidar_driver_node']['ros__parameters']
            
            lidar_launch = GroupAction(
                actions=[
                    LifecycleNode(
                        package='lslidar_driver',
                        executable='lslidar_driver_node',
                        name='lslidar_driver_node',
                        namespace='cx',
                        parameters=[cx_cfg],
                        #remappings=[('/scan', '/scan_raw')], # Remapping voor cx filter RT
                        output='screen'
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_launch.py'))
                    )
                ]
            )
        else:
            # LS_X10 serie (M10/N10 etc)
            template_yaml = Path(get_package_share_directory('lslidar_driver'), 'config', 'lslidar_x10.yaml')
            x10_cfg = yaml.safe_load(template_yaml.read_text())['x10']['lslidar_driver_node']['ros__parameters']
            lidar_port = cfg['x10']['lidar_port']

            if lidar_type.endswith('net'): x10_cfg['serial_port'] = ''
            elif lidar_type.endswith('uart'): x10_cfg['serial_port'] = lidar_port

            # Model selectie
            if lidar_type.startswith('ls_M10'):
                x10_cfg['lidar_model'] = 'M10P' if lidar_type.startswith('ls_M10P') else 'M10'
            elif lidar_type.startswith('ls_N10'):
                x10_cfg['lidar_model'] = 'N10Plus' if lidar_type.startswith('ls_N10Plus') else 'N10'

            # Hoek-instellingen (Integers voor driver)
            # RT orig de 3 regels hieronder
            #if cfg['x10']['angle_disable_min'] != 0:
                #x10_cfg['angle_disable_min'] = int(cfg['x10']['angle_disable_min'])
                #x10_cfg['angle_disable_max'] = int(cfg['x10']['angle_disable_max'])
                
            # Hoek-instellingen (Zorg dat we alleen het eerste element pakken als het een lijst is)
            angle_min = cfg['x10']['angle_disable_min'] #RT bijgevoegd
            angle_max = cfg['x10']['angle_disable_max'] #RT bijgevoegd

            if angle_min != 0:
                # We maken er altijd een lijst van: [getal]
                val_min = angle_min[0] if isinstance(angle_min, list) else angle_min
                val_max = angle_max[0] if isinstance(angle_max, list) else angle_max
                x10_cfg['angle_disable_min'] = [int(val_min)]
                x10_cfg['angle_disable_max'] = [int(val_max)]
            else:
                # Als het 0 is, stuur een lege lijst om de driver tevreden te stellen
                x10_cfg['angle_disable_min'] = []
                x10_cfg['angle_disable_max'] = []


            lidar_launch = LifecycleNode(
                package='lslidar_driver',
                executable='lslidar_driver_node',
                name='lslidar_driver_node',
                namespace='x10',
                parameters=[x10_cfg],
                output='screen'#,
                #remappings=[('/scan', '/scan_raw')]  RT terug weg gedaan
            )

    # Overige modellen met toegevoegde remapping
    elif lidar_type == 'ld14':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar_sl_ros2'), 'launch','ld14.launch.py')),
            launch_arguments={'topic_name': 'scan_raw'}.items() # Direct remapping via argument indien ondersteund
        )
    elif lidar_type == 'ld06':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch','ld06.launch.py')),
        )
    elif lidar_type == 'rplidar_c1':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch','rplidar_c1_launch.py')),
        )
    else:
        raise ValueError(f'Unsupported lidar: {lidar_type}')

    actions.append(lidar_launch)

    # 3. Laser Filter Node Toevoegen (Slechts één keer hier)
#    laser_filter_config = os.path.join(
#        get_package_share_directory('turn_on_wheeltec_robot'),
#        'config',
#        'laser_filter.yaml'
#   )

#    laser_filter_node = Node(
#        package='laser_filters',
#       executable='scan_to_scan_filter_chain',
#        name='laser_filter',
#        parameters=[laser_filter_config],
#       remappings=[
#            ('scan', 'scan_raw'),        # Input
#            ('scan_filtered', 'scan')    # Output naar Nav2
#       ],
#        output='screen'
#    )
    
#    actions.append(laser_filter_node)

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type_yaml',
            default_value=os.path.join(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'wheeltec_param.yaml'),
            description='Path to lidar_type.yaml'),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Lidar model'),
        OpaqueFunction(function=include_lidar_launch),
    ])

