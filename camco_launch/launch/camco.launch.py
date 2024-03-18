from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import os

def generate_launch_description():
    ld = LaunchDescription()

    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='False',
    description='Whether to start RVIZ')

    # Get the path to camco_launch/rviz/camco.rviz and kobuki params
    pkg_camco_launch = get_package_share_directory('camco_launch')
    
    camco_launch_rviz_config_file = PathJoinSubstitution(
        [pkg_camco_launch, 'rviz', 'camco.rviz'])

    kobuki_node_config_file = os.path.join(pkg_camco_launch, 'config', 'kobuki_node_params.yaml')

    # Get the path to camco_description.launch.py
    pkg_camco_description = get_package_share_directory('camco_description')
    
    camco_description_launch_file = PathJoinSubstitution(
        [pkg_camco_description, 'launch', 'camco_description.launch.py'])
    

    # Include camco_description.launch.py and set rviz argument to false
    camco_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camco_description_launch_file]),
        launch_arguments={'rviz_config_file': camco_launch_rviz_config_file, 'use_rviz' : use_rviz}.items()
        )
    
    # Launch kobuki node and remap /commands/velocity to /cmd_vel

    with open(kobuki_node_config_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    
    kobuki_ros_node = Node(
        package='kobuki_node',
        node_executable='kobuki_ros_node',
        output='both',
        parameters=[kobuki_params],
        remappings=[('commands/velocity', 'cmd_vel')]
        )

    #Launching RPLIDAR node
    rplidar_node = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/RPLIDAR',
                'serial_baudrate': 115200,
                'frame_id': 'rplidar_link',
                'inverted': False,
                'angle_compensate': True,
                'auto_standby': True,
            }],
        )

    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(camco_description_launch_include)
    ld.add_action(kobuki_ros_node)
    ld.add_action(rplidar_node)

    return ld

