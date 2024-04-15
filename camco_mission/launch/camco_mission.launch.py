from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('params_nav2',
        default_value=PathJoinSubstitution([
            get_package_share_directory('camco_navigation'), 'config', 'nav2.yaml']), 
            description='Nav2 parameters'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('params_amcl',
        default_value=PathJoinSubstitution(
            [get_package_share_directory('camco_navigation'), 'config', 'localization.yaml']),
            description='Localization parameters'),
    DeclareLaunchArgument('map', default_value='w12.yaml',
        description='name of the map yaml file to load'),
     DeclareLaunchArgument('rviz', default_value='true',
        description='Whether to use Rviz2 or not')
]


def generate_launch_description():

    pkg_camco_navigation = get_package_share_directory('camco_navigation')

    path_camco_navigation_launch = PathJoinSubstitution([pkg_camco_navigation, 'launch', 'navigation.launch.py'])
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_nav2 = LaunchConfiguration('params_nav2')
    params_amcl = LaunchConfiguration('params_amcl')
    rviz = LaunchConfiguration('rviz')


    camco_navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_camco_navigation_launch),
            launch_arguments=[
                  ('params_nav2', params_nav2),
                  ('use_sim_time', use_sim_time),
                  ('namespace', namespace),
                  ('params_amcl', params_amcl),
                  ('rviz', rviz),
                ]
        )
    
    read_battery_state_server_node = Node(
    package='camco_kobuki_interface',
    executable='read_battery_state_server',
    output='screen')

    camco_mission_node = Node(
    package='camco_mission',
    executable='camco_mission',
    output='screen')

    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(camco_navigation_launch)
    ld.add_action(read_battery_state_server_node)
    ld.add_action(camco_mission_node)
    return ld