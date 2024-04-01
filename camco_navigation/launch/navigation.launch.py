from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace

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
        description='name of the map yaml file to load')
]


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_camco_navigation = get_package_share_directory('camco_navigation')
    

    map_full_path = PathJoinSubstitution([pkg_camco_navigation, 'maps', LaunchConfiguration('map')])
    launch_nav2 = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_nav2 = LaunchConfiguration('params_nav2')
    params_amcl = LaunchConfiguration('params_amcl')


    localization = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
            launch_arguments={'namespace': namespace,
                              'map': map_full_path,
                              'use_sim_time': use_sim_time,
                              'params_file': params_amcl}.items()),
    ])
    

    nav2 = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                  ('params_file', params_nav2),
                  ('use_composition', 'False'),
                ]
        ),
    ])
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization)
    ld.add_action(nav2)
    return ld