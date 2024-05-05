from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition

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
        description='Whether to use Rviz2 or not'),
    DeclareLaunchArgument('gui', default_value='true',
        description='Whether to use GUI or not')
]


def generate_launch_description():

    pkg_camco_navigation = get_package_share_directory('camco_navigation')
    pkg_rosbridge_server = get_package_share_directory('rosbridge_server')
    pkg_robot_pose_publisher_ros2 = get_package_share_directory('robot_pose_publisher_ros2')

    path_camco_navigation_launch = PathJoinSubstitution([pkg_camco_navigation, 'launch', 'navigation.launch.py'])
    path_rosbridge_websocket_launch = PathJoinSubstitution([pkg_rosbridge_server, 'launch', 'rosbridge_websocket_launch.xml'])
    path_robot_pose_publisher_launch = PathJoinSubstitution([pkg_robot_pose_publisher_ros2, 'launch', 'robot_pose_publisher_launch.py'])

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_nav2 = LaunchConfiguration('params_nav2')
    params_amcl = LaunchConfiguration('params_amcl')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')


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

    robot_pose_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_robot_pose_publisher_launch)
    )

    rosbridge_websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(path_rosbridge_websocket_launch),
        condition=IfCondition(gui)
    )

    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(camco_navigation_launch)
    ld.add_action(read_battery_state_server_node)
    ld.add_action(camco_mission_node)
    ld.add_action(robot_pose_publisher_launch)
    ld.add_action(rosbridge_websocket_launch)
    return ld