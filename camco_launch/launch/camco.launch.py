from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Get the path to camco_launch/rviz/camco.rviz
    pkg_camco_launch = get_package_share_directory('camco_launch')
    
    camco_launch_rviz_config_file = PathJoinSubstitution(
        [pkg_camco_launch, 'rviz', 'camco.rviz'])

    # Get the path to camco_description.launch.py
    pkg_camco_description = get_package_share_directory('camco_description')
    
    camco_description_launch_file = PathJoinSubstitution(
        [pkg_camco_description, 'launch', 'camco_description.launch.py'])
    
    # Get the path to kobuki_node-launch.py
    pkg_kobuki_node = get_package_share_directory('kobuki_node')
    
    kobuki_node_launch_file = PathJoinSubstitution(
        [pkg_kobuki_node, 'launch', 'kobuki_node-launch.py'])
    

    # Include camco_description.launch.py and set rviz argument to false
    camco_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camco_description_launch_file]),
        launch_arguments={'rviz_config_file': camco_launch_rviz_config_file}.items()
        )
    
    # Include kobuki_node-launch.py and remap /commands/velocity to /cmd_vel
    kobuki_node_launch_include = GroupAction(
        actions=[
            SetRemap(src='commands/velocity',dst='cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([kobuki_node_launch_file])
            )
        ]
    )

    #Launching RPLIDAR node
    rplidar_node = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/RPLIDAR',
                'serial_baudrate': 57600,
                'frame_id': 'rplidar_link',
                'inverted': False,
                'angle_compensate': True,
                'auto_standby': True,
            }],
        )

    ld.add_action(camco_description_launch_include)
    ld.add_action(kobuki_node_launch_include)
    ld.add_action(rplidar_node)

    return ld