from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

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
        launch_arguments={'use_rviz': "False"}.items()
        )
    
    # Include kobuki_node-launch.py and remap /commands/velocity to /cmd_vel
    kobuki_node_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kobuki_node_launch_file]),
        remappings=[('commands/velocity', 'cmd_vel')]
        )


    ld.add_action(camco_description_launch_include)
    ld.add_action(kobuki_node_launch_include)

    return ld