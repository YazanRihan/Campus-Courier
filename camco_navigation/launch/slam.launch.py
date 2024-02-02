from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNameSpace
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

# Create Slam arguments
arguments = [
    DeclareLaunchArgument('sync', default_value='true', description= 'Enable synced SLAM')
]

def generate_launch_description():
    pkg_camco_navigation = get_package_share_directory('camco_navigation')
    
    sync = LaunchConfiguration('sync')
    slam_params_arguments = DeclareLaunchArgument('params',
        default_value=PathJoinSubstitution([pkg_camco_navigation,'config', 'slam.yaml']), description='Robot namespace'
    )
    
