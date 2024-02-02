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
    # Get path to camco_navigation/config/slam.yaml
    pkg_camco_navigation = get_package_share_directory('camco_navigation')
    slam_node_config_file = PathJoinSubstitution([pkg_camco_navigation,'config', 'slam.yaml'])
    
    # Launch configuration variables specific to SLAM
    sync = LaunchConfiguration('sync')

    # Declare the launch arguments
    slam_params_arguments = DeclareLaunchArgument('params',
        default_value= slam_node_config_file, description='Robot namespace'
    )

    # Rewrite yaml configuration file for the SLAM
    slam_params = RewrittenYaml(
        source_file=LaunchConfiguration('params'),
        root_key='',
        param_rewrites={},
        convert_types=True
    )

    # Remap topics' names
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
    ]

    # Create a SLAM group action
    slam = GroupAction([
        # PushRosNamespace(namespace), # Test without namespacing these nodes to check if it is not needed

        # Create a synchronized SLAM node
        Node(package='slam_toolbox',
             executable='sync_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[slam_params],
             remappings=remappings,
             condition=IfCondition(sync)),
        # Create an asyncronized SLAM node
        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[slam_params],
             remappings=remappings,
             condition=UnlessCondition(sync))
    ])

    # Declare launch options and add action
    ld = LaunchDescription(arguments)
    ld.add_action(slam_params_arguments)
    ld.add_action(slam)
    return ld
