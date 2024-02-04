from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory

arguments = [
    DeclareLaunchArgument('params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('camco_navigation'), 'config', 'nav2.yaml']), 
            description='Nav2 parameters')
]


def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = LaunchConfiguration('params_file')
    # namespace = LaunchConfiguration('namespace')
    # use_sim_time = LaunchConfiguration('use_sim_time')

    # namespace_str = namespace.perform(context)
    # if (namespace_str and not namespace_str.startswith('/')):
    #     namespace_str = '/' + namespace_str

    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        # PushRosNamespace(namespace),
        # SetRemap(namespace_str + '/global_costmap/scan', namespace_str + '/scan'),
        # SetRemap(namespace_str + '/local_costmap/scan', namespace_str + '/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                  ('params_file', nav2_params.perform(context)),
                  ('use_composition', 'False'),
                ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(arguments)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld