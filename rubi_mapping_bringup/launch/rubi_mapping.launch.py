import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def optional_backend_overrides(cloud_topic, base_frame):
    """Only explicit, non-empty CLI values override the selected backend YAML."""
    overrides = {}
    if cloud_topic:
        overrides['subscribers.rubi_lidar.topic_name'] = cloud_topic
    if base_frame:
        overrides['base_frame'] = base_frame
    return overrides


def _launch_nodes(context):
    backend = get_package_share_directory('elevation_mapping_cupy')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    overrides = {'use_sim_time': sim_time}
    overrides.update(optional_backend_overrides(
        LaunchConfiguration('cloud_topic').perform(context),
        LaunchConfiguration('base_frame').perform(context),
    ))
    return [
        Node(
            package='elevation_mapping_cupy', executable='elevation_mapping_node.py',
            name='elevation_mapping_node', output='screen',
            parameters=[
                os.path.join(backend, 'config', 'core', 'core_param.yaml'),
                LaunchConfiguration('backend_config'), overrides,
            ],
        ),
        Node(
            package='rubi_global_heightmap_wrapper', executable='global_heightmap_node',
            name='rubi_global_heightmap_wrapper', output='screen',
            parameters=[LaunchConfiguration('wrapper_config'), {'use_sim_time': sim_time}],
        ),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('rubi_mapping_bringup')
    wrapper = get_package_share_directory('rubi_global_heightmap_wrapper')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Empty means "use backend_config". The RUBI YAML selects the corrected
        # MID-360 topic and base_link; explicit CLI values still win.
        DeclareLaunchArgument('cloud_topic', default_value=''),
        DeclareLaunchArgument('base_frame', default_value=''),
        DeclareLaunchArgument(
            'backend_config', default_value=os.path.join(bringup, 'config', 'rubi_cupy.yaml')),
        DeclareLaunchArgument(
            'wrapper_config', default_value=os.path.join(wrapper, 'config', 'global_heightmap.yaml')),
        OpaqueFunction(function=_launch_nodes),
    ])
