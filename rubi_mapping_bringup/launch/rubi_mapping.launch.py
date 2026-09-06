import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    backend = get_package_share_directory('elevation_mapping_cupy')
    bringup = get_package_share_directory('rubi_mapping_bringup')
    wrapper = get_package_share_directory('rubi_global_heightmap_wrapper')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('base_frame', default_value='body'),
        DeclareLaunchArgument('backend_config', default_value=os.path.join(bringup, 'config', 'rubi_cupy.yaml')),
        DeclareLaunchArgument('wrapper_config', default_value=os.path.join(wrapper, 'config', 'global_heightmap.yaml')),
        Node(package='elevation_mapping_cupy', executable='elevation_mapping_node.py',
             name='elevation_mapping_node', output='screen',
             parameters=[os.path.join(backend, 'config', 'core', 'core_param.yaml'),
                         LaunchConfiguration('backend_config'),
                         {'use_sim_time': sim_time,
                          'base_frame': ParameterValue(LaunchConfiguration('base_frame'), value_type=str),
                          'subscribers.rubi_lidar.topic_name': ParameterValue(
                              LaunchConfiguration('cloud_topic'), value_type=str)}]),
        Node(package='rubi_global_heightmap_wrapper', executable='global_heightmap_node',
             name='rubi_global_heightmap_wrapper', output='screen',
             parameters=[LaunchConfiguration('wrapper_config'), {'use_sim_time': sim_time}]),
    ])
