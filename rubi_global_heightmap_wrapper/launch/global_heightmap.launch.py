from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rubi_global_heightmap_wrapper'),
                          'config', 'global_heightmap.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config', default_value=config),
        Node(package='rubi_global_heightmap_wrapper', executable='global_heightmap_node',
             name='rubi_global_heightmap_wrapper', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
