# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    #    dev_name_arg = DeclareLaunchArgument(
    #        "dev_name", default_value=TextSubstitution(text="STM32")
    #    )

    # include another launch file
    # launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('demo_nodes_cpp'),
    #             'launch/topics/talker_listener.launch.py'))
    # )

    pkg_dir = get_package_share_directory('fast_lio')

    config_file = LaunchConfiguration('lidar_config')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_lidar_config_file_cmd = DeclareLaunchArgument(
        'lidar_config',
        default_value=os.path.join(pkg_dir, 'config', 'mid360.yaml'),
        description='Full path to the filter chain config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'viz.rviz'),
        description='Full path to the RVIZ config file to use')
    # and use args to set parameters
    lio_node = Node(
        package='fast_lio',
        executable='fast_lio_node',
        name='fast_lio',
        output='screen',
        parameters=[{"feature_extract_enable": True},
                    {"point_filter_num": 1},
                    {"max_iteration": 3},
                    {"filter_size_surf": 0.6},
                    {"filter_size_map": 0.6},
                    {"cube_side_length": 1000.},
                    {"runtime_pos_log_enable": True},
                    config_file])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        declare_lidar_config_file_cmd,
        declare_rviz_config_file_cmd,
        lio_node,
        rviz2_node
    ])
