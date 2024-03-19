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
    pkg_dir = get_package_share_directory('r3live')


    config_file = LaunchConfiguration('lidar_config')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_lidar_config_file_cmd = DeclareLaunchArgument(
        'lidar_config',
        default_value=os.path.join(pkg_dir, 'config', 'r3live_config.yaml'),
        description='Full path to the filter chain config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'viz.rviz'),
        description='Full path to the RVIZ config file to use')

    param = [{"LiDAR_pointcloud_topic": "/laser_cloud_flat"},
             {"IMU_topic": "/os_cloud_node/imu"},
             {"Image_topic": "/NotAvail"},
             {"r3live_common/map_output_dir": os.path.join(pkg_dir,"r3live_output")},
             {"/Lidar_front_end/lidar_type":3},
             {"/Lidar_front_end/point_step":1},
             {"/r3live_lio/lio_update_point_step":6},
             config_file]


    r3live_LiDAR_front_end = Node(
        package='r3live',
        executable='r3live_LiDAR_front_end',
        name='r3live_LiDAR_front_end',
        output='screen',
        parameters=param
    )

    r3live_mapping = Node(
        package='r3live',
        executable='r3live_mapping',
        name='r3live_mapping',
        output='screen',
        parameters=param
    )

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
        r3live_LiDAR_front_end,
        r3live_mapping,
        rviz2_node
    ])
