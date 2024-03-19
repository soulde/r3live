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

    param = [{"add_keyframe_R": 10.0},
             {"add_keyframe_t": 0.15},
             {"insert_pt_dis": 0.0},
             {"if_use_free_space_support": False},
             {"thickness_factor": 1.0},
             {"quality_factor": 0.0},
             {"decimate_mesh": 1.0},
             {"if_remove_spurious": 40.0},
             {"if_remove_spikes": True},
             {"close_holes_dist": 20.0},
             {"smooth_mesh_factor": 5.0},
             {"working_dir": os.path.join(pkg_dir, "r3live_output")},
             {"texturing_smooth_factor": 10.0},
             {"offline_map_name": "test.r3live"}]

    r3live_meshing = Node(
        package='r3live',
        executable='r3live_meshing',
        name='r3live_meshing',
        output='screen',
        parameters=param
    )

    return LaunchDescription([
        r3live_meshing
    ])
