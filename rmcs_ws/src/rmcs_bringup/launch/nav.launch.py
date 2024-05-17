from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("fast_lio"), "launch"),
                "/mapping.launch.py",
            ]
        )
    )
    local_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("local_nav"), "launch"),
                "/run.py",
            ]
        )
    )
    livox_ros_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("livox_ros_driver2"), "launch"
                ),
                "/msg_MID360_launch.py",
            ]
        )
    )

    return LaunchDescription([fast_lio, local_nav, livox_ros_driver])
