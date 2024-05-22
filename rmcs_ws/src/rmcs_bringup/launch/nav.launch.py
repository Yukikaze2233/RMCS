from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
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
    tlarc = Node(package="decision_maker", executable="decision_maker")
    navmesh = ExecuteProcess(
        cmd=[
            get_package_share_directory("decision_maker")
            + "/UnityNavMesh/UnityNavMesh.x86_64"
        ],
        output="screen",
    )

    return LaunchDescription([fast_lio, local_nav, livox_ros_driver, tlarc, navmesh])