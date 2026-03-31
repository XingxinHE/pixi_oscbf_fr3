from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _arm_include(namespace_arg: str, host_arg: str, base_frame_arg: str):
    single_launch = str(Path(__file__).resolve().parent / "fr3_cbf_teleop.launch.py")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_launch),
        launch_arguments={
            "namespace": LaunchConfiguration(namespace_arg),
            "robot_hostname": LaunchConfiguration(host_arg),
            "realtime": LaunchConfiguration("realtime"),
            "base_frame": LaunchConfiguration(base_frame_arg),
            "launch_franka_node": LaunchConfiguration("launch_franka_node"),
        }.items(),
    )


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "leader_namespace", default_value="left", description="Leader arm namespace"
        ),
        DeclareLaunchArgument(
            "follower_namespace",
            default_value="right",
            description="Follower arm namespace",
        ),
        DeclareLaunchArgument(
            "leader_robot_hostname",
            default_value="172.16.0.33",
            description="Leader Franka robot IP/hostname",
        ),
        DeclareLaunchArgument(
            "follower_robot_hostname",
            default_value="172.16.0.3",
            description="Follower Franka robot IP/hostname",
        ),
        DeclareLaunchArgument(
            "leader_base_frame",
            default_value="base",
            description="Leader bridge frame_id for pose/twist output",
        ),
        DeclareLaunchArgument(
            "follower_base_frame",
            default_value="base",
            description="Follower bridge frame_id for pose/twist output",
        ),
        DeclareLaunchArgument(
            "realtime", default_value="true", description="Use libfranka realtime mode"
        ),
        DeclareLaunchArgument(
            "launch_franka_node",
            default_value="true",
            description="Launch franka_impedance_controller for each arm",
        ),
    ]

    leader = _arm_include(
        "leader_namespace",
        "leader_robot_hostname",
        "leader_base_frame",
    )
    follower = _arm_include(
        "follower_namespace",
        "follower_robot_hostname",
        "follower_base_frame",
    )

    return LaunchDescription(launch_args + [leader, follower])
