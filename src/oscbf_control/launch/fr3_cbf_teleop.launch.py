from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    robot_hostname = LaunchConfiguration("robot_hostname")
    realtime = LaunchConfiguration("realtime")
    base_frame = LaunchConfiguration("base_frame")
    launch_franka_node = LaunchConfiguration("launch_franka_node")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for all OSCBF/bridge nodes",
            ),
            DeclareLaunchArgument(
                "robot_hostname",
                default_value="172.16.0.3",
                description="Franka robot IP/hostname",
            ),
            DeclareLaunchArgument(
                "realtime",
                default_value="true",
                description="Use libfranka realtime mode",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base",
                description="Frame id used in bridge pose/twist output",
            ),
            DeclareLaunchArgument(
                "launch_franka_node",
                default_value="true",
                description="Launch franka_impedance_controller from this workspace",
            ),
            Node(
                package="oscbf_control",
                executable="franka_impedance_controller",
                name="franka_impedance_controller",
                namespace=namespace,
                condition=IfCondition(launch_franka_node),
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_hostname": robot_hostname,
                        "realtime": realtime,
                    }
                ],
            ),
            Node(
                package="oscbf_control",
                executable="franka_control_node.py",
                name="oscbf_control_node",
                namespace=namespace,
                output="screen",
                emulate_tty=True,
                parameters=[{"robot_model": "fr3"}],
            ),
            Node(
                package="crisp_oscbf_bridge",
                executable="crisp_bridge_node.py",
                name="crisp_oscbf_bridge",
                namespace=namespace,
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "joint_state_topic": "franka/joint_states",
                        "target_pose_topic": "target_pose",
                        "ee_state_topic": "ee_state",
                        "current_pose_topic": "current_pose",
                        "current_twist_topic": "current_twist",
                        "current_joint_topic": "joint_states",
                        "base_frame": base_frame,
                    }
                ],
            ),
        ]
    )
