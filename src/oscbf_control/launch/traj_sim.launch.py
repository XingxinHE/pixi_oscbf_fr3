from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="oscbf_control",
                executable="pybullet_sim_node.py",
                name="sim_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="oscbf_control",
                executable="traj_node.py",
                name="traj_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="oscbf_control",
                executable="franka_control_node.py",
                name="control_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
