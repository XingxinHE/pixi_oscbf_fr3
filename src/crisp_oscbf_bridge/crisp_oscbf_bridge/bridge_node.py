#!/usr/bin/env python3
"""Bridge CRISP teleop topics to OSCBF EEState and publish CRISP state topics.

This node connects two topic conventions:

Inputs:
- target pose from CRISP teleop (`target_pose`)
- joint states from OSCBF hardware node (`franka/joint_states`)

Outputs:
- desired end-effector state for OSCBF (`ee_state`)
- current pose/twist for CRISP (`current_pose`, `current_twist`)
- relayed joint states for CRISP (`joint_states`)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TwistStamped, Vector3
from oscbf.core.manipulator import load_panda
from oscbf_control_msgs.msg import EEState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def _stamp_to_seconds(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def _quat_normalize_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def _quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_delta_to_omega(
    q_prev: np.ndarray, q_curr: np.ndarray, dt: float
) -> np.ndarray:
    if dt <= 1e-6:
        return np.zeros(3, dtype=float)
    q_rel = _quat_multiply_xyzw(q_curr, _quat_conjugate_xyzw(q_prev))
    q_rel = _quat_normalize_xyzw(q_rel)
    if q_rel[3] < 0.0:
        q_rel = -q_rel
    v = q_rel[:3]
    w = float(np.clip(q_rel[3], -1.0, 1.0))
    v_norm = float(np.linalg.norm(v))
    if v_norm < 1e-9:
        return np.zeros(3, dtype=float)
    angle = 2.0 * math.atan2(v_norm, w)
    axis = v / v_norm
    return axis * (angle / dt)


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if max_norm <= 0.0 or norm <= max_norm or norm < 1e-12:
        return v
    return v * (max_norm / norm)


def _rotation_matrix_to_quat_xyzw(rmat: np.ndarray) -> np.ndarray:
    tr = rmat[0, 0] + rmat[1, 1] + rmat[2, 2]
    if tr >= 0.0:
        s4 = 2.0 * math.sqrt(tr + 1.0)
        x = (rmat[2, 1] - rmat[1, 2]) / s4
        y = (rmat[0, 2] - rmat[2, 0]) / s4
        z = (rmat[1, 0] - rmat[0, 1]) / s4
        w = s4 / 4.0
    elif rmat[0, 0] > rmat[1, 1] and rmat[0, 0] > rmat[2, 2]:
        s4 = 2.0 * math.sqrt(1.0 + rmat[0, 0] - rmat[1, 1] - rmat[2, 2])
        x = s4 / 4.0
        y = (rmat[0, 1] + rmat[1, 0]) / s4
        z = (rmat[2, 0] + rmat[0, 2]) / s4
        w = (rmat[2, 1] - rmat[1, 2]) / s4
    elif rmat[1, 1] > rmat[2, 2]:
        s4 = 2.0 * math.sqrt(1.0 + rmat[1, 1] - rmat[0, 0] - rmat[2, 2])
        x = (rmat[0, 1] + rmat[1, 0]) / s4
        y = s4 / 4.0
        z = (rmat[1, 2] + rmat[2, 1]) / s4
        w = (rmat[0, 2] - rmat[2, 0]) / s4
    else:
        s4 = 2.0 * math.sqrt(1.0 + rmat[2, 2] - rmat[0, 0] - rmat[1, 1])
        x = (rmat[2, 0] + rmat[0, 2]) / s4
        y = (rmat[1, 2] + rmat[2, 1]) / s4
        z = s4 / 4.0
        w = (rmat[1, 0] - rmat[0, 1]) / s4
    return _quat_normalize_xyzw(np.array([x, y, z, w], dtype=float))


@dataclass
class TargetEEState:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    vel: np.ndarray
    omega: np.ndarray


class CrispOscbfBridge(Node):
    def __init__(self):
        super().__init__("crisp_oscbf_bridge")

        self.declare_parameter("joint_state_topic", "franka/joint_states")
        self.declare_parameter("target_pose_topic", "target_pose")
        self.declare_parameter("ee_state_topic", "ee_state")
        self.declare_parameter("current_pose_topic", "current_pose")
        self.declare_parameter("current_twist_topic", "current_twist")
        self.declare_parameter("current_joint_topic", "joint_states")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("publish_ee_rate_hz", 200.0)
        self.declare_parameter("max_linear_speed", 0.7)
        self.declare_parameter("max_angular_speed", 3.0)
        self.declare_parameter("twist_alpha", 0.25)
        self.declare_parameter("robot_model", "panda")

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.ee_state_topic = str(self.get_parameter("ee_state_topic").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.current_twist_topic = str(self.get_parameter("current_twist_topic").value)
        self.current_joint_topic = str(self.get_parameter("current_joint_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_ee_rate_hz = float(self.get_parameter("publish_ee_rate_hz").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.twist_alpha = float(self.get_parameter("twist_alpha").value)
        self.robot_model = str(self.get_parameter("robot_model").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.ee_state_pub = self.create_publisher(EEState, self.ee_state_topic, qos)
        self.current_pose_pub = self.create_publisher(
            PoseStamped, self.current_pose_topic, qos
        )
        self.current_twist_pub = self.create_publisher(
            TwistStamped, self.current_twist_topic, qos
        )
        self.current_joint_pub = self.create_publisher(
            JointState, self.current_joint_topic, qos
        )

        self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            qos,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos,
        )

        self.robot = load_panda()
        if self.robot_model != "panda":
            self.get_logger().warning(
                "robot_model=%s requested, but only panda model is currently available. Using panda model.",
                self.robot_model,
            )

        self._target_state: TargetEEState | None = None
        self._last_target_pos: np.ndarray | None = None
        self._last_target_quat: np.ndarray | None = None
        self._last_target_time: float | None = None
        self._last_target_vel: np.ndarray = np.zeros(3, dtype=float)
        self._last_target_omega: np.ndarray = np.zeros(3, dtype=float)

        timer_dt = 1.0 / max(self.publish_ee_rate_hz, 1.0)
        self.create_timer(timer_dt, self._publish_ee_state)

        self.get_logger().info(
            "Bridge ready: target_pose=%s, joint_states=%s, ee_state=%s",
            self.target_pose_topic,
            self.joint_state_topic,
            self.ee_state_topic,
        )

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = _quat_normalize_xyzw(
            np.array(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ],
                dtype=float,
            )
        )

        stamp = msg.header.stamp
        t = _stamp_to_seconds(stamp.sec, stamp.nanosec)
        if t <= 0.0:
            now = self.get_clock().now().to_msg()
            t = _stamp_to_seconds(now.sec, now.nanosec)

        if (
            self._last_target_pos is None
            or self._last_target_quat is None
            or self._last_target_time is None
        ):
            vel = np.zeros(3, dtype=float)
            omega = np.zeros(3, dtype=float)
        else:
            dt = max(t - self._last_target_time, 1e-6)
            vel_raw = (pos - self._last_target_pos) / dt
            omega_raw = _quat_delta_to_omega(self._last_target_quat, quat, dt)
            vel = (
                self.twist_alpha * vel_raw
                + (1.0 - self.twist_alpha) * self._last_target_vel
            )
            omega = (
                self.twist_alpha * omega_raw
                + (1.0 - self.twist_alpha) * self._last_target_omega
            )

        vel = _clip_norm(vel, self.max_linear_speed)
        omega = _clip_norm(omega, self.max_angular_speed)

        self._target_state = TargetEEState(
            pos=pos, quat_xyzw=quat, vel=vel, omega=omega
        )
        self._last_target_pos = pos
        self._last_target_quat = quat
        self._last_target_time = t
        self._last_target_vel = vel
        self._last_target_omega = omega

    def _joint_state_callback(self, msg: JointState) -> None:
        n = self.robot.num_joints
        if len(msg.position) < n:
            return

        q = np.asarray(msg.position[:n], dtype=float)
        if len(msg.velocity) >= n:
            dq = np.asarray(msg.velocity[:n], dtype=float)
        else:
            dq = np.zeros(n, dtype=float)

        T = np.asarray(self.robot.ee_transform(q))
        J = np.asarray(self.robot.ee_jacobian(q))
        twist = J @ dq

        now_msg = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = self.base_frame
        pose_msg.pose.position = Point(
            x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])
        )
        quat = _rotation_matrix_to_quat_xyzw(T[:3, :3])
        pose_msg.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])
        )
        self.current_pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = now_msg
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear = Vector3(
            x=float(twist[0]), y=float(twist[1]), z=float(twist[2])
        )
        twist_msg.twist.angular = Vector3(
            x=float(twist[3]), y=float(twist[4]), z=float(twist[5])
        )
        self.current_twist_pub.publish(twist_msg)

        if self.current_joint_topic != self.joint_state_topic:
            relay_msg = JointState()
            relay_msg.header = msg.header
            relay_msg.name = list(msg.name)
            relay_msg.position = list(msg.position)
            relay_msg.velocity = list(msg.velocity)
            relay_msg.effort = list(msg.effort)
            self.current_joint_pub.publish(relay_msg)

        if self._target_state is None:
            self._target_state = TargetEEState(
                pos=np.array([T[0, 3], T[1, 3], T[2, 3]], dtype=float),
                quat_xyzw=quat,
                vel=np.zeros(3, dtype=float),
                omega=np.zeros(3, dtype=float),
            )

    def _publish_ee_state(self) -> None:
        if self._target_state is None:
            return

        msg = EEState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position = Point(
            x=float(self._target_state.pos[0]),
            y=float(self._target_state.pos[1]),
            z=float(self._target_state.pos[2]),
        )
        msg.pose.orientation = Quaternion(
            x=float(self._target_state.quat_xyzw[0]),
            y=float(self._target_state.quat_xyzw[1]),
            z=float(self._target_state.quat_xyzw[2]),
            w=float(self._target_state.quat_xyzw[3]),
        )
        msg.twist.linear = Vector3(
            x=float(self._target_state.vel[0]),
            y=float(self._target_state.vel[1]),
            z=float(self._target_state.vel[2]),
        )
        msg.twist.angular = Vector3(
            x=float(self._target_state.omega[0]),
            y=float(self._target_state.omega[1]),
            z=float(self._target_state.omega[2]),
        )
        self.ee_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrispOscbfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
