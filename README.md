# pixi_oscbf_fr3

RT-side OSCBF control workspace for FR3 experiments.

This repo is a Pixi-managed ROS2 workspace based on `oscbf_hardware_ws`, with an added
`crisp_oscbf_bridge` package to connect CRISP teleop topics to OSCBF control topics.

## What this workspace does

Runtime nodes in the FR3 CBF mode:

1. `franka_impedance_controller` (C++):
   - subscribes `franka/torque_command`
   - publishes `franka/joint_states`
2. `franka_control_node.py` (Python):
   - subscribes `franka/joint_states`, `ee_state`
   - publishes `franka/torque_command`
3. `crisp_bridge_node.py` (Python, new):
   - subscribes `target_pose`, `franka/joint_states`
   - publishes `ee_state`, `current_pose`, `current_twist`, `joint_states`

This lets existing CRISP teleop frontends keep publishing `target_pose` while OSCBF owns the
low-level torque loop.

## Important safety and ownership rule

Do **not** run `module_run_on_RT_pc/pixi_franka_ros2` robot control launch and OSCBF hardware
control at the same time for the same robot.

Pick exactly one robot owner.

## Setup

```bash
cd module_run_on_RT_pc/pixi_oscbf_fr3
pixi install
pixi run build
```

If build output says `Franka package not found. Skipping franka_impedance_controller build`,
simulation and bridge nodes are still available, but hardware torque node is not built in this workspace.

If needed, source overlays manually in the current shell:

```bash
source scripts/setup.sh
```

## Main tasks

```bash
pixi run traj-sim
pixi run teleop-hardware
pixi run teleop-fr3-cbf
pixi run teleop-fr3-cbf-all
pixi run teleop-fr3-cbf-dual
pixi run teleop-fr3-cbf-dual-all

pixi run franka-node
pixi run cbf-node
pixi run bridge-node
pixi run traj-node
pixi run oculus-node
```

`teleop-fr3-cbf` launches CBF + bridge only (expects a separate Franka torque node).
`teleop-fr3-cbf-all` also launches `franka_impedance_controller` from this workspace.
`teleop-fr3-cbf-dual` launches leader + follower CBF/bridge stacks (no local torque nodes).
`teleop-fr3-cbf-dual-all` also launches two `franka_impedance_controller` nodes.

## Recommended first hardware experiment (single FR3)

1. On RT PC, run CBF stack:

```bash
pixi run teleop-fr3-cbf-all -- --robot_hostname 172.16.0.3
```

2. On operator/GPU side, run your existing teleop workflow (for example gamepad):

```bash
cd /home/hex/Documents/github/playground/understand_crisp/robofab_crisp
pixi run teleop-gamepad-fr3-3cams
```

The bridge should expose `current_pose/current_twist/joint_states` and consume `target_pose`.

## Namespace usage (dual-arm follower experiment)

To run follower as `right` namespace:

```bash
pixi run teleop-fr3-cbf -- --namespace right --robot_hostname 172.16.0.3
```

Then CRISP side should target right-namespaced env configs (`--follower-namespace right`, etc.).

To launch both leader and follower OSCBF stacks from this workspace:

```bash
pixi run teleop-fr3-cbf-dual -- --leader_namespace left --follower_namespace right --leader_robot_hostname 172.16.0.33 --follower_robot_hostname 172.16.0.3
```

## Current limitations

- Upstream OSCBF hardware scripts currently use `load_panda()` model assumptions.
- FR3-specific model/collision tuning is a follow-up task for production safety validation.
- Gripper control is still outside OSCBF torque loop and remains separate.

## Citation

If you use OSCBF in publications, please cite:

```text
@inproceedings{morton2025oscbf,
  author={Morton, Daniel and Pavone, Marco},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Safe, Task-Consistent Manipulation with Operational Space Control Barrier Functions},
  year={2025},
  pages={187-194},
  doi={10.1109/IROS60139.2025.11246389}
}
```
