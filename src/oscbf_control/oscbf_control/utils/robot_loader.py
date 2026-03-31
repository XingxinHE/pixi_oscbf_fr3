"""Robot model loader utilities for OSCBF hardware nodes."""

from __future__ import annotations

from oscbf.core.manipulator import Manipulator, load_panda


def load_robot_model(robot_model: str, logger) -> Manipulator:  # noqa: ANN001
    model = robot_model.strip().lower()
    if model in {"panda", "franka"}:
        return load_panda()

    if model == "fr3":
        logger.warning(
            "robot_model=fr3 requested, but upstream oscbf currently provides load_panda(). "
            "Using panda model as temporary fallback."
        )
        return load_panda()

    raise ValueError(
        f"Unsupported robot_model={robot_model}. Supported values: panda, franka, fr3"
    )
