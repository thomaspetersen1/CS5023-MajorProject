"""Loads landmarks from YAML.

No ROS imports here so this can be tested in isolation. Returns a
dict-of-dicts that the executor and planner can use directly.

YAML shape:

    frame_id: map
    landmarks:
      reception:  {x: 1.5, y: 0.2, yaw: 0.0}
      whiteboard: {x: 4.0, y: 3.5, yaw: 1.57}
"""
from __future__ import annotations

import yaml


def load_landmarks(path: str) -> dict[str, dict[str, float]]:
    """Load landmarks from a YAML file, keyed by name.

    Each value is {"x": float, "y": float, "yaw": float}. yaw defaults
    to 0.0 when it isn't in the file.

    Raises FileNotFoundError if the path doesn't exist, and ValueError
    if the YAML is empty or doesn't have the expected shape.
    """
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    raw = data.get("landmarks") or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: 'landmarks' must be a mapping")
    if not raw:
        raise ValueError(f"{path}: no landmarks defined")

    landmarks: dict[str, dict[str, float]] = {}
    for name, pose in raw.items():
        if not isinstance(pose, dict) or "x" not in pose or "y" not in pose:
            raise ValueError(f"{path}: landmark '{name}' needs x and y")
        landmarks[str(name)] = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose.get("yaw", 0.0)),
        }
    return landmarks