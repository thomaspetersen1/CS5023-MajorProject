"""Nearest-neighbor TSP heuristic for tour routing.

No ROS imports here so this can be tested on its own. route_planner
imports nearest_neighbor_order, so if we ever swap in a smarter
heuristic we just keep the signature the same.
"""
from __future__ import annotations

import math


def nearest_neighbor_order(
    start: tuple[float, float] | None,
    points: list[tuple[str, float, float]],
) -> list[str]:
    """Return the point names in nearest-neighbor visit order, starting
    from `start`.

    `start` is an (x, y) pose, or None when we don't have one yet
    because AMCL hasn't published. `points` is a list of (name, x, y)
    triples. If `start` is None we just return the input order
    untouched.
    """
    if start is None:
        return [name for name, _, _ in points]

    cx, cy = start
    remaining = list(points)
    order: list[str] = []
    while remaining:
        nearest_idx = min(
            range(len(remaining)),
            key=lambda i: math.hypot(
                remaining[i][1] - cx, remaining[i][2] - cy
            ),
        )
        name, x, y = remaining.pop(nearest_idx)
        order.append(name)
        cx, cy = x, y
    return order