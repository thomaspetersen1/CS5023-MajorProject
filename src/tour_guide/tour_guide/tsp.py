"""Nearest-neighbor TSP heuristic for tour routing.

Pure-Python module with no ROS imports — unit-testable in isolation.
The deliberative layer's planner imports this; replace the body of
:func:`nearest_neighbor_order` with a smarter heuristic while keeping
the signature stable.
"""
from __future__ import annotations

import math


def nearest_neighbor_order(
    start: tuple[float, float] | None,
    points: list[tuple[str, float, float]],
) -> list[str]:
    """Return point names in nearest-neighbor visit order from ``start``.

    Args:
        start: ``(x, y)`` starting position, or ``None`` if no pose is
            available yet (e.g. AMCL hasn't published).
        points: List of ``(name, x, y)`` triples to visit.

    Returns:
        Names in visit order. If ``start`` is ``None``, the input order
        is returned unchanged.
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