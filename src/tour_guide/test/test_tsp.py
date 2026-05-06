from __future__ import annotations

import re
from pathlib import Path

import pytest

from tour_guide.tsp import nearest_neighbor_order


LANDMARKS_YAML = (
    Path(__file__).resolve().parent.parent / "config" / "landmarks.yaml"
)

# Hand-parsed to avoid pulling PyYAML into the test environment.
_LANDMARK_RE = re.compile(
    r"^\s+(?P<name>\w+):\s*\{x:\s*(?P<x>-?\d+\.?\d*),"
    r"\s*y:\s*(?P<y>-?\d+\.?\d*),"
)


def _load_points() -> list[tuple[str, float, float]]:
    """Return ``(name, x, y)`` triples in the order they appear in YAML."""
    points: list[tuple[str, float, float]] = []
    for line in LANDMARKS_YAML.read_text().splitlines():
        match = _LANDMARK_RE.match(line)
        if match:
            points.append(
                (match["name"], float(match["x"]), float(match["y"]))
            )
    assert points, f"No landmarks parsed from {LANDMARKS_YAML}"
    return points


# Expected orders computed by hand; recompute if landmarks.yaml changes.
@pytest.mark.parametrize(
    "start, expected",
    [
        (
            None,
            [
                "computer_door",
                "whiteboard_pillar",
                "cardboard_corner",
                "unnamed_stop",
                "charging_stations",
                "cardboard_room",
            ],
        ),
        (
            (0.0, 0.0),
            [
                "computer_door",
                "cardboard_room",
                "cardboard_corner",
                "whiteboard_pillar",
                "unnamed_stop",
                "charging_stations",
            ],
        ),
        (
            (-3.0, -8.0),
            [
                "charging_stations",
                "unnamed_stop",
                "cardboard_corner",
                "whiteboard_pillar",
                "cardboard_room",
                "computer_door",
            ],
        ),
    ],
)
def test_nearest_neighbor_order(start, expected):
    points = _load_points()
    assert nearest_neighbor_order(start, points) == expected
