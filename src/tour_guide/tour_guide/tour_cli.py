"""Interactive CLI for adding landmark visits mid-tour.

We subscribe to /tour_status to track what the executor is currently
doing, and publish JSON to /tour_config to ask for more landmarks to
be tacked onto the active tour. The executor's mid-flight replan
keeps the current target and re-runs TSP over
[current_target] + remaining + requested.

Run with:

    ros2 run tour_guide tour_cli
    ros2 run tour_guide tour_cli --ros-args -p landmarks_file:=/path/to/landmarks.yaml
"""
from __future__ import annotations

import json
import threading
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from tour_guide.landmark_manager import load_landmarks


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class TourCLI(Node):
    def __init__(self) -> None:
        super().__init__("tour_cli")

        default_path = f"{get_package_share_directory('tour_guide')}/config/landmarks.yaml"
        self.declare_parameter("landmarks_file", default_path)
        path = self.get_parameter("landmarks_file").get_parameter_value().string_value

        self.landmarks = load_landmarks(path)
        self.names = sorted(self.landmarks.keys())

        self._status_lock = threading.Lock()
        self._status: Optional[dict] = None

        self.create_subscription(String, "/tour_status", self._on_status, LATCHED_QOS)
        self.config_pub = self.create_publisher(String, "/tour_config", LATCHED_QOS)

        self.get_logger().info(f"Loaded {len(self.names)} landmarks from {path}")

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._status_lock:
            self._status = payload

    def snapshot(self) -> Optional[dict]:
        with self._status_lock:
            return None if self._status is None else dict(self._status)

    def request(self, requested: list[str]) -> list[str]:
        """Build the new tour list, publish it to /tour_config, and
        return whatever was actually sent."""
        status = self.snapshot()
        current = (status or {}).get("current_target")
        remaining = list((status or {}).get("remaining") or [])

        tour: list[str] = []
        if current:
            tour.append(current)
        tour.extend(remaining)
        for name in requested:
            if name not in tour:
                tour.append(name)

        self.config_pub.publish(String(data=json.dumps({"landmarks": tour})))
        return tour


HELP = """
Commands:
  l, list           Show available landmarks
  s, status         Show current tour status
  a <name|#> ...    Queue landmark(s) for visit (by name or list number)
  c, clear          Cancel the active tour (publish empty list)
  h, help           Show this help
  q, quit           Exit
""".rstrip()


def print_landmarks(names: list[str]) -> None:
    print("\nAvailable landmarks:")
    for i, name in enumerate(names, 1):
        print(f"  {i:>2}. {name}")
    print()


def print_status(status: Optional[dict]) -> None:
    if status is None:
        print("\nNo /tour_status received yet. Is tour_executor running?\n")
        return
    print()
    print(f"  state:          {status.get('state')}")
    print(f"  current target: {status.get('current_target')}")
    print(f"  remaining:      {status.get('remaining')}")
    print(f"  visited:        {status.get('visited')}")
    print(f"  last event:     {status.get('last_event')}")
    print()


def resolve(token: str, names: list[str]) -> Optional[str]:
    if token.isdigit():
        idx = int(token) - 1
        return names[idx] if 0 <= idx < len(names) else None
    return token if token in names else None


def repl(node: TourCLI) -> None:
    print_landmarks(node.names)
    print(HELP)

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in ("q", "quit", "exit"):
            return
        if cmd in ("h", "help", "?"):
            print(HELP)
            continue
        if cmd in ("l", "list", "ls"):
            print_landmarks(node.names)
            continue
        if cmd in ("s", "status"):
            print_status(node.snapshot())
            continue
        if cmd in ("c", "clear", "cancel"):
            node.config_pub.publish(String(data=json.dumps({"landmarks": []})))
            print("Sent: cancel tour")
            continue
        if cmd in ("a", "add"):
            if not args:
                print("usage: a <name|#> [<name|#> ...]")
                continue
            resolved: list[str] = []
            bad: list[str] = []
            for token in args:
                name = resolve(token, node.names)
                (resolved if name else bad).append(name or token)
            if bad:
                print(f"Unknown: {', '.join(bad)}")
                continue
            tour = node.request(resolved)
            print(f"Sent tour ({len(tour)}): {tour}")
            continue

        # bare number/name shortcut, treat it as `add`
        resolved = []
        bad = []
        for token in parts:
            name = resolve(token, node.names)
            (resolved if name else bad).append(name or token)
        if bad or not resolved:
            print(f"Unknown command or landmark: {line!r}. Type 'h' for help.")
            continue
        tour = node.request(resolved)
        print(f"Sent tour ({len(tour)}): {tour}")


def main() -> None:
    rclpy.init()
    node = TourCLI()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        repl(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
