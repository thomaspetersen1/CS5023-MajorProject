"""Route planner — deliberative layer.

Consumes ``plan_request`` and replies on ``plan_result``. Both topics
carry ``std_msgs/String`` with JSON payloads, matching the constraint
that the project uses only standard message types.

Request schema::

    {
      "request_id": "<uuid or counter>",
      "start": [x, y] | null,
      "landmarks": ["a", "b", "c"]
    }

Reply schema::

    {
      "request_id": "<echoes request>",
      "order": ["b", "a", "c"],
      "success": true,
      "error": null
    }
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from tour_guide.landmark_manager import load_landmarks
from tour_guide.tsp import nearest_neighbor_order


class RoutePlanner(Node):
    def __init__(self) -> None:
        super().__init__("route_planner")

        self.declare_parameter("landmarks_file", "")
        path = (
            self.get_parameter("landmarks_file").get_parameter_value().string_value
        )
        if not path:
            raise RuntimeError("route_planner: landmarks_file parameter is required")

        self.landmarks = load_landmarks(path)
        self.get_logger().info(
            f"Loaded {len(self.landmarks)} landmarks: "
            f"{list(self.landmarks.keys())}"
        )

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Both topics are latched: plan_request so a late-starting planner picks
        # up the executor's request, and plan_result so the executor receives
        # the reply even if subscription discovery hadn't completed when the
        # planner published.
        self.result_pub = self.create_publisher(String, "plan_result", latched_qos)
        self.create_subscription(
            String, "plan_request", self._on_request, latched_qos
        )
        self.get_logger().info("route_planner ready.")

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid plan_request JSON: {e}")
            return

        request_id = payload.get("request_id")
        names = payload.get("landmarks", [])
        start_raw = payload.get("start")

        if not isinstance(names, list):
            self._reply(request_id, [], False, "'landmarks' must be a list")
            return

        unknown = [n for n in names if n not in self.landmarks]
        if unknown:
            self._reply(request_id, [], False, f"unknown landmarks: {unknown}")
            return

        start: tuple[float, float] | None = None
        if isinstance(start_raw, list) and len(start_raw) == 2:
            start = (float(start_raw[0]), float(start_raw[1]))

        points = [
            (n, self.landmarks[n]["x"], self.landmarks[n]["y"]) for n in names
        ]
        order = nearest_neighbor_order(start, points)
        self.get_logger().info(f"Planned: {order}")
        self._reply(request_id, order, True, None)

    def _reply(
        self,
        request_id,
        order: list[str],
        success: bool,
        error: str | None,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "request_id": request_id,
                "order": order,
                "success": success,
                "error": error,
            }
        )
        self.result_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()