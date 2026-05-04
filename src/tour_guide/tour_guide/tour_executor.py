import json
import math
from enum import Enum

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class State(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"


def yaw_to_quat_zw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TourExecutor(Node):
    TICK_PERIOD = 0.5  # seconds

    def __init__(self) -> None:
        super().__init__("tour_executor")

        self.declare_parameter("landmarks_file", "")
        landmarks_path = (
            self.get_parameter("landmarks_file").get_parameter_value().string_value
        )
        if not landmarks_path:
            raise RuntimeError("tour_executor: landmarks_file parameter is required")

        self.landmarks: dict[str, dict[str, float]] = self._load_landmarks(
            landmarks_path
        )
        self.get_logger().info(
            f"Loaded {len(self.landmarks)} landmarks from {landmarks_path}"
        )

        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        self.state: State = State.IDLE
        self.queue: list[str] = []
        self.visited: list[str] = []
        self.current_name: str | None = None
        self.current_pose_xy: tuple[float, float] | None = None

        # Latched status so a fresh subscriber (e.g. webapp on reconnect) sees current state.
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "tour_status", latched_qos)

        self.create_subscription(String, "tour_config", self._on_tour_config, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10
        )
        self.create_service(Trigger, "start_tour", self._start_tour_cb)
        self.create_timer(self.TICK_PERIOD, self._tick)

        self._publish_status("ready")
        self.get_logger().info("Publish to /tour_config or call /start_tour to begin.")

    @staticmethod
    def _load_landmarks(path: str) -> dict[str, dict[str, float]]:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("landmarks", {}) or {}

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_pose_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _start_tour_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        ok, message = self._set_tour(list(self.landmarks.keys()))
        response.success = ok
        response.message = message
        return response

    def _on_tour_config(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid tour_config JSON: {e}")
            self._publish_status(f"invalid tour_config: {e}")
            return
        names = payload.get("landmarks", [])
        if not isinstance(names, list):
            self._publish_status("invalid tour_config: 'landmarks' must be a list")
            return
        ok, message = self._set_tour(names)
        if not ok:
            self.get_logger().warn(message)

    def _set_tour(self, names: list[str]) -> tuple[bool, str]:
        unknown = [n for n in names if n not in self.landmarks]
        if unknown:
            msg = f"unknown landmarks: {unknown}"
            self._publish_status(msg)
            return False, msg

        # Empty list = stop.
        if not names:
            if self.state == State.NAVIGATING:
                self.navigator.cancelTask()
            self.queue = []
            self.visited = []
            self.current_name = None
            self.state = State.IDLE
            self._publish_status("tour cleared")
            return True, "tour cleared"

        # If currently navigating to a landmark still in the new set, let it finish
        # and reorder the rest from that landmark's position.
        if self.state == State.NAVIGATING and self.current_name in names:
            rest = [n for n in names if n != self.current_name]
            anchor = (
                self.landmarks[self.current_name]["x"],
                self.landmarks[self.current_name]["y"],
            )
            self.queue = self._tsp_order_from(rest, anchor)
            self._publish_status(
                f"tour updated; finishing {self.current_name} then continuing"
            )
            return True, "tour updated mid-flight"

        # Otherwise cancel any in-flight goal and replan from the robot's current pose.
        if self.state == State.NAVIGATING:
            self.navigator.cancelTask()
            self.current_name = None

        ordered = self._tsp_order_from(names, self.current_pose_xy)
        self.queue = ordered
        self.visited = []
        self.state = State.PLANNING
        self._publish_status(f"tour set: {ordered}")
        return True, f"tour set with {len(ordered)} landmarks"

    def _tsp_order_from(
        self,
        names: list[str],
        start_xy: tuple[float, float] | None,
    ) -> list[str]:
        # Nearest-neighbor over Euclidean distance. Falls back to input order
        # if we don't have a starting position yet (e.g. AMCL hasn't published).
        if start_xy is None:
            return list(names)
        cx, cy = start_xy
        ordered: list[str] = []
        remaining = list(names)
        while remaining:
            nearest = min(
                remaining,
                key=lambda n: math.hypot(
                    self.landmarks[n]["x"] - cx,
                    self.landmarks[n]["y"] - cy,
                ),
            )
            ordered.append(nearest)
            remaining.remove(nearest)
            cx = self.landmarks[nearest]["x"]
            cy = self.landmarks[nearest]["y"]
        return ordered

    def _landmark_to_pose(self, name: str) -> PoseStamped:
        lm = self.landmarks[name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(lm["x"])
        pose.pose.position.y = float(lm["y"])
        z, w = yaw_to_quat_zw(float(lm.get("yaw", 0.0)))
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def _publish_status(self, last_event: str) -> None:
        payload = {
            "state": self.state.value,
            "current_target": self.current_name,
            "remaining": list(self.queue),
            "visited": list(self.visited),
            "last_event": last_event,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _tick(self) -> None:
        if self.state == State.IDLE:
            return

        if self.state == State.PLANNING:
            if not self.queue:
                self.get_logger().info("Tour complete.")
                self.current_name = None
                self.state = State.IDLE
                self._publish_status("tour complete")
                return
            self.current_name = self.queue.pop(0)
            pose = self._landmark_to_pose(self.current_name)
            self.get_logger().info(
                f'Navigating to "{self.current_name}" '
                f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
            )
            self.navigator.goToPose(pose)
            self.state = State.NAVIGATING
            self._publish_status(f"navigating to {self.current_name}")
            return

        if self.state == State.NAVIGATING:
            if not self.navigator.isTaskComplete():
                return
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached "{self.current_name}"')
                self.visited.append(self.current_name)
                event = f"reached {self.current_name}"
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Goal to "{self.current_name}" was canceled')
                event = f"canceled at {self.current_name}"
            else:
                self.get_logger().warn(
                    f'Goal to "{self.current_name}" failed; skipping'
                )
                event = f"failed at {self.current_name}; skipping"
            self.current_name = None
            self.state = State.PLANNING
            self._publish_status(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TourExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
