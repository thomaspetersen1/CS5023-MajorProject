"""Tour executor — sequencer / FSM layer.

State machine:
    IDLE -> PLANNING -> NAVIGATING -> DWELLING -> NAVIGATING -> ...

Holds the queue of unvisited stops, sends goals to Nav2 one at a time
via :class:`BasicNavigator`, and handles dynamic ``add_landmark``
requests by asking ``route_planner`` for a new order over the
``plan_request`` / ``plan_result`` topics.

Inputs:
    * ``tour_config`` (std_msgs/String, JSON): ``{"landmarks": ["a", "b"]}``.
      Empty list cancels the tour.
    * ``start_tour`` (std_srvs/Trigger): plans over all loaded landmarks.

Outputs:
    * ``tour_status`` (std_msgs/String, JSON, latched): FSM state, target,
      remaining queue, visited list, last event.
    * ``plan_request`` (std_msgs/String, JSON): outgoing planning queries.

Subscribes to ``plan_result`` for replies from the deliberative layer.
"""
from __future__ import annotations

import json
import math
import uuid
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tour_guide.landmark_manager import load_landmarks


class State(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    DWELLING = "DWELLING"


def yaw_to_quat_zw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TourExecutor(Node):
    TICK_PERIOD = 0.5  # seconds

    def __init__(self) -> None:
        super().__init__("tour_executor")

        self.declare_parameter("landmarks_file", "")
        self.declare_parameter("dwell_seconds", 3.0)
        landmarks_path = (
            self.get_parameter("landmarks_file").get_parameter_value().string_value
        )
        if not landmarks_path:
            raise RuntimeError("tour_executor: landmarks_file parameter is required")
        self.dwell_seconds: float = float(
            self.get_parameter("dwell_seconds").value
        )

        self.landmarks = load_landmarks(landmarks_path)
        self.get_logger().info(
            f"Loaded {len(self.landmarks)} landmarks from {landmarks_path}"
        )

        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        # FSM state
        self.state: State = State.IDLE
        self.queue: list[str] = []
        self.visited: list[str] = []
        self.current_name: str | None = None
        self.current_pose_xy: tuple[float, float] | None = None
        self.dwell_started_ns: int | None = None
        self.pending_request_id: str | None = None

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "tour_status", latched_qos)
        self.plan_request_pub = self.create_publisher(String, "plan_request", 10)

        self.create_subscription(String, "tour_config", self._on_tour_config, 10)
        self.create_subscription(String, "plan_result", self._on_plan_result, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10
        )
        self.create_service(Trigger, "start_tour", self._start_tour_cb)
        self.create_timer(self.TICK_PERIOD, self._tick)

        self._publish_status("ready")
        self.get_logger().info("Publish to /tour_config or call /start_tour to begin.")

    # ----------------------------------------------------------------------
    # Inputs
    # ----------------------------------------------------------------------
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

        # Empty list = stop everything.
        if not names:
            self._cancel_active_goal()
            self.queue = []
            self.visited = []
            self.current_name = None
            self.dwell_started_ns = None
            self.pending_request_id = None
            self.state = State.IDLE
            self._publish_status("tour cleared")
            return True, "tour cleared"

        # Mid-flight: if current target is still wanted, finish it and reorder
        # the rest from that landmark's position.
        if self.state == State.NAVIGATING and self.current_name in names:
            rest = [n for n in names if n != self.current_name]
            anchor = (
                self.landmarks[self.current_name]["x"],
                self.landmarks[self.current_name]["y"],
            )
            self.queue = []  # clear stale queue; plan_result will repopulate
            self._send_plan_request(rest, anchor)
            self._publish_status(
                f"replanning rest of tour from {self.current_name}"
            )
            return True, "replanning mid-flight"

        # Otherwise, cancel anything in flight and replan from the robot's pose.
        self._cancel_active_goal()
        self.current_name = None
        self.dwell_started_ns = None
        self.visited = []
        self.queue = []
        self.state = State.PLANNING
        self._send_plan_request(names, self.current_pose_xy)
        self._publish_status(f"planning tour with {len(names)} landmarks")
        return True, f"planning tour with {len(names)} landmarks"

    def _on_plan_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid plan_result JSON: {e}")
            return
        if payload.get("request_id") != self.pending_request_id:
            return  # stale or unrelated

        self.pending_request_id = None
        if not payload.get("success"):
            err = payload.get("error", "unknown error")
            self.get_logger().error(f"Planner failed: {err}")
            self._publish_status(f"plan failed: {err}")
            self.state = State.IDLE
            return

        order = payload.get("order", [])
        if not isinstance(order, list):
            self.get_logger().error("Planner returned non-list 'order'")
            self.state = State.IDLE
            return

        if self.state == State.PLANNING:
            self.queue = list(order)
            self._publish_status(f"tour planned: {order}")
            self._dispatch_next()
        elif self.state == State.NAVIGATING:
            # Mid-flight reorder: queue is the rest after current_name.
            self.queue = list(order)
            self._publish_status(
                f"queue updated mid-flight; finishing {self.current_name}"
            )
        elif self.state == State.DWELLING:
            self.queue = list(order)
            self._publish_status(f"queue updated during dwell: {order}")
        # IDLE → ignore (we got canceled while plan was in flight)

    # ----------------------------------------------------------------------
    # FSM helpers
    # ----------------------------------------------------------------------
    def _send_plan_request(
        self,
        names: list[str],
        start: tuple[float, float] | None,
    ) -> None:
        request_id = str(uuid.uuid4())
        self.pending_request_id = request_id
        payload = {
            "request_id": request_id,
            "landmarks": names,
            "start": list(start) if start is not None else None,
        }
        self.plan_request_pub.publish(String(data=json.dumps(payload)))

    def _dispatch_next(self) -> None:
        """Pop next from queue and start navigating, or go IDLE."""
        if not self.queue:
            self.current_name = None
            self.state = State.IDLE
            self._publish_status("tour complete")
            self.get_logger().info("Tour complete.")
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

    def _cancel_active_goal(self) -> None:
        if self.state == State.NAVIGATING:
            self.navigator.cancelTask()

    def _tick(self) -> None:
        if self.state == State.NAVIGATING:
            if not self.navigator.isTaskComplete():
                return
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached "{self.current_name}"')
                self.visited.append(self.current_name)
                self.state = State.DWELLING
                self.dwell_started_ns = self.get_clock().now().nanoseconds
                self._publish_status(f"reached {self.current_name}; dwelling")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(
                    f'Goal to "{self.current_name}" was canceled'
                )
                self._publish_status(f"canceled at {self.current_name}")
                self.current_name = None
                self._dispatch_next()
            else:
                self.get_logger().warn(
                    f'Goal to "{self.current_name}" failed; skipping'
                )
                self._publish_status(f"failed at {self.current_name}; skipping")
                self.current_name = None
                self._dispatch_next()
        elif self.state == State.DWELLING:
            assert self.dwell_started_ns is not None
            elapsed = (
                self.get_clock().now().nanoseconds - self.dwell_started_ns
            ) / 1e9
            if elapsed < self.dwell_seconds:
                return
            if self.pending_request_id is not None:
                # Dwell is up but we're still waiting on a mid-flight plan.
                # Hold off on dispatching until the queue is updated.
                return
            prev = self.current_name
            self.current_name = None
            self.dwell_started_ns = None
            self._publish_status(f"dwell complete at {prev}")
            self._dispatch_next()

    # ----------------------------------------------------------------------
    # Pose construction
    # ----------------------------------------------------------------------
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
        self.status_pub.publish(String(data=json.dumps(payload)))


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