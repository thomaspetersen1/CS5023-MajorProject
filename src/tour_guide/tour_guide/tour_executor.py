import math
from enum import Enum

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
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
        self.get_logger().info("Nav2 is active. Call /start_tour to begin.")

        self.state: State = State.IDLE
        self.queue: list[str] = []
        self.current_name: str | None = None

        self.create_service(Trigger, "start_tour", self._start_tour_cb)
        self.create_timer(self.TICK_PERIOD, self._tick)

    @staticmethod
    def _load_landmarks(path: str) -> dict[str, dict[str, float]]:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("landmarks", {}) or {}

    def _start_tour_cb(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        if self.state != State.IDLE:
            response.success = False
            response.message = f"tour already in progress (state={self.state.value})"
            return response
        if not self.landmarks:
            response.success = False
            response.message = "no landmarks loaded"
            return response

        # stub: skip route_planner, visit landmarks in YAML order
        self.queue = list(self.landmarks.keys())
        self.state = State.PLANNING
        response.success = True
        response.message = f"tour started with {len(self.queue)} landmarks"
        self.get_logger().info(response.message)
        return response

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

    def _tick(self) -> None:
        if self.state == State.IDLE:
            return

        if self.state == State.PLANNING:
            if not self.queue:
                self.get_logger().info("Tour complete.")
                self.state = State.IDLE
                return
            self.current_name = self.queue.pop(0)
            pose = self._landmark_to_pose(self.current_name)
            self.get_logger().info(
                f'Navigating to "{self.current_name}" '
                f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
            )
            self.navigator.goToPose(pose)
            self.state = State.NAVIGATING
            return

        if self.state == State.NAVIGATING:
            if not self.navigator.isTaskComplete():
                return
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached "{self.current_name}"')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Goal to "{self.current_name}" was canceled')
            else:
                self.get_logger().warn(
                    f'Goal to "{self.current_name}" failed; skipping'
                )
            self.current_name = None
            self.state = State.PLANNING


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
