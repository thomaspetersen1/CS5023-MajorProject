"""FSM that runs the tour.

States cycle IDLE -> PLANNING -> NAVIGATING -> DWELLING -> NAVIGATING.
Goals go out to Nav2 one at a time via navigate_to_pose, and the visit
order comes from route_planner over plan_request / plan_result.

We don't use nav2_simple_commander.BasicNavigator because its blocking
helpers call rclpy.spin_until_future_complete on the global executor,
which crashes with "Executor is already spinning" the moment we send a
goal from inside a callback under rclpy.spin(node).
"""
from __future__ import annotations

import json
import math
import uuid
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
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
    TICK_PERIOD = 0.5

    # We spin in place during PLANNING and DWELLING so you can read the
    # state off the robot's body. The cue gets republished at 10 Hz or
    # the create3's velocity timeout kicks in and stops the motion.
    CUE_TIMER_PERIOD = 0.1
    CUE_PLANNING_WZ = -1.5  # clockwise
    CUE_DWELLING_WZ = 1.5   # counter-clockwise

    def __init__(self) -> None:
        super().__init__("tour_executor")

        self.declare_parameter("landmarks_file", "")
        self.declare_parameter("dwell_seconds", 6.0)
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

        # All post-__init__ transitions go through _set_state.
        self.state: State = State.IDLE
        self._active_cue_wz: float | None = None
        self.queue: list[str] = []
        self.visited: list[str] = []
        self.current_name: str | None = None
        self.current_pose_xy: tuple[float, float] | None = None
        self.dwell_started_ns: int | None = None
        self.pending_request_id: str | None = None
        self._last_event: str = "init"

        # Every send or cancel bumps _goal_generation. Each callback
        # closes over the generation it was registered for and bails on
        # mismatch, so a stale result from a canceled goal can't drive
        # the FSM after we've already moved on.
        self._goal_handle = None
        self._result_future = None
        self._goal_generation: int = 0

        # Bring publishers up before we wait on the Nav2 action server
        # so /tour_status is observable during the wait.
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "tour_status", latched_qos)
        self.plan_request_pub = self.create_publisher(String, "plan_request", latched_qos)

        # teleop and collision_monitor also publish to /cmd_vel, and the
        # create3 just takes whichever message arrived most recently. So
        # we only push to it during PLANNING and DWELLING, which keeps
        # us out of Nav2's way and the joystick's way.
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self._publish_status("waiting for Nav2")

        # On this turtlebot4 image bt_navigator's autostart races
        # planner_server and ends up stuck in INACTIVE, so the
        # navigate_to_pose action server never comes up on its own.
        # We kick the lifecycle manager first, then force bt_navigator
        # to ACTIVE ourselves before waiting on the action.
        self._kick_nav2_lifecycle()
        self._force_activate_bt_navigator()

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 action servers...")
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(
                "navigate_to_pose action server not yet up, waiting..."
            )
        self.get_logger().info("Nav2 ready.")

        self.create_subscription(String, "tour_config", self._on_tour_config, 10)
        self.create_subscription(
            String, "plan_result", self._on_plan_result, latched_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10
        )
        self.create_service(Trigger, "start_tour", self._start_tour_cb)
        self.create_timer(self.TICK_PERIOD, self._tick)
        self.create_timer(self.CUE_TIMER_PERIOD, self._cue_tick)

        self._publish_status("ready")
        self.get_logger().info("Publish to /tour_config or call /start_tour to begin.")

    def _kick_nav2_lifecycle(self) -> None:
        """Ask /lifecycle_manager_navigation to STARTUP. No-op when the
        service isn't running or the stack is already ACTIVE anyway."""
        client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                "lifecycle_manager_navigation/manage_nodes not available; "
                "skipping auto-startup. If goals get rejected as INACTIVE, "
                "run: ros2 service call /lifecycle_manager_navigation/"
                "manage_nodes nav2_msgs/srv/ManageLifecycleNodes "
                "'{command: 0}'  # 0 = STARTUP"
            )
            return
        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP  # = 0
        future = client.call_async(req)
        future.add_done_callback(self._on_lifecycle_startup)
        self.get_logger().info(
            "Requested /lifecycle_manager_navigation STARTUP "
            "(transitioning nav stack to ACTIVE)"
        )

    def _on_lifecycle_startup(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"lifecycle_manager_navigation STARTUP call failed: {exc!r}"
            )
            return
        if result is None:
            self.get_logger().warn(
                "lifecycle_manager_navigation STARTUP returned no result"
            )
            return
        if result.success:
            self.get_logger().info(
                "lifecycle_manager_navigation STARTUP succeeded; nav stack ACTIVE"
            )
        else:
            self.get_logger().warn(
                "lifecycle_manager_navigation STARTUP returned success=false; "
                "the nav stack may still be INACTIVE"
            )

    def _force_activate_bt_navigator(self) -> None:
        """Force bt_navigator to ACTIVE so the autostart race against
        planner_server doesn't leave it stuck.

        When bt_navigator activates it loads a BT XML, and that XML
        wants an action client on /compute_path_to_pose. But
        planner_server owns that action and isn't necessarily up yet,
        so under autostart bt_navigator loses the race and
        self-deactivates with "Action server compute_path_to_pose not
        available". The lifecycle manager won't retry on its own. So
        we wait for /compute_path_to_pose to come up and then send the
        activate transition straight to /bt_navigator/change_state.
        """
        plan_check = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.get_logger().info(
            "Waiting for /compute_path_to_pose (planner_server) before "
            "re-activating bt_navigator..."
        )
        if not plan_check.wait_for_server(timeout_sec=30.0):
            self.get_logger().warn(
                "/compute_path_to_pose not advertised after 30s; "
                "bt_navigator activation will likely fail again."
            )
        else:
            self.get_logger().info("/compute_path_to_pose is up.")
        plan_check.destroy()

        get_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        if not get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/bt_navigator/get_state unavailable; cannot force-activate."
            )
            return
        state_future = get_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, state_future, timeout_sec=5.0)
        state_resp = state_future.result()
        if state_resp is None:
            self.get_logger().warn(
                "bt_navigator get_state returned no result; cannot force-activate."
            )
            return
        state_id = state_resp.current_state.id
        state_label = state_resp.current_state.label
        self.get_logger().info(
            f"bt_navigator current state: {state_label} (id={state_id})"
        )
        if state_id == 3:  # PRIMARY_STATE_ACTIVE
            self.get_logger().info("bt_navigator already ACTIVE; nothing to do.")
            return

        change_state_client = self.create_client(
            ChangeState, "/bt_navigator/change_state"
        )
        if not change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/bt_navigator/change_state unavailable; cannot force-activate."
            )
            return

        if state_id == 1:  # PRIMARY_STATE_UNCONFIGURED
            if not self._send_change_state(
                change_state_client,
                Transition.TRANSITION_CONFIGURE,
                label="configure",
                timeout_sec=10.0,
            ):
                return

        self._send_change_state(
            change_state_client,
            Transition.TRANSITION_ACTIVATE,
            label="activate",
            timeout_sec=15.0,
        )

    def _send_change_state(
        self,
        client,
        transition_id: int,
        label: str,
        timeout_sec: float,
    ) -> bool:
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        resp = future.result()
        if resp is None:
            self.get_logger().error(
                f"bt_navigator {label} transition timed out after {timeout_sec}s"
            )
            return False
        if not resp.success:
            self.get_logger().error(
                f"bt_navigator {label} transition returned success=false"
            )
            return False
        self.get_logger().info(f"bt_navigator {label} transition OK")
        return True

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
            self._set_state(State.IDLE)
            self._publish_status("tour cleared")
            return True, "tour cleared"

        # If someone adds landmarks mid-tour, one of the new ones might
        # actually be closer than the target we're driving to, so we
        # cancel the active goal and re-run TSP from the current pose.
        # visited stays as it was.
        self._cancel_active_goal()
        self.current_name = None
        self.dwell_started_ns = None
        self.queue = []
        self._set_state(State.PLANNING)
        self._send_plan_request(names, self.current_pose_xy)
        self._publish_status(f"planning tour with {len(names)} landmarks")
        return True, f"planning tour with {len(names)} landmarks"

    def _on_plan_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid plan_result JSON: {e}")
            return
        incoming_id = payload.get("request_id")
        self.get_logger().info(
            f"plan_result received: request_id={incoming_id} "
            f"(pending={self.pending_request_id}, success={payload.get('success')})"
        )
        if incoming_id != self.pending_request_id:
            return  # stale or unrelated

        self.pending_request_id = None
        if not payload.get("success"):
            err = payload.get("error", "unknown error")
            self.get_logger().error(f"Planner failed: {err}")
            self._publish_status(f"plan failed: {err}")
            self._set_state(State.IDLE)
            return

        order = payload.get("order", [])
        if not isinstance(order, list):
            self.get_logger().error("Planner returned non-list 'order'")
            self._set_state(State.IDLE)
            return

        if self.state == State.PLANNING:
            self.queue = list(order)
            self._publish_status(f"tour planned: {order}")
            self._dispatch_next()
        elif self.state == State.NAVIGATING:
            # mid-flight reorder, so this is just the remainder of the tour
            self.queue = list(order)
            self._publish_status(
                f"queue updated mid-flight; finishing {self.current_name}"
            )
        elif self.state == State.DWELLING:
            self.queue = list(order)
            self._publish_status(f"queue updated during dwell: {order}")
        # IDLE → ignore (we got canceled while plan was in flight)

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
        if not self.queue:
            self.current_name = None
            self._set_state(State.IDLE)
            self._publish_status("tour complete")
            self.get_logger().info("Tour complete.")
            return
        self.current_name = self.queue.pop(0)
        pose = self._landmark_to_pose(self.current_name)
        self.get_logger().info(
            f'Navigating to "{self.current_name}" '
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )
        self._set_state(State.NAVIGATING)
        self._publish_status(f"navigating to {self.current_name}")

        self._goal_generation += 1
        gen = self._goal_generation
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda fut, g=gen: self._on_goal_response(fut, g)
        )

    def _on_goal_response(self, future, gen: int) -> None:
        if gen != self._goal_generation:
            return  # superseded by a newer dispatch or cancel
        try:
            goal_handle = future.result()
        except Exception as exc:  # send_goal_async failed, action probably gone
            self.get_logger().error(f"send_goal_async failed: {exc!r}")
            self._handle_goal_unavailable("send_goal_async failed")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal to "{self.current_name}" was REJECTED by Nav2 '
                "(bt_navigator probably isn't ACTIVE, try setting an initial pose in RViz)"
            )
            self._handle_goal_unavailable("goal rejected")
            return

        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            lambda fut, g=gen: self._on_nav_result(fut, g)
        )

    def _on_nav_result(self, future, gen: int) -> None:
        if gen != self._goal_generation:
            return  # this was a stale (canceled or replaced) goal
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"get_result_async failed: {exc!r}")
            self._publish_status(f"nav result error at {self.current_name}; skipping")
            self.current_name = None
            self._goal_handle = None
            self._result_future = None
            self._dispatch_next()
            return

        status = result.status
        self._goal_handle = None
        self._result_future = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached "{self.current_name}"')
            self.visited.append(self.current_name)
            self._set_state(State.DWELLING)
            self.dwell_started_ns = self.get_clock().now().nanoseconds
            self._publish_status(f"reached {self.current_name}; dwelling")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal to "{self.current_name}" was canceled')
            self._publish_status(f"canceled at {self.current_name}")
            self.current_name = None
            self._dispatch_next()
        else:
            self.get_logger().warn(
                f'Goal to "{self.current_name}" failed (status={status}); skipping'
            )
            self._publish_status(f"failed at {self.current_name}; skipping")
            self.current_name = None
            self._dispatch_next()

    def _handle_goal_unavailable(self, reason: str) -> None:
        """Put the current target back at the front of the queue and use
        the dwell timer as a backoff before we retry."""
        if self.current_name is not None:
            self.queue.insert(0, self.current_name)
        self.current_name = None
        self._goal_handle = None
        self._result_future = None
        self._set_state(State.DWELLING)
        self.dwell_started_ns = self.get_clock().now().nanoseconds
        self._publish_status(
            f"{reason}; retrying after {self.dwell_seconds:.0f}s backoff"
        )

    def _cancel_active_goal(self) -> None:
        # Bump the generation first so any in-flight goal-response or
        # result callbacks for the previous goal bail out instead of
        # driving the FSM after we've already moved to a new state.
        self._goal_generation += 1
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None

    def _tick(self) -> None:
        if self.state == State.DWELLING:
            assert self.dwell_started_ns is not None
            elapsed = (
                self.get_clock().now().nanoseconds - self.dwell_started_ns
            ) / 1e9
            if elapsed >= self.dwell_seconds and self.pending_request_id is None:
                prev = self.current_name
                self.current_name = None
                self.dwell_started_ns = None
                self._publish_status(f"dwell complete at {prev}")
                # Re-run TSP from the pose we just reached so the next
                # target is whichever remaining landmark is closest, and
                # so any landmarks added mid-tour since the last plan
                # get picked up too.
                if self.queue:
                    remaining = list(self.queue)
                    self.queue = []
                    self._set_state(State.PLANNING)
                    self._send_plan_request(remaining, self.current_pose_xy)
                    self._publish_status(
                        f"replanning {len(remaining)} remaining from current pose"
                    )
                else:
                    self._set_state(State.IDLE)
                    self._publish_status("tour complete")
                    self.get_logger().info("Tour complete.")

        # Heartbeat at 2 Hz so default-VOLATILE subscribers like
        # `ros2 topic echo` keep seeing that the FSM is alive.
        self._publish_status()

    def _set_state(self, new_state: State) -> None:
        """Every FSM transition has to go through here.

        The cue rotation gets started and stopped on state entry and
        exit, not from a timer that polls self.state. The old version
        kept publishing cue cmd_vel as Nav2 took over, and the create3
        saw fighting commands on every PLANNING -> NAVIGATING handoff.
        """
        if new_state == self.state:
            return

        prev_cue = self._active_cue_wz
        self.state = new_state

        if new_state == State.PLANNING:
            self._active_cue_wz = self.CUE_PLANNING_WZ
        elif new_state == State.DWELLING:
            self._active_cue_wz = self.CUE_DWELLING_WZ
        else:  # IDLE or NAVIGATING -- no cue motion
            self._active_cue_wz = None

        # If we were emitting a cue and we aren't anymore, send one
        # explicit zero cmd_vel before yielding the topic. Cue-to-cue
        # transitions just update the angular velocity and let the
        # next _cue_tick pick it up.
        if prev_cue is not None and self._active_cue_wz is None:
            self._publish_cue_velocity(0.0)

    def _publish_cue_velocity(self, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def _cue_tick(self) -> None:
        """Republish the active cue rotation at 10 Hz so the create3's
        velocity timeout doesn't stop it. No-op when no cue is active.

        We bypass velocity_smoother and collision_monitor here because
        the cue is just in-place rotation, and the global costmap's
        inflation already gives the robot's 0.22m radius enough room.
        """
        if self._active_cue_wz is None:
            return
        self._publish_cue_velocity(self._active_cue_wz)

    def _landmark_to_pose(self, name: str) -> PoseStamped:
        lm = self.landmarks[name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(lm["x"])
        pose.pose.position.y = float(lm["y"])
        z, w = yaw_to_quat_zw(float(lm.get("yaw", 0.0)))
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def _publish_status(self, last_event: str | None = None) -> None:
        if last_event is not None:
            self._last_event = last_event
        payload = {
            "state": self.state.value,
            "current_target": self.current_name,
            "remaining": list(self.queue),
            "visited": list(self.visited),
            "last_event": self._last_event,
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