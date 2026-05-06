"""Tour executor — sequencer / FSM layer.

State machine:
    IDLE -> PLANNING -> NAVIGATING -> DWELLING -> NAVIGATING -> ...

Holds the queue of unvisited stops, sends goals to Nav2 one at a time
via the ``navigate_to_pose`` action client (native async pattern), and
handles dynamic ``add_landmark`` requests by asking ``route_planner``
for a new order over the ``plan_request`` / ``plan_result`` topics.

We intentionally do NOT use ``nav2_simple_commander.BasicNavigator``
because its blocking helpers call ``rclpy.spin_until_future_complete``
on the global executor, which is incompatible with running this node
under ``rclpy.spin(node)`` — it raises ``Executor is already spinning``
the moment a goal is sent from a callback.

Inputs:
    * ``tour_config`` (std_msgs/String, JSON): ``{"landmarks": ["a", "b"]}``.
      Empty list cancels the tour.
    * ``start_tour`` (std_srvs/Trigger): plans over all loaded landmarks.

Outputs:
    * ``tour_status`` (std_msgs/String, JSON, latched): FSM state, target,
      remaining queue, visited list, last event.
    * ``plan_request`` (std_msgs/String, JSON, latched): outgoing planning queries.

Subscribes to ``plan_result`` (latched) for replies from the deliberative layer.
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
    TICK_PERIOD = 0.5  # seconds — FSM heartbeat / dwell timer

    # State-cue motion: a small in-place rotation per FSM state, used
    # so observers can read the current state from the robot's body
    # language without subscribing to /tour_status. Published at
    # CUE_TIMER_PERIOD so the create3 base keeps receiving fresh
    # commands and doesn't trip its own safety stop. Negative
    # angular.z is clockwise (right-hand rule), positive is CCW.
    CUE_TIMER_PERIOD = 0.1  # seconds (10 Hz)
    CUE_PLANNING_WZ = -1.5  # rad/s, clockwise (~86 deg/s)
    CUE_DWELLING_WZ = 1.5   # rad/s, counter-clockwise (~86 deg/s)

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

        # FSM state — initialized before any publishing so _publish_status is safe.
        # All transitions after __init__ MUST go through _set_state, which is
        # also where the visual-cue rotation is started/stopped. Direct
        # mutation of self.state desyncs the cue and produces "fighting"
        # commands at hand-offs to Nav2 (see _set_state docstring).
        self.state: State = State.IDLE
        self._active_cue_wz: float | None = None
        self.queue: list[str] = []
        self.visited: list[str] = []
        self.current_name: str | None = None
        self.current_pose_xy: tuple[float, float] | None = None
        self.dwell_started_ns: int | None = None
        self.pending_request_id: str | None = None
        self._last_event: str = "init"

        # Active Nav2 action handles. _goal_generation invalidates stale
        # callbacks: every send/cancel bumps it, and callbacks captured the
        # generation they were registered for and bail out on mismatch.
        self._goal_handle = None
        self._result_future = None
        self._goal_generation: int = 0

        # Publishers come up BEFORE waiting on the Nav2 action server so
        # /tour_status is observable during the wait.
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, "tour_status", latched_qos)
        self.plan_request_pub = self.create_publisher(String, "plan_request", latched_qos)

        # State-cue cmd_vel publisher. /cmd_vel already has multiple
        # publishers in this stack (teleop_twist_joy_node + collision_monitor
        # from the Nav2 pipeline); the create3 base just takes the latest
        # message. We only ever publish during PLANNING and DWELLING --
        # see _cue_tick -- so we don't fight Nav2 (NAVIGATING) or the
        # joystick (IDLE).
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self._publish_status("waiting for Nav2")

        # Order matters here. The navigate_to_pose action server is
        # ONLY advertised once bt_navigator reaches ACTIVE — and on this
        # turtlebot4 image bt_navigator's autostart_node self-activate
        # races planner_server, fails, and lands in INACTIVE [2]. So
        # before we wait on navigate_to_pose, we (a) ask the lifecycle
        # manager to STARTUP (best-effort, free if it already worked),
        # and (b) directly drive bt_navigator to ACTIVE ourselves once
        # /compute_path_to_pose is up. Without (b) the wait below loops
        # forever on this stack.
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

    # ----------------------------------------------------------------------
    # Nav2 lifecycle bootstrap
    # ----------------------------------------------------------------------
    def _kick_nav2_lifecycle(self) -> None:
        """Send STARTUP to /lifecycle_manager_navigation if it exists.

        Idempotent and best-effort: if the service isn't there (e.g. user is
        running a stack without a lifecycle manager, or it's namespaced
        differently) we just log and continue. If it IS there and already
        ACTIVE, STARTUP is a no-op on the manager's side.
        """
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
        """Drive /bt_navigator to ACTIVE explicitly, repairing the
        autostart-time race against planner_server.

        The other nav2 nodes (planner_server, controller_server, etc.)
        self-activate cleanly via autostart_node:true because they
        have no inter-node dependencies at activate-time. bt_navigator
        is the exception: its on_activate loads the BT XML which
        binds an action client to /compute_path_to_pose, owned by
        planner_server. On this turtlebot4 image the two activations
        run in parallel under autostart_node, bt_navigator loses the
        race, and self-deactivates back to INACTIVE [2] with
        "Action server compute_path_to_pose not available". The
        surrounding lifecycle_manager_navigation does not re-drive
        configure/activate on this stack, so STARTUP via
        _kick_nav2_lifecycle alone does NOT recover it.

        Recovery: wait until /compute_path_to_pose is actually up
        (proves planner_server is fully active), then send the
        activate transition straight to /bt_navigator/change_state.
        On this attempt the BT XML's action-client bind succeeds.

        Synchronous on purpose -- runs in __init__ before main()'s
        rclpy.spin(node), so spin_until_future_complete is safe and
        doesn't conflict with any outer executor.
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
            self._set_state(State.IDLE)
            self._publish_status("tour cleared")
            return True, "tour cleared"

        # Mid-flight add: do NOT cancel the active goal yet. Let the robot
        # keep moving toward current_name while we ask the planner to
        # re-run TSP over the full set (including current_name) from the
        # live pose. _on_plan_result then reconciles:
        #   * order[0] == current_name → keep going, queue := order[1:]
        #   * else → cancel, dispatch order[0]; previously-current target
        #     stays in the new queue and gets visited later.
        # IDLE / no in-flight goal: plan and dispatch normally.
        if self.current_name is None:
            self.queue = []
            self._set_state(State.PLANNING)
            self._send_plan_request(names, self.current_pose_xy)
            self._publish_status(f"planning tour with {len(names)} landmarks")
            return True, f"planning tour with {len(names)} landmarks"

        self.queue = []
        self._send_plan_request(names, self.current_pose_xy)
        msg = f"replanning {len(names)} landmarks; {self.current_name} still in flight"
        self._publish_status(msg)
        return True, msg

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
            # Mid-flight reorder: queue is the rest after current_name.
            if order and order[0] == self.current_name:
                self.queue = list(order[1:])
                self._publish_status(
                    f"queue updated mid-flight; finishing {self.current_name}"
                )
            else:
                prev = self.current_name
                self._cancel_active_goal()
                self.current_name = None
                self.queue = list(order)
                self._publish_status(
                    f"diverting from {prev} to {order[0]} (closer)"
                )
                self._set_state(State.PLANNING)
                self._dispatch_next()


        elif self.state == State.DWELLING:
            # Robot already at a stop — current_name is still set until
            # dwell completes. Stage the new queue; _tick replans again
            # from the dwell pose when the dwell timer fires.
            if order and order[0] == self.current_name:
                self.queue = list(order[1:])
            else:
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
        """Goal accept/reject came back from Nav2."""
        if gen != self._goal_generation:
            return  # superseded by a newer dispatch or cancel
        try:
            goal_handle = future.result()
        except Exception as exc:  # send_goal_async failed (e.g. action gone)
            self.get_logger().error(f"send_goal_async failed: {exc!r}")
            self._handle_goal_unavailable("send_goal_async failed")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal to "{self.current_name}" was REJECTED by Nav2 '
                "(bt_navigator likely INACTIVE — set an initial pose in RViz)"
            )
            self._handle_goal_unavailable("goal rejected")
            return

        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            lambda fut, g=gen: self._on_nav_result(fut, g)
        )

    def _on_nav_result(self, future, gen: int) -> None:
        """Final navigation result for the dispatched goal."""
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
        """Nav2 server rejected/dropped the goal — requeue current target and
        back off via the dwell mechanism so we retry once it becomes active."""
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
        # Bumping the generation invalidates any in-flight goal-response or
        # result callbacks for the current goal, so they can't drive the FSM
        # after _set_tour has moved us to a new state.
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
                # Replan from the just-reached pose: re-run TSP over what's
                # left so the next target is the closest remaining landmark
                # to where we actually are. Without this, _dispatch_next
                # would just pop queue[0] from the original plan, which
                # ignores any mid-tour additions or drift since the last
                # plan was computed.
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

        # Heartbeat: republish current status every tick so observers (including
        # default-VOLATILE subscribers like `ros2 topic echo`) always see a
        # 2 Hz pulse confirming the FSM is alive. NAVIGATING transitions are
        # event-driven (via _on_nav_result), so there's no polling work here.
        self._publish_status()

    def _set_state(self, new_state: State) -> None:
        """Single chokepoint for FSM transitions. ALL post-__init__
        state writes must go through here.

        Routing every transition through one method lets the visual-
        cue rotation be driven by state ENTRY/EXIT instead of by a
        free-running timer that polls self.state. The previous design
        kept publishing CCW/CW cmd_vel right up until the moment
        Nav2's controller_server started publishing its own commands;
        the create3 base saw the two streams overlap and the robot
        looked like it was being yanked in opposite directions on
        every PLANNING->NAVIGATING and DWELLING->NAVIGATING hand-off.
        Now every cue->non-cue transition emits a single decisive
        zero cmd_vel, then yields the topic.

        Cue selection per state (see also CUE_*_WZ class constants):
          IDLE       -> silent (joystick can drive)
          PLANNING   -> clockwise spin ("thinking")
          NAVIGATING -> silent (Nav2 owns cmd_vel)
          DWELLING   -> counter-clockwise spin ("look at me")
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

        # Hand-off cleanly: if we were producing a cue and we no longer
        # are, stop the rotation explicitly before the next owner of
        # /cmd_vel takes over. Cue->cue transitions (e.g. DWELLING->
        # PLANNING from a mid-flight tour_config) skip this and just
        # update the angular velocity, so direction reversals are
        # picked up on the next _cue_tick.
        if prev_cue is not None and self._active_cue_wz is None:
            self._publish_cue_velocity(0.0)

    def _publish_cue_velocity(self, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def _cue_tick(self) -> None:
        """Republish the currently active cue rotation at 10 Hz.

        Cue selection happens in _set_state at transition time -- this
        timer just keeps fresh commands flowing to the create3 base so
        it doesn't trip its velocity-timeout safety stop. When no cue
        is active (IDLE / NAVIGATING) this is a no-op, so /cmd_vel is
        fully owned by the joystick or Nav2 respectively.

        Bypasses velocity_smoother and collision_monitor on purpose:
        the cue is pure in-place rotation, the global_costmap's
        inflation already guarantees enough wall clearance for the
        robot's 0.22m radius to rotate safely, and the speeds are
        conservative (CUE_*_WZ = 0.5 rad/s = ~28 deg/s).
        """
        if self._active_cue_wz is None:
            return
        self._publish_cue_velocity(self._active_cue_wz)

    # ----------------------------------------------------------------------
    # Pose construction
    # ----------------------------------------------------------------------
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