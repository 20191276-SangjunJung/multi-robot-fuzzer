
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .behaviors.move_to_goal import MoveToGoal
from .behaviors.follow import Follow
from .behaviors.maintain_formation import MaintainFormation
from .behaviors.avoid_obstacle import AvoidObstacle
from .behaviors.explore_area import ExploreArea
from .behaviors.search_target import SearchTarget

from .missions import (
    navigation,
    leader_follower,
    formation_navigation,
    formation_obstacle_avoidance,
    exploration,
    search,
    rendezvous,
)


MISSIONS = {
    mission.MISSION_NAME: mission
    for mission in (
        navigation,
        leader_follower,
        formation_navigation,
        formation_obstacle_avoidance,
        exploration,
        search,
        rendezvous,
    )
}

BEHAVIOR_CLASSES = {
    "move_to_goal": MoveToGoal,
    "follow": Follow,
    "maintain_formation": MaintainFormation,
    "explore_area": ExploreArea,
    "search_target": SearchTarget,
}


class AvoidanceMux(Node):
    """Combine movement commands with obstacle avoidance."""

    def __init__(
        self,
        robot_name,
        safe_distance=0.5,
        turn_speed=0.6,
    ):
        super().__init__(f"avoidance_mux_{robot_name}")

        self.robot_name = robot_name
        self.avoid = AvoidObstacle(
            robot_name=robot_name,
            safe_distance=safe_distance,
            turn_speed=turn_speed,
        )

        self.latest_command = Twist()
        self.last_command_time = 0.0
        self.enabled = True

        self.raw_sub = self.create_subscription(
            Twist,
            f"/{robot_name}/cmd_vel_raw",
            self.raw_callback,
            10,
        )

        self.safe_pub = self.create_publisher(
            Twist,
            f"/{robot_name}/cmd_vel_safe",
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_command,
        )

    def raw_callback(self, msg):
        self.latest_command = msg
        self.last_command_time = time.monotonic()

    def publish_command(self):
        if not self.enabled:
            self.safe_pub.publish(Twist())
            return

        # Do not continue an old movement command.
        if (
            time.monotonic() - self.last_command_time
            > 0.3
        ):
            self.safe_pub.publish(Twist())
            return

        avoidance_command = (
            self.avoid.get_avoidance_command()
        )

        if avoidance_command is not None:
            self.safe_pub.publish(avoidance_command)
        else:
            self.safe_pub.publish(
                self.latest_command
            )

    def stop(self):
        self.enabled = False
        self.safe_pub.publish(Twist())

    def close(self):
        self.stop()
        self.avoid.destroy_node()
        self.destroy_node()


class MissionExecutor:
    """Execute and monitor a mission specification."""

    def __init__(self, mission_spec, timeout=120.0):
        self.spec = mission_spec
        self.timeout = timeout

        self.nodes = {}
        self.threads = {}
        self.results = {}
        self.muxes = {}

        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.state = "PENDING"
        self.termination_reason = None

        self._validate()

    def _validate(self):
        name = self.spec.get("name")

        if name not in MISSIONS:
            raise ValueError(
                f"Unknown mission: {name}"
            )

        behaviors = self.spec.get("behaviors", [])

        if not behaviors:
            raise ValueError(
                "Mission has no behaviors"
            )

        required = set(
            MISSIONS[name].SUPPORTED_BEHAVIORS
        )

        present = {
            behavior["type"]
            for behavior in behaviors
        }

        if not required.issubset(present):
            raise ValueError(
                f"Missing behaviors: "
                f"{sorted(required - present)}"
            )

        if not present.issubset(required):
            raise ValueError(
                f"Unsupported behaviors: "
                f"{sorted(present - required)}"
            )

    def _avoidance_robots(self, spec):
        if "robots" in spec:
            return list(spec["robots"])

        if "robot" in spec:
            return [spec["robot"]]

        return []

    def _prepare(self):
        for spec in self.spec["behaviors"]:
            if spec["type"] != "avoid_obstacle":
                continue

            params = spec.get("parameters", {})

            for robot_name in self._avoidance_robots(spec):
                if robot_name in self.muxes:
                    raise ValueError(
                        f"Duplicate avoidance: {robot_name}"
                    )

                self.muxes[robot_name] = AvoidanceMux(
                    robot_name=robot_name,
                    **params,
                )

        for index, spec in enumerate(
            self.spec["behaviors"]
        ):
            kind = spec["type"]

            if kind == "avoid_obstacle":
                continue

            kwargs = dict(
                spec.get("parameters", {})
            )

            if kind in (
                "move_to_goal",
                "explore_area",
                "search_target",
            ):
                kwargs.setdefault(
                    "robot_name",
                    spec["robot"],
                )

            elif kind == "follow":
                kwargs.setdefault(
                    "follower_name",
                    spec["robot"],
                )

            elif kind == "maintain_formation":
                kwargs["continuous"] = True

            behavior = BEHAVIOR_CLASSES[kind](
                **kwargs
            )

            behavior_id = f"{kind}_{index}"

            self.nodes[behavior_id] = behavior

            self.results[behavior_id] = {
                "status": "PENDING",
                "success": None,
                "metric": None,
                "execution_time": None,
                "error": None,
            }

    def input_topic(self, robot_name):
        if robot_name in self.muxes:
            return (
                f"/{robot_name}/cmd_vel_safe"
            )

        return (
            f"/{robot_name}/cmd_vel_raw"
        )

    def _run_behavior(self, behavior_id, behavior):
        try:
            value = behavior.execute()

            if isinstance(value, tuple):
                success = bool(value[0])
                metric = (
                    value[1]
                    if len(value) > 1
                    else None
                )
                elapsed = (
                    value[2]
                    if len(value) > 2
                    else None
                )
            else:
                success = bool(value)
                metric = None
                elapsed = None

            with self.lock:
                self.results[behavior_id].update({
                    "status": (
                        "COMPLETED"
                        if success
                        else "FAILED"
                    ),
                    "success": success,
                    "metric": metric,
                    "execution_time": elapsed,
                })

        except Exception as exc:
            with self.lock:
                self.results[behavior_id].update({
                    "status": "ERROR",
                    "success": False,
                    "error": repr(exc),
                })

    def _spin_muxes(self):
        from rclpy.executors import SingleThreadedExecutor

        executors = []
        try:
            for mux in self.muxes.values():
                for node in (mux, mux.avoid):
                    executor = SingleThreadedExecutor(
                        context=node.context
                    )
                    executors.append((executor, node))

            while rclpy.ok() and not self.stop_event.is_set():
                for executor, node in executors:
                    rclpy.spin_once(
                        node,
                        executor=executor,
                        timeout_sec=0.005,
                    )
        finally:
            for executor, _ in executors:
                executor.shutdown()

    def _snapshot(self):
        with self.lock:
            return {
                key: value.copy()
                for key, value in self.results.items()
            }

    def _primary_ids(self):
        return [
            key
            for key in self.nodes
            if key.startswith(
                ("move_to_goal_", "explore_area_")
            )
        ]

    def _support_ids(self):
        return [
            key
            for key in self.nodes
            if key.startswith(
                ("follow_", "maintain_formation_")
            )
        ]

    def _evaluate_execution(self):
        name = self.spec["name"]
        results = self._snapshot()

        primary = self._primary_ids()
        support = self._support_ids()

        # A behavior exception is an execution error.
        if any(
            result["status"] == "ERROR"
            for result in results.values()
        ):
            return "ERROR", "behavior_error"

        # Search: detecting the target completes the mission.
        if name == "search":
            for key, result in results.items():
                if (
                    key.startswith("search_target_")
                    and result["success"] is True
                ):
                    return "COMPLETED", "target_found"

            if any(
                key in results
                and results[key]["success"] is False
                for key in primary
            ):
                return "FAILED", "exploration_failed"

            if all(
                results[key]["status"] != "PENDING"
                for key in primary
            ):
                return "FAILED", "target_not_found"

            return None, None

        # A failed primary behavior ends the mission.
        if any(
            results[key]["success"] is False
            for key in primary
        ):
            return "FAILED", "movement_failed"

        # A support behavior failing before mission end.
        if any(
            results[key]["success"] is False
            for key in support
        ):
            return "FAILED", "support_failed"

        # Wait until every primary movement finishes.
        if not all(
            results[key]["success"] is True
            for key in primary
        ):
            return None, None

        # Leader-Follower: inspect final tracking distance.
        if name == "leader_follower":
            followers = [
                node
                for key, node in self.nodes.items()
                if key.startswith("follow_")
            ]

            if not followers:
                return "ERROR", "missing_follower"

            for follower in followers:
                distance = follower.distance_to_leader()

                if distance is None:
                    return "FAILED", "missing_follower_odom"

                limit = (
                    follower.follow_distance
                    + follower.tolerance
                )

                if distance > limit:
                    return "FAILED", "follow_distance_exceeded"

        # Formation: inspect the final formation error.
        if name in (
            "formation_navigation",
            "formation_obstacle_avoidance",
        ):
            formations = [
                node
                for key, node in self.nodes.items()
                if key.startswith("maintain_formation_")
            ]

            if not formations:
                return "ERROR", "missing_formation"

            for formation in formations:
                if not formation.all_odom_received():
                    return "FAILED", "missing_formation_odom"

                for follower_name in formation.follower_names:
                    target = formation.get_target_position(
                        follower_name
                    )

                    if target is None:
                        return "FAILED", "missing_formation_target"

                    error = formation.distance_to_target(
                        follower_name,
                        *target,
                    )

                    if error > formation.tolerance:
                        return "FAILED", "formation_error"

        # Rendezvous: all MoveToGoal nodes must succeed.
        # This is already enforced by checking every primary.
        return "COMPLETED", "primary_behaviors_completed"

    def _request_stop(self):
        self.stop_event.set()

        for behavior in self.nodes.values():
            behavior.stop_requested = True

        for mux in self.muxes.values():
            mux.stop()

    def _cleanup(self, mux_thread):
        self._request_stop()

        for thread in self.threads.values():
            thread.join(timeout=3.0)

        if mux_thread is not None:
            mux_thread.join(timeout=3.0)

        active = [
            key
            for key, thread in self.threads.items()
            if thread.is_alive()
        ]

        if active:
            self.state = "ERROR"
            self.termination_reason = (
                "behavior_threads_not_stopped"
            )

        for key, behavior in self.nodes.items():
            thread = self.threads.get(key)

            if thread is None or not thread.is_alive():
                if hasattr(behavior, "stop_all_robots"):
                    behavior.stop_all_robots()
                elif hasattr(behavior, "stop_robot"):
                    behavior.stop_robot()

                if hasattr(behavior, "_local_executor"):
                    behavior._local_executor.shutdown()

                behavior.destroy_node()

        if mux_thread is None or not mux_thread.is_alive():
            for mux in self.muxes.values():
                mux.close()
        else:
            self.state = "ERROR"
            self.termination_reason = "mux_thread_not_stopped"

    def execute(self, on_prepared=None):
        if self.state != "PENDING":
            raise RuntimeError(
                "MissionExecutor can only execute once"
            )

        start_time = time.monotonic()
        deadline = start_time + self.timeout

        self.state = "RUNNING"
        mux_thread = None

        try:
            self._prepare()

            if on_prepared is not None:
                on_prepared(self)

            if self.muxes:
                mux_thread = threading.Thread(
                    target=self._spin_muxes,
                    daemon=True,
                )
                mux_thread.start()

            for key, behavior in self.nodes.items():
                thread = threading.Thread(
                    target=self._run_behavior,
                    args=(key, behavior),
                    daemon=True,
                )

                self.threads[key] = thread
                thread.start()

            while rclpy.ok():
                status, reason = (
                    self._evaluate_execution()
                )

                if status is not None:
                    self.state = status
                    self.termination_reason = reason
                    break

                if time.monotonic() >= deadline:
                    self.state = "TIMEOUT"
                    self.termination_reason = (
                        "mission_timeout"
                    )
                    break

                time.sleep(0.05)

            if not rclpy.ok():
                self.state = "INTERRUPTED"
                self.termination_reason = (
                    "ros_shutdown"
                )

        except Exception as exc:
            self.state = "ERROR"
            self.termination_reason = repr(exc)

        finally:
            self._cleanup(mux_thread)

        return {
            "mission": self.spec["name"],
            "status": self.state,
            "reason": self.termination_reason,
            "behavior_results": self._snapshot(),
            "execution_time": (
                time.monotonic() - start_time
            ),
        }
