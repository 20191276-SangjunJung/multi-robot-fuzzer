
class BugOracle:
    """
    Mission-level result evaluator (v1).

    Input:
        mission_spec: generated mission specification
        execution_result: MissionExecutor.execute() return value

    Output:
        {
            "result": "SUCCESS" | "FAILURE",
            "reason": str,
            "mission_status": str,
        }
    """

    def evaluate(self, mission_spec, execution_result):
        mission_name = mission_spec["name"]
        status = execution_result.get("status", "ERROR")
        reason = execution_result.get("reason")
        behaviors = execution_result.get("behavior_results", {})

        def outcome(result, explanation):
            return {
                "result": result,
                "reason": explanation,
                "mission_status": status,
            }

        # Execution errors and timeouts are never successes.
        if status != "COMPLETED":
            return outcome(
                "FAILURE",
                f"{status}: {reason or 'mission_not_completed'}",
            )

        def matching(prefix):
            return [
                value
                for key, value in behaviors.items()
                if key.startswith(prefix + "_")
            ]

        def all_succeeded(prefix, minimum=1):
            values = matching(prefix)
            return (
                len(values) >= minimum
                and all(v.get("success") is True for v in values)
            )

        if mission_name == "navigation":
            if all_succeeded("move_to_goal"):
                return outcome("SUCCESS", "goal_reached")
            return outcome("FAILURE", "navigation_not_verified")

        if mission_name == "leader_follower":
            if (
                all_succeeded("move_to_goal")
                and reason == "primary_behaviors_completed"
            ):
                # MissionExecutor checks the follower's
                # final tracking distance.
                return outcome(
                    "SUCCESS",
                    "leader_goal_and_final_follow_condition_met",
                )
            return outcome(
                "FAILURE",
                "leader_follower_not_verified",
            )

        if mission_name in (
            "formation_navigation",
            "formation_obstacle_avoidance",
        ):
            if (
                all_succeeded("move_to_goal")
                and reason == "primary_behaviors_completed"
            ):
                # MissionExecutor checks final formation
                # errors before reporting COMPLETED.
                return outcome(
                    "SUCCESS",
                    "goal_and_final_formation_condition_met",
                )
            return outcome(
                "FAILURE",
                "formation_not_verified",
            )

        if mission_name == "exploration":
            if all_succeeded("explore_area"):
                return outcome(
                    "SUCCESS",
                    "waypoints_completed",
                )
            return outcome(
                "FAILURE",
                "exploration_not_verified",
            )

        if mission_name == "search":
            if (
                reason == "target_found"
                and all_succeeded("search_target")
            ):
                return outcome(
                    "SUCCESS",
                    "target_found",
                )
            return outcome(
                "FAILURE",
                "target_not_verified",
            )

        if mission_name == "rendezvous":
            robot_count = len({
                b["robot"]
                for b in mission_spec["behaviors"]
                if b["type"] == "move_to_goal"
            })

            if all_succeeded(
                "move_to_goal",
                minimum=robot_count,
            ):
                return outcome(
                    "SUCCESS",
                    "all_robots_reached_rendezvous",
                )
            return outcome(
                "FAILURE",
                "rendezvous_not_verified",
            )

        return outcome(
            "FAILURE",
            f"unknown_mission: {mission_name}",
        )
