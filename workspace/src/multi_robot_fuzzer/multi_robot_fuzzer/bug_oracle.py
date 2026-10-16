class BugOracle:
    """Evaluate mission semantics; never manage ROS threads or lifecycle."""

    @staticmethod
    def _matching(results, prefix):
        return [v for k, v in results.items() if k.startswith(prefix + '_')]

    @classmethod
    def _all_succeeded(cls, results, prefix, minimum=1):
        values = cls._matching(results, prefix)
        return len(values) >= minimum and all(v.get('success') is True for v in values)

    @staticmethod
    def _formation_ready(nodes):
        formations = [v for k, v in nodes.items() if k.startswith('maintain_formation_')]
        if not formations:
            return False
        for formation in formations:
            if not formation.all_odom_received():
                return False
            for name in formation.follower_names:
                target = formation.get_target_position(name)
                if target is None:
                    return False
                error = formation.distance_to_target(name, *target)
                if error is None or error > formation.tolerance:
                    return False
        return True

    @staticmethod
    def _follow_ready(nodes):
        followers = [v for k, v in nodes.items() if k.startswith('follow_')]
        if not followers:
            return False
        for follower in followers:
            distance = follower.distance_to_leader()
            if distance is None or distance > follower.follow_distance + follower.tolerance:
                return False
        return True

    def assess_progress(self, mission_spec, behavior_results, nodes):
        """Return (status, reason, stop_support). None means keep running.

        This is a semantic decision only. The executor applies stop_support.
        """
        name = mission_spec['name']
        results = behavior_results
        primary = [k for k in nodes if k.startswith(('move_to_goal_', 'explore_area_'))]
        support = [k for k in nodes if k.startswith(('follow_', 'maintain_formation_'))]

        if name == 'search':
            if any(k.startswith('search_target_') and v.get('success') is True
                   for k, v in results.items()):
                return 'COMPLETED', 'target_found', False
            if any(results[k].get('success') is False for k in primary):
                return 'FAILED', 'exploration_failed', False
            if all(results[k].get('status') != 'PENDING' for k in primary):
                return 'FAILED', 'target_not_found', False
            return None, None, False

        if any(results[k].get('success') is False for k in primary):
            return 'FAILED', 'movement_failed', False
        if any(results[k].get('success') is False for k in support):
            return 'FAILED', 'support_failed', False
        if not all(results[k].get('success') is True for k in primary):
            return None, None, False

        if name in ('formation_navigation', 'formation_obstacle_avoidance'):
            if not self._formation_ready(nodes):
                return None, None, False
        if name == 'leader_follower' and not self._follow_ready(nodes):
            return None, None, False

        if any(results[k].get('status') == 'PENDING' for k in support):
            return None, None, True

        # Recheck at completion: odometry can change while support threads stop.
        if name in ('formation_navigation', 'formation_obstacle_avoidance'):
            if not self._formation_ready(nodes):
                return 'FAILED', 'formation_error', False
        if name == 'leader_follower' and not self._follow_ready(nodes):
            return 'FAILED', 'follow_distance_exceeded', False
        return 'COMPLETED', 'primary_behaviors_completed', False

    def evaluate(self, mission_spec, execution_result):
        """Final public result format consumed by fuzzer.py."""
        name = mission_spec['name']
        status = execution_result.get('status', 'ERROR')
        reason = execution_result.get('reason')
        behaviors = execution_result.get('behavior_results', {})

        def outcome(result, explanation):
            return {'result': result, 'reason': explanation, 'mission_status': status}

        if status != 'COMPLETED':
            return outcome('FAILURE', f"{status}: {reason or 'mission_not_completed'}")

        if name == 'navigation':
            if self._all_succeeded(behaviors, 'move_to_goal'):
                return outcome('SUCCESS', 'goal_reached')
            return outcome('FAILURE', 'navigation_not_verified')

        if name == 'leader_follower':
            if self._all_succeeded(behaviors, 'move_to_goal') and reason == 'primary_behaviors_completed':
                return outcome('SUCCESS', 'leader_goal_and_final_follow_condition_met')
            return outcome('FAILURE', 'leader_follower_not_verified')

        if name in ('formation_navigation', 'formation_obstacle_avoidance'):
            if self._all_succeeded(behaviors, 'move_to_goal') and reason == 'primary_behaviors_completed':
                return outcome('SUCCESS', 'goal_and_final_formation_condition_met')
            return outcome('FAILURE', 'formation_not_verified')

        if name == 'exploration':
            if self._all_succeeded(behaviors, 'explore_area'):
                return outcome('SUCCESS', 'waypoints_completed')
            return outcome('FAILURE', 'exploration_not_verified')

        if name == 'search':
            if reason == 'target_found' and self._all_succeeded(behaviors, 'search_target'):
                return outcome('SUCCESS', 'target_found')
            return outcome('FAILURE', 'target_not_verified')

        if name == 'rendezvous':
            robot_count = len({b['robot'] for b in mission_spec['behaviors']
                               if b['type'] == 'move_to_goal'})
            if self._all_succeeded(behaviors, 'move_to_goal', minimum=robot_count):
                return outcome('SUCCESS', 'all_robots_reached_rendezvous')
            return outcome('FAILURE', 'rendezvous_not_verified')

        return outcome('FAILURE', f'unknown_mission: {name}')
