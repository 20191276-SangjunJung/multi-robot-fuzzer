class BugOracle:

    def __init__(self, tolerance=0.1):
        self.tolerance = tolerance

    def evaluate(self, mission_completed, final_distance):
        if final_distance is None:
            return {
                "result": "FAILURE",
                "reason": "No odometry received",
            }

        if mission_completed and final_distance <= self.tolerance:
            return {
                "result": "SUCCESS",
                "reason": "Goal reached",
            }

        return {
            "result": "FAILURE",
            "reason": "Goal not reached",
        }