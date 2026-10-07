import math
import random


WORLD_CONFIGS = {
    "empty": {
        "spawn_regions": [
            {
                "x_min": -3.0,
                "x_max": 3.0,
                "y_min": -3.0,
                "y_max": 3.0,
            },
        ],
    },

    "turtlebot3_world": {
        "spawn_regions": [
            {
                "x_min": -3.0,
                "x_max": -2.3,
                "y_min": -1.5,
                "y_max": 1.5,
            },
            {
                "x_min": 2.3,
                "x_max": 3.0,
                "y_min": -1.5,
                "y_max": 1.5,
            },
        ],
    },

    "dqn_stage4": {
        "spawn_regions": [
            {
                "x_min": -1.5,
                "x_max": 1.5,
                "y_min": -1.5,
                "y_max": 1.5,
            },
        ],
    },

    "house": {
        "spawn_regions": [
            {
                "x_min": -4.5,
                "x_max": -2.0,
                "y_min": -3.3,
                "y_max": -2.3,
            },
            {
                "x_min": 0.0,
                "x_max": 4.3,
                "y_min": -4.5,
                "y_max": -1.0,
            },
            {
                "x_min": 3.0,
                "x_max": 6.5,
                "y_min": 0.5,
                "y_max": 2.2,
            },
        ],
    },
}


DELAY_MIN_MS = 100
DELAY_MAX_MS = 500

MIN_GOAL_DISTANCE = 0.8


class TestCaseGenerator:

    def generate_position(self, region):
        return {
            "x": round(
                random.uniform(
                    region["x_min"],
                    region["x_max"],
                ),
                2,
            ),
            "y": round(
                random.uniform(
                    region["y_min"],
                    region["y_max"],
                ),
                2,
            ),
        }

    def generate_spawn_and_goal(self, world):
        region = random.choice(
            WORLD_CONFIGS[world]["spawn_regions"]
        )

        spawn_position = self.generate_position(region)

        while True:
            goal_position = self.generate_position(region)

            distance = math.sqrt(
                (
                    goal_position["x"]
                    - spawn_position["x"]
                ) ** 2
                +
                (
                    goal_position["y"]
                    - spawn_position["y"]
                ) ** 2
            )

            if distance >= MIN_GOAL_DISTANCE:
                break

        spawn = {
            "x": spawn_position["x"],
            "y": spawn_position["y"],
            "yaw": round(
                random.uniform(-math.pi, math.pi),
                2,
            ),
        }

        return spawn, goal_position

    def generate(self):
        world = random.choice(
            list(WORLD_CONFIGS.keys())
        )

        spawn, goal = self.generate_spawn_and_goal(world)

        delay_ms = random.randint(
            DELAY_MIN_MS,
            DELAY_MAX_MS,
        )

        return {
            "scenario": {
                "world": world,
                "robots": [
                    {
                        "name": "tb0_1",
                        "spawn": spawn,
                    }
                ],
            },

            "mission": {
                "name": "navigation",
                "behaviors": [
                    {
                        "type": "move_to_goal",
                        "robot": "tb0_1",
                        "parameters": {
                            "goal_x": goal["x"],
                            "goal_y": goal["y"],
                        },
                    }
                ],
            },

            "faults": [
                {
                    "domain": "network",
                    "type": "delay",
                    "target": "tb0_1",
                    "parameters": {
                        "delay_ms": delay_ms,
                    },
                }
            ],
        }