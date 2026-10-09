
import math
import random


WORLD_CONFIGS = {
    "empty": {
        "spawn_regions": [
            {"x_min": -3.0, "x_max": 3.0,
             "y_min": -3.0, "y_max": 3.0},
        ],
    },
    "turtlebot3_world": {
        "spawn_regions": [
            {"x_min": -3.0, "x_max": -2.3,
             "y_min": -1.5, "y_max": 1.5},
            {"x_min": 2.3, "x_max": 3.0,
             "y_min": -1.5, "y_max": 1.5},
        ],
    },
    "dqn_stage4": {
        "spawn_regions": [
            {"x_min": -1.5, "x_max": 1.5,
             "y_min": -1.5, "y_max": 1.5},
        ],
    },
    "house": {
        "spawn_regions": [
            {"x_min": -4.5, "x_max": -2.0,
             "y_min": -3.3, "y_max": -2.3},
            {"x_min": 0.0, "x_max": 4.3,
             "y_min": -4.5, "y_max": -1.0},
            {"x_min": 3.0, "x_max": 6.5,
             "y_min": 0.5, "y_max": 2.2},
        ],
    },
}

MISSION_NAMES = [
    "navigation",
    "leader_follower",
    "formation_navigation",
    "formation_obstacle_avoidance",
    "exploration",
    "search",
    "rendezvous",
]

DELAY_MIN_MS = 100
DELAY_MAX_MS = 500
MIN_GOAL_DISTANCE = 0.8
MIN_ROBOT_DISTANCE = 0.5


class TestCaseGenerator:

    def generate_position(self, region):
        return {
            "x": round(random.uniform(
                region["x_min"], region["x_max"]), 2),
            "y": round(random.uniform(
                region["y_min"], region["y_max"]), 2),
        }

    def distance(self, a, b):
        return math.hypot(
            a["x"] - b["x"],
            a["y"] - b["y"],
        )

    def generate_spawn(self, position, yaw=None):
        return {
            "x": position["x"],
            "y": position["y"],
            "yaw": round(
                random.uniform(-math.pi, math.pi)
                if yaw is None else yaw, 2
            ),
        }

    def generate_spawn_and_goal(self, world):
        region = random.choice(
            WORLD_CONFIGS[world]["spawn_regions"]
        )

        for _ in range(1000):
            start = self.generate_position(region)
            goal = self.generate_position(region)

            if self.distance(start, goal) >= MIN_GOAL_DISTANCE:
                return self.generate_spawn(start), goal

        raise ValueError(
            f"Cannot generate spawn and goal in {world}"
        )

    def generate_robot_positions(self, world, count):
        regions = WORLD_CONFIGS[world]["spawn_regions"]

        positions = []

        for _ in range(count):
            for _ in range(1000):
                region = random.choice(regions)
                position = self.generate_position(region)

                if all(
                    self.distance(position, existing)
                    >= MIN_ROBOT_DISTANCE
                    for existing in positions
                ):
                    positions.append(position)
                    break
            else:
                raise ValueError(
                    f"Cannot place {count} robots in {world}"
                )

        return positions

    def make_robot(self, name, position, yaw=None):
        return {
            "name": name,
            "spawn": self.generate_spawn(position, yaw),
        }

    def make_behavior(self, kind, robot=None, **parameters):
        behavior = {
            "type": kind,
            "parameters": parameters,
        }

        if robot is not None:
            behavior["robot"] = robot

        return behavior

    def generate_navigation(self, world):
        spawn, goal = self.generate_spawn_and_goal(world)

        robots = [
            {"name": "tb0_1", "spawn": spawn}
        ]

        behaviors = [
            self.make_behavior(
                "move_to_goal", "tb0_1",
                goal_x=goal["x"],
                goal_y=goal["y"],
            )
        ]

        return robots, behaviors

    def generate_leader_follower(self, world):
        spawn, goal = self.generate_spawn_and_goal(world)

        leader_position = {
            "x": spawn["x"],
            "y": spawn["y"],
        }

        regions = WORLD_CONFIGS[world]["spawn_regions"]

        for _ in range(1000):
            region = random.choice(regions)
            follower_position = self.generate_position(region)

            if (
                MIN_ROBOT_DISTANCE
                <= self.distance(
                    leader_position, follower_position
                )
                <= 1.2
            ):
                break
        else:
            raise ValueError(
                "Cannot generate leader-follower positions"
            )

        robots = [
            {"name": "tb0_0", "spawn": spawn},
            self.make_robot("tb0_1", follower_position),
        ]

        behaviors = [
            self.make_behavior(
                "move_to_goal", "tb0_0",
                goal_x=goal["x"],
                goal_y=goal["y"],
            ),
            self.make_behavior(
                "follow", "tb0_1",
                leader_name="tb0_0",
                follower_name="tb0_1",
                follow_distance=0.8,
            ),
        ]

        return robots, behaviors

    def generate_formation(self, world, avoidance=False):
        # Use a shared orientation for the initial V formation.
        regions = WORLD_CONFIGS[world]["spawn_regions"]
        spacing = 0.8

        for _ in range(1000):
            region = random.choice(regions)
            leader = self.generate_position(region)

            yaw = random.uniform(-math.pi, math.pi)

            offsets = [
                (-spacing, spacing),
                (-spacing, -spacing),
            ]

            follower_positions = []

            for dx, dy in offsets:
                x = leader["x"] + (
                    dx * math.cos(yaw)
                    - dy * math.sin(yaw)
                )
                y = leader["y"] + (
                    dx * math.sin(yaw)
                    + dy * math.cos(yaw)
                )

                follower_positions.append({
                    "x": round(x, 2),
                    "y": round(y, 2),
                })

            # Ensure all robots start inside a known region.
            if not all(
                any(
                    r["x_min"] <= p["x"] <= r["x_max"]
                    and r["y_min"] <= p["y"] <= r["y_max"]
                    for r in regions
                )
                for p in [leader] + follower_positions
            ):
                continue

            goal = self.generate_position(region)

            if self.distance(leader, goal) >= MIN_GOAL_DISTANCE:
                break
        else:
            raise ValueError(
                f"Cannot generate formation in {world}"
            )

        follower_names = ["tb0_1", "tb0_2"]

        robots = [
            self.make_robot("tb0_0", leader, yaw),
            self.make_robot(
                "tb0_1", follower_positions[0], yaw
            ),
            self.make_robot(
                "tb0_2", follower_positions[1], yaw
            ),
        ]

        behaviors = [
            self.make_behavior(
                "move_to_goal", "tb0_0",
                goal_x=goal["x"],
                goal_y=goal["y"],
            ),
            self.make_behavior(
                "maintain_formation",
                leader_name="tb0_0",
                follower_names=follower_names,
                spacing=spacing,
            ),
        ]

        if avoidance:
            behaviors.append(
                self.make_behavior(
                    "avoid_obstacle",
                    robots=["tb0_0"] + follower_names,
                )
            )

        return robots, behaviors

    def generate_exploration(self, world, search=False):
        regions = WORLD_CONFIGS[world]["spawn_regions"]
        region = random.choice(regions)

        spawn_position = self.generate_position(region)
        waypoint_count = random.randint(2, 4)

        waypoints = []
        previous = spawn_position

        for _ in range(waypoint_count):
            for _ in range(1000):
                point = self.generate_position(region)

                if self.distance(previous, point) >= 0.5:
                    waypoints.append(point)
                    previous = point
                    break
            else:
                raise ValueError(
                    "Cannot generate exploration waypoints"
                )

        robots = [
            self.make_robot("tb0_1", spawn_position)
        ]

        behaviors = [
            self.make_behavior(
                "explore_area", "tb0_1",
                waypoints=waypoints,
            ),
            self.make_behavior(
                "avoid_obstacle", "tb0_1",
            ),
        ]

        if search:
            # Target is placed near one generated waypoint.
            target = random.choice(waypoints)

            behaviors.append(
                self.make_behavior(
                    "search_target", "tb0_1",
                    target_x=target["x"],
                    target_y=target["y"],
                    detection_range=0.5,
                )
            )

        return robots, behaviors

    def generate_rendezvous(self, world):
        count = random.randint(2, 3)
        regions = WORLD_CONFIGS[world]["spawn_regions"]

        region = random.choice(regions)

        for _ in range(1000):
            goal = self.generate_position(region)

            positions = []
            for _ in range(count):
                for _ in range(1000):
                    point = self.generate_position(region)

                    if (
                        self.distance(point, goal)
                        >= MIN_GOAL_DISTANCE
                        and all(
                            self.distance(point, p)
                            >= MIN_ROBOT_DISTANCE
                            for p in positions
                        )
                    ):
                        positions.append(point)
                        break
                else:
                    break

            if len(positions) == count:
                break
        else:
            raise ValueError(
                f"Cannot generate rendezvous in {world}"
            )

        robots = []
        behaviors = []
        robot_names = []

        for index, position in enumerate(positions):
            name = f"tb0_{index}"
            robot_names.append(name)

            robots.append(
                self.make_robot(name, position)
            )

            behaviors.append(
                self.make_behavior(
                    "move_to_goal", name,
                    goal_x=goal["x"],
                    goal_y=goal["y"],
                )
            )

        behaviors.append(
            self.make_behavior(
                "avoid_obstacle",
                robots=robot_names,
            )
        )

        return robots, behaviors

    def generate(self, mission_name=None, world=None):
        if mission_name is None:
            mission_name = random.choice(MISSION_NAMES)

        if mission_name not in MISSION_NAMES:
            raise ValueError(
                f"Unknown mission: {mission_name}"
            )

        if world is None:
            world = random.choice(
                list(WORLD_CONFIGS.keys())
            )

        if world not in WORLD_CONFIGS:
            raise ValueError(f"Unknown world: {world}")

        if mission_name == "navigation":
            robots, behaviors = self.generate_navigation(world)

        elif mission_name == "leader_follower":
            robots, behaviors = self.generate_leader_follower(world)

        elif mission_name == "formation_navigation":
            robots, behaviors = self.generate_formation(world)

        elif mission_name == "formation_obstacle_avoidance":
            robots, behaviors = self.generate_formation(
                world, avoidance=True
            )

        elif mission_name == "exploration":
            robots, behaviors = self.generate_exploration(world)

        elif mission_name == "search":
            robots, behaviors = self.generate_exploration(
                world, search=True
            )

        elif mission_name == "rendezvous":
            robots, behaviors = self.generate_rendezvous(world)

        faults = []

        for robot in robots:
            faults.append({
                "domain": "network",
                "type": "delay",
                "target": robot["name"],
                "parameters": {
                    "delay_ms": random.randint(
                        DELAY_MIN_MS, DELAY_MAX_MS
                    ),
                },
            })

        return {
            "scenario": {
                "world": world,
                "robots": robots,
            },
            "mission": {
                "name": mission_name,
                "behaviors": behaviors,
            },
            "faults": faults,
        }
