import os
import subprocess
import time
import signal


WORLD_DIR = "/opt/ros/humble/share/turtlebot3_gazebo/worlds"

WORLD_PATHS = {
    "empty": os.path.join(WORLD_DIR, "empty_world.world"),
    "turtlebot3_world": os.path.join(WORLD_DIR, "turtlebot3_world.world"),
    "dqn_stage4": os.path.join(WORLD_DIR, "turtlebot3_dqn_stage4.world"),
    "house": os.path.join(WORLD_DIR, "turtlebot3_house.world"),
}

ROBOT_SDF = (
    "/opt/ros/humble/share/turtlebot3_gazebo/"
    "models/turtlebot3_burger/model.sdf"
)


class ScenarioManager:

    def __init__(self):
        self.gazebo_process = None
        self.gazebo_log = None

    def _wait_for_old_gazebo_to_disappear(self, timeout=30):
        start_time = time.time()

        while time.time() - start_time < timeout:
            result = subprocess.run(
                ["ros2", "service", "list"],
                capture_output=True,
                text=True,
            )

            if "/spawn_entity" not in result.stdout:
                return

            time.sleep(0.5)

        raise RuntimeError(
            "Previous Gazebo /spawn_entity service is still available."
        )

    def start_world(self, world_name):
        self._wait_for_old_gazebo_to_disappear()

        if world_name not in WORLD_PATHS:
            raise ValueError(f"Unknown world: {world_name}")

        world_path = WORLD_PATHS[world_name]

        print("\n================================")
        print("Starting Gazebo World")
        print("================================")
        print(f"World : {world_name}")
        print(f"Path  : {world_path}")
        print("================================\n")

        command = [
            "ros2",
            "launch",
            "gazebo_ros",
            "gazebo.launch.py",
            f"world:={world_path}",
            "gui:=false",
        ]

        os.makedirs("/workspace/logs", exist_ok=True)

        self.gazebo_log = open(
            "/workspace/logs/gazebo.log",
            "a",
        )

        self.gazebo_process = subprocess.Popen(
            command,
            stdout=self.gazebo_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
)

        self._wait_for_spawn_service()

    def _wait_for_spawn_service(self, timeout=120):
        print("Waiting for Gazebo ROS services...")

        start_time = time.time()

        while time.time() - start_time < timeout:
            result = subprocess.run(
                ["ros2", "service", "list"],
                capture_output=True,
                text=True,
            )

            if "/spawn_entity" in result.stdout:
                elapsed = time.time() - start_time
                print(
                    f"Gazebo started and /spawn_entity is ready "
                    f"({elapsed:.1f} s)."
                )
                return

            elapsed = int(time.time() - start_time)

            if elapsed > 0 and elapsed % 10 == 0:
                print(f"Still waiting... {elapsed} s")

            time.sleep(1)

        raise RuntimeError(
            f"Gazebo started, but /spawn_entity service "
            f"was not available within {timeout} seconds."
        )

    def spawn_robot(self, robot_name, x, y, yaw=0.0):
        namespace = f"/{robot_name}"

        print("\n================================")
        print("Spawning Robot")
        print("================================")
        print(f"Robot     : {robot_name}")
        print(f"Namespace : {namespace}")
        print(f"Position  : ({x}, {y}, 0.01)")
        print(f"Yaw       : {yaw}")
        print("================================\n")

        command = [
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity", robot_name,
            "-file", ROBOT_SDF,
            "-robot_namespace", namespace,
            "-x", str(x),
            "-y", str(y),
            "-z", "0.01",
            "-Y", str(yaw),
        ]

        result = subprocess.run(
            command,
            stdout=self.gazebo_log,
            stderr=subprocess.STDOUT,
        )

        if result.returncode != 0:
            raise RuntimeError(f"Failed to spawn robot: {robot_name}")

        print(f"Robot {robot_name} spawned.")

    def spawn_robots(self, robot_count):
        spacing = 1.0

        for i in range(robot_count):
            robot_name = f"tb0_{i + 1}"

            self.spawn_robot(
                robot_name=robot_name,
                x=0.0,
                y=i * spacing,
            )

    def stop_simulation(self):
        print("\nStopping Gazebo...")

        # Stop the ros2 launch process first.
        if self.gazebo_process is not None:
            try:
                os.killpg(
                    os.getpgid(self.gazebo_process.pid),
                    signal.SIGTERM,
                )

                self.gazebo_process.wait(timeout=5)

            except (subprocess.TimeoutExpired, ProcessLookupError):
                pass

            self.gazebo_process = None

        # Make sure no Gazebo server remains in the container.
        subprocess.run(
            ["pkill", "-TERM", "gzserver"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        time.sleep(1)

        # Force kill only if graceful termination failed.
        result = subprocess.run(
            ["pgrep", "gzserver"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        if result.returncode == 0:
            subprocess.run(
                ["pkill", "-KILL", "gzserver"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        time.sleep(2)

        if self.gazebo_log is not None:
            self.gazebo_log.close()
            self.gazebo_log = None

        print("Gazebo stopped.")