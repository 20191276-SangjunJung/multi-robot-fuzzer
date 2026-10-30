import argparse
import csv
import json
import math
import os
import random
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor

from multi_robot_fuzzer.scenario_manager import ScenarioManager
from multi_robot_fuzzer.mission_executor import MissionExecutor
from multi_robot_fuzzer.fault_injector import NetworkFaultInjector
from multi_robot_fuzzer.bug_oracle import BugOracle
from multi_robot_fuzzer.test_case_generator import TestCaseGenerator


TEST_COUNT = 3

RESULT_FILE = "/workspace/fuzzing_results_v2.csv"
EKF_CONFIG = "/workspace/src/multi_robot_fuzzer/config/ekf.yaml"
INJECTOR_SCRIPT = (
    "/workspace/src/multi_robot_fuzzer/"
    "multi_robot_fuzzer/fault_injector.py"
)


def start_process(command):
    proc = subprocess.Popen(
        command,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )

    time.sleep(0.4)

    if proc.poll() is not None:
        raise RuntimeError(
            f"Process failed: {command} "
            f"({proc.stderr.read().decode(errors='replace')})"
        )

    return proc


def stop_process(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)

        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


CSV_FIELDS = [
    "test_number",
    "world",
    "mission",
    "robot_count",
    "robot_names",
    "scenario_json",
    "mission_json",
    "faults_json",
    "execution_status",
    "execution_reason",
    "behavior_results_json",
    "result",
    "reason",
    "execution_time",
]


def save_result(
    filename,
    test_number,
    test_case,
    oracle_result,
    execution_result,
):
    file_exists = os.path.exists(filename)

    scenario = test_case["scenario"]
    mission = test_case["mission"]
    robots = scenario["robots"]

    row = {
        "test_number": test_number,
        "world": scenario["world"],
        "mission": mission["name"],
        "robot_count": len(robots),
        "robot_names": ",".join(
            robot["name"] for robot in robots
        ),
        "scenario_json": json.dumps(
            scenario, ensure_ascii=False
        ),
        "mission_json": json.dumps(
            mission, ensure_ascii=False
        ),
        "faults_json": json.dumps(
            test_case.get("faults", []),
            ensure_ascii=False,
        ),
        "execution_status": execution_result.get(
            "status", "ERROR"
        ),
        "execution_reason": execution_result.get(
            "reason", ""
        ),
        "behavior_results_json": json.dumps(
            execution_result.get("behavior_results", {}),
            ensure_ascii=False,
        ),
        "result": oracle_result["result"],
        "reason": oracle_result["reason"],
        "execution_time": execution_result.get(
            "execution_time", 0.0
        ),
    }

    with open(
        filename,
        "a",
        newline="",
        encoding="utf-8",
    ) as csvfile:
        writer = csv.DictWriter(
            csvfile,
            fieldnames=CSV_FIELDS,
        )

        if (
            not file_exists
            or os.path.getsize(filename) == 0
        ):
            writer.writeheader()

        writer.writerow(row)


def run_test(test_number, test_case, result_file):
    scenario_spec = test_case["scenario"]
    mission_spec = test_case["mission"]
    robots = scenario_spec["robots"]
    faults = test_case.get("faults", [])

    print(
        f"\nTest Case #{test_number}: "
        f"{mission_spec['name']} "
        f"world={scenario_spec['world']} "
        f"robots={len(robots)}"
    )

    scenario = ScenarioManager()

    injectors = []
    sensor_processes = []

    ros_executor = None
    ros_thread = None

    execution_result = {
        "status": "ERROR",
        "reason": "not_started",
        "behavior_results": {},
        "execution_time": 0.0,
    }

    start_time = time.monotonic()

    try:
        # Start Gazebo world.
        scenario.start_world(scenario_spec["world"])

        # Spawn robots.
        for robot in robots:
            spawn = robot["spawn"]

            scenario.spawn_robot(
                robot_name=robot["name"],
                x=spawn["x"],
                y=spawn["y"],
                yaw=spawn["yaw"],
            )

        # Collect IMU faults.
        imu_faults = [
            fault
            for fault in faults
            if (
                fault.get("domain") == "sensor"
                and fault.get("sensor") == "imu"
            )
        ]

        if imu_faults:
            # Static transform for IMU frame.
            sensor_processes.append(
                start_process([
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x", "-0.032",
                    "--y", "0",
                    "--z", "0.078",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "imu_link",
                ])
            )

            # Start IMU injectors and EKF nodes.
            for fault in imu_faults:
                name = fault["target"]
                params = fault["parameters"]

                if fault["type"] != "acoustic":
                    raise ValueError(
                        f"Unsupported IMU fault: "
                        f"{fault['type']}"
                    )

                amplitude = params["amplitude"]
                frequency = params["frequency"]
                attack_start = params["start_time"]

                print(
                    f"[IMU Attack] "
                    f"robot={name}, "
                    f"A={amplitude} rad/s, "
                    f"f={frequency} Hz, "
                    f"start={attack_start} s"
                )

                # Acoustic gyroscope fault injector.
                sensor_processes.append(
                    start_process([
                        "python3",
                        INJECTOR_SCRIPT,
                        "--robot", name,
                        "--amplitude", str(amplitude),
                        "--frequency", str(frequency),
                        "--start-time", str(attack_start),
                    ])
                )

                # Load robot-specific EKF configuration.
                config = (
                    Path(EKF_CONFIG)
                    .read_text()
                    .replace("/tb0_1/", f"/{name}/")
                )

                config = config.replace(
                    "ekf_filter_node:\n",
                    "/**:\n",
                    1,
                )

                # Preserve the existing wheel odometry
                # configuration used in IMU tests.
                config, matches = re.subn(
                    r"odom0_config:\s*\[[^]]*\]",
                    (
                        "odom0_config: ["
                        "true, true, false, "
                        "false, false, true, "
                        "true, false, false, "
                        "false, false, false, "
                        "false, false, false]"
                    ),
                    config,
                    count=1,
                    flags=re.S,
                )

                if matches != 1:
                    raise RuntimeError(
                        "odom0_config not found "
                        "in EKF config"
                    )

                config_file = Path(
                    f"/tmp/ekf_{name}.yaml"
                )

                config_file.write_text(config)

                # Start robot_localization EKF.
                sensor_processes.append(
                    start_process([
                        "ros2",
                        "run",
                        "robot_localization",
                        "ekf_node",
                        "--ros-args",
                        "--params-file",
                        str(config_file),
                        "-r",
                        f"__node:=ekf_{name}",
                        "-r",
                        (
                            "/odometry/filtered:="
                            f"/{name}/odometry/filtered"
                        ),
                    ])
                )

            # Allow sensor nodes to initialize.
            time.sleep(2.0)

        # Prepare mission executor.
        mission_executor = MissionExecutor(
            mission_spec,
            timeout=120.0,
        )

        def prepare(execution):
            nonlocal ros_executor, ros_thread

            fault_by_robot = {
                fault["target"]: fault
                for fault in faults
                if (
                    fault["domain"] == "network"
                    and fault["type"] == "delay"
                )
            }

            # Create network injectors.
            for robot in robots:
                name = robot["name"]
                fault = fault_by_robot.get(name)

                delay_ms = (
                    fault["parameters"]["delay_ms"]
                    if fault
                    else 0
                )

                injector = NetworkFaultInjector(
                    robot_name=name,
                    delay_ms=delay_ms,
                    input_topic=execution.input_topic(name),
                )

                injectors.append(injector)

            ros_executor = MultiThreadedExecutor(
                num_threads=max(
                    2,
                    len(injectors),
                )
            )

            for injector in injectors:
                ros_executor.add_node(injector)

            ros_thread = threading.Thread(
                target=ros_executor.spin,
                daemon=True,
            )

            ros_thread.start()

        # Execute mission.
        execution_result = mission_executor.execute(
            on_prepared=prepare
        )

    except Exception as exc:
        execution_result = {
            "status": "ERROR",
            "reason": repr(exc),
            "behavior_results": execution_result.get(
                "behavior_results", {}
            ),
            "execution_time": (
                time.monotonic() - start_time
            ),
        }

        print(
            f"Test #{test_number} execution error: "
            f"{exc}"
        )

    finally:
        # Stop ROS executor.
        if ros_executor is not None:
            ros_executor.shutdown()

        if ros_thread is not None:
            ros_thread.join(timeout=3.0)

        for injector in injectors:
            injector.destroy_node()

        # Stop IMU and EKF processes.
        for proc in reversed(sensor_processes):
            stop_process(proc)

        scenario.stop_simulation()
        time.sleep(2)

    # Evaluate mission result.
    oracle = BugOracle()

    result = oracle.evaluate(
        mission_spec,
        execution_result,
    )

    print(
        f"Test #{test_number}: "
        f"{result['result']} "
        f"(Mission: {execution_result['status']}, "
        f"Reason: {result['reason']})"
    )

    save_result(
        result_file,
        test_number,
        test_case,
        result,
        execution_result,
    )

    return result


def choose_parameter(value, bounds, mode):
    low, high = bounds

    if not all(
        math.isfinite(v) for v in (low, high)
    ):
        raise ValueError(
            f"Non-finite parameter range: {bounds}"
        )

    if low < 0 or high < low:
        raise ValueError(
            f"Invalid parameter range: {bounds}"
        )

    if value is not None:
        if (
            not math.isfinite(value)
            or value < 0
        ):
            raise ValueError(
                f"Invalid parameter: {value}"
            )

        return value

    if mode == "fixed":
        return low

    return random.uniform(low, high)


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Run multi-robot mission tests"
    )

    parser.add_argument(
        "--mode",
        choices=["random", "fixed"],
        default="random",
    )

    parser.add_argument(
        "--mission",
        default="leader_follower",
    )

    parser.add_argument(
        "--world",
        default="dqn_stage4",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
    )

    parser.add_argument(
        "--count",
        type=int,
        default=None,
    )

    parser.add_argument(
        "--result-file",
        default=RESULT_FILE,
    )

    # IMU attack type.
    parser.add_argument(
        "--imu-fault",
        choices=["acoustic"],
        default=None,
    )

    # Explicit acoustic attack parameters.
    parser.add_argument(
        "--imu-amplitude",
        type=float,
        default=None,
        help="Gyroscope error amplitude (rad/s)",
    )

    parser.add_argument(
        "--imu-frequency",
        type=float,
        default=None,
        help="Gyroscope error frequency (Hz)",
    )

    parser.add_argument(
        "--imu-start-time",
        type=float,
        default=None,
        help="Attack start time (seconds)",
    )

    # Temporary parameter ranges for initial testing.
    parser.add_argument(
        "--imu-amplitude-range",
        type=float,
        nargs=2,
        default=[0.0, 0.05],
        metavar=("MIN", "MAX"),
    )

    parser.add_argument(
        "--imu-frequency-range",
        type=float,
        nargs=2,
        default=[0.1, 2.0],
        metavar=("MIN", "MAX"),
    )

    parser.add_argument(
        "--imu-start-time-range",
        type=float,
        nargs=2,
        default=[0.0, 10.0],
        metavar=("MIN", "MAX"),
    )

    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)

    test_count = (
        options.count
        if options.count is not None
        else (
            1
            if options.mode == "fixed"
            else TEST_COUNT
        )
    )

    if test_count < 1:
        raise ValueError(
            "--count must be at least 1"
        )

    ros_args = [
        "--ros-args",
        "--log-level",
        "fatal",
    ]

    # Route behavior odometry through EKF
    # when IMU attack is enabled.
    if options.imu_fault:
        for name in (
            "tb0_0",
            "tb0_1",
            "tb0_2",
        ):
            ros_args += [
                "-r",
                (
                    f"/{name}/odom:="
                    f"/{name}/odometry/filtered"
                ),
            ]

    rclpy.init(args=ros_args)

    success_count = 0
    failure_count = 0

    generator = TestCaseGenerator()

    try:
        for index in range(
            1,
            test_count + 1,
        ):
            try:
                if options.mode == "fixed":
                    random.seed(options.seed)

                    test_case = generator.generate(
                        mission_name=options.mission,
                        world=options.world,
                    )

                    # Disable network delay
                    # during fixed baseline tests.
                    for fault in test_case.get(
                        "faults", []
                    ):
                        if (
                            fault["domain"] == "network"
                            and fault["type"] == "delay"
                        ):
                            fault["parameters"][
                                "delay_ms"
                            ] = 0

                else:
                    test_case = generator.generate()

                # Add acoustic IMU faults.
                if options.imu_fault:
                    amplitude = choose_parameter(
                        options.imu_amplitude,
                        options.imu_amplitude_range,
                        options.mode,
                    )

                    frequency = choose_parameter(
                        options.imu_frequency,
                        options.imu_frequency_range,
                        options.mode,
                    )

                    attack_start = choose_parameter(
                        options.imu_start_time,
                        options.imu_start_time_range,
                        options.mode,
                    )

                    generator.add_imu_faults(
                        test_case,
                        amplitude=amplitude,
                        frequency=frequency,
                        start_time=attack_start,
                    )

                # Run test.
                result = run_test(
                    index,
                    test_case,
                    options.result_file,
                )

                if result["result"] == "SUCCESS":
                    success_count += 1
                else:
                    failure_count += 1

            except Exception as exc:
                failure_count += 1

                print(
                    f"\n[Test #{index}] ERROR: {exc}"
                )

        print("\n================================")
        print("Test Summary")
        print("================================")
        print(f"Mode        : {options.mode}")
        print(f"Total Tests : {test_count}")
        print(f"SUCCESS     : {success_count}")
        print(f"FAILURE     : {failure_count}")
        print(f"Results     : {options.result_file}")
        print("================================")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()