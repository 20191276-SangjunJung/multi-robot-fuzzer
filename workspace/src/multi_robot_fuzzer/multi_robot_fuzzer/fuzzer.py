
import csv
import json
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from multi_robot_fuzzer.scenario_manager import ScenarioManager
from multi_robot_fuzzer.mission_executor import MissionExecutor
from multi_robot_fuzzer.fault_injector import NetworkFaultInjector
from multi_robot_fuzzer.bug_oracle import BugOracle
from multi_robot_fuzzer.test_case_generator import TestCaseGenerator


TEST_COUNT = 3
RESULT_FILE = "/workspace/fuzzing_results_v2.csv"

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
        filename, "a", newline="", encoding="utf-8"
    ) as csvfile:
        writer = csv.DictWriter(
            csvfile,
            fieldnames=CSV_FIELDS,
        )

        if not file_exists or os.path.getsize(filename) == 0:
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
        scenario.start_world(
            scenario_spec["world"]
        )

        for robot in robots:
            spawn = robot["spawn"]

            scenario.spawn_robot(
                robot_name=robot["name"],
                x=spawn["x"],
                y=spawn["y"],
                yaw=spawn["yaw"],
            )

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

            for robot in robots:
                name = robot["name"]
                fault = fault_by_robot.get(name)

                delay_ms = (
                    fault["parameters"]["delay_ms"]
                    if fault is not None
                    else 0
                )

                injector = NetworkFaultInjector(
                    robot_name=name,
                    delay_ms=delay_ms,
                    input_topic=execution.input_topic(name),
                )

                injectors.append(injector)

            ros_executor = MultiThreadedExecutor(
                num_threads=max(2, len(injectors))
            )

            for injector in injectors:
                ros_executor.add_node(injector)

            ros_thread = threading.Thread(
                target=ros_executor.spin,
                daemon=True,
            )
            ros_thread.start()

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
            f"Test #{test_number} execution error: {exc}"
        )

    finally:
        if ros_executor is not None:
            ros_executor.shutdown()

        if ros_thread is not None:
            ros_thread.join(timeout=3.0)

        for injector in injectors:
            injector.destroy_node()

        scenario.stop_simulation()

        time.sleep(2)

    oracle = BugOracle()

    result = oracle.evaluate(
        mission_spec,
        execution_result,
    )

    print(
        f"Test #{test_number}: {result['result']} "
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


def main(args=None):
    rclpy.init(args=["--ros-args", "--log-level", "fatal"])

    success_count = 0
    failure_count = 0

    generator = TestCaseGenerator()

    try:
        for index in range(1, TEST_COUNT + 1):
            test_case = generator.generate()

            try:
                result = run_test(
                    index,
                    test_case,
                    RESULT_FILE,
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
        print("Fuzzing Summary")
        print("================================")
        print(f"Total Tests : {TEST_COUNT}")
        print(f"SUCCESS     : {success_count}")
        print(f"FAILURE     : {failure_count}")
        print(f"Results     : {RESULT_FILE}")
        print("================================")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
