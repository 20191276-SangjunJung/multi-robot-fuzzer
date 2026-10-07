import csv
import os
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from multi_robot_fuzzer.scenario_manager import ScenarioManager
from multi_robot_fuzzer.behaviors.move_to_goal import MoveToGoal
from multi_robot_fuzzer.fault_injector import NetworkFaultInjector
from multi_robot_fuzzer.bug_oracle import BugOracle
from multi_robot_fuzzer.test_case_generator import TestCaseGenerator


TEST_COUNT = 3

def save_result(filename, test_case, result, execution_time):
    file_exists = os.path.exists(filename)

    scenario_spec = test_case["scenario"]
    mission_spec = test_case["mission"]
    fault_spec = test_case["faults"][0]

    robot_spec = scenario_spec["robots"][0]
    behavior_spec = mission_spec["behaviors"][0]

    with open(filename, "a", newline="") as csvfile:
        fieldnames = [
            "world",
            "mission",
            "robot",
            "spawn_x",
            "spawn_y",
            "spawn_yaw",
            "fault_domain",
            "fault_type",
            "delay_ms",
            "goal_x",
            "goal_y",
            "result",
            "reason",
            "execution_time",
        ]

        writer = csv.DictWriter(
            csvfile,
            fieldnames=fieldnames,
        )

        if not file_exists:
            writer.writeheader()

        writer.writerow({
            "world": scenario_spec["world"],
            "mission": mission_spec["name"],
            "robot": robot_spec["name"],

            "spawn_x": robot_spec["spawn"]["x"],
            "spawn_y": robot_spec["spawn"]["y"],
            "spawn_yaw": robot_spec["spawn"]["yaw"],

            "fault_domain": fault_spec["domain"],
            "fault_type": fault_spec["type"],
            "delay_ms": fault_spec["parameters"]["delay_ms"],

            "goal_x": behavior_spec["parameters"]["goal_x"],
            "goal_y": behavior_spec["parameters"]["goal_y"],

            "result": result["result"],
            "reason": result["reason"],
            "execution_time": execution_time,
        })


def run_test(test_number, test_case, result_file):
    scenario_spec = test_case["scenario"]
    mission_spec = test_case["mission"]
    fault_spec = test_case["faults"][0]

    robot_spec = scenario_spec["robots"][0]
    behavior_spec = mission_spec["behaviors"][0]

    world = scenario_spec["world"]

    robot_name = robot_spec["name"]
    spawn = robot_spec["spawn"]

    mission_name = mission_spec["name"]

    fault_type = fault_spec["type"]
    delay_ms = fault_spec["parameters"]["delay_ms"]

    goal_x = behavior_spec["parameters"]["goal_x"]
    goal_y = behavior_spec["parameters"]["goal_y"]

    print("\n================================")
    print(f"Test Case #{test_number}")
    print("================================")
    print(f"World       : {world}")
    print(f"Mission     : {mission_name}")
    print(f"Target Robot: {robot_name}")
    print(f"Fault Type  : {fault_type}")
    print(f"Delay Time  : {delay_ms} ms")
    print(f"Goal        : ({goal_x}, {goal_y})")
    print("================================\n")

    scenario = ScenarioManager()

    fault_injector = None
    behavior = None
    executor = None
    executor_thread = None

    try:
        scenario.start_world(world)

        scenario.spawn_robot(
            robot_name=robot_name,
            x=spawn["x"],
            y=spawn["y"],
            yaw=spawn["yaw"],
        )

        fault_injector = NetworkFaultInjector(
            robot_name=robot_name,
            delay_ms=delay_ms,
        )

        behavior = MoveToGoal(
            robot_name=robot_name,
            goal_x=goal_x,
            goal_y=goal_y,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(fault_injector)

        # Fault injector must process messages while
        # the mission is running.
        import threading

        executor_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
        )
        executor_thread.start()

        mission_completed, final_distance, execution_time = (
            behavior.execute()
        )

        oracle = BugOracle(tolerance=0.1)

        result = oracle.evaluate(
            mission_completed,
            final_distance,
        )

        print("\n================================")
        print(f"Test #{test_number} Result")
        print("================================")
        print(f"World      : {world}")
        print(f"Result     : {result['result']}")
        print(f"Reason     : {result['reason']}")
        print(f"Test Time  : {execution_time:.3f} s")
        print("================================")

        save_result(
            result_file,
            test_case,
            result,
            execution_time,
        )

        return result

    finally:
        if behavior is not None:
            behavior.stop_robot()
            behavior.destroy_node()

        if executor is not None:
            executor.shutdown()

        if executor_thread is not None:
            executor_thread.join(timeout=2)

        if fault_injector is not None:
            fault_injector.destroy_node()

        scenario.stop_simulation()

        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)

    result_file = "/workspace/fuzzing_results.csv"

    success_count = 0
    failure_count = 0

    generator = TestCaseGenerator()

    try:
        for i in range(1, TEST_COUNT + 1):
            test_case = generator.generate()

            try:
                result = run_test(
                    i,
                    test_case,
                    result_file,
                )

                if result["result"] == "SUCCESS":
                    success_count += 1
                else:
                    failure_count += 1

            except Exception as e:
                failure_count += 1

                print(
                    f"\n[Test #{i}] ERROR: {e}"
                )

        print("\n================================")
        print("Fuzzing Summary")
        print("================================")
        print(f"Total Tests : {TEST_COUNT}")
        print(f"SUCCESS     : {success_count}")
        print(f"FAILURE     : {failure_count}")
        print(f"Results     : {result_file}")
        print("================================")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()