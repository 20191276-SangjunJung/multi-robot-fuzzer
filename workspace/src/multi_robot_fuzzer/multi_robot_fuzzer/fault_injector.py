import argparse
import copy
import json
import math
import os
import signal
import time
import random
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

def save_fault_feedback(filename, data):
    if filename is None:
        return

    directory = os.path.dirname(filename)
    if directory:
        os.makedirs(directory, exist_ok=True)

    temp_file = f"{filename}.tmp"

    with open(temp_file, "w", encoding="utf-8") as file:
        json.dump(
            data,
            file,
            ensure_ascii=False,
            allow_nan=False,
        )

    os.replace(temp_file, filename)


def serialize_distance(value):
    if math.isnan(value):
        return "nan"
    if value == float("inf"):
        return "inf"
    if value == float("-inf"):
        return "-inf"
    return float(value)

class NetworkFaultInjector(Node):

    def __init__(
        self,
        robot_name="tb0_1",
        delay_ms=300,
        input_topic=None,
    ):
        super().__init__("network_fault_injector")

        self.robot_name = robot_name
        self.delay_sec = delay_ms / 1000.0
        self.queue = deque()

        self.subscription = self.create_subscription(
            Twist,
            input_topic or f"/{robot_name}/cmd_vel_raw",
            self.cmd_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Twist,
            f"/{robot_name}/cmd_vel",
            10,
        )

        self.timer = self.create_timer(
            0.01,
            self.process_queue,
        )

        print(
            f"Network Fault Injector started: "
            f"robot={robot_name}, delay={delay_ms} ms"
        )

    def cmd_callback(self, msg):
        release_time = time.time() + self.delay_sec
        self.queue.append((release_time, msg))

    def process_queue(self):
        now = time.time()

        while self.queue and self.queue[0][0] <= now:
            _, msg = self.queue.popleft()
            self.publisher.publish(msg)



class IMUFaultInjector(Node):

    def __init__(
        self,
        robot_name="tb0_1",
        imu_target="accelerometer",
        amplitude=0.0,
        frequency=0.0,
        phase=0.0,
        jitter_std_us=10.0,
        start_time=0.0,
        duration=None,
        seed=0,
        feedback_file=None,
    ):
        super().__init__(f"imu_fault_injector_{robot_name}")

        if imu_target not in ("accelerometer", "gyroscope"):
            raise ValueError(f"Invalid IMU target: {imu_target}")

        self.imu_target = imu_target        

        values = (amplitude, frequency, jitter_std_us, start_time)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("Acoustic parameters must be finite and non-negative")

        if not math.isfinite(phase):
            raise ValueError("Phase must be finite")

        if duration is not None and (
            not math.isfinite(duration) or duration <= 0
        ):
            raise ValueError("Duration must be positive")

        self.robot_name = robot_name
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = math.radians(phase)
        self.jitter_std = jitter_std_us * 1e-6
        self.start_time = start_time
        self.duration = duration

        self.rng = random.Random(seed)
        self.time_origin = None

        self.feedback_file = feedback_file
        self.feedback = {
            "sensor": "imu",
            "robot": robot_name,
            "attack_type": "acoustic",
            "target": imu_target,
            "configured_start_time": start_time,
            "configured_duration": duration,
            "configured_amplitude": amplitude,
            "configured_frequency": frequency,
            "configured_phase_deg": phase,
            "configured_jitter_std_us": jitter_std_us,
            "seed": seed,
            "mission_start_received": False,
            "attack_triggered": False,
            "message_count": 0,
            "attacked_message_count": 0,
            "nonzero_error_count": 0,
            "first_attack_elapsed": None,
            "last_attack_elapsed": None,
            "first_nonzero_elapsed": None,
            "last_nonzero_elapsed": None,
            "max_abs_error": 0.0,
            "sum_squared_error": 0.0,
            "sum_error": 0.0,
            "min_error": None,
            "max_error": None,
            "feedback_complete": False,
        }

        self.feedback_timer = self.create_timer(
            0.5,
            self.flush_feedback,
        )

        self.flush_feedback()

        start_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.start_subscription = self.create_subscription(
            Float64,
            f"/{robot_name}/mission_start",
            self.start_callback,
            start_qos,
        )

        self.subscription = self.create_subscription(
            Imu,
            f"/{robot_name}/imu",
            self.imu_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Imu,
            f"/{robot_name}/imu_faulty",
            10,
        )

        self.get_logger().info(
            f"Acoustic {imu_target} Injector started: "
            f"robot={robot_name}, "
            f"amplitude={amplitude} "
            f"{'m/s^2' if imu_target == 'accelerometer' else 'rad/s'}, "
            f"frequency={frequency} Hz, "
            f"phase={phase} deg, "
            f"jitter_std={jitter_std_us} us, "
            f"start_time={start_time} s, "
            f"duration={duration}, "
            f"seed={seed}"
        )

    def start_callback(self, msg):
        self.time_origin = msg.data
        self.feedback["mission_start_received"] = True

    def flush_feedback(self):
        data = self.feedback.copy()

        count = data["attacked_message_count"]

        data["rms_error"] = (
            math.sqrt(data["sum_squared_error"] / count)
            if count > 0 else None
        )

        data["mean_error"] = (
            data["sum_error"] / count
            if count > 0 else None
        )

        save_fault_feedback(self.feedback_file, data)

    def imu_callback(self, msg):
        faulty_msg = copy.deepcopy(msg)

        if self.time_origin is None:
            self.publisher.publish(faulty_msg)
            self.feedback["message_count"] += 1
            return

        elapsed = time.monotonic() - self.time_origin

        attack_active = elapsed >= self.start_time

        if self.duration is not None:
            attack_active = (
                attack_active
                and elapsed < self.start_time + self.duration
            )

        error = 0.0

        if attack_active:
            jitter = self.rng.gauss(0.0, self.jitter_std)

            attack_t = elapsed - self.start_time + jitter

            error = self.amplitude * math.cos(
                2.0 * math.pi * self.frequency * attack_t
                + self.phase
            )

            if self.imu_target == "accelerometer":
                faulty_msg.linear_acceleration.x += error
                faulty_msg.linear_acceleration.y += error
                faulty_msg.linear_acceleration.z += error
            else:
                faulty_msg.angular_velocity.x += error
                faulty_msg.angular_velocity.y += error
                faulty_msg.angular_velocity.z += error

        self.publisher.publish(faulty_msg)

        self.feedback["message_count"] += 1

        if attack_active:
            feedback = self.feedback

            feedback["attack_triggered"] = True
            feedback["attacked_message_count"] += 1
            feedback["last_attack_elapsed"] = elapsed

            if feedback["first_attack_elapsed"] is None:
                feedback["first_attack_elapsed"] = elapsed

            feedback["sum_error"] += error
            feedback["sum_squared_error"] += error * error

            feedback["max_abs_error"] = max(
                feedback["max_abs_error"],
                abs(error),
            )

            if feedback["min_error"] is None:
                feedback["min_error"] = error
                feedback["max_error"] = error
            else:
                feedback["min_error"] = min(
                    feedback["min_error"], error
                )
                feedback["max_error"] = max(
                    feedback["max_error"], error
                )

            if error != 0.0:
                feedback["nonzero_error_count"] += 1
                feedback["last_nonzero_elapsed"] = elapsed

                if feedback["first_nonzero_elapsed"] is None:
                    feedback["first_nonzero_elapsed"] = elapsed


class LiDARFaultInjector(Node):

    SECTORS = {
        "global": [(-180, 180)],
        "front": [(-30, 30)],
        "left": [(30, 90)],
        "right": [(-90, -30)],
        "rear": [(-180, -90), (90, 180)],
    }

    def __init__(
        self,
        robot_name="tb0_1",
        attack_type="removal",
        sector="front",
        attack_angle=0.0,
        angular_width=0.0,
        ghost_distance=0.3,
        start_time=0.0,
        feedback_file=None,
    ):
        super().__init__(f"lidar_fault_injector_{robot_name}")

        if attack_type not in ("removal", "ghost"):
            raise ValueError("Invalid LiDAR attack type")

        if sector not in self.SECTORS:
            raise ValueError("Invalid LiDAR sector")

        if not math.isfinite(attack_angle):
            raise ValueError("Invalid attack angle")

        self.sector = sector

        if not self.in_sector(
            (attack_angle + 180.0) % 360.0 - 180.0
        ):
            raise ValueError("Attack angle is outside the selected sector")

        if angular_width not in (0.0, 1.0):
            raise ValueError("Angular width must be 0 or 1 beam")

        if not math.isfinite(start_time) or start_time < 0:
            raise ValueError("Invalid start time")

        if attack_type == "ghost":
            if not math.isfinite(ghost_distance) or not (
                0.12 <= ghost_distance <= 3.5
            ):
                raise ValueError("Ghost distance must be in [0.12, 3.5] m")

        self.robot_name = robot_name
        self.attack_type = attack_type
        self.sector = sector
        self.attack_angle = (
            (attack_angle + 180.0) % 360.0 - 180.0
        )
        self.angular_width = angular_width
        self.ghost_distance = ghost_distance
        self.start_time = start_time

        self.time_origin = None
        self.attack_applied = False

        self.feedback_file = feedback_file
        self.feedback = {
            "sensor": "lidar",
            "robot": robot_name,
            "attack_type": attack_type,
            "sector": sector,
            "configured_angle_deg": self.attack_angle,
            "configured_width_deg": angular_width,
            "configured_start_time": start_time,
            "configured_ghost_distance": (
                ghost_distance if attack_type == "ghost" else None
            ),
            "mission_start_received": False,
            "attack_triggered": False,
            "attack_applied": False,
            "measurement_changed": False,
            "actual_elapsed": None,
            "selected_beams": [],
            "modified_count": 0,
            "modified_beams": [],
            "feedback_complete": False,
        }

        save_fault_feedback(self.feedback_file, self.feedback)

        start_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.start_subscription = self.create_subscription(
            Float64,
            f"/{robot_name}/mission_start",
            self.start_callback,
            start_qos,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            f"/{robot_name}/scan",
            self.scan_callback,
            10,
        )

        self.publisher = self.create_publisher(
            LaserScan,
            f"/{robot_name}/scan_faulty",
            10,
        )

        self.get_logger().info(
            f"LiDAR Injector started: "
            f"robot={robot_name}, "
            f"type={attack_type}, "
            f"sector={sector}, "
            f"attack_angle={self.attack_angle} deg, "
            f"angular_width={angular_width} deg, "
            f"start_time={start_time} s"
            + (
                f", ghost_distance={ghost_distance} m"
                if attack_type == "ghost" else ""
            )
        )

    def in_sector(self, angle_deg):
        for start, end in self.SECTORS[self.sector]:
            if start <= angle_deg <= end:
                return True
        return False

    def start_callback(self, msg):
        self.time_origin = msg.data
        self.feedback["mission_start_received"] = True

    def scan_callback(self, msg):
        faulty_msg = copy.deepcopy(msg)

        if self.time_origin is None or self.attack_applied:
            self.publisher.publish(faulty_msg)
            return

        elapsed = time.monotonic() - self.time_origin

        if elapsed < self.start_time:
            self.publisher.publish(faulty_msg)
            return

        if not math.isfinite(msg.angle_increment) or msg.angle_increment <= 0:
            self.get_logger().error("Invalid LaserScan angle_increment")
            self.publisher.publish(faulty_msg)
            return

        modified = []

        beam_count = len(msg.ranges)

        if beam_count == 0:
            self.get_logger().error("Empty LaserScan ranges")
            self.publisher.publish(faulty_msg)
            return

        center_index = min(
            range(beam_count),
            key=lambda i: abs(
                (
                    (
                        math.degrees(
                            msg.angle_min + i * msg.angle_increment
                        )
                        - self.attack_angle
                        + 180.0
                    ) % 360.0
                ) - 180.0
            ),
        )

        selected_indices = {
            (center_index + offset) % beam_count
            for offset in range(
                -int(self.angular_width),
                int(self.angular_width) + 1
            )
        }

        for i, distance in enumerate(msg.ranges):
            if i not in selected_indices:
                continue

            angle_deg = (
                math.degrees(
                    msg.angle_min + i * msg.angle_increment
                ) + 180.0
            ) % 360.0 - 180.0

            if not self.in_sector(angle_deg):
                continue

            if self.attack_type == "ghost":
                if not (
                    msg.range_min <= self.ghost_distance <= msg.range_max
                ):
                    self.get_logger().error(
                        "Ghost distance outside LaserScan range"
                    )
                    break

                new_distance = self.ghost_distance

            else:
                if not (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    continue

                new_distance = float("inf")

            faulty_msg.ranges[i] = new_distance

            if distance != new_distance:
                modified.append(
                    (i, angle_deg, distance, new_distance)
                )

        self.publisher.publish(faulty_msg)

        self.attack_applied = True

        self.feedback.update({
            "attack_triggered": True,
            "attack_applied": True,
            "measurement_changed": len(modified) > 0,
            "actual_elapsed": elapsed,
            "selected_beams": sorted(selected_indices),
            "modified_count": len(modified),
            "modified_beams": [
                {
                    "index": index,
                    "angle_deg": angle,
                    "original_distance": serialize_distance(original),
                    "faulty_distance": serialize_distance(faulty),
                }
                for index, angle, original, faulty in modified
            ],
        })

        save_fault_feedback(self.feedback_file, self.feedback)

        self.get_logger().info(
            f"LiDAR attack triggered: "
            f"robot={self.robot_name}, "
            f"type={self.attack_type}, "
            f"elapsed={elapsed:.3f}s, "
            f"selected_beams={sorted(selected_indices)}, "
            f"modified_count={len(modified)}, "
            f"modified_beams={modified}"
        )

def handle_sigterm(signum, frame):
    raise KeyboardInterrupt

def main():
    signal.signal(signal.SIGTERM, handle_sigterm)

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sensor",
        choices=["imu", "lidar"],
        default="imu",
    )

    parser.add_argument(
        "--imu-target",
        choices=["accelerometer", "gyroscope"],
        default="accelerometer",
    )

    parser.add_argument("--robot", default="tb0_1")

    # IMU parameters
    parser.add_argument("--amplitude", type=float, default=0.0)
    parser.add_argument("--frequency", type=float, default=0.0)
    parser.add_argument("--phase", type=float, default=0.0)
    parser.add_argument("--jitter-std-us", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--seed", type=int, default=0)

    # LiDAR parameters
    parser.add_argument(
        "--attack-type",
        choices=["removal", "ghost"],
        default="removal",
    )
    parser.add_argument(
        "--sector",
        choices=["global", "front", "left", "right", "rear"],
        default="front",
    )
    parser.add_argument(
        "--attack-angle",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--angular-width",
        type=float,
        choices=[0.0, 1.0],
        default=0.0,
    )
    parser.add_argument(
        "--ghost-distance",
        type=float,
        default=0.3,
    )

    # Shared parameter
    parser.add_argument("--start-time", type=float, default=0.0)
    parser.add_argument("--feedback-file", type=str, default=None)

    args = parser.parse_args()

    rclpy.init()

    if args.sensor == "imu":
        node = IMUFaultInjector(
            robot_name=args.robot,
            amplitude=args.amplitude,
            frequency=args.frequency,
            phase=args.phase,
            jitter_std_us=args.jitter_std_us,
            start_time=args.start_time,
            duration=args.duration,
            seed=args.seed,
            imu_target=args.imu_target,
            feedback_file=args.feedback_file,
        )
    else:
        node = LiDARFaultInjector(
            robot_name=args.robot,
            attack_type=args.attack_type,
            sector=args.sector,
            attack_angle=args.attack_angle,
            angular_width=args.angular_width,
            ghost_distance=args.ghost_distance,
            start_time=args.start_time,
            feedback_file=args.feedback_file,
        )

    try:
        rclpy.spin(node)
    except (
        KeyboardInterrupt,
        ExternalShutdownException,
        _rclpy.RCLError,
    ):
        pass
    finally:
        if isinstance(node, IMUFaultInjector):
            node.feedback["feedback_complete"] = True
            node.flush_feedback()
        elif isinstance(node, LiDARFaultInjector):
            node.feedback["feedback_complete"] = True
            save_fault_feedback(
                node.feedback_file,
                node.feedback,
            )

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()