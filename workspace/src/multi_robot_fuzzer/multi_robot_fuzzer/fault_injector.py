import argparse
import copy
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Imu, LaserScan


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
        amplitude=0.0,
        frequency=0.0,
        start_time=0.0,
    ):
        super().__init__(f"imu_fault_injector_{robot_name}")

        if not all(
            math.isfinite(value) and value >= 0
            for value in (amplitude, frequency, start_time)
        ):
            raise ValueError(
                "Acoustic parameters must be finite and non-negative"
            )

        self.robot_name = robot_name
        self.amplitude = amplitude
        self.frequency = frequency
        self.start_time = start_time

        # Time origin: first received IMU sample.
        self.time_origin = None

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
            f"Acoustic IMU Injector started: "
            f"robot={robot_name}, "
            f"amplitude={amplitude} rad/s, "
            f"frequency={frequency} Hz, "
            f"start_time={start_time} s"
        )

    def imu_callback(self, msg):
        faulty_msg = copy.deepcopy(msg)

        now = time.monotonic()

        if self.time_origin is None:
            self.time_origin = now

        elapsed = now - self.time_origin

        if elapsed >= self.start_time:
            attack_t = elapsed - self.start_time

            error = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * attack_t
            )

            faulty_msg.angular_velocity.z += error

        self.publisher.publish(faulty_msg)

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
        ghost_distance=0.3,
        start_time=0.0,
    ):
        super().__init__(f"lidar_fault_injector_{robot_name}")

        if attack_type not in ("removal", "ghost"):
            raise ValueError("Invalid LiDAR attack type")

        if sector not in self.SECTORS:
            raise ValueError("Invalid LiDAR sector")

        if not math.isfinite(start_time) or start_time < 0:
            raise ValueError("Invalid start time")

        if not math.isfinite(ghost_distance) or ghost_distance <= 0:
            raise ValueError("Invalid ghost distance")

        self.robot_name = robot_name
        self.attack_type = attack_type
        self.sector = sector
        self.ghost_distance = ghost_distance
        self.start_time = start_time
        self.time_origin = None

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
            f"ghost_distance={ghost_distance}, "
            f"start_time={start_time}"
        )

    def in_sector(self, angle_deg):
        for start, end in self.SECTORS[self.sector]:
            if start <= angle_deg <= end:
                return True
        return False

    def scan_callback(self, msg):
        faulty_msg = copy.deepcopy(msg)

        now = time.monotonic()

        if self.time_origin is None:
            self.time_origin = now

        elapsed = now - self.time_origin

        if elapsed >= self.start_time:

            for i, distance in enumerate(faulty_msg.ranges):

                angle_deg = (
                    math.degrees(
                        msg.angle_min + i * msg.angle_increment
                    ) + 180
                ) % 360 - 180

                if not self.in_sector(angle_deg):
                    continue

                if self.attack_type == "removal":
                    if (
                        math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max
                    ):
                        faulty_msg.ranges[i] = float("inf")

                elif self.attack_type == "ghost":
                    if msg.range_min <= self.ghost_distance <= msg.range_max:
                        faulty_msg.ranges[i] = self.ghost_distance

        self.publisher.publish(faulty_msg)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--sensor",
        choices=["imu", "lidar"],
        default="imu",
    )

    parser.add_argument("--robot", default="tb0_1")

    # IMU parameters
    parser.add_argument("--amplitude", type=float, default=0.0)
    parser.add_argument("--frequency", type=float, default=0.0)

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
        "--ghost-distance",
        type=float,
        default=0.3,
    )

    # Shared parameter
    parser.add_argument("--start-time", type=float, default=0.0)

    args = parser.parse_args()

    rclpy.init()

    if args.sensor == "imu":
        node = IMUFaultInjector(
            robot_name=args.robot,
            amplitude=args.amplitude,
            frequency=args.frequency,
            start_time=args.start_time,
        )
    else:
        node = LiDARFaultInjector(
            robot_name=args.robot,
            attack_type=args.attack_type,
            sector=args.sector,
            ghost_distance=args.ghost_distance,
            start_time=args.start_time,
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()