import argparse
import copy
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


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


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--robot", default="tb0_1")
    parser.add_argument("--amplitude", type=float, default=0.0)
    parser.add_argument("--frequency", type=float, default=0.0)
    parser.add_argument("--start-time", type=float, default=0.0)

    args = parser.parse_args()

    rclpy.init()

    node = IMUFaultInjector(
        robot_name=args.robot,
        amplitude=args.amplitude,
        frequency=args.frequency,
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