
import argparse
import copy
import random
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
        fault_type="bias",
        accel_param=0.0,
        gyro_param=0.0,
    ):
        super().__init__("imu_fault_injector")

        if fault_type not in ("bias", "noise", "scale"):
            raise ValueError(f"Unknown fault type: {fault_type}")

        if fault_type == "noise" and (
            accel_param < 0 or gyro_param < 0
        ):
            raise ValueError(
                "Noise standard deviation must be non-negative"
            )

        self.robot_name = robot_name
        self.fault_type = fault_type
        self.accel_param = accel_param
        self.gyro_param = gyro_param

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
            f"IMU Fault Injector started: "
            f"robot={robot_name}, "
            f"fault={fault_type}, "
            f"accel={accel_param}, "
            f"gyro={gyro_param}"
        )

    def apply_fault(self, value, parameter):
        if self.fault_type == "bias":
            return value + parameter

        if self.fault_type == "noise":
            return value + random.gauss(0.0, parameter)

        if self.fault_type == "scale":
            return value * (1.0 + parameter)

        return value

    def imu_callback(self, msg):
        faulty_msg = copy.deepcopy(msg)

        for vector, parameter in (
            (faulty_msg.linear_acceleration, self.accel_param),
            (faulty_msg.angular_velocity, self.gyro_param),
        ):
            vector.x = self.apply_fault(vector.x, parameter)
            vector.y = self.apply_fault(vector.y, parameter)
            vector.z = self.apply_fault(vector.z, parameter)

        self.publisher.publish(faulty_msg)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--robot", default="tb0_1")
    parser.add_argument(
        "--fault",
        choices=["bias", "noise", "scale"],
        default="bias",
    )
    parser.add_argument("--accel", type=float, default=0.0)
    parser.add_argument("--gyro", type=float, default=0.0)

    args = parser.parse_args()

    rclpy.init()

    node = IMUFaultInjector(
        robot_name=args.robot,
        fault_type=args.fault,
        accel_param=args.accel,
        gyro_param=args.gyro,
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
