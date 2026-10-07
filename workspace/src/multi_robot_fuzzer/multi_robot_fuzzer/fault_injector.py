import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class NetworkFaultInjector(Node):

    def __init__(
        self,
        robot_name="tb0_1",
        delay_ms=300,
    ):
        super().__init__("network_fault_injector")

        self.robot_name = robot_name
        self.delay_sec = delay_ms / 1000.0

        self.queue = deque()

        self.subscription = self.create_subscription(
            Twist,
            f"/{robot_name}/cmd_vel_raw",
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

        self.queue.append(
            (release_time, msg)
        )

    def process_queue(self):
        now = time.time()

        while self.queue and self.queue[0][0] <= now:
            _, msg = self.queue.popleft()
            self.publisher.publish(msg)