from rclpy.executors import SingleThreadedExecutor
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class SearchTarget(Node):

    def __init__(
        self,
        robot_name,
        target_x,
        target_y,
        detection_range=0.5,
        timeout=60.0,
    ):
        super().__init__("search_target_node")

        self._local_executor = SingleThreadedExecutor(context=self.context)
        self.robot_name = robot_name

        self.target_x = target_x
        self.target_y = target_y

        self.detection_range = detection_range
        self.timeout = timeout

        self.current_x = None
        self.current_y = None

        self.target_found = False

        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{robot_name}/odom",
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def get_target_distance(self):
        if (
            self.current_x is None
            or self.current_y is None
        ):
            return None

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        return math.sqrt(
            dx * dx
            + dy * dy
        )

    def is_target_detected(self):
        distance = self.get_target_distance()

        if distance is None:
            return False

        return distance <= self.detection_range

    def execute(self):
        start_time = time.time()

        self.get_logger().info(
            f"Searching for target at "
            f"({self.target_x:.2f}, "
            f"{self.target_y:.2f}), "
            f"detection range: "
            f"{self.detection_range:.2f} m"
        )

        while rclpy.ok() and not getattr(self, "stop_requested", False):

            if time.time() - start_time >= self.timeout:
                self.get_logger().warn(
                    "Target search timed out."
                )

                return False

            rclpy.spin_once(self, executor=self._local_executor,
                timeout_sec=0.05,
            )

            if self.is_target_detected():
                self.target_found = True

                distance = self.get_target_distance()

                self.get_logger().info(
                    f"Target found. "
                    f"Distance: {distance:.2f} m"
                )

                return True

        return False