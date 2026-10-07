import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AvoidObstacle(Node):

    def __init__(
        self,
        robot_name="tb0_1",
        safe_distance=0.5,
        turn_speed=0.6,
    ):
        super().__init__("avoid_obstacle_node")

        self.robot_name = robot_name
        self.safe_distance = safe_distance
        self.turn_speed = turn_speed

        self.scan_data = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            f"/{robot_name}/scan",
            self.scan_callback,
            10,
        )

    def scan_callback(self, msg):
        self.scan_data = msg

    def get_distance_in_range(
        self,
        start_angle,
        end_angle,
    ):
        if self.scan_data is None:
            return float("inf")

        ranges = self.scan_data.ranges

        valid_distances = []

        for i, distance in enumerate(ranges):

            angle = (
                self.scan_data.angle_min
                + i * self.scan_data.angle_increment
            )

            if start_angle <= angle <= end_angle:

                if (
                    math.isfinite(distance)
                    and distance >= self.scan_data.range_min
                    and distance <= self.scan_data.range_max
                ):
                    valid_distances.append(distance)

        if not valid_distances:
            return float("inf")

        return min(valid_distances)

    def get_obstacle_distances(self):
        # 전방 약 ±30도
        front_distance = self.get_distance_in_range(
            math.radians(-30),
            math.radians(30),
        )

        # 왼쪽 약 30~90도
        left_distance = self.get_distance_in_range(
            math.radians(30),
            math.radians(90),
        )

        # 오른쪽 약 -90~-30도
        right_distance = self.get_distance_in_range(
            math.radians(-90),
            math.radians(-30),
        )

        return (
            front_distance,
            left_distance,
            right_distance,
        )

    def obstacle_detected(self):
        front_distance, _, _ = (
            self.get_obstacle_distances()
        )

        return front_distance <= self.safe_distance

    def get_avoidance_command(self):
        if self.scan_data is None:
            return None

        (
            front_distance,
            left_distance,
            right_distance,
        ) = self.get_obstacle_distances()

        # 전방이 안전하면 회피할 필요 없음
        if front_distance > self.safe_distance:
            return None

        msg = Twist()

        # 장애물이 있으면 전진하지 않음
        msg.linear.x = 0.0

        # 더 넓은 방향으로 회전
        if left_distance >= right_distance:
            msg.angular.z = self.turn_speed
        else:
            msg.angular.z = -self.turn_speed
        return msg