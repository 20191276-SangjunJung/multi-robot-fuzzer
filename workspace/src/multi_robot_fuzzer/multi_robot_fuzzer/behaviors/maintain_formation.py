import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MaintainFormation(Node):

    def __init__(
        self,
        leader_name="tb0_0",
        follower_names=None,
        spacing=0.8,
        tolerance=0.15,
        timeout=15.0,
    ):
        super().__init__("maintain_formation_node")

        if follower_names is None:
            follower_names = ["tb0_1", "tb0_2"]

        self.leader_name = leader_name
        self.follower_names = follower_names
        self.spacing = spacing
        self.tolerance = tolerance
        self.timeout = timeout

        self.leader_position = None
        self.leader_yaw = None

        self.follower_positions = {}
        self.follower_yaws = {}

        self.cmd_publishers = {}
        self.odom_subscribers = []

        # 각 follower의 V-formation offset 생성
        self.formation_offsets = self.generate_formation_offsets()

        # Leader odom 구독
        self.leader_odom_sub = self.create_subscription(
            Odometry,
            f"/{leader_name}/odom",
            self.leader_odom_callback,
            10,
        )

        # 각 follower의 odom 구독 + cmd_vel publisher 생성
        for follower_name in self.follower_names:

            self.cmd_publishers[follower_name] = (
                self.create_publisher(
                    Twist,
                    f"/{follower_name}/cmd_vel_raw",
                    10,
                )
            )

            subscription = self.create_subscription(
                Odometry,
                f"/{follower_name}/odom",
                lambda msg, name=follower_name:
                    self.follower_odom_callback(msg, name),
                10,
            )

            self.odom_subscribers.append(subscription)

    def generate_formation_offsets(self):
        offsets = {}

        for i, follower_name in enumerate(
            self.follower_names,
            start=1,
        ):
            level = (i + 1) // 2

            dx = -self.spacing * level

            if i % 2 == 1:
                dy = self.spacing * level
            else:
                dy = -self.spacing * level

            offsets[follower_name] = (dx, dy)

        return offsets

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def leader_odom_callback(self, msg):
        self.leader_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        self.leader_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def follower_odom_callback(
        self,
        msg,
        follower_name,
    ):
        self.follower_positions[follower_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        self.follower_yaws[follower_name] = (
            self.quaternion_to_yaw(
                msg.pose.pose.orientation
            )
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def get_target_position(self, follower_name):
        if (
            self.leader_position is None
            or self.leader_yaw is None
        ):
            return None

        leader_x, leader_y = self.leader_position

        dx, dy = self.formation_offsets[
            follower_name
        ]

        # Leader의 yaw를 기준으로 offset 회전
        target_x = (
            leader_x
            + dx * math.cos(self.leader_yaw)
            - dy * math.sin(self.leader_yaw)
        )

        target_y = (
            leader_y
            + dx * math.sin(self.leader_yaw)
            + dy * math.cos(self.leader_yaw)
        )

        return target_x, target_y

    def distance_to_target(
        self,
        follower_name,
        target_x,
        target_y,
    ):
        follower_x, follower_y = (
            self.follower_positions[follower_name]
        )

        return math.sqrt(
            (target_x - follower_x) ** 2
            + (target_y - follower_y) ** 2
        )

    def stop_robot(self, follower_name):
        msg = Twist()

        self.cmd_publishers[
            follower_name
        ].publish(msg)

    def stop_all_robots(self):
        for follower_name in self.follower_names:
            self.stop_robot(follower_name)

    def all_odom_received(self):
        if (
            self.leader_position is None
            or self.leader_yaw is None
        ):
            return False

        for follower_name in self.follower_names:

            if (
                follower_name
                not in self.follower_positions
            ):
                return False

            if (
                follower_name
                not in self.follower_yaws
            ):
                return False

        return True

    def control_follower(self, follower_name):
        target = self.get_target_position(
            follower_name
        )

        if target is None:
            return None

        target_x, target_y = target

        distance = self.distance_to_target(
            follower_name,
            target_x,
            target_y,
        )

        # Formation 위치에 충분히 가까우면 정지
        if distance <= self.tolerance:
            self.stop_robot(follower_name)
            return distance

        follower_x, follower_y = (
            self.follower_positions[follower_name]
        )

        follower_yaw = self.follower_yaws[
            follower_name
        ]

        target_angle = math.atan2(
            target_y - follower_y,
            target_x - follower_x,
        )

        angle_error = self.normalize_angle(
            target_angle - follower_yaw
        )

        msg = Twist()

        # 목표 방향을 먼저 맞춘 후 전진
        if abs(angle_error) > 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.8 * angle_error

        else:
            msg.linear.x = 0.3
            msg.angular.z = 0.0

        self.cmd_publishers[
            follower_name
        ].publish(msg)

        return distance

    def execute(self):
        print(
            f"MaintainFormation started: "
            f"leader={self.leader_name}, "
            f"followers={self.follower_names}, "
            f"spacing={self.spacing}"
        )

        print(
            f"Formation offsets: "
            f"{self.formation_offsets}"
        )

        start_time = time.time()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            elapsed_time = (
                time.time() - start_time
            )

            # odom이 아직 전부 들어오지 않은 경우
            if not self.all_odom_received():

                if elapsed_time > self.timeout:
                    self.stop_all_robots()

                    return (
                        False,
                        None,
                        elapsed_time,
                    )

                continue

            max_error = 0.0

            # 모든 follower 제어
            for follower_name in self.follower_names:

                error = self.control_follower(
                    follower_name
                )

                if error is not None:
                    max_error = max(
                        max_error,
                        error,
                    )

            # 현재 formation이 만들어졌으면 성공
            if max_error <= self.tolerance:

                self.stop_all_robots()

                print(
                    f"Formation reached: "
                    f"max_error={max_error:.3f} m"
                )

                return (
                    True,
                    max_error,
                    elapsed_time,
                )

            if elapsed_time > self.timeout:

                self.stop_all_robots()

                print(
                    f"Formation timeout: "
                    f"max_error={max_error:.3f} m"
                )

                return (
                    False,
                    max_error,
                    elapsed_time,
                )

        self.stop_all_robots()

        return (
            False,
            None,
            time.time() - start_time,
        )