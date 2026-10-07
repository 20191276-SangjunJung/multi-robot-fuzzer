import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Follow(Node):

    def __init__(
        self,
        leader_name="tb0_0",
        follower_name="tb0_1",
        follow_distance=0.8,
        tolerance=0.1,
        timeout=15.0,
    ):
        super().__init__("follow_node")

        self.leader_name = leader_name
        self.follower_name = follower_name
        self.follow_distance = follow_distance
        self.tolerance = tolerance
        self.timeout = timeout

        self.leader_position = None
        self.follower_position = None
        self.follower_yaw = None

        self.cmd_pub = self.create_publisher(
            Twist,
            f"/{follower_name}/cmd_vel_raw",
            10,
        )

        self.leader_odom_sub = self.create_subscription(
            Odometry,
            f"/{leader_name}/odom",
            self.leader_odom_callback,
            10,
        )

        self.follower_odom_sub = self.create_subscription(
            Odometry,
            f"/{follower_name}/odom",
            self.follower_odom_callback,
            10,
        )

    def leader_odom_callback(self, msg):
        self.leader_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def follower_odom_callback(self, msg):
        self.follower_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        self.follower_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def distance_to_leader(self):
        if (
            self.leader_position is None
            or self.follower_position is None
        ):
            return None

        leader_x, leader_y = self.leader_position
        follower_x, follower_y = self.follower_position

        return math.sqrt(
            (leader_x - follower_x) ** 2
            + (leader_y - follower_y) ** 2
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def execute(self):
        print(
            f"Follow started: leader={self.leader_name}, "
            f"follower={self.follower_name}, "
            f"distance={self.follow_distance}"
        )

        start_time = time.time()

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.leader_position is None
                or self.follower_position is None
                or self.follower_yaw is None
            ):
                if time.time() - start_time > self.timeout:
                    self.stop_robot()
                    return False, None, time.time() - start_time

                continue

            distance = self.distance_to_leader()

            # 원하는 거리보다 충분히 가까우면 정지
            if distance <= self.follow_distance + self.tolerance:
                self.stop_robot()

            else:
                leader_x, leader_y = self.leader_position
                follower_x, follower_y = self.follower_position

                target_angle = math.atan2(
                    leader_y - follower_y,
                    leader_x - follower_x,
                )

                angle_error = self.normalize_angle(
                    target_angle - self.follower_yaw
                )

                msg = Twist()

                if abs(angle_error) > 0.1:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.8 * angle_error
                else:
                    msg.linear.x = 0.3
                    msg.angular.z = 0.0

                self.cmd_pub.publish(msg)

            if time.time() - start_time > self.timeout:
                self.stop_robot()

                final_distance = self.distance_to_leader()

                print(
                    f"Follow finished: "
                    f"distance={final_distance:.3f} m"
                )

                success = (
                    final_distance
                    <= self.follow_distance + self.tolerance
                )

                return (
                    success,
                    final_distance,
                    time.time() - start_time,
                )

        self.stop_robot()
        return False, None, time.time() - start_time