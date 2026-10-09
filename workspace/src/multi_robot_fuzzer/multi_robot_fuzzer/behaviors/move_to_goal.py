from rclpy.executors import SingleThreadedExecutor
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveToGoal(Node):

    def __init__(
        self,
        robot_name="tb0_1",
        goal_x=1.0,
        goal_y=0.0,
        tolerance=0.1,
        timeout=15.0,
    ):
        super().__init__("move_to_goal_node")

        self._local_executor = SingleThreadedExecutor(context=self.context)
        self.robot_name = robot_name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.tolerance = tolerance
        self.timeout = timeout

        self.position = None
        self.yaw = None

        self.cmd_pub = self.create_publisher(
            Twist,
            f"/{robot_name}/cmd_vel_raw",
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{robot_name}/odom",
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        self.position = (
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

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def distance_to_goal(self):
        if self.position is None:
            return None

        x, y = self.position

        return math.sqrt(
            (self.goal_x - x) ** 2
            + (self.goal_y - y) ** 2
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
            f"MoveToGoal started: robot={self.robot_name}, "
            f"goal=({self.goal_x}, {self.goal_y})"
        )

        start_time = time.time()

        while rclpy.ok() and not getattr(self, "stop_requested", False):

            rclpy.spin_once(self, executor=self._local_executor, timeout_sec=0.05)

            if self.position is None or self.yaw is None:
                if time.time() - start_time > self.timeout:
                    self.stop_robot()
                    return False, None, time.time() - start_time

                continue

            distance = self.distance_to_goal()

            if distance <= self.tolerance:
                self.stop_robot()

                print(
                    f"Goal reached: position={self.position}, "
                    f"distance={distance:.3f} m"
                )

                return True, distance, time.time() - start_time

            if time.time() - start_time > self.timeout:
                self.stop_robot()

                print(
                    f"Mission timeout: distance={distance:.3f} m"
                )

                return False, distance, time.time() - start_time

            x, y = self.position

            goal_angle = math.atan2(
                self.goal_y - y,
                self.goal_x - x,
            )

            angle_error = self.normalize_angle(
                goal_angle - self.yaw
            )

            msg = Twist()

            # 먼저 목표 방향으로 회전
            if abs(angle_error) > 0.1:
                msg.linear.x = 0.0
                msg.angular.z = 0.8 * angle_error

            # 목표 방향을 보고 있으면 전진
            else:
                msg.linear.x = 0.3
                msg.angular.z = 0.0

            self.cmd_pub.publish(msg)

        self.stop_robot()
        return False, None, time.time() - start_time