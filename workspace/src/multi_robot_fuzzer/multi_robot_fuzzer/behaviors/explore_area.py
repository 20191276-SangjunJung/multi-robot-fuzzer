from rclpy.executors import SingleThreadedExecutor
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ExploreArea(Node):

    def __init__(
        self,
        robot_name,
        waypoints,
        tolerance=0.15,
        timeout=60.0,
    ):
        super().__init__("explore_area_node")

        self._local_executor = SingleThreadedExecutor(context=self.context)
        self.robot_name = robot_name
        self.waypoints = waypoints
        self.tolerance = tolerance
        self.timeout = timeout

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

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
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        self.current_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
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

    def move_to_waypoint(self, goal_x, goal_y, deadline):
        while rclpy.ok() and not getattr(self, "stop_requested", False):

            if time.time() >= deadline:
                self.stop_robot()
                return False

            rclpy.spin_once(self, executor=self._local_executor,
                timeout_sec=0.05,
            )

            if (
                self.current_x is None
                or self.current_y is None
                or self.current_yaw is None
            ):
                continue

            dx = goal_x - self.current_x
            dy = goal_y - self.current_y

            distance = math.sqrt(
                dx * dx
                + dy * dy
            )

            if distance <= self.tolerance:
                self.stop_robot()
                return True

            goal_angle = math.atan2(
                dy,
                dx,
            )

            angle_error = self.normalize_angle(
                goal_angle - self.current_yaw
            )

            msg = Twist()

            if abs(angle_error) > 0.1:
                msg.linear.x = 0.0
                msg.angular.z = 0.8 * angle_error
            else:
                msg.linear.x = 0.3
                msg.angular.z = 0.0

            self.cmd_pub.publish(msg)

        self.stop_robot()
        return False

    def execute(self):
        if not self.waypoints:
            self.get_logger().error(
                "No waypoints provided."
            )
            return False

        start_time = time.time()
        deadline = start_time + self.timeout

        self.get_logger().info(
            f"Starting exploration with "
            f"{len(self.waypoints)} waypoints."
        )

        for index, waypoint in enumerate(
            self.waypoints
        ):
            if getattr(self, "stop_requested", False):
                self.stop_robot()
                return False
            goal_x = waypoint["x"]
            goal_y = waypoint["y"]

            self.get_logger().info(
                f"Moving to waypoint "
                f"{index + 1}/{len(self.waypoints)}: "
                f"({goal_x:.2f}, {goal_y:.2f})"
            )

            reached = self.move_to_waypoint(
                goal_x,
                goal_y,
                deadline,
            )

            if not reached:
                self.stop_robot()

                self.get_logger().warn(
                    f"Failed to reach waypoint "
                    f"{index + 1}."
                )

                return False

            self.get_logger().info(
                f"Reached waypoint "
                f"{index + 1}/{len(self.waypoints)}."
            )

        self.stop_robot()

        execution_time = (
            time.time() - start_time
        )

        self.get_logger().info(
            f"Exploration completed in "
            f"{execution_time:.2f} seconds."
        )

        return True