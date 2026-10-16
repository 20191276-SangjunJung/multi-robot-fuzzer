"""Coverage-guided exploration using the existing MoveToGoal behavior.

Requires ROS2 Humble and the existing .move_to_goal.MoveToGoal class.
"""
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from .move_to_goal import MoveToGoal


class ExploreArea(Node):
    def __init__(self, robot_name, waypoints=None, bounds=None,
                 resolution=0.25, target_coverage=0.75, timeout=120.0,
                 goal_timeout=25.0, sensor_range=2.0, goal_tolerance=0.15,
                 mode="waypoint"):
        super().__init__(f"explore_area_{robot_name}")
        self.robot_name = robot_name
        self.mode = mode
        self.timeout = float(timeout)
        self.goal_timeout = float(goal_timeout)
        self.target_coverage = float(target_coverage)
        self.resolution = float(resolution)
        self.sensor_range = float(sensor_range)
        self.goal_tolerance = float(goal_tolerance)
        if not 0 < self.target_coverage <= 1 or self.resolution <= 0 or self.sensor_range <= 0:
            raise ValueError("Invalid coverage configuration")

        self.candidates = [(float(w['x']), float(w['y'])) for w in (waypoints or [])]
        if bounds is None:
            if not self.candidates:
                raise ValueError("Provide bounds or candidate waypoints")
            xs, ys = zip(*self.candidates)
            margin = self.sensor_range
            bounds = (min(xs)-margin, max(xs)+margin,
                      min(ys)-margin, max(ys)+margin)
        self.xmin, self.xmax, self.ymin, self.ymax = map(float, bounds)
        self.nx = math.ceil((self.xmax-self.xmin)/self.resolution)
        self.ny = math.ceil((self.ymax-self.ymin)/self.resolution)
        if self.nx <= 0 or self.ny <= 0:
            raise ValueError("Invalid exploration bounds")
        if self.mode == "coverage":
            spacing = 1.0

            self.candidates = [
                (
                    self.xmin + spacing / 2 + i * spacing,
                    self.ymin + spacing / 2 + j * spacing,
                )
                for i in range(int((self.xmax - self.xmin) / spacing))
                for j in range(int((self.ymax - self.ymin) / spacing))
            ]

        elif not self.candidates:
            raise ValueError("Waypoint mode requires waypoints")

        self.observed = set()
        self.blocked = set()
        self.failed_goals = set()
        self.position = None
        self.yaw = None
        self._lock = threading.Lock()
        self._executor = SingleThreadedExecutor(context=self.context)
        self.create_subscription(Odometry, f'/{robot_name}/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, f'/{robot_name}/scan', self._scan_cb, 10)

    def _cell(self, x, y):
        i = math.floor((x-self.xmin)/self.resolution)
        j = math.floor((y-self.ymin)/self.resolution)
        return (i,j) if 0 <= i < self.nx and 0 <= j < self.ny else None

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self._lock:
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.yaw = yaw

    def _scan_cb(self, msg):
        with self._lock:
            if self.position is None or self.yaw is None:
                return
            x, y = self.position
            yaw = self.yaw
            origin = self._cell(x,y)
            if origin is not None:
                self.observed.add(origin)
            step = max(1, len(msg.ranges)//180)
            for k in range(0,len(msg.ranges),step):
                r = msg.ranges[k]
                if math.isnan(r) or r < msg.range_min:
                    continue
                hit = math.isfinite(r) and r <= min(msg.range_max,self.sensor_range)
                reach = min(r,self.sensor_range) if math.isfinite(r) else self.sensor_range
                theta = yaw + msg.angle_min + k*msg.angle_increment
                distance = 0.0
                while distance < reach:
                    c = self._cell(x+distance*math.cos(theta),y+distance*math.sin(theta))
                    if c is not None:
                        self.observed.add(c)
                    distance += self.resolution/2
                if hit:
                    c = self._cell(x+reach*math.cos(theta),y+reach*math.sin(theta))
                    if c is not None:
                        self.observed.add(c)
                        self.blocked.add(c)

    def coverage(self):
        with self._lock:
            return len(self.observed)/(self.nx*self.ny)

    def _choose_goal(self):
        with self._lock:
            if self.position is None:
                return None
            x,y = self.position
            observed = set(self.observed)
            blocked = set(self.blocked)
        best, best_score = None, -float('inf')
        for idx,(gx,gy) in enumerate(self.candidates):
            cell = self._cell(gx,gy)
            if idx in self.failed_goals or cell is None or cell in blocked:
                continue
            distance = math.hypot(gx-x,gy-y)
            if distance < self.goal_tolerance:
                continue
            # Approximate gain using unknown cells in the sensor footprint.
            gain = 0
            r = math.ceil(self.sensor_range/self.resolution)
            for di in range(-r,r+1):
                for dj in range(-r,r+1):
                    c = (cell[0]+di,cell[1]+dj)
                    if (0 <= c[0] < self.nx and 0 <= c[1] < self.ny
                        and c not in observed and c not in blocked
                        and (di*di+dj*dj)*self.resolution**2 <= self.sensor_range**2):
                        gain += 1
            score = gain - 2.0*distance
            if score > best_score and gain > 0:
                best, best_score = (idx,gx,gy),score
        return best

    def _execute_waypoints(self):
        """Preserve the original waypoint semantics for Search missions."""
        started = time.monotonic()
        for gx, gy in self.candidates:
            if getattr(self, 'stop_requested', False):
                return False, None, time.monotonic()-started
            remaining = self.timeout-(time.monotonic()-started)
            if remaining <= 0:
                return False, None, time.monotonic()-started
            mover = MoveToGoal(robot_name=self.robot_name, goal_x=gx, goal_y=gy,
                               tolerance=self.goal_tolerance,
                               timeout=min(self.goal_timeout, remaining))
            try:
                ok, _, _ = mover.execute()
            finally:
                mover.stop_robot()
                mover._local_executor.shutdown()
                mover.destroy_node()
            if not ok:
                return False, None, time.monotonic()-started
        return True, None, time.monotonic()-started

    def execute(self):
        if self.mode == "waypoint":
            return self._execute_waypoints()
        if self.mode != "coverage":
            raise ValueError(f"Unknown exploration mode: {self.mode}")
        started = time.monotonic()
        deadline = started+self.timeout
        stop_spin = threading.Event()

        def spin():
            while rclpy.ok() and not stop_spin.is_set():
                rclpy.spin_once(self, executor=self._executor, timeout_sec=0.05)

        thread = threading.Thread(target=spin,daemon=True)
        thread.start()
        mover = None
        try:
            while rclpy.ok() and not getattr(self,'stop_requested',False):
                cov = self.coverage()
                if cov >= self.target_coverage:
                    self.get_logger().info(f'Coverage target reached: {cov:.3f}')
                    return True,cov,time.monotonic()-started
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return False,cov,time.monotonic()-started
                # Wait until the robot's initial position is received.
                with self._lock:
                    position_ready = self.position is not None

                if not position_ready:
                    time.sleep(min(0.1, remaining))
                    continue

                goal = self._choose_goal()

                if goal is None:
                    self.get_logger().warn(
                        f"No exploration goal available: "
                        f"coverage={cov:.3f}, "
                        f"candidates={len(self.candidates)}, "
                        f"failed_goals={len(self.failed_goals)}"
                    )
                    return False, self.coverage(), time.monotonic()-started
                idx,gx,gy = goal
                self.get_logger().info(f'Exploring goal ({gx:.2f}, {gy:.2f}), coverage={cov:.3f}')
                mover = MoveToGoal(robot_name=self.robot_name,goal_x=gx,goal_y=gy,
                                   tolerance=self.goal_tolerance,
                                   timeout=min(self.goal_timeout,remaining))
                # A mission-level stop needs to propagate to the active movement.
                def propagate_stop():
                    while not getattr(mover,'_finished',False):
                        if getattr(self,'stop_requested',False):
                            mover.stop_requested = True
                            return
                        time.sleep(0.05)
                watcher = threading.Thread(target=propagate_stop,daemon=True)
                watcher.start()
                try:
                    ok,_,_ = mover.execute()
                finally:
                    mover._finished = True
                    watcher.join(timeout=0.2)
                    mover.stop_robot()
                    mover._local_executor.shutdown()
                    mover.destroy_node()
                    mover = None
                if not ok:
                    self.failed_goals.add(idx)
            return False,self.coverage(),time.monotonic()-started
        finally:
            if mover is not None:
                mover.stop_requested = True
            stop_spin.set()
            thread.join(timeout=1.0)
            self._executor.shutdown()
