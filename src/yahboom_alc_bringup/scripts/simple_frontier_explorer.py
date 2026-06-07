#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class SimpleFrontierExplorer(Node):
    def __init__(self):
        super().__init__("simple_frontier_explorer")
        self.declare_parameter("enabled", True)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("cycle_period_sec", 5.0)
        self.declare_parameter("min_goal_distance", 0.45)
        self.declare_parameter("max_goal_distance", 3.0)
        self.declare_parameter("frontier_stride", 3)
        self.declare_parameter("occupied_threshold", 50)

        self.enabled = self.get_parameter("enabled").value
        self.map_topic = self.get_parameter("map_topic").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.global_frame = self.get_parameter("global_frame").value
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_active = False
        self.last_goal: Optional[Tuple[float, float]] = None
        self.wait_log_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, map_qos
        )
        self.timer = self.create_timer(
            float(self.get_parameter("cycle_period_sec").value), self.on_timer
        )

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_timer(self):
        if not self.enabled or self.goal_active or self.map_msg is None:
            if self.map_msg is None:
                self.log_waiting("Waiting for /map")
            return
        if not self.nav_client.server_is_ready():
            self.log_waiting("Waiting for /navigate_to_pose action server")
            return

        robot_pose = self.lookup_robot_pose()
        if robot_pose is None:
            return

        goal_xy = self.select_frontier(robot_pose)
        if goal_xy is None:
            self.log_waiting(self.describe_frontiers(robot_pose))
            return

        self.send_goal(robot_pose, goal_xy)

    def log_waiting(self, message: str):
        self.wait_log_count += 1
        if self.wait_log_count == 1 or self.wait_log_count % 6 == 0:
            self.get_logger().info(message)

    def lookup_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for robot pose TF: {exc}")
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    def select_frontier(
        self, robot_pose: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        msg = self.map_msg
        info = msg.info
        width = info.width
        height = info.height
        if width == 0 or height == 0:
            return None

        best = None
        best_score = float("inf")
        data = msg.data
        for y in range(1, height - 1, self.frontier_stride):
            row = y * width
            for x in range(1, width - 1, self.frontier_stride):
                index = row + x
                if data[index] < 0 or data[index] >= self.occupied_threshold:
                    continue
                if not self.has_unknown_neighbor(data, width, index):
                    continue

                wx, wy = self.map_to_world(x, y)
                dist = math.hypot(wx - robot_pose[0], wy - robot_pose[1])
                if dist < self.min_goal_distance or dist > self.max_goal_distance:
                    continue
                if self.last_goal is not None:
                    if math.hypot(wx - self.last_goal[0], wy - self.last_goal[1]) < 0.35:
                        continue
                if dist < best_score:
                    best_score = dist
                    best = (wx, wy)

        return best

    def has_unknown_neighbor(self, data, width: int, index: int) -> bool:
        neighbors = (
            index - width,
            index + width,
            index - 1,
            index + 1,
            index - width - 1,
            index - width + 1,
            index + width - 1,
            index + width + 1,
        )
        return any(data[n] == -1 for n in neighbors)

    def describe_frontiers(self, robot_pose: Tuple[float, float]) -> str:
        msg = self.map_msg
        info = msg.info
        width = info.width
        height = info.height
        data = msg.data
        free = 0
        unknown = 0
        occupied = 0
        frontier = 0
        in_range = 0
        nearest = None

        for value in data:
            if value == -1:
                unknown += 1
            elif 0 <= value < self.occupied_threshold:
                free += 1
            else:
                occupied += 1

        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                index = row + x
                if data[index] < 0 or data[index] >= self.occupied_threshold:
                    continue
                if not self.has_unknown_neighbor(data, width, index):
                    continue
                frontier += 1
                wx, wy = self.map_to_world(x, y)
                dist = math.hypot(wx - robot_pose[0], wy - robot_pose[1])
                nearest = dist if nearest is None else min(nearest, dist)
                if self.min_goal_distance <= dist <= self.max_goal_distance:
                    in_range += 1

        nearest_text = "none" if nearest is None else f"{nearest:.2f}m"
        return (
            "No reachable frontier candidate found "
            f"(free={free}, unknown={unknown}, occupied={occupied}, "
            f"frontier={frontier}, in_range={in_range}, nearest={nearest_text})"
        )

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def send_goal(self, robot_pose: Tuple[float, float], goal_xy: Tuple[float, float]):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_xy[0]
        goal.pose.pose.position.y = goal_xy[1]
        yaw = math.atan2(goal_xy[1] - robot_pose[1], goal_xy[0] - robot_pose[0])
        goal.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.goal_active = True
        self.last_goal = goal_xy
        self.wait_log_count = 0
        self.get_logger().info(
            f"Sending frontier goal x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected")
            self.goal_active = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Frontier goal finished with status {status}")
        self.goal_active = False

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main():
    rclpy.init()
    node = SimpleFrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
