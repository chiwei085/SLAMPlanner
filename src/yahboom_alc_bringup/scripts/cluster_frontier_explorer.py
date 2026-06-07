#!/usr/bin/env python3
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


Cell = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass
class FrontierCandidate:
    cells: List[Cell]
    goal_cell: Cell
    goal_xy: WorldPoint
    path_length: float
    information: int
    reachable: bool
    score: float


class ClusterFrontierExplorer(Node):
    def __init__(self):
        super().__init__("cluster_frontier_explorer")
        self.declare_parameter("enabled", True)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("cycle_period_sec", 3.0)
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_clearance_cells", 2)
        self.declare_parameter("path_clearance_cells", 2)
        self.declare_parameter("min_frontier_distance", 0.45)
        self.declare_parameter("max_frontier_distance", 2.5)
        self.declare_parameter("frontier_goal_search_radius", 0.6)
        self.declare_parameter("robot_start_search_radius", 2.0)
        self.declare_parameter("information_radius", 1.5)
        self.declare_parameter("path_weight", 1.0)
        self.declare_parameter("information_weight", 0.02)
        self.declare_parameter("frontier_size_weight", 0.02)
        self.declare_parameter("unreachable_goal_penalty", 3.0)
        self.declare_parameter("allow_unreachable_goal", True)
        self.declare_parameter("goal_update_distance", 0.5)
        self.declare_parameter("goal_publish_period", 3.0)
        self.declare_parameter("marker_topic", "/frontier_markers")
        self.declare_parameter("goal_marker_topic", "/frontier_goal_marker")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.min_clearance_cells = int(self.get_parameter("min_clearance_cells").value)
        self.path_clearance_cells = int(self.get_parameter("path_clearance_cells").value)
        self.min_frontier_distance = float(
            self.get_parameter("min_frontier_distance").value
        )
        self.max_frontier_distance = float(
            self.get_parameter("max_frontier_distance").value
        )
        self.frontier_goal_search_radius = float(
            self.get_parameter("frontier_goal_search_radius").value
        )
        self.robot_start_search_radius = float(
            self.get_parameter("robot_start_search_radius").value
        )
        self.information_radius = float(self.get_parameter("information_radius").value)
        self.path_weight = float(self.get_parameter("path_weight").value)
        self.information_weight = float(self.get_parameter("information_weight").value)
        self.frontier_size_weight = float(
            self.get_parameter("frontier_size_weight").value
        )
        self.unreachable_goal_penalty = float(
            self.get_parameter("unreachable_goal_penalty").value
        )
        self.allow_unreachable_goal = bool(
            self.get_parameter("allow_unreachable_goal").value
        )
        self.goal_update_distance = float(self.get_parameter("goal_update_distance").value)
        self.goal_publish_period = float(
            self.get_parameter("goal_publish_period").value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_active = False
        self.last_goal: Optional[WorldPoint] = None
        self.last_goal_yaw = 0.0
        self.last_goal_time = self.get_clock().now()
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
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, map_qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), marker_qos
        )
        self.goal_marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("goal_marker_topic").value), marker_qos
        )
        self.timer = self.create_timer(
            float(self.get_parameter("cycle_period_sec").value), self.on_timer
        )

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_timer(self):
        if not self.enabled:
            return
        if self.map_msg is None:
            self.log_waiting("Waiting for /map")
            return
        if self.goal_active:
            return
        if not self.nav_client.server_is_ready():
            self.log_waiting("Waiting for /navigate_to_pose action server")
            return

        robot_xy = self.lookup_robot_pose()
        if robot_xy is None:
            return

        candidates, counts = self.find_candidates(robot_xy)
        self.publish_frontier_markers(candidates)
        if not candidates:
            self.clear_goal_marker()
            self.log_waiting(
                "No frontier goal candidate found "
                f"(free={counts['free']}, unknown={counts['unknown']}, "
                f"occupied={counts['occupied']}, frontier_cells={counts['frontier']}, "
                f"clusters={counts['clusters']})"
            )
            return

        best = min(candidates, key=lambda candidate: candidate.score)
        if not self.should_publish_goal(best.goal_xy):
            return
        self.send_goal(robot_xy, best)

    def log_waiting(self, message: str):
        self.wait_log_count += 1
        if self.wait_log_count == 1 or self.wait_log_count % 6 == 0:
            self.get_logger().info(message)

    def lookup_robot_pose(self) -> Optional[WorldPoint]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for robot pose TF: {exc}")
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    def find_candidates(
        self, robot_xy: WorldPoint
    ) -> Tuple[List[FrontierCandidate], Dict[str, int]]:
        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        counts = {"free": 0, "unknown": 0, "occupied": 0, "frontier": 0, "clusters": 0}
        if width == 0 or height == 0:
            return [], counts

        for value in data:
            if value == -1:
                counts["unknown"] += 1
            elif self.is_free_value(value):
                counts["free"] += 1
            else:
                counts["occupied"] += 1

        frontier_cells = self.find_frontier_cells(width, height, data)
        counts["frontier"] = len(frontier_cells)
        clusters = self.cluster_frontiers(frontier_cells, width)
        clusters = [cluster for cluster in clusters if len(cluster) >= self.min_frontier_size]
        counts["clusters"] = len(clusters)
        if not clusters:
            return [], counts

        blocked = self.make_blocked_grid(width, height, data, self.path_clearance_cells)
        robot_cell = self.world_to_map(robot_xy[0], robot_xy[1])
        if robot_cell is None:
            return [], counts
        start_cell = self.nearest_free_cell(
            robot_cell, blocked, width, height, self.robot_start_search_radius
        )
        if start_cell is None:
            return [], counts

        candidates = []
        for cluster in clusters:
            candidate = self.make_candidate(cluster, start_cell, blocked, width, height)
            if candidate is not None:
                dist = math.hypot(
                    candidate.goal_xy[0] - robot_xy[0], candidate.goal_xy[1] - robot_xy[1]
                )
                if dist < self.min_frontier_distance:
                    continue
                if self.max_frontier_distance > 0.0 and dist > self.max_frontier_distance:
                    continue
                candidates.append(candidate)
        return candidates, counts

    def find_frontier_cells(self, width: int, height: int, data) -> List[Cell]:
        cells = []
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                index = row + x
                if not self.is_free_value(data[index]):
                    continue
                if self.has_unknown_neighbor(data, width, index):
                    cells.append((x, y))
        return cells

    def cluster_frontiers(self, frontier_cells: List[Cell], width: int) -> List[List[Cell]]:
        frontier_set: Set[Cell] = set(frontier_cells)
        visited: Set[Cell] = set()
        clusters = []
        for seed in frontier_cells:
            if seed in visited:
                continue
            cluster = []
            queue = deque([seed])
            visited.add(seed)
            while queue:
                cell = queue.popleft()
                cluster.append(cell)
                for neighbor in self.neighbor_cells(cell[0], cell[1], width):
                    if neighbor in frontier_set and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            clusters.append(cluster)
        return clusters

    def make_candidate(
        self, cluster: List[Cell], start_cell: Cell, blocked: List[bool], width: int, height: int
    ) -> Optional[FrontierCandidate]:
        goal_options = self.goal_options_for_cluster(cluster, blocked, width, height)
        if not goal_options:
            return None

        best: Optional[FrontierCandidate] = None
        for goal_cell in goal_options:
            path_length = self.grid_path_length(start_cell, goal_cell, blocked, width, height)
            reachable = path_length is not None
            if not reachable and not self.allow_unreachable_goal:
                continue
            if path_length is None:
                path_length = self.euclidean_cells(start_cell, goal_cell) * self.map_resolution()
            goal_xy = self.map_to_world(goal_cell[0], goal_cell[1])
            information = self.count_unknown_near(goal_cell)
            score = (
                self.path_weight * path_length
                - self.information_weight * information
                - self.frontier_size_weight * len(cluster)
            )
            if not reachable:
                score += self.unreachable_goal_penalty
            candidate = FrontierCandidate(
                cells=cluster,
                goal_cell=goal_cell,
                goal_xy=goal_xy,
                path_length=path_length,
                information=information,
                reachable=reachable,
                score=score,
            )
            if best is None or candidate.score < best.score:
                best = candidate
        return best

    def goal_options_for_cluster(
        self, cluster: List[Cell], blocked: List[bool], width: int, height: int
    ) -> List[Cell]:
        radius_cells = max(1, int(self.frontier_goal_search_radius / self.map_resolution()))
        options: Set[Cell] = set()
        for fx, fy in cluster:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    x = fx + dx
                    y = fy + dy
                    if not self.in_bounds(x, y, width, height):
                        continue
                    if math.hypot(dx, dy) > radius_cells:
                        continue
                    if blocked[self.index(x, y, width)]:
                        continue
                    if self.has_clearance(x, y, width, height, self.min_clearance_cells):
                        options.add((x, y))
        return list(options)

    def make_blocked_grid(self, width: int, height: int, data, clearance: int) -> List[bool]:
        blocked = [not self.is_free_value(value) for value in data]
        if clearance <= 0:
            return blocked
        inflated = blocked[:]
        for y in range(height):
            for x in range(width):
                if not blocked[self.index(x, y, width)]:
                    continue
                for dy in range(-clearance, clearance + 1):
                    for dx in range(-clearance, clearance + 1):
                        nx = x + dx
                        ny = y + dy
                        if self.in_bounds(nx, ny, width, height):
                            inflated[self.index(nx, ny, width)] = True
        return inflated

    def nearest_free_cell(
        self, origin: Cell, blocked: List[bool], width: int, height: int, radius_m: float
    ) -> Optional[Cell]:
        max_radius = max(1, int(radius_m / self.map_resolution()))
        ox, oy = origin
        if self.in_bounds(ox, oy, width, height) and not blocked[self.index(ox, oy, width)]:
            return origin
        best = None
        best_dist = float("inf")
        for dy in range(-max_radius, max_radius + 1):
            for dx in range(-max_radius, max_radius + 1):
                x = ox + dx
                y = oy + dy
                if not self.in_bounds(x, y, width, height):
                    continue
                if blocked[self.index(x, y, width)]:
                    continue
                dist = math.hypot(dx, dy)
                if dist < best_dist:
                    best_dist = dist
                    best = (x, y)
        return best

    def grid_path_length(
        self, start: Cell, goal: Cell, blocked: List[bool], width: int, height: int
    ) -> Optional[float]:
        if start == goal:
            return 0.0
        queue = deque([start])
        distance = {start: 0}
        while queue:
            cell = queue.popleft()
            for neighbor in self.neighbor_cells(cell[0], cell[1], width):
                if not self.in_bounds(neighbor[0], neighbor[1], width, height):
                    continue
                if blocked[self.index(neighbor[0], neighbor[1], width)]:
                    continue
                if neighbor in distance:
                    continue
                distance[neighbor] = distance[cell] + 1
                if neighbor == goal:
                    return distance[neighbor] * self.map_resolution()
                queue.append(neighbor)
        return None

    def count_unknown_near(self, cell: Cell) -> int:
        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        radius_cells = max(1, int(self.information_radius / self.map_resolution()))
        count = 0
        cx, cy = cell
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = cx + dx
                y = cy + dy
                if not self.in_bounds(x, y, width, height):
                    continue
                if math.hypot(dx, dy) > radius_cells:
                    continue
                if data[self.index(x, y, width)] == -1:
                    count += 1
        return count

    def should_publish_goal(self, goal_xy: WorldPoint) -> bool:
        now = self.get_clock().now()
        elapsed = (now - self.last_goal_time).nanoseconds / 1e9
        if self.last_goal is None:
            return True
        moved = math.hypot(goal_xy[0] - self.last_goal[0], goal_xy[1] - self.last_goal[1])
        return moved >= self.goal_update_distance or elapsed >= self.goal_publish_period

    def send_goal(self, robot_xy: WorldPoint, candidate: FrontierCandidate):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = candidate.goal_xy[0]
        goal.pose.pose.position.y = candidate.goal_xy[1]
        yaw = math.atan2(candidate.goal_xy[1] - robot_xy[1], candidate.goal_xy[0] - robot_xy[0])
        goal.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.goal_active = True
        self.last_goal = candidate.goal_xy
        self.last_goal_yaw = yaw
        self.last_goal_time = self.get_clock().now()
        self.wait_log_count = 0
        self.publish_goal_marker(candidate.goal_xy, yaw)
        self.get_logger().info(
            "Sending cluster frontier goal "
            f"x={candidate.goal_xy[0]:.2f}, y={candidate.goal_xy[1]:.2f}, "
            f"frontier_cells={len(candidate.cells)}, path={candidate.path_length:.2f}m, "
            f"info={candidate.information}, score={candidate.score:.2f}, "
            f"reachable={candidate.reachable}"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Cluster frontier goal rejected")
            self.goal_active = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Cluster frontier goal finished with status {status}")
        self.goal_active = False

    def publish_frontier_markers(self, candidates: List[FrontierCandidate]):
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.global_frame
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "frontier_cells"
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        cells_marker = Marker()
        cells_marker.header = delete_marker.header
        cells_marker.ns = "frontier_cells"
        cells_marker.id = 1
        cells_marker.type = Marker.CUBE_LIST
        cells_marker.action = Marker.ADD
        cells_marker.scale.x = self.map_resolution()
        cells_marker.scale.y = self.map_resolution()
        cells_marker.scale.z = 0.02
        cells_marker.color.r = 1.0
        cells_marker.color.g = 0.55
        cells_marker.color.b = 0.05
        cells_marker.color.a = 0.85
        cells_marker.pose.orientation.w = 1.0

        for candidate in candidates:
            for x, y in candidate.cells:
                wx, wy = self.map_to_world(x, y)
                point = Point()
                point.x = wx
                point.y = wy
                point.z = 0.03
                cells_marker.points.append(point)
        markers.markers.append(cells_marker)
        self.marker_pub.publish(markers)

    def publish_goal_marker(self, goal_xy: WorldPoint, yaw: float):
        marker = self.make_goal_marker(goal_xy, yaw)
        self.goal_marker_pub.publish(marker)
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def make_goal_marker(self, goal_xy: WorldPoint, yaw: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontier_goal"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = goal_xy[0]
        marker.pose.position.y = goal_xy[1]
        marker.pose.position.z = 0.12
        marker.pose.orientation = self.yaw_to_quaternion(yaw)
        marker.scale.x = 0.35
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.05
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def clear_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontier_goal"
        marker.id = 1
        marker.action = Marker.DELETE
        self.goal_marker_pub.publish(marker)

    def is_free_value(self, value: int) -> bool:
        return 0 <= value < self.occupied_threshold

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

    def has_clearance(self, x: int, y: int, width: int, height: int, radius: int) -> bool:
        data = self.map_msg.data
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = x + dx
                ny = y + dy
                if not self.in_bounds(nx, ny, width, height):
                    return False
                if not self.is_free_value(data[self.index(nx, ny, width)]):
                    return False
        return True

    def neighbor_cells(self, x: int, y: int, width: int) -> Tuple[Cell, ...]:
        return (
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
            (x - 1, y - 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
            (x + 1, y + 1),
        )

    def index(self, x: int, y: int, width: int) -> int:
        return y * width + x

    def in_bounds(self, x: int, y: int, width: int, height: int) -> bool:
        return 0 <= x < width and 0 <= y < height

    def euclidean_cells(self, a: Cell, b: Cell) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def map_resolution(self) -> float:
        return float(self.map_msg.info.resolution)

    def map_to_world(self, mx: int, my: int) -> WorldPoint:
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def world_to_map(self, wx: float, wy: float) -> Optional[Cell]:
        info = self.map_msg.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if not self.in_bounds(mx, my, info.width, info.height):
            return None
        return mx, my

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main():
    rclpy.init()
    node = ClusterFrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
