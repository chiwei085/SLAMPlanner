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
    goal_yaw: float
    path: List[Cell]
    path_length: float
    information: int
    reachable: bool
    unknown_overlap_ratio: float
    wall_clearance: float
    score: float


@dataclass
class FootprintCheck:
    occupied: bool
    unknown: int
    total: int

    @property
    def unknown_ratio(self) -> float:
        if self.total <= 0:
            return 1.0
        return self.unknown / self.total


@dataclass
class FootprintGeometry:
    front: float
    rear: float
    half_width: float
    radius_cells: int
    resolution: float


@dataclass
class ObstacleNeighborhood:
    wall_clearance: float
    corner_penalty: float


@dataclass
class GoalAttemptRecord:
    xy: WorldPoint
    failures: int
    expires_at_sec: float


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
        self.declare_parameter("allow_unreachable_goal", False)
        self.declare_parameter("goal_update_distance", 0.5)
        self.declare_parameter("goal_publish_period", 3.0)
        self.declare_parameter("footprint_front_m", 0.16)
        self.declare_parameter("footprint_rear_m", 0.15)
        self.declare_parameter("footprint_half_width_m", 0.11)
        self.declare_parameter("footprint_padding_m", 0.03)
        self.declare_parameter("swept_step_m", 0.05)
        self.declare_parameter("unknown_overlap_max_ratio", 0.35)
        self.declare_parameter("unknown_overlap_weight", 4.0)
        self.declare_parameter("wall_clearance_m", 0.22)
        self.declare_parameter("corner_penalty_weight", 1.0)
        self.declare_parameter("corridor_progress_weight", 0.4)
        self.declare_parameter("goal_timeout_sec", 45.0)
        self.declare_parameter("stuck_progress_radius_m", 0.12)
        self.declare_parameter("stuck_progress_timeout_sec", 12.0)
        self.declare_parameter("failed_goal_blacklist_radius_m", 0.45)
        self.declare_parameter("failed_goal_blacklist_sec", 60.0)
        self.declare_parameter("failed_goal_max_retries", 1)
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
        self.footprint_front_m = float(self.get_parameter("footprint_front_m").value)
        self.footprint_rear_m = float(self.get_parameter("footprint_rear_m").value)
        self.footprint_half_width_m = float(
            self.get_parameter("footprint_half_width_m").value
        )
        self.footprint_padding_m = float(
            self.get_parameter("footprint_padding_m").value
        )
        self.swept_step_m = float(self.get_parameter("swept_step_m").value)
        self.unknown_overlap_max_ratio = float(
            self.get_parameter("unknown_overlap_max_ratio").value
        )
        self.unknown_overlap_weight = float(
            self.get_parameter("unknown_overlap_weight").value
        )
        self.wall_clearance_m = float(self.get_parameter("wall_clearance_m").value)
        self.corner_penalty_weight = float(
            self.get_parameter("corner_penalty_weight").value
        )
        self.corridor_progress_weight = float(
            self.get_parameter("corridor_progress_weight").value
        )
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.stuck_progress_radius_m = float(
            self.get_parameter("stuck_progress_radius_m").value
        )
        self.stuck_progress_timeout_sec = float(
            self.get_parameter("stuck_progress_timeout_sec").value
        )
        self.failed_goal_blacklist_radius_m = float(
            self.get_parameter("failed_goal_blacklist_radius_m").value
        )
        self.failed_goal_blacklist_sec = float(
            self.get_parameter("failed_goal_blacklist_sec").value
        )
        self.failed_goal_max_retries = int(
            self.get_parameter("failed_goal_max_retries").value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_active = False
        self.goal_handle = None
        self.active_goal: Optional[WorldPoint] = None
        self.active_goal_sent_time = self.get_clock().now()
        self.last_progress_pose: Optional[WorldPoint] = None
        self.last_progress_time = self.get_clock().now()
        self.last_goal: Optional[WorldPoint] = None
        self.last_goal_yaw = 0.0
        self.last_goal_time = self.get_clock().now()
        self.goal_attempts: List[GoalAttemptRecord] = []
        self.footprint_geometry_: Optional[FootprintGeometry] = None
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
        self.footprint_geometry_ = None

    def on_timer(self):
        if not self.enabled:
            return
        if self.map_msg is None:
            self.log_waiting("Waiting for /map")
            return
        if self.goal_active:
            robot_xy = self.lookup_robot_pose()
            if robot_xy is not None:
                self.check_active_goal_progress(robot_xy)
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

        self.prune_blacklist()
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
            candidate = self.make_candidate(
                cluster, start_cell, robot_xy, blocked, width, height
            )
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
        self,
        cluster: List[Cell],
        start_cell: Cell,
        robot_xy: WorldPoint,
        blocked: List[bool],
        width: int,
        height: int,
    ) -> Optional[FrontierCandidate]:
        goal_options = self.goal_options_for_cluster(cluster, blocked, width, height)
        if not goal_options:
            return None

        best: Optional[FrontierCandidate] = None
        cluster_axis = self.cluster_axis(cluster)
        start_xy = self.map_to_world_unchecked(start_cell[0], start_cell[1])
        for goal_cell in goal_options:
            goal_xy = self.map_to_world_unchecked(goal_cell[0], goal_cell[1])
            if self.is_blacklisted(goal_xy):
                continue

            path = self.grid_path(start_cell, goal_cell, blocked, width, height)
            reachable = path is not None
            if not reachable and not self.allow_unreachable_goal:
                continue
            if path is None:
                path = [start_cell, goal_cell]
            goal_yaw = self.path_goal_yaw(path, robot_xy, goal_xy)
            footprint = self.footprint_check_at_world(goal_xy[0], goal_xy[1], goal_yaw)
            if footprint.occupied:
                continue
            if footprint.unknown_ratio > self.unknown_overlap_max_ratio:
                continue
            swept_unknown_ratio = self.swept_path_unknown_ratio(path)
            if swept_unknown_ratio is None:
                continue

            path_length = self.path_length(path)
            information = self.count_unknown_near(goal_cell)
            obstacle_neighborhood = self.analyze_obstacle_neighborhood(
                goal_cell, width, height
            )
            corridor_progress = self.corridor_progress(
                start_xy, goal_xy, cluster_axis
            )
            unknown_ratio = max(footprint.unknown_ratio, swept_unknown_ratio)
            score = (
                self.path_weight * path_length
                - self.information_weight * information
                - self.frontier_size_weight * len(cluster)
                - self.corridor_progress_weight * corridor_progress
                + self.unknown_overlap_weight * unknown_ratio
                + self.corner_penalty_weight * obstacle_neighborhood.corner_penalty
            )
            if obstacle_neighborhood.wall_clearance < self.wall_clearance_m:
                score += self.corner_penalty_weight * (
                    self.wall_clearance_m - obstacle_neighborhood.wall_clearance
                )
            if not reachable:
                score += self.unreachable_goal_penalty
            candidate = FrontierCandidate(
                cells=cluster,
                goal_cell=goal_cell,
                goal_xy=goal_xy,
                goal_yaw=goal_yaw,
                path=path,
                path_length=path_length,
                information=information,
                reachable=reachable,
                unknown_overlap_ratio=unknown_ratio,
                wall_clearance=obstacle_neighborhood.wall_clearance,
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
                    if self.has_occupied_clearance(
                        x, y, width, height, self.min_clearance_cells
                    ):
                        options.add((x, y))
        return list(options)

    def make_blocked_grid(self, width: int, height: int, data, clearance: int) -> List[bool]:
        blocked = [self.is_occupied_value(value) for value in data]
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

    def grid_path(
        self, start: Cell, goal: Cell, blocked: List[bool], width: int, height: int
    ) -> Optional[List[Cell]]:
        if start == goal:
            return [start]
        queue = deque([start])
        parent: Dict[Cell, Optional[Cell]] = {start: None}
        while queue:
            cell = queue.popleft()
            for neighbor in self.neighbor_cells(cell[0], cell[1], width):
                if not self.in_bounds(neighbor[0], neighbor[1], width, height):
                    continue
                if blocked[self.index(neighbor[0], neighbor[1], width)]:
                    continue
                if neighbor in parent:
                    continue
                parent[neighbor] = cell
                if neighbor == goal:
                    return self.reconstruct_path(parent, goal)
                queue.append(neighbor)
        return None

    def reconstruct_path(
        self, parent: Dict[Cell, Optional[Cell]], goal: Cell
    ) -> List[Cell]:
        path = []
        cell: Optional[Cell] = goal
        while cell is not None:
            path.append(cell)
            cell = parent[cell]
        path.reverse()
        return path

    def path_length(self, path: List[Cell]) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for prev, current in zip(path, path[1:]):
            total += self.euclidean_cells(prev, current) * self.map_resolution()
        return total

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
        yaw = candidate.goal_yaw
        goal.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.goal_active = True
        self.active_goal = candidate.goal_xy
        self.active_goal_sent_time = self.get_clock().now()
        self.last_progress_pose = robot_xy
        self.last_progress_time = self.get_clock().now()
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
            f"unknown={candidate.unknown_overlap_ratio:.2f}, "
            f"clearance={candidate.wall_clearance:.2f}m, reachable={candidate.reachable}"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Cluster frontier goal rejected")
            self.blacklist_active_goal()
            self.clear_active_goal()
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Cluster frontier goal finished with status {status}")
        if status != 4:
            self.blacklist_active_goal()
        self.clear_active_goal()

    def check_active_goal_progress(self, robot_xy: WorldPoint):
        now = self.get_clock().now()
        goal_elapsed = (now - self.active_goal_sent_time).nanoseconds / 1e9
        if self.goal_timeout_sec > 0.0 and goal_elapsed >= self.goal_timeout_sec:
            self.get_logger().warn("Abandoning frontier goal after timeout")
            self.abandon_active_goal()
            return

        moved = math.hypot(
            robot_xy[0] - self.last_progress_pose[0],
            robot_xy[1] - self.last_progress_pose[1],
        )
        if moved >= self.stuck_progress_radius_m:
            self.last_progress_pose = robot_xy
            self.last_progress_time = now
            return

        stuck_elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        if (
            self.stuck_progress_timeout_sec > 0.0
            and stuck_elapsed >= self.stuck_progress_timeout_sec
        ):
            self.get_logger().warn("Abandoning frontier goal after no progress")
            self.abandon_active_goal()

    def abandon_active_goal(self):
        self.blacklist_active_goal()
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.clear_active_goal()

    def clear_active_goal(self):
        self.goal_active = False
        self.goal_handle = None
        self.active_goal = None
        self.last_progress_pose = None

    def blacklist_active_goal(self):
        if self.active_goal is None:
            return
        expires_at = self.now_sec() + self.failed_goal_blacklist_sec
        record = self.find_goal_attempt(self.active_goal)
        if record is None:
            self.goal_attempts.append(
                GoalAttemptRecord(self.active_goal, 1, expires_at)
            )
            return

        record.xy = self.active_goal
        record.failures += 1
        record.expires_at_sec = expires_at

    def prune_blacklist(self):
        now = self.now_sec()
        self.goal_attempts = [
            record for record in self.goal_attempts if record.expires_at_sec > now
        ]

    def is_blacklisted(self, goal_xy: WorldPoint) -> bool:
        record = self.find_goal_attempt(goal_xy)
        if record is None:
            return False
        return record.failures >= self.failed_goal_max_retries

    def find_goal_attempt(self, goal_xy: WorldPoint) -> Optional[GoalAttemptRecord]:
        for record in self.goal_attempts:
            if (
                math.hypot(goal_xy[0] - record.xy[0], goal_xy[1] - record.xy[1])
                <= self.failed_goal_blacklist_radius_m
            ):
                return record
        return None

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
        resolution = self.map_resolution()
        cells_marker.scale.x = resolution
        cells_marker.scale.y = resolution
        cells_marker.scale.z = 0.02
        cells_marker.color.r = 1.0
        cells_marker.color.g = 0.55
        cells_marker.color.b = 0.05
        cells_marker.color.a = 0.85
        cells_marker.pose.orientation.w = 1.0

        for candidate in candidates:
            for x, y in candidate.cells:
                wx, wy = self.map_to_world_unchecked(x, y)
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

    def is_occupied_value(self, value: int) -> bool:
        return value >= self.occupied_threshold

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

    def has_occupied_clearance(
        self, x: int, y: int, width: int, height: int, radius: int
    ) -> bool:
        data = self.map_msg.data
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = x + dx
                ny = y + dy
                if not self.in_bounds(nx, ny, width, height):
                    return False
                if self.is_occupied_value(data[self.index(nx, ny, width)]):
                    return False
        return True

    def path_goal_yaw(
        self, path: List[Cell], robot_xy: WorldPoint, goal_xy: WorldPoint
    ) -> float:
        if len(path) >= 2:
            prev_xy = self.map_to_world_unchecked(path[-2][0], path[-2][1])
            return math.atan2(goal_xy[1] - prev_xy[1], goal_xy[0] - prev_xy[0])
        return math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])

    def footprint_check_at_world(self, wx: float, wy: float, yaw: float) -> FootprintCheck:
        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        footprint = self.footprint_geometry()
        center = self.world_to_map(wx, wy)
        if center is None:
            return FootprintCheck(True, 0, 0)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        occupied = False
        unknown = 0
        total = 0
        cx, cy = center
        for dy in range(-footprint.radius_cells, footprint.radius_cells + 1):
            for dx in range(-footprint.radius_cells, footprint.radius_cells + 1):
                x = cx + dx
                y = cy + dy
                cell_xy = self.map_to_world_unchecked(x, y)
                rel_x = cell_xy[0] - wx
                rel_y = cell_xy[1] - wy
                body_x = cos_yaw * rel_x + sin_yaw * rel_y
                body_y = -sin_yaw * rel_x + cos_yaw * rel_y
                if body_x < -footprint.rear or body_x > footprint.front:
                    continue
                if abs(body_y) > footprint.half_width:
                    continue
                total += 1
                if not self.in_bounds(x, y, width, height):
                    occupied = True
                    continue
                value = msg.data[self.index(x, y, width)]
                if self.is_occupied_value(value):
                    occupied = True
                elif value == -1:
                    unknown += 1
        return FootprintCheck(occupied, unknown, total)

    def footprint_geometry(self) -> FootprintGeometry:
        resolution = self.map_resolution()
        if (
            self.footprint_geometry_ is not None
            and self.footprint_geometry_.resolution == resolution
        ):
            return self.footprint_geometry_

        front = self.footprint_front_m + self.footprint_padding_m
        rear = self.footprint_rear_m + self.footprint_padding_m
        half_width = self.footprint_half_width_m + self.footprint_padding_m
        radius_cells = max(
            1, int(math.ceil(max(front, rear, half_width) / resolution)) + 1
        )
        self.footprint_geometry_ = FootprintGeometry(
            front=front,
            rear=rear,
            half_width=half_width,
            radius_cells=radius_cells,
            resolution=resolution,
        )
        return self.footprint_geometry_

    def swept_path_unknown_ratio(self, path: List[Cell]) -> Optional[float]:
        if not path:
            return None
        if len(path) == 1:
            xy = self.map_to_world_unchecked(path[0][0], path[0][1])
            check = self.footprint_check_at_world(xy[0], xy[1], self.last_goal_yaw)
            if check.occupied or check.unknown_ratio > self.unknown_overlap_max_ratio:
                return None
            return check.unknown_ratio

        max_ratio = 0.0
        step = max(self.swept_step_m, self.map_resolution())
        for prev, current in zip(path, path[1:]):
            prev_xy = self.map_to_world_unchecked(prev[0], prev[1])
            current_xy = self.map_to_world_unchecked(current[0], current[1])
            dx = current_xy[0] - prev_xy[0]
            dy = current_xy[1] - prev_xy[1]
            yaw = math.atan2(dy, dx)
            distance = math.hypot(dx, dy)
            samples = max(1, int(math.ceil(distance / step)))
            for sample in range(samples + 1):
                ratio = sample / samples
                wx = prev_xy[0] + dx * ratio
                wy = prev_xy[1] + dy * ratio
                check = self.footprint_check_at_world(wx, wy, yaw)
                if check.occupied or check.unknown_ratio > self.unknown_overlap_max_ratio:
                    return None
                max_ratio = max(max_ratio, check.unknown_ratio)
        return max_ratio

    def analyze_obstacle_neighborhood(
        self, cell: Cell, width: int, height: int
    ) -> ObstacleNeighborhood:
        radius_cells = max(1, int(math.ceil(self.wall_clearance_m / self.map_resolution())))
        best = float("inf")
        occupied_quadrants: Set[Tuple[int, int]] = set()
        cx, cy = cell
        data = self.map_msg.data
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = cx + dx
                y = cy + dy
                if dx == 0 and dy == 0:
                    continue
                blocked = not self.in_bounds(x, y, width, height)
                if not blocked:
                    blocked = self.is_occupied_value(data[self.index(x, y, width)])
                if blocked:
                    best = min(best, math.hypot(dx, dy) * self.map_resolution())
                if blocked and (dx != 0 or dy != 0):
                    occupied_quadrants.add((1 if dx >= 0 else -1, 1 if dy >= 0 else -1))

        wall_clearance = self.wall_clearance_m if math.isinf(best) else best
        corner_penalty = max(0.0, float(len(occupied_quadrants) - 1))
        return ObstacleNeighborhood(wall_clearance, corner_penalty)

    def cluster_axis(self, cluster: List[Cell]) -> WorldPoint:
        if len(cluster) < 2:
            return (1.0, 0.0)
        mean_x = sum(cell[0] for cell in cluster) / len(cluster)
        mean_y = sum(cell[1] for cell in cluster) / len(cluster)
        cov_xx = 0.0
        cov_xy = 0.0
        cov_yy = 0.0
        for x, y in cluster:
            dx = x - mean_x
            dy = y - mean_y
            cov_xx += dx * dx
            cov_xy += dx * dy
            cov_yy += dy * dy
        if abs(cov_xy) < 1e-6 and cov_xx >= cov_yy:
            return (1.0, 0.0)
        if abs(cov_xy) < 1e-6:
            return (0.0, 1.0)
        theta = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
        return (math.cos(theta), math.sin(theta))

    def corridor_progress(
        self, start_xy: WorldPoint, goal_xy: WorldPoint, axis: WorldPoint
    ) -> float:
        dx = goal_xy[0] - start_xy[0]
        dy = goal_xy[1] - start_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0
        ux = dx / dist
        uy = dy / dist
        return dist * abs(ux * axis[0] + uy * axis[1])

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
        if not self.in_bounds(
            mx, my, self.map_msg.info.width, self.map_msg.info.height
        ):
            raise ValueError(f"map cell out of bounds: ({mx}, {my})")
        return self.map_to_world_unchecked(mx, my)

    def map_to_world_unchecked(self, mx: int, my: int) -> WorldPoint:
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

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

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
