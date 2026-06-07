import sys
from pathlib import Path

from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from cluster_frontier_explorer import (
    ClusterFrontierExplorer,
    GoalAttemptRecord,
)


class FakeDuration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return FakeDuration(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self, seconds=0.0):
        self.seconds = seconds

    def now(self):
        return FakeTime(self.seconds)


class FakeLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def debug(self, _message):
        pass


class FakeGoalHandle:
    def __init__(self):
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


def build_grid(width, height, resolution=0.1, default=0):
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.orientation.w = 1.0
    grid.data = [default] * (width * height)
    return grid


def set_cell(grid, x, y, value):
    grid.data[y * grid.info.width + x] = value


def make_explorer(grid):
    explorer = ClusterFrontierExplorer.__new__(ClusterFrontierExplorer)
    explorer.map_msg = grid
    explorer.min_clearance_cells = 0
    explorer.path_clearance_cells = 0
    explorer.occupied_threshold = 50
    explorer.frontier_goal_search_radius = 0.01
    explorer.robot_start_search_radius = 1.0
    explorer.information_radius = 0.3
    explorer.path_weight = 1.0
    explorer.information_weight = 0.02
    explorer.frontier_size_weight = 0.02
    explorer.unreachable_goal_penalty = 3.0
    explorer.allow_unreachable_goal = False
    explorer.goal_update_distance = 0.5
    explorer.goal_publish_period = 3.0
    explorer.footprint_front_m = 0.16
    explorer.footprint_rear_m = 0.15
    explorer.footprint_half_width_m = 0.11
    explorer.footprint_padding_m = 0.03
    explorer.swept_step_m = 0.05
    explorer.unknown_overlap_max_ratio = 0.35
    explorer.unknown_overlap_weight = 4.0
    explorer.wall_clearance_m = 0.22
    explorer.corner_penalty_weight = 1.0
    explorer.corridor_progress_weight = 0.4
    explorer.goal_timeout_sec = 45.0
    explorer.stuck_progress_radius_m = 0.12
    explorer.stuck_progress_timeout_sec = 12.0
    explorer.failed_goal_blacklist_radius_m = 0.45
    explorer.failed_goal_blacklist_sec = 60.0
    explorer.failed_goal_max_retries = 1
    explorer.last_goal = None
    explorer.last_goal_yaw = 0.0
    explorer.goal_attempts = []
    explorer.footprint_geometry_ = None
    explorer.goal_active = False
    explorer.goal_handle = None
    explorer.active_goal = None
    explorer.active_goal_sent_time = FakeTime(0.0)
    explorer.last_progress_pose = None
    explorer.last_progress_time = FakeTime(0.0)
    explorer.get_clock = lambda: FakeClock(0.0)
    explorer.get_logger = lambda: FakeLogger()
    return explorer


def test_large_corridor_frontier_beats_near_small_corner_frontier():
    grid = build_grid(30, 12)
    explorer = make_explorer(grid)
    explorer.unknown_overlap_max_ratio = 1.0
    explorer.unknown_overlap_weight = 0.0
    explorer.wall_clearance_m = 0.1
    start = (2, 5)
    robot_xy = explorer.map_to_world(*start)

    small_corner = [(6, 5), (6, 6)]
    large_corridor = [(8 + i, 5) for i in range(12)]

    small = explorer.make_candidate(
        small_corner, start, robot_xy, [False] * len(grid.data), 30, 12
    )
    large = explorer.make_candidate(
        large_corridor, start, robot_xy, [False] * len(grid.data), 30, 12
    )

    assert small is not None
    assert large is not None
    assert large.score < small.score


def test_occupied_footprint_overlap_rejects_candidate():
    grid = build_grid(12, 12, 0.05)
    explorer = make_explorer(grid)
    set_cell(grid, 6, 5, 100)

    check = explorer.footprint_check_at_world(0.30, 0.30, 0.0)

    assert check.occupied


def test_limited_unknown_overlap_is_allowed_but_scores_worse_than_free():
    free_grid = build_grid(16, 10, 0.1)
    unknown_grid = build_grid(16, 10, 0.1)
    set_cell(unknown_grid, 6, 5, -1)
    start = (2, 5)
    cluster = [(8, 5), (9, 5), (10, 5), (11, 5)]

    free_explorer = make_explorer(free_grid)
    unknown_explorer = make_explorer(unknown_grid)
    free_explorer.unknown_overlap_max_ratio = 1.0
    unknown_explorer.unknown_overlap_max_ratio = 1.0

    free_candidate = free_explorer.make_candidate(
        cluster,
        start,
        free_explorer.map_to_world(*start),
        [False] * len(free_grid.data),
        16,
        10,
    )
    unknown_candidate = unknown_explorer.make_candidate(
        cluster,
        start,
        unknown_explorer.map_to_world(*start),
        [False] * len(unknown_grid.data),
        16,
        10,
    )

    assert free_candidate is not None
    assert unknown_candidate is not None
    assert unknown_candidate.unknown_overlap_ratio > 0.0
    assert unknown_candidate.score > free_candidate.score


def test_excessive_unknown_overlap_rejects_candidate():
    grid = build_grid(12, 12, 0.05)
    explorer = make_explorer(grid)
    explorer.unknown_overlap_max_ratio = 0.1
    for y in range(4, 9):
        for x in range(4, 9):
            set_cell(grid, x, y, -1)

    check = explorer.footprint_check_at_world(0.30, 0.30, 0.0)

    assert check.unknown_ratio > explorer.unknown_overlap_max_ratio
    assert explorer.swept_path_unknown_ratio([(2, 6), (6, 6)]) is None


def test_swept_footprint_rejects_path_that_clips_wall():
    grid = build_grid(12, 12, 0.05)
    explorer = make_explorer(grid)
    set_cell(grid, 6, 6, 100)

    assert explorer.swept_path_unknown_ratio([(2, 6), (9, 6)]) is None


def test_failed_goal_blacklist_prevents_immediate_reselection():
    grid = build_grid(20, 10, 0.1)
    explorer = make_explorer(grid)
    explorer.goal_attempts = [
        GoalAttemptRecord(explorer.map_to_world(8, 5), 1, 60.0)
    ]
    explorer.now_sec = lambda: 0.0

    candidate = explorer.make_candidate(
        [(8, 5), (9, 5), (10, 5)],
        (2, 5),
        explorer.map_to_world(2, 5),
        [False] * len(grid.data),
        20,
        10,
    )

    assert candidate is None


def test_goal_timeout_abandons_active_goal_and_cancels_handle():
    grid = build_grid(20, 10, 0.1)
    explorer = make_explorer(grid)
    handle = FakeGoalHandle()
    explorer.goal_active = True
    explorer.goal_handle = handle
    explorer.active_goal = explorer.map_to_world(8, 5)
    explorer.active_goal_sent_time = FakeTime(0.0)
    explorer.last_progress_pose = explorer.map_to_world(2, 5)
    explorer.last_progress_time = FakeTime(0.0)
    explorer.get_clock = lambda: FakeClock(46.0)
    explorer.now_sec = lambda: 46.0

    explorer.check_active_goal_progress(explorer.map_to_world(2, 5))

    assert not explorer.goal_active
    assert handle.cancelled
    assert explorer.goal_attempts


def test_expired_goal_attempt_prunes_failure_count():
    grid = build_grid(20, 10, 0.1)
    explorer = make_explorer(grid)
    goal_xy = explorer.map_to_world(8, 5)
    explorer.goal_attempts = [GoalAttemptRecord(goal_xy, 3, 10.0)]
    explorer.now_sec = lambda: 11.0

    explorer.prune_blacklist()

    assert explorer.goal_attempts == []
    assert not explorer.is_blacklisted(goal_xy)


def test_nearby_failed_goals_share_retry_count_by_world_radius():
    grid = build_grid(20, 10, 0.1)
    explorer = make_explorer(grid)
    explorer.failed_goal_max_retries = 2
    explorer.now_sec = lambda: 0.0

    explorer.active_goal = explorer.map_to_world(8, 5)
    explorer.blacklist_active_goal()
    assert len(explorer.goal_attempts) == 1
    assert explorer.goal_attempts[0].failures == 1
    assert not explorer.is_blacklisted(explorer.map_to_world(8, 5))

    explorer.active_goal = explorer.map_to_world(9, 5)
    explorer.blacklist_active_goal()

    assert len(explorer.goal_attempts) == 1
    assert explorer.goal_attempts[0].failures == 2
    assert explorer.is_blacklisted(explorer.map_to_world(8, 5))
