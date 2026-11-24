#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/math_util.hpp"
#include "yahboom_alc_bringup/straight_tail.hpp"

namespace yahboom_alc_bringup
{

FrontierExplorerCore::FrontierExplorerCore(FrontierConfig config)
    : config_(std::move(config)) {}

void FrontierExplorerCore::setConfig(const FrontierConfig& config) {
    config_ = config;
    footprint_geometry_.reset();
    last_strict_candidate_sec_.reset();
    relaxation_active_ = false;
}

bool FrontierExplorerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
    if (map.info.resolution <= 0.0f ||
        map.data.size() !=
            static_cast<std::size_t>(map.info.width) * map.info.height) {
        return false;
    }
    map_ = map;
    has_map_ = true;
    footprint_geometry_.reset();
    return true;
}

CandidateResult FrontierExplorerCore::findCandidates(
    const WorldPoint& robot_xy, const double now_sec,
    std::optional<double> robot_yaw) {
    // Prune before any path returns: coverage revisits run exactly when no
    // frontier cluster exists, and must see blacklist entries expire too.
    pruneBlacklist(now_sec);
    if (config_.coverage_revisit_enabled) {
        noteRobotVisited(robot_xy);
    }
    // Prune a committed intention when the robot has arrived nearby, when it
    // has timed out, or when the goal has since been blacklisted.
    if (intention_) {
        const double dist =
            std::hypot(robot_xy.first - intention_->goal_xy.first,
                       robot_xy.second - intention_->goal_xy.second);
        const bool arrived = dist <= config_.intention_clear_radius_m;
        const bool timed_out = config_.intention_timeout_sec > 0.0 &&
                               now_sec - intention_->committed_at_sec >=
                                   config_.intention_timeout_sec;
        const bool blacklisted = isBlacklisted(intention_->goal_xy);
        const bool hazardous = isInHazardRegion(intention_->goal_xy);
        if (arrived || timed_out || blacklisted || hazardous) {
            intention_.reset();
        }
    }
    const auto grid = prepareSearchGrid(robot_xy);
    auto result = findCandidatesOnce(robot_xy, grid, strictLimits(), robot_yaw);
    if (!result.candidates.empty()) {
        last_strict_candidate_sec_ = now_sec;
        relaxation_active_ = false;
        // In blend mode, always merge coverage candidates into the main pool
        // even when frontier candidates exist (coverage_blend_weight > 0).
        // In fallback-only mode the guard inside appendCoverageCandidates keeps
        // this a no-op when candidates are present.
        if (config_.coverage_blend_weight != 0.0) {
            appendCoverageCandidates(result, robot_xy, grid, strictLimits(),
                                     robot_yaw);
        }
        return result;
    }
    if (config_.no_frontier_relax_sec <= 0.0) {
        // Frontier search is exhausted with relaxation disabled: fall back to
        // revisiting known-but-unapproached space rather than idling.
        appendCoverageCandidates(result, robot_xy, grid, strictLimits(),
                                 robot_yaw);
        return result;
    }
    if (!last_strict_candidate_sec_) {
        last_strict_candidate_sec_ = now_sec;
        return result;
    }
    if (!relaxation_active_ &&
        now_sec - *last_strict_candidate_sec_ < config_.no_frontier_relax_sec) {
        return result;
    }

    relaxation_active_ = true;
    auto relaxed_result =
        findCandidatesOnce(robot_xy, grid, relaxedLimits(), robot_yaw);
    relaxed_result.relaxed = true;
    // Relaxed frontier search also came up empty: revisit
    // known-but-unapproached space instead of giving up.
    appendCoverageCandidates(relaxed_result, robot_xy, grid, relaxedLimits(),
                             robot_yaw);
    return relaxed_result;
}

FrontierExplorerCore::SearchLimits FrontierExplorerCore::strictLimits() const {
    return {config_.min_frontier_size, config_.max_frontier_distance};
}

FrontierExplorerCore::SearchLimits FrontierExplorerCore::relaxedLimits() const {
    auto limits = strictLimits();
    limits.min_frontier_size =
        std::min(limits.min_frontier_size, config_.relaxed_min_frontier_size);
    if (config_.relaxed_max_frontier_distance > 0.0) {
        limits.max_frontier_distance =
            std::max(limits.max_frontier_distance,
                     config_.relaxed_max_frontier_distance);
    }
    return limits;
}

std::optional<FrontierExplorerCore::SearchGrid>
FrontierExplorerCore::prepareSearchGrid(const WorldPoint& robot_xy) const {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    if (!has_map_ || width == 0 || height == 0) {
        return std::nullopt;
    }
    const auto robot_cell = worldToMap(robot_xy.first, robot_xy.second);
    if (!robot_cell) {
        return std::nullopt;
    }
    const int exploratory_clearance =
        std::min(config_.path_clearance_cells,
                 std::max(0, config_.exploratory_path_clearance_cells));
    SearchGrid grid;
    grid.width = width;
    grid.height = height;
    grid.blocked = makeBlockedGrid(width, height, exploratory_clearance);
    const auto start_cell =
        nearestFreeCell(*robot_cell, grid.blocked, width, height,
                        config_.robot_start_search_radius);
    if (!start_cell) {
        return std::nullopt;
    }
    grid.start_cell = *start_cell;
    grid.strict_blocked =
        makeBlockedGrid(width, height, config_.path_clearance_cells);
    return grid;
}

CandidateResult FrontierExplorerCore::findCandidatesOnce(
    const WorldPoint& robot_xy, const std::optional<SearchGrid>& grid,
    const SearchLimits& limits, std::optional<double> robot_yaw) {
    CandidateResult result;
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    if (!has_map_ || width == 0 || height == 0) {
        return result;
    }

    for (const auto value : map_.data) {
        if (value == -1) {
            ++result.counts.unknown;
        }
        else if (isFreeValue(value)) {
            ++result.counts.free;
        }
        else {
            ++result.counts.occupied;
        }
    }

    const auto frontier_cells = findFrontierCells(width, height);
    result.counts.frontier = static_cast<int>(frontier_cells.size());
    auto clusters = clusterFrontiers(frontier_cells, width);
    std::erase_if(clusters, [&limits](const auto& cluster) {
        return static_cast<int>(cluster.size()) < limits.min_frontier_size;
    });
    result.counts.clusters = static_cast<int>(clusters.size());
    if (clusters.empty() || !grid) {
        return result;
    }

    for (const auto& cluster : clusters) {
        auto candidate = makeCandidateImpl(
            cluster, grid->start_cell, robot_xy, grid->blocked,
            grid->strict_blocked, width, height, limits.max_frontier_distance,
            result.counts, robot_yaw);
        if (!candidate) {
            continue;
        }
        result.candidates.push_back(std::move(*candidate));
    }
    return result;
}

void FrontierExplorerCore::appendCoverageCandidates(
    CandidateResult& result, const WorldPoint& robot_xy,
    const std::optional<SearchGrid>& grid, const SearchLimits& limits,
    std::optional<double> robot_yaw) {
    if (!config_.coverage_revisit_enabled) {
        return;
    }
    // In pure-fallback mode (blend disabled) only run when no frontier
    // candidates exist; in blend mode always include coverage candidates so
    // unvisited known-free regions compete directly with live frontiers.
    if (config_.coverage_blend_weight == 0.0 && !result.candidates.empty()) {
        return;
    }
    if (!grid) {
        return;
    }
    const auto seeds = findCoverageSeeds(robot_xy, limits.max_frontier_distance,
                                         grid->width, grid->height);
    FrontierCounts coverage_counts;
    for (const auto& seed : seeds) {
        auto candidate = makeCandidateImpl(
            {seed}, grid->start_cell, robot_xy, grid->blocked,
            grid->strict_blocked, grid->width, grid->height,
            limits.max_frontier_distance, coverage_counts, robot_yaw);
        if (!candidate) {
            continue;
        }
        candidate->is_coverage = true;
        // Subtract blend weight from score (positive weight = better score =
        // more competitive against frontier candidates).
        candidate->score -= config_.coverage_blend_weight;
        result.candidates.push_back(std::move(*candidate));
    }
    result.counts.coverage_candidates =
        static_cast<int>(result.candidates.size());
}

std::optional<FrontierCandidate> FrontierExplorerCore::makeCandidate(
    const std::vector<Cell>& cluster, const Cell& start_cell,
    const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
    const int width, const int height, FrontierCounts& counts,
    std::optional<double> robot_yaw) {
    const auto strict_blocked =
        makeBlockedGrid(width, height, config_.path_clearance_cells);
    return makeCandidateImpl(cluster, start_cell, robot_xy, blocked,
                             strict_blocked, width, height,
                             config_.max_frontier_distance, counts, robot_yaw);
}

std::optional<FrontierCandidate> FrontierExplorerCore::makeCandidateImpl(
    const std::vector<Cell>& cluster, const Cell& start_cell,
    const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
    const std::vector<std::uint8_t>& strict_blocked, const int width,
    const int height, const double max_frontier_distance,
    FrontierCounts& counts, std::optional<double> robot_yaw) {
    const auto goal_options =
        goalOptionsForCluster(cluster, blocked, width, height);
    if (goal_options.empty()) {
        ++counts.rejected_no_options;
        return std::nullopt;
    }

    std::optional<FrontierCandidate> best;
    const auto axis = clusterAxis(cluster);
    const auto start_xy =
        mapToWorldUnchecked(start_cell.first, start_cell.second);
    for (const auto& goal_cell : goal_options) {
        const auto goal_xy =
            mapToWorldUnchecked(goal_cell.first, goal_cell.second);
        if (isBlacklisted(goal_xy)) {
            ++counts.rejected_blacklist;
            continue;
        }
        if (isInHazardRegion(goal_xy)) {
            ++counts.rejected_hazard;
            continue;
        }
        const double goal_distance = std::hypot(
            goal_xy.first - robot_xy.first, goal_xy.second - robot_xy.second);
        if (goal_distance < config_.min_frontier_distance) {
            ++counts.rejected_too_near;
            continue;
        }
        if (robot_yaw && config_.min_turning_radius_m > 0.0) {
            const double dx = goal_xy.first - robot_xy.first;
            const double dy = goal_xy.second - robot_xy.second;
            const double body_x =
                std::cos(*robot_yaw) * dx + std::sin(*robot_yaw) * dy;
            const double body_y =
                -std::sin(*robot_yaw) * dx + std::cos(*robot_yaw) * dy;
            const double radius = config_.min_turning_radius_m;
            if (std::hypot(body_x, body_y - radius) < radius ||
                std::hypot(body_x, body_y + radius) < radius) {
                ++counts.rejected_turning_circle;
                continue;
            }
        }
        if (max_frontier_distance > 0.0 &&
            goal_distance > max_frontier_distance) {
            ++counts.rejected_too_far;
            continue;
        }

        auto strict_path =
            gridPath(start_cell, goal_cell, strict_blocked, width, height);
        auto path = strict_path ? strict_path
                                : gridPath(start_cell, goal_cell, blocked,
                                           width, height);
        const bool reachable = strict_path.has_value();
        const bool soft_reachable = !reachable && path.has_value();
        if (!reachable && !soft_reachable && !config_.allow_unreachable_goal) {
            ++counts.rejected_no_path;
            continue;
        }
        if (!path) {
            path = std::vector<Cell>{start_cell, goal_cell};
        }

        double goal_yaw = 0.0;
        FootprintCheck footprint;
        std::optional<ViewTail> view;
        if (config_.view_tail_length_m > 0.0 && require_view_tail_) {
            // The yaw is the direction the car will face on its straight
            // approach, chosen for what it sees; no tail fits = no view.
            view = bestViewTail(goal_xy);
            if (!view) {
                ++counts.rejected_footprint;
                continue;
            }
            goal_yaw = view->yaw;
            footprint = view->goal_footprint;
        }
        else {
            goal_yaw = pathGoalYaw(*path, robot_xy, goal_xy);
            footprint =
                footprintCheckAtWorld(goal_xy.first, goal_xy.second, goal_yaw);
            if (footprint.occupied) {
                ++counts.rejected_footprint;
                continue;
            }
        }
        const auto swept_unknown_ratio = sweptPathUnknownRatio(*path);
        if (!swept_unknown_ratio) {
            ++counts.rejected_swept;
            continue;
        }

        const double path_length = pathLength(*path);
        const int information = countUnknownNear(goal_cell);
        const auto obstacle =
            analyzeObstacleNeighborhood(goal_cell, width, height);
        const double progress = corridorProgress(start_xy, goal_xy, axis);
        const double unknown_ratio =
            std::max(footprint.unknownRatio(), *swept_unknown_ratio);
        if (unknown_ratio > config_.unknown_overlap_max_ratio) {
            ++counts.rejected_unknown_ratio;
            continue;
        }
        if (config_.min_wall_clearance_m > 0.0 &&
            obstacle.wall_clearance < config_.min_wall_clearance_m) {
            ++counts.rejected_clearance;
            continue;
        }
        // Mirror Nav2's goal feasibility: a pose inside the global inflation
        // band is unplannable, so drop it here instead of letting Nav2 abort
        // and blacklist it. This floor stays in force under relaxed search.
        if (config_.nav2_goal_clearance_m > 0.0 &&
            obstacle.wall_clearance < config_.nav2_goal_clearance_m) {
            ++counts.rejected_nav2_clearance;
            continue;
        }
        // Stage 2 viewpoint term is additive and only computed when enabled, so
        // with weight 0 the score (and thus selection) is unchanged.
        int visible_unknown = view ? static_cast<int>(view->cells.size()) : 0;
        double viewpoint_bonus = 0.0;
        if (config_.viewpoint_information_weight > 0.0) {
            if (!view) {
                visible_unknown = countVisibleUnknown(goal_cell, goal_yaw);
            }
            viewpoint_bonus =
                config_.viewpoint_information_weight * visible_unknown;
        }
        double score = config_.path_weight * path_length -
                       config_.information_weight * information -
                       config_.frontier_size_weight * cluster.size() -
                       config_.corridor_progress_weight * progress +
                       config_.unknown_overlap_weight * unknown_ratio +
                       config_.corner_penalty_weight * obstacle.corner_penalty -
                       viewpoint_bonus;
        score += hazardPathPenalty(*path);
        // Intention continuation bonus: reduce score (make more attractive) for
        // candidates near the committed intention goal so the planner sustains
        // progress toward a chosen distant region across replanning cycles.
        if (intention_ && config_.intention_continuation_bonus > 0.0) {
            const double dist_to_intention =
                std::hypot(goal_xy.first - intention_->goal_xy.first,
                           goal_xy.second - intention_->goal_xy.second);
            if (dist_to_intention <= config_.intention_clear_radius_m * 2.0) {
                score -= config_.intention_continuation_bonus;
            }
        }
        if (obstacle.wall_clearance < config_.wall_clearance_m) {
            score += config_.corner_penalty_weight *
                     (config_.wall_clearance_m - obstacle.wall_clearance);
        }
        if (!reachable) {
            score += soft_reachable ? config_.soft_unreachable_penalty
                                    : config_.unreachable_goal_penalty;
        }
        if (const auto attempt = findGoalAttemptIndex(goal_xy)) {
            score += config_.soft_unreachable_penalty *
                     static_cast<double>(goal_attempts_[*attempt].failures);
        }
        double heading_delta = 0.0;
        if (robot_yaw && config_.heading_penalty_weight > 0.0 &&
            config_.heading_penalty_max_rad > 0.0) {
            heading_delta = headingDelta(
                *robot_yaw, pathInitialYaw(*path, robot_xy, goal_xy));
            score += config_.heading_penalty_weight *
                     std::min(heading_delta, config_.heading_penalty_max_rad) /
                     config_.heading_penalty_max_rad;
        }

        FrontierCandidate candidate;
        candidate.cells = cluster;
        candidate.goal_cell = goal_cell;
        candidate.goal_xy = goal_xy;
        candidate.goal_yaw = goal_yaw;
        candidate.path = *path;
        candidate.path_length = path_length;
        candidate.information = information;
        candidate.visible_unknown = visible_unknown;
        if (view) {
            candidate.view_cells = std::move(view->cells);
        }
        candidate.reachable = reachable;
        candidate.soft_reachable = soft_reachable;
        candidate.unknown_overlap_ratio = unknown_ratio;
        candidate.wall_clearance = obstacle.wall_clearance;
        candidate.heading_delta = heading_delta;
        candidate.score = score;
        if (!best || candidate.score < best->score) {
            best = std::move(candidate);
        }
    }
    // When viewpoint scoring is disabled it is not computed in the loop; fill
    // it in for the single chosen candidate so the metric is still logged (and
    // can be validated) without paying the per-option raycast cost or changing
    // selection.
    if (best && config_.viewpoint_information_weight <= 0.0 &&
        config_.view_tail_length_m <= 0.0) {
        best->visible_unknown =
            countVisibleUnknown(best->goal_cell, best->goal_yaw);
    }
    return best;
}

std::vector<Cell> FrontierExplorerCore::findFrontierCells(
    const int width, const int height) const {
    std::vector<Cell> cells;
    for (int y = 1; y < height - 1; ++y) {
        const int row = y * width;
        for (int x = 1; x < width - 1; ++x) {
            const int cell_index = row + x;
            if (!isFreeValue(map_.data[cell_index])) {
                continue;
            }
            if (hasUnknownNeighbor(width, cell_index)) {
                cells.emplace_back(x, y);
            }
        }
    }
    return cells;
}

std::vector<std::vector<Cell>> FrontierExplorerCore::clusterFrontiers(
    const std::vector<Cell>& frontier_cells, const int width) const {
    const int height = static_cast<int>(map_.info.height);
    const int total = width * height;
    std::vector<std::uint8_t> is_frontier(total, false);
    for (const auto& cell : frontier_cells) {
        is_frontier[index(cell.first, cell.second, width)] = true;
    }
    std::vector<std::uint8_t> visited(total, false);
    std::vector<std::vector<Cell>> clusters;
    for (const auto& seed : frontier_cells) {
        const int seed_idx = index(seed.first, seed.second, width);
        if (visited[seed_idx]) {
            continue;
        }
        std::vector<Cell> cluster;
        std::queue<Cell> queue;
        queue.push(seed);
        visited[seed_idx] = true;
        while (!queue.empty()) {
            const auto cell = queue.front();
            queue.pop();
            cluster.push_back(cell);
            for (const auto& neighbor :
                 neighborCells(cell.first, cell.second)) {
                if (!inBounds(neighbor.first, neighbor.second, width, height)) {
                    continue;
                }
                const int n_idx = index(neighbor.first, neighbor.second, width);
                if (is_frontier[n_idx] && !visited[n_idx]) {
                    visited[n_idx] = true;
                    queue.push(neighbor);
                }
            }
        }
        clusters.push_back(std::move(cluster));
    }
    return clusters;
}

std::vector<Cell> FrontierExplorerCore::goalOptionsForCluster(
    const std::vector<Cell>& cluster, const std::vector<std::uint8_t>& blocked,
    const int width, const int height) const {
    const int radius_cells =
        std::max(1, static_cast<int>(config_.frontier_goal_search_radius /
                                     mapResolution()));
    std::vector<std::uint8_t> seen(static_cast<std::size_t>(width * height),
                                   false);
    std::vector<Cell> options;
    const int radius_sq = radius_cells * radius_cells;
    for (const auto& frontier : cluster) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const int x = frontier.first + dx;
                const int y = frontier.second + dy;
                if (!inBounds(x, y, width, height)) {
                    continue;
                }
                if (dx * dx + dy * dy > radius_sq) {
                    continue;
                }
                const int idx = index(x, y, width);
                if (!isFreeValue(map_.data[idx])) {
                    continue;
                }
                if (blocked[idx]) {
                    continue;
                }
                if (hasOccupiedClearance(x, y, width, height,
                                         config_.min_clearance_cells)) {
                    if (!seen[idx]) {
                        seen[idx] = true;
                        options.emplace_back(x, y);
                    }
                }
            }
        }
    }
    return options;
}

int FrontierExplorerCore::countUnknownNear(const Cell& cell) const {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int radius_cells = std::max(
        1, static_cast<int>(config_.information_radius / mapResolution()));
    const int radius_sq = radius_cells * radius_cells;
    int count = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int x = cell.first + dx;
            const int y = cell.second + dy;
            if (!inBounds(x, y, width, height)) {
                continue;
            }
            if (dx * dx + dy * dy > radius_sq) {
                continue;
            }
            if (map_.data[index(x, y, width)] == -1) {
                ++count;
            }
        }
    }
    return count;
}

std::vector<Cell> FrontierExplorerCore::findCoverageSeeds(
    const WorldPoint& robot_xy, const double max_distance, const int width,
    const int height) const {
    const double resolution = mapResolution();
    if (resolution <= 0.0) {
        return {};
    }
    const int step = std::max(
        1, static_cast<int>(config_.coverage_sample_spacing_m / resolution));
    std::vector<std::pair<double, Cell>> ranked;
    for (int y = 0; y < height; y += step) {
        for (int x = 0; x < width; x += step) {
            if (!isFreeValue(map_.data[index(x, y, width)])) {
                continue;
            }
            if (!hasOccupiedClearance(x, y, width, height,
                                      config_.min_clearance_cells)) {
                continue;
            }
            const auto xy = mapToWorldUnchecked(x, y);
            const double distance = std::hypot(xy.first - robot_xy.first,
                                               xy.second - robot_xy.second);
            if (distance < config_.min_frontier_distance) {
                continue;
            }
            if (max_distance > 0.0 && distance > max_distance) {
                continue;
            }
            if (!isUnderObserved(xy)) {
                continue;
            }
            ranked.emplace_back(distance, Cell{x, y});
        }
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<Cell> seeds;
    const int limit = std::max(0, config_.coverage_max_targets);
    for (const auto& entry : ranked) {
        if (static_cast<int>(seeds.size()) >= limit) {
            break;
        }
        seeds.push_back(entry.second);
    }
    return seeds;
}

int FrontierExplorerCore::countVisibleUnknown(const Cell& from,
                                              const double yaw) const {
    return static_cast<int>(visibleUnknownIndices(from, yaw).size());
}

std::vector<int> FrontierExplorerCore::visibleUnknownIndices(
    const Cell& from, const double yaw) const {
    std::vector<int> cells;
    const double resolution = mapResolution();
    if (resolution <= 0.0 || config_.viewpoint_range_m <= 0.0) {
        return cells;
    }
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int range_cells =
        std::max(1, static_cast<int>(config_.viewpoint_range_m / resolution));
    // Angular step keeps adjacent rays within one cell at max range so the fan
    // does not skip cells; clamped so a tiny FOV still casts at least one ray.
    const double angular_step =
        std::clamp(resolution / config_.viewpoint_range_m, 1e-3, 0.2);
    std::unordered_set<int> seen;
    for (double offset = -config_.viewpoint_half_fov_rad;
         offset <= config_.viewpoint_half_fov_rad + 1e-9;
         offset += angular_step) {
        const double angle = yaw + offset;
        const double dx = std::cos(angle);
        const double dy = std::sin(angle);
        for (int step = 1; step <= range_cells; ++step) {
            const int x = from.first + static_cast<int>(std::lround(dx * step));
            const int y =
                from.second + static_cast<int>(std::lround(dy * step));
            if (!inBounds(x, y, width, height)) {
                break;
            }
            const int idx = index(x, y, width);
            const int value = map_.data[idx];
            if (isOccupiedValue(value)) {
                break;  // line of sight blocked by an obstacle
            }
            if (value == -1 && seen.insert(idx).second) {
                cells.push_back(idx);
            }
        }
    }
    return cells;
}

std::optional<ViewTail> FrontierExplorerCore::bestViewTail(
    const WorldPoint& goal_xy) {
    const int width = static_cast<int>(map_.info.width);
    // Adjacent fans overlap by half, so every bearing from the goal lies
    // inside the fan of some direction.
    const double half_fov = std::max(config_.viewpoint_half_fov_rad, 1e-3);
    const int directions = static_cast<int>(std::ceil(2.0 * M_PI / half_fov));
    std::optional<ViewTail> best;
    for (int k = 0; k < directions; ++k) {
        const double yaw = -M_PI + 2.0 * M_PI * k / directions;
        const TailPose goal{goal_xy.first, goal_xy.second, yaw};
        // The car drives every tail pose; each must pass the goal pose rule.
        std::optional<FootprintCheck> at_goal;
        bool feasible = true;
        for (const auto& pose :
             tailPoses(goal, 0.0, -config_.view_tail_length_m,
                       config_.swept_step_m)) {
            const auto check = footprintCheckAtWorld(pose.x, pose.y, yaw);
            if (check.occupied ||
                check.unknownRatio() > config_.unknown_overlap_max_ratio) {
                feasible = false;
                break;
            }
            if (!at_goal) {
                at_goal = check;
            }
        }
        if (!feasible) {
            continue;
        }
        // Score from the earliest stop the goal checker allows on the tail.
        const auto stop = worldToMap(
            goal_xy.first - config_.view_stop_offset_m * std::cos(yaw),
            goal_xy.second - config_.view_stop_offset_m * std::sin(yaw));
        if (!stop) {
            continue;
        }
        const auto indices = visibleUnknownIndices(*stop, yaw);
        if (best && indices.size() <= best->cells.size()) {
            continue;
        }
        ViewTail view{yaw, *at_goal, {}};
        view.cells.reserve(indices.size());
        for (const int idx : indices) {
            view.cells.push_back(mapToWorldUnchecked(idx % width, idx / width));
        }
        best = std::move(view);
    }
    return best;
}

int FrontierExplorerCore::countObserved(
    const std::vector<WorldPoint>& cells) const {
    const int width = static_cast<int>(map_.info.width);
    int observed = 0;
    for (const auto& xy : cells) {
        const auto cell = worldToMap(xy.first, xy.second);
        if (cell && map_.data[index(cell->first, cell->second, width)] != -1) {
            ++observed;
        }
    }
    return observed;
}

ObstacleNeighborhood FrontierExplorerCore::analyzeObstacleNeighborhood(
    const Cell& cell, const int width, const int height) const {
    const double search_radius_m =
        std::max({config_.wall_clearance_m, config_.min_wall_clearance_m,
                  config_.nav2_goal_clearance_m});
    const int radius_cells = std::max(
        1, static_cast<int>(std::ceil(search_radius_m / mapResolution())));
    double best = std::numeric_limits<double>::infinity();
    uint8_t quadrant_mask = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int x = cell.first + dx;
            const int y = cell.second + dy;
            if (dx == 0 && dy == 0) {
                continue;
            }
            bool blocked = !inBounds(x, y, width, height);
            if (!blocked) {
                blocked = isOccupiedValue(map_.data[index(x, y, width)]);
            }
            if (!blocked) {
                continue;
            }
            best = std::min(best, std::hypot(dx, dy) * mapResolution());
            quadrant_mask |= static_cast<uint8_t>(
                1u << ((dx >= 0 ? 1 : 0) | ((dy >= 0 ? 1 : 0) << 1)));
        }
    }
    ObstacleNeighborhood result;
    result.wall_clearance = std::isinf(best) ? search_radius_m : best;
    result.corner_penalty = std::max(
        0.0, static_cast<double>(std::bitset<4>(quadrant_mask).count()) - 1.0);
    return result;
}

WorldPoint FrontierExplorerCore::clusterAxis(
    const std::vector<Cell>& cluster) const {
    if (cluster.size() < 2) {
        return {1.0, 0.0};
    }
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& cell : cluster) {
        mean_x += cell.first;
        mean_y += cell.second;
    }
    mean_x /= cluster.size();
    mean_y /= cluster.size();
    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const auto& cell : cluster) {
        const double dx = cell.first - mean_x;
        const double dy = cell.second - mean_y;
        cov_xx += dx * dx;
        cov_xy += dx * dy;
        cov_yy += dy * dy;
    }
    if (std::abs(cov_xy) < 1e-6 && cov_xx >= cov_yy) {
        return {1.0, 0.0};
    }
    if (std::abs(cov_xy) < 1e-6) {
        return {0.0, 1.0};
    }
    const double theta = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    return {std::cos(theta), std::sin(theta)};
}

double FrontierExplorerCore::corridorProgress(const WorldPoint& start_xy,
                                              const WorldPoint& goal_xy,
                                              const WorldPoint& axis) const {
    const double dx = goal_xy.first - start_xy.first;
    const double dy = goal_xy.second - start_xy.second;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
        return 0.0;
    }
    const double ux = dx / dist;
    const double uy = dy / dist;
    return dist * std::abs(ux * axis.first + uy * axis.second);
}

bool FrontierExplorerCore::isFreeValue(const int value) const {
    return 0 <= value && value < config_.occupied_threshold;
}

bool FrontierExplorerCore::isOccupiedValue(const int value) const {
    return value >= config_.occupied_threshold;
}

bool FrontierExplorerCore::hasUnknownNeighbor(const int width,
                                              const int i) const {
    const int neighbors[] = {i - width,     i + width,     i - 1,
                             i + 1,         i - width - 1, i - width + 1,
                             i + width - 1, i + width + 1};
    return std::any_of(std::begin(neighbors), std::end(neighbors),
                       [this](const int n) { return map_.data[n] == -1; });
}

bool FrontierExplorerCore::hasOccupiedClearance(const int x, const int y,
                                                const int width,
                                                const int height,
                                                const int radius) const {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if (!inBounds(nx, ny, width, height)) {
                return false;
            }
            if (isOccupiedValue(map_.data[index(nx, ny, width)])) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace yahboom_alc_bringup
