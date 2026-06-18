#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

#include "yahboom_alc_bringup/math_util.hpp"

namespace yahboom_alc_bringup
{

double FootprintCheck::unknownRatio() const {
    return total <= 0 ? 1.0 : static_cast<double>(unknown) / total;
}

FrontierExplorerCore::FrontierExplorerCore(FrontierConfig config)
    : config_(std::move(config)) {}

void FrontierExplorerCore::setConfig(const FrontierConfig& config) {
    config_ = config;
    footprint_geometry_.reset();
    last_strict_candidate_sec_.reset();
    relaxation_active_ = false;
}

void FrontierExplorerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_ = map;
    has_map_ = true;
    footprint_geometry_.reset();
}

CandidateResult FrontierExplorerCore::findCandidates(
    const WorldPoint& robot_xy, const double now_sec,
    std::optional<double> robot_yaw) {
    auto result = findCandidatesOnce(robot_xy, now_sec, robot_yaw);
    if (!result.candidates.empty()) {
        last_strict_candidate_sec_ = now_sec;
        relaxation_active_ = false;
        return result;
    }
    if (config_.no_frontier_relax_sec <= 0.0) {
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
    struct ConfigRestore
    {
        FrontierExplorerCore& core;
        FrontierConfig strict;
        ~ConfigRestore() {
            core.config_ = std::move(strict);
            core.footprint_geometry_.reset();
        }
    } restore{*this, config_};
    config_ = relaxedConfig();
    footprint_geometry_.reset();
    auto relaxed_result = findCandidatesOnce(robot_xy, now_sec, robot_yaw);
    relaxed_result.relaxed = true;
    return relaxed_result;
}

FrontierConfig FrontierExplorerCore::relaxedConfig() const {
    FrontierConfig relaxed = config_;
    relaxed.min_frontier_size =
        std::min(relaxed.min_frontier_size, config_.relaxed_min_frontier_size);
    if (config_.relaxed_max_frontier_distance > 0.0) {
        relaxed.max_frontier_distance =
            std::max(relaxed.max_frontier_distance,
                     config_.relaxed_max_frontier_distance);
    }
    return relaxed;
}

CandidateResult FrontierExplorerCore::findCandidatesOnce(
    const WorldPoint& robot_xy, const double now_sec,
    std::optional<double> robot_yaw) {
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
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [this](const auto& cluster) {
                                      return static_cast<int>(cluster.size()) <
                                             config_.min_frontier_size;
                                  }),
                   clusters.end());
    result.counts.clusters = static_cast<int>(clusters.size());
    if (clusters.empty()) {
        return result;
    }

    pruneBlacklist(now_sec);
    const int exploratory_clearance =
        std::min(config_.path_clearance_cells,
                 std::max(0, config_.exploratory_path_clearance_cells));
    const auto blocked = makeBlockedGrid(width, height, exploratory_clearance);
    const auto strict_blocked =
        makeBlockedGrid(width, height, config_.path_clearance_cells);
    const auto robot_cell = worldToMap(robot_xy.first, robot_xy.second);
    if (!robot_cell) {
        return result;
    }
    const auto start_cell = nearestFreeCell(*robot_cell, blocked, width, height,
                                            config_.robot_start_search_radius);
    if (!start_cell) {
        return result;
    }

    for (const auto& cluster : clusters) {
        auto candidate = makeCandidateImpl(cluster, *start_cell, robot_xy,
                                           blocked, strict_blocked, width,
                                           height, result.counts, robot_yaw);
        if (!candidate) {
            continue;
        }
        result.candidates.push_back(std::move(*candidate));
    }
    return result;
}

std::optional<FrontierCandidate> FrontierExplorerCore::makeCandidate(
    const std::vector<Cell>& cluster, const Cell& start_cell,
    const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
    const int width, const int height, FrontierCounts& counts,
    std::optional<double> robot_yaw) {
    const auto strict_blocked =
        makeBlockedGrid(width, height, config_.path_clearance_cells);
    return makeCandidateImpl(cluster, start_cell, robot_xy, blocked,
                             strict_blocked, width, height, counts, robot_yaw);
}

std::optional<FrontierCandidate> FrontierExplorerCore::makeCandidateImpl(
    const std::vector<Cell>& cluster, const Cell& start_cell,
    const WorldPoint& robot_xy, const std::vector<std::uint8_t>& blocked,
    const std::vector<std::uint8_t>& strict_blocked, const int width,
    const int height, FrontierCounts& counts, std::optional<double> robot_yaw) {
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
        const double goal_distance = std::hypot(
            goal_xy.first - robot_xy.first, goal_xy.second - robot_xy.second);
        if (goal_distance < config_.min_frontier_distance) {
            ++counts.rejected_too_near;
            continue;
        }
        if (config_.max_frontier_distance > 0.0 &&
            goal_distance > config_.max_frontier_distance) {
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

        const double goal_yaw = pathGoalYaw(*path, robot_xy, goal_xy);
        const auto footprint =
            footprintCheckAtWorld(goal_xy.first, goal_xy.second, goal_yaw);
        if (footprint.occupied) {
            ++counts.rejected_footprint;
            continue;
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
        double score = config_.path_weight * path_length -
                       config_.information_weight * information -
                       config_.frontier_size_weight * cluster.size() -
                       config_.corridor_progress_weight * progress +
                       config_.unknown_overlap_weight * unknown_ratio +
                       config_.corner_penalty_weight * obstacle.corner_penalty;
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
        candidate.reachable = reachable;
        candidate.soft_reachable = soft_reachable;
        candidate.approach_xy = approachPointForPath(*path, goal_xy);
        candidate.approach_yaw = yawBetween(robot_xy, candidate.approach_xy);
        candidate.unknown_overlap_ratio = unknown_ratio;
        candidate.wall_clearance = obstacle.wall_clearance;
        candidate.heading_delta = heading_delta;
        candidate.score = score;
        if (!best || candidate.score < best->score) {
            best = std::move(candidate);
        }
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

std::vector<std::uint8_t> FrontierExplorerCore::makeBlockedGrid(
    const int width, const int height, const int clearance) const {
    std::vector<std::uint8_t> inflated(map_.data.size());
    for (std::size_t i = 0; i < map_.data.size(); ++i) {
        inflated[i] = !isFreeValue(map_.data[i]);
    }
    if (clearance <= 0) {
        return inflated;
    }
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!isOccupiedValue(map_.data[index(x, y, width)])) {
                continue;
            }
            for (int dy = -clearance; dy <= clearance; ++dy) {
                for (int dx = -clearance; dx <= clearance; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (inBounds(nx, ny, width, height)) {
                        inflated[index(nx, ny, width)] = true;
                    }
                }
            }
        }
    }
    return inflated;
}

std::optional<Cell> FrontierExplorerCore::nearestFreeCell(
    const Cell& origin, const std::vector<std::uint8_t>& blocked,
    const int width, const int height, const double radius_m) const {
    const int max_radius =
        std::max(1, static_cast<int>(radius_m / mapResolution()));
    const auto [ox, oy] = origin;
    if (inBounds(ox, oy, width, height) && !blocked[index(ox, oy, width)]) {
        return origin;
    }
    std::optional<Cell> best;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int dy = -max_radius; dy <= max_radius; ++dy) {
        for (int dx = -max_radius; dx <= max_radius; ++dx) {
            const int x = ox + dx;
            const int y = oy + dy;
            if (!inBounds(x, y, width, height)) {
                continue;
            }
            if (blocked[index(x, y, width)]) {
                continue;
            }
            const double dist = std::hypot(dx, dy);
            if (dist < best_dist) {
                best_dist = dist;
                best = Cell{x, y};
            }
        }
    }
    return best;
}

std::optional<std::vector<Cell>> FrontierExplorerCore::gridPath(
    const Cell& start, const Cell& goal,
    const std::vector<std::uint8_t>& blocked, const int width,
    const int height) const {
    if (start == goal) {
        return std::vector<Cell>{start};
    }
    const int start_index = index(start.first, start.second, width);
    const int goal_index = index(goal.first, goal.second, width);
    std::queue<Cell> queue;
    std::vector<int> parent(width * height, -1);
    queue.push(start);
    parent[start_index] = start_index;
    while (!queue.empty()) {
        const auto cell = queue.front();
        queue.pop();
        for (const auto& neighbor : neighborCells(cell.first, cell.second)) {
            if (!inBounds(neighbor.first, neighbor.second, width, height)) {
                continue;
            }
            const int next_index =
                index(neighbor.first, neighbor.second, width);
            if (blocked[next_index] || parent[next_index] != -1) {
                continue;
            }
            parent[next_index] = index(cell.first, cell.second, width);
            if (next_index == goal_index) {
                return reconstructPath(parent, start_index, goal_index, width);
            }
            queue.push(neighbor);
        }
    }
    return std::nullopt;
}

std::vector<Cell> FrontierExplorerCore::reconstructPath(
    const std::vector<int>& parent, const int start_index, const int goal_index,
    const int width) const {
    std::vector<Cell> path;
    int current = goal_index;
    while (true) {
        path.emplace_back(current % width, current / width);
        if (current == start_index) {
            break;
        }
        current = parent[current];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double FrontierExplorerCore::pathLength(const std::vector<Cell>& path) const {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += euclideanCells(path[i - 1], path[i]) * mapResolution();
    }
    return total;
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

bool FrontierExplorerCore::shouldPublishGoal(const WorldPoint& goal_xy,
                                             const double now_sec) const {
    if (!last_goal_) {
        return true;
    }
    const double elapsed = now_sec - last_goal_time_sec_;
    const double moved = std::hypot(goal_xy.first - last_goal_->first,
                                    goal_xy.second - last_goal_->second);
    return moved >= config_.goal_update_distance ||
           elapsed >= config_.goal_publish_period;
}

void FrontierExplorerCore::noteGoalPublished(const WorldPoint& goal_xy,
                                             const double goal_yaw,
                                             const double now_sec) {
    last_goal_ = goal_xy;
    last_goal_yaw_ = goal_yaw;
    last_goal_time_sec_ = now_sec;
}

FootprintCheck FrontierExplorerCore::footprintCheckAtWorld(const double wx,
                                                           const double wy,
                                                           const double yaw) {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const auto footprint = footprintGeometry();
    const auto center = worldToMap(wx, wy);
    if (!center) {
        return {true, 0, 0};
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    FootprintCheck result;
    for (int dy = -footprint.radius_cells; dy <= footprint.radius_cells; ++dy) {
        for (int dx = -footprint.radius_cells; dx <= footprint.radius_cells;
             ++dx) {
            const int x = center->first + dx;
            const int y = center->second + dy;
            const auto cell_xy = mapToWorldUnchecked(x, y);
            const double rel_x = cell_xy.first - wx;
            const double rel_y = cell_xy.second - wy;
            const double body_x = cos_yaw * rel_x + sin_yaw * rel_y;
            const double body_y = -sin_yaw * rel_x + cos_yaw * rel_y;
            if (body_x < -footprint.rear || body_x > footprint.front) {
                continue;
            }
            if (std::abs(body_y) > footprint.half_width) {
                continue;
            }
            ++result.total;
            if (!inBounds(x, y, width, height)) {
                result.occupied = true;
                continue;
            }
            const int value = map_.data[index(x, y, width)];
            if (isOccupiedValue(value)) {
                result.occupied = true;
            }
            else if (value == -1) {
                ++result.unknown;
            }
        }
    }
    return result;
}

std::optional<double> FrontierExplorerCore::sweptPathUnknownRatio(
    const std::vector<Cell>& path) {
    if (path.empty()) {
        return std::nullopt;
    }
    if (path.size() == 1) {
        const auto xy =
            mapToWorldUnchecked(path.front().first, path.front().second);
        const auto check =
            footprintCheckAtWorld(xy.first, xy.second, last_goal_yaw_);
        if (check.occupied) {
            return std::nullopt;
        }
        return check.unknownRatio();
    }

    double max_ratio = 0.0;
    const double step = std::max(config_.swept_step_m, mapResolution());
    for (std::size_t i = 1; i < path.size(); ++i) {
        const auto prev_xy =
            mapToWorldUnchecked(path[i - 1].first, path[i - 1].second);
        const auto current_xy =
            mapToWorldUnchecked(path[i].first, path[i].second);
        const double dx = current_xy.first - prev_xy.first;
        const double dy = current_xy.second - prev_xy.second;
        const double yaw = std::atan2(dy, dx);
        const double distance = std::hypot(dx, dy);
        const int samples =
            std::max(1, static_cast<int>(std::ceil(distance / step)));
        for (int sample = 0; sample <= samples; ++sample) {
            const double ratio = static_cast<double>(sample) / samples;
            const double wx = prev_xy.first + dx * ratio;
            const double wy = prev_xy.second + dy * ratio;
            const auto check = footprintCheckAtWorld(wx, wy, yaw);
            if (check.occupied) {
                return std::nullopt;
            }
            max_ratio = std::max(max_ratio, check.unknownRatio());
        }
    }
    return max_ratio;
}

std::optional<double> FrontierExplorerCore::findClearestEscapeDirection(
    const WorldPoint& pos, const double max_dist_m,
    const int num_angles) const {
    if (!has_map_ || max_dist_m <= 0.0 || num_angles <= 0 ||
        mapResolution() <= 0.0) {
        return std::nullopt;
    }
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    if (width <= 0 || height <= 0 || map_.data.empty()) {
        return std::nullopt;
    }
    const auto start = worldToMap(pos.first, pos.second);
    if (!start) {
        return std::nullopt;
    }

    std::optional<double> best_yaw;
    int best_free_cells = 0;
    double best_clear_distance = 0.0;
    const double step = mapResolution();
    const int samples =
        std::max(1, static_cast<int>(std::ceil(max_dist_m / step)));

    for (int angle_index = 0; angle_index < num_angles; ++angle_index) {
        const double yaw = evenlySpacedYaw(angle_index, num_angles);
        int free_cells = 0;
        double clear_distance = 0.0;
        std::optional<Cell> previous_cell;

        for (int sample = 1; sample <= samples; ++sample) {
            const double distance = static_cast<double>(sample) * step;
            const auto point = projectPoint(pos, yaw, distance);
            const auto cell = worldToMap(point.first, point.second);
            if (!cell) {
                break;
            }
            if (previous_cell && *previous_cell == *cell) {
                continue;
            }
            previous_cell = cell;

            const int value =
                map_.data[index(cell->first, cell->second, width)];
            if (isOccupiedValue(value)) {
                break;
            }
            clear_distance = distance;
            if (isFreeValue(value) && *cell != *start) {
                ++free_cells;
            }
        }

        if (free_cells > best_free_cells ||
            (free_cells == best_free_cells &&
             clear_distance > best_clear_distance)) {
            best_free_cells = free_cells;
            best_clear_distance = clear_distance;
            best_yaw = yaw;
        }
    }

    if (best_free_cells <= 0) {
        return std::nullopt;
    }
    return best_yaw;
}

ObstacleNeighborhood FrontierExplorerCore::analyzeObstacleNeighborhood(
    const Cell& cell, const int width, const int height) const {
    const double search_radius_m =
        std::max(config_.wall_clearance_m, config_.min_wall_clearance_m);
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

std::optional<PathProjection> FrontierExplorerCore::projectOntoPath(
    const WorldPoint& point, const std::vector<Cell>& path) const {
    if (path.empty()) {
        return std::nullopt;
    }
    if (path.size() == 1) {
        const auto xy =
            mapToWorldUnchecked(path.front().first, path.front().second);
        return PathProjection{
            std::hypot(point.first - xy.first, point.second - xy.second), 0.0};
    }

    std::optional<PathProjection> best;
    double arc_before = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const auto a =
            mapToWorldUnchecked(path[i - 1].first, path[i - 1].second);
        const auto b = mapToWorldUnchecked(path[i].first, path[i].second);
        const double dx = b.first - a.first;
        const double dy = b.second - a.second;
        const double len_sq = dx * dx + dy * dy;
        if (len_sq <= 1e-12) {
            continue;
        }
        const double len = std::sqrt(len_sq);
        const double raw_t =
            ((point.first - a.first) * dx + (point.second - a.second) * dy) /
            len_sq;
        const double t = std::clamp(raw_t, 0.0, 1.0);
        const double px = a.first + t * dx;
        const double py = a.second + t * dy;
        const double dist = std::hypot(point.first - px, point.second - py);
        const double arc = arc_before + t * len;
        if (!best || dist < best->distance ||
            (std::abs(dist - best->distance) <= 1e-9 &&
             arc < best->arc_length)) {
            best = PathProjection{dist, arc, i};
        }
        arc_before += len;
    }
    return best;
}

std::vector<FrontierCandidate> FrontierExplorerCore::buildSubgoalsForRoute(
    const FrontierCandidate& final_candidate,
    const std::vector<FrontierCandidate>& candidates) const {
    if (!config_.enable_subgoals || config_.max_subgoals <= 0 ||
        final_candidate.path.empty()) {
        return {};
    }

    struct ProjectedCandidate
    {
        FrontierCandidate candidate;
        double arc_length = 0.0;
    };
    std::vector<ProjectedCandidate> accepted;
    const auto spaced_from_accepted = [&](const WorldPoint& xy) {
        if (std::hypot(xy.first - final_candidate.goal_xy.first,
                       xy.second - final_candidate.goal_xy.second) <
            config_.min_subgoal_spacing_m) {
            return false;
        }
        return std::all_of(
            accepted.begin(), accepted.end(), [&](const auto& subgoal) {
                return std::hypot(
                           xy.first - subgoal.candidate.goal_xy.first,
                           xy.second - subgoal.candidate.goal_xy.second) >=
                       config_.min_subgoal_spacing_m;
            });
    };

    for (const auto& candidate : candidates) {
        if (accepted.size() >= static_cast<std::size_t>(config_.max_subgoals)) {
            break;
        }
        if (candidate.score < final_candidate.score) {
            continue;
        }
        if (std::hypot(candidate.goal_xy.first - final_candidate.goal_xy.first,
                       candidate.goal_xy.second -
                           final_candidate.goal_xy.second) < 1e-6) {
            continue;
        }
        if (isBlacklisted(candidate.goal_xy)) {
            continue;
        }
        const auto projection =
            projectOntoPath(candidate.goal_xy, final_candidate.path);
        if (!projection ||
            projection->distance > config_.subgoal_path_tolerance_m) {
            continue;
        }
        if (!spaced_from_accepted(candidate.goal_xy)) {
            continue;
        }
        accepted.push_back({candidate, projection->arc_length});
    }

    std::sort(accepted.begin(), accepted.end(),
              [](const auto& a, const auto& b) {
                  return a.arc_length < b.arc_length;
              });
    std::vector<FrontierCandidate> subgoals;
    subgoals.reserve(accepted.size());
    for (auto& subgoal : accepted) {
        subgoals.push_back(std::move(subgoal.candidate));
    }
    return subgoals;
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

bool FrontierExplorerCore::isBlacklisted(const WorldPoint& goal_xy) const {
    const auto index = findGoalAttemptIndex(goal_xy);
    if (!index) {
        return false;
    }
    return goal_attempts_[*index].failures >= config_.failed_goal_max_retries;
}

void FrontierExplorerCore::blacklistGoal(const WorldPoint& goal_xy,
                                         const double now_sec) {
    recordGoalFailure(goal_xy, now_sec, 1);
}

void FrontierExplorerCore::recordGoalFailure(const WorldPoint& goal_xy,
                                             const double now_sec,
                                             const int failure_weight) {
    const int weight = std::max(1, failure_weight);
    const double expires_at = now_sec + config_.failed_goal_blacklist_sec;
    const auto existing = findGoalAttemptIndex(goal_xy);
    if (!existing) {
        goal_attempts_.push_back({goal_xy, weight, expires_at});
        return;
    }
    auto& record = goal_attempts_[*existing];
    record.failures += weight;
    record.expires_at_sec = expires_at;
}

void FrontierExplorerCore::pruneBlacklist(const double now_sec) {
    goal_attempts_.erase(
        std::remove_if(goal_attempts_.begin(), goal_attempts_.end(),
                       [now_sec](const auto& record) {
                           return record.expires_at_sec <= now_sec;
                       }),
        goal_attempts_.end());
}

std::optional<std::size_t> FrontierExplorerCore::findGoalAttemptIndex(
    const WorldPoint& goal_xy) const {
    for (std::size_t i = 0; i < goal_attempts_.size(); ++i) {
        const auto& record = goal_attempts_[i];
        if (std::hypot(goal_xy.first - record.xy.first,
                       goal_xy.second - record.xy.second) <=
            config_.failed_goal_blacklist_radius_m) {
            return i;
        }
    }
    return std::nullopt;
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

std::optional<double> FrontierExplorerCore::pathTargetYawNear(
    const WorldPoint& robot_xy, const std::vector<Cell>& path) const {
    if (path.size() < 2) {
        return std::nullopt;
    }
    const auto projection = projectOntoPath(robot_xy, path);
    if (!projection || projection->segment == 0) {
        return std::nullopt;
    }
    const auto a = mapToWorldUnchecked(path[projection->segment - 1].first,
                                       path[projection->segment - 1].second);
    const auto b = mapToWorldUnchecked(path[projection->segment].first,
                                       path[projection->segment].second);
    return yawBetween(a, b);
}

AlignmentGraceStatus FrontierExplorerCore::evaluateAlignmentGrace(
    const WorldPoint& robot_xy, const double robot_yaw,
    const std::optional<double> previous_robot_yaw,
    const std::vector<Cell>& path, const WorldPoint& active_goal_xy,
    const double elapsed_since_waypoint_sent_sec) const {
    AlignmentGraceStatus status;
    if (!config_.alignment_grace_enabled ||
        config_.alignment_max_grace_sec <= 0.0) {
        return status;
    }

    const auto target_yaw = pathTargetYawNear(robot_xy, path);
    status.target_yaw =
        target_yaw ? *target_yaw : yawBetween(robot_xy, active_goal_xy);
    status.heading_error = headingDelta(robot_yaw, status.target_yaw);
    status.previous_heading_error =
        previous_robot_yaw
            ? headingDelta(*previous_robot_yaw, status.target_yaw)
            : status.heading_error;
    status.heading_error_delta =
        status.previous_heading_error - status.heading_error;

    if (elapsed_since_waypoint_sent_sec >= config_.alignment_max_grace_sec) {
        return status;
    }
    if (status.heading_error <= config_.alignment_heading_threshold_rad) {
        return status;
    }
    if (!previous_robot_yaw ||
        status.heading_error_delta < config_.alignment_min_rotation_delta_rad) {
        return status;
    }

    status.active = true;
    return status;
}

double FrontierExplorerCore::pathInitialYaw(const std::vector<Cell>& path,
                                            const WorldPoint& robot_xy,
                                            const WorldPoint& goal_xy) const {
    if (path.size() >= 2) {
        const auto next_xy = mapToWorldUnchecked(path[1].first, path[1].second);
        return yawBetween(robot_xy, next_xy);
    }
    return yawBetween(robot_xy, goal_xy);
}

double FrontierExplorerCore::pathGoalYaw(const std::vector<Cell>& path,
                                         const WorldPoint& robot_xy,
                                         const WorldPoint& goal_xy) const {
    if (path.size() >= 2) {
        const auto prev_xy = mapToWorldUnchecked(path[path.size() - 2].first,
                                                 path[path.size() - 2].second);
        return yawBetween(prev_xy, goal_xy);
    }
    return yawBetween(robot_xy, goal_xy);
}

std::vector<Cell> FrontierExplorerCore::neighborCells(const int x,
                                                      const int y) const {
    return {{x - 1, y},     {x + 1, y},     {x, y - 1},     {x, y + 1},
            {x - 1, y - 1}, {x + 1, y - 1}, {x - 1, y + 1}, {x + 1, y + 1}};
}

int FrontierExplorerCore::index(const int x, const int y,
                                const int width) const {
    return y * width + x;
}

bool FrontierExplorerCore::inBounds(const int x, const int y, const int width,
                                    const int height) const {
    return 0 <= x && x < width && 0 <= y && y < height;
}

double FrontierExplorerCore::euclideanCells(const Cell& a,
                                            const Cell& b) const {
    return std::hypot(a.first - b.first, a.second - b.second);
}

double FrontierExplorerCore::mapResolution() const {
    return map_.info.resolution;
}

WorldPoint FrontierExplorerCore::mapToWorld(const int mx, const int my) const {
    if (!inBounds(mx, my, static_cast<int>(map_.info.width),
                  static_cast<int>(map_.info.height))) {
        throw std::out_of_range("map cell out of bounds");
    }
    return mapToWorldUnchecked(mx, my);
}

WorldPoint FrontierExplorerCore::mapToWorldUnchecked(const int mx,
                                                     const int my) const {
    return {map_.info.origin.position.x + (mx + 0.5) * map_.info.resolution,
            map_.info.origin.position.y + (my + 0.5) * map_.info.resolution};
}

std::optional<Cell> FrontierExplorerCore::worldToMap(const double wx,
                                                     const double wy) const {
    const int mx = static_cast<int>((wx - map_.info.origin.position.x) /
                                    map_.info.resolution);
    const int my = static_cast<int>((wy - map_.info.origin.position.y) /
                                    map_.info.resolution);
    if (!inBounds(mx, my, static_cast<int>(map_.info.width),
                  static_cast<int>(map_.info.height))) {
        return std::nullopt;
    }
    return Cell{mx, my};
}

WorldPoint FrontierExplorerCore::approachPointForPath(
    const std::vector<Cell>& path, const WorldPoint& goal_xy) const {
    if (path.empty() || config_.approach_offset_m <= 0.0) {
        return goal_xy;
    }

    double remaining = config_.approach_offset_m;
    WorldPoint previous = goal_xy;
    for (std::size_t i = path.size(); i-- > 0;) {
        const auto current = mapToWorldUnchecked(path[i].first, path[i].second);
        const double segment = std::hypot(previous.first - current.first,
                                          previous.second - current.second);
        if (segment >= remaining && segment > 1e-9) {
            const double ratio = remaining / segment;
            return {
                previous.first + (current.first - previous.first) * ratio,
                previous.second + (current.second - previous.second) * ratio};
        }
        remaining -= segment;
        previous = current;
    }
    return mapToWorldUnchecked(path.front().first, path.front().second);
}

FootprintGeometry FrontierExplorerCore::footprintGeometry() {
    const double resolution = mapResolution();
    if (footprint_geometry_ && footprint_geometry_->resolution == resolution) {
        return *footprint_geometry_;
    }

    FootprintGeometry geometry;
    geometry.front = config_.footprint_front_m + config_.footprint_padding_m;
    geometry.rear = config_.footprint_rear_m + config_.footprint_padding_m;
    geometry.half_width =
        config_.footprint_half_width_m + config_.footprint_padding_m;
    geometry.resolution = resolution;
    geometry.radius_cells = std::max(
        1, static_cast<int>(std::ceil(
               std::max({geometry.front, geometry.rear, geometry.half_width}) /
               resolution)) +
               1);
    footprint_geometry_ = geometry;
    return geometry;
}

}  // namespace yahboom_alc_bringup
