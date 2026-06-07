#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_set>

namespace yahboom_alc_bringup
{

namespace
{

int signBucket(const int value) {
    return value >= 0 ? 1 : -1;
}

}  // namespace

double FootprintCheck::unknownRatio() const {
    return total <= 0 ? 1.0 : static_cast<double>(unknown) / total;
}

FrontierExplorerCore::FrontierExplorerCore(FrontierConfig config)
    : config_(std::move(config)) {}

void FrontierExplorerCore::setConfig(const FrontierConfig& config) {
    config_ = config;
    footprint_geometry_.reset();
}

void FrontierExplorerCore::setMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_ = map;
    has_map_ = true;
    footprint_geometry_.reset();
}

CandidateResult FrontierExplorerCore::findCandidates(const WorldPoint& robot_xy,
                                                     const double now_sec) {
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
    const auto blocked =
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
        auto candidate = makeCandidate(cluster, *start_cell, robot_xy, blocked,
                                       width, height);
        if (!candidate) {
            continue;
        }
        const double dist =
            std::hypot(candidate->goal_xy.first - robot_xy.first,
                       candidate->goal_xy.second - robot_xy.second);
        if (dist < config_.min_frontier_distance) {
            continue;
        }
        if (config_.max_frontier_distance > 0.0 &&
            dist > config_.max_frontier_distance) {
            continue;
        }
        result.candidates.push_back(std::move(*candidate));
    }
    return result;
}

std::optional<FrontierCandidate> FrontierExplorerCore::makeCandidate(
    const std::vector<Cell>& cluster, const Cell& start_cell,
    const WorldPoint& robot_xy, const std::vector<bool>& blocked,
    const int width, const int height) {
    const auto goal_options =
        goalOptionsForCluster(cluster, blocked, width, height);
    if (goal_options.empty()) {
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
            continue;
        }

        auto path = gridPath(start_cell, goal_cell, blocked, width, height);
        const bool reachable = path.has_value();
        if (!reachable && !config_.allow_unreachable_goal) {
            continue;
        }
        if (!path) {
            path = std::vector<Cell>{start_cell, goal_cell};
        }

        const double goal_yaw = pathGoalYaw(*path, robot_xy, goal_xy);
        const auto footprint =
            footprintCheckAtWorld(goal_xy.first, goal_xy.second, goal_yaw);
        if (footprint.occupied) {
            continue;
        }
        const auto swept_unknown_ratio = sweptPathUnknownRatio(*path);
        if (!swept_unknown_ratio) {
            continue;
        }

        const double path_length = pathLength(*path);
        const int information = countUnknownNear(goal_cell);
        const auto obstacle =
            analyzeObstacleNeighborhood(goal_cell, width, height);
        const double progress = corridorProgress(start_xy, goal_xy, axis);
        const double unknown_ratio =
            std::max(footprint.unknownRatio(), *swept_unknown_ratio);
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
            score += config_.unreachable_goal_penalty;
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
        candidate.unknown_overlap_ratio = unknown_ratio;
        candidate.wall_clearance = obstacle.wall_clearance;
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
    std::set<Cell> frontier_set(frontier_cells.begin(), frontier_cells.end());
    std::set<Cell> visited;
    std::vector<std::vector<Cell>> clusters;
    for (const auto& seed : frontier_cells) {
        if (visited.count(seed) != 0) {
            continue;
        }
        std::vector<Cell> cluster;
        std::queue<Cell> queue;
        queue.push(seed);
        visited.insert(seed);
        while (!queue.empty()) {
            const auto cell = queue.front();
            queue.pop();
            cluster.push_back(cell);
            for (const auto& neighbor :
                 neighborCells(cell.first, cell.second)) {
                (void)width;
                if (frontier_set.count(neighbor) != 0 &&
                    visited.count(neighbor) == 0) {
                    visited.insert(neighbor);
                    queue.push(neighbor);
                }
            }
        }
        clusters.push_back(std::move(cluster));
    }
    return clusters;
}

std::vector<Cell> FrontierExplorerCore::goalOptionsForCluster(
    const std::vector<Cell>& cluster, const std::vector<bool>& blocked,
    const int width, const int height) const {
    const int radius_cells =
        std::max(1, static_cast<int>(config_.frontier_goal_search_radius /
                                     mapResolution()));
    std::set<Cell> options;
    for (const auto& frontier : cluster) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const int x = frontier.first + dx;
                const int y = frontier.second + dy;
                if (!inBounds(x, y, width, height)) {
                    continue;
                }
                if (std::hypot(dx, dy) > radius_cells) {
                    continue;
                }
                if (blocked[index(x, y, width)]) {
                    continue;
                }
                if (hasOccupiedClearance(x, y, width, height,
                                         config_.min_clearance_cells)) {
                    options.emplace(x, y);
                }
            }
        }
    }
    return {options.begin(), options.end()};
}

std::vector<bool> FrontierExplorerCore::makeBlockedGrid(
    const int width, const int height, const int clearance) const {
    std::vector<bool> blocked(map_.data.size(), false);
    for (std::size_t i = 0; i < map_.data.size(); ++i) {
        blocked[i] = isOccupiedValue(map_.data[i]);
    }
    if (clearance <= 0) {
        return blocked;
    }
    auto inflated = blocked;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!blocked[index(x, y, width)]) {
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
    const Cell& origin, const std::vector<bool>& blocked, const int width,
    const int height, const double radius_m) const {
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
    const Cell& start, const Cell& goal, const std::vector<bool>& blocked,
    const int width, const int height) const {
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
    int count = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int x = cell.first + dx;
            const int y = cell.second + dy;
            if (!inBounds(x, y, width, height)) {
                continue;
            }
            if (std::hypot(dx, dy) > radius_cells) {
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

ObstacleNeighborhood FrontierExplorerCore::analyzeObstacleNeighborhood(
    const Cell& cell, const int width, const int height) const {
    const int radius_cells =
        std::max(1, static_cast<int>(
                        std::ceil(config_.wall_clearance_m / mapResolution())));
    double best = std::numeric_limits<double>::infinity();
    std::set<std::pair<int, int>> occupied_quadrants;
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
            occupied_quadrants.emplace(signBucket(dx), signBucket(dy));
        }
    }
    ObstacleNeighborhood result;
    result.wall_clearance = std::isinf(best) ? config_.wall_clearance_m : best;
    result.corner_penalty =
        std::max(0.0, static_cast<double>(occupied_quadrants.size()) - 1.0);
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

bool FrontierExplorerCore::isBlacklisted(const WorldPoint& goal_xy) const {
    const auto index = findGoalAttemptIndex(goal_xy);
    if (!index) {
        return false;
    }
    return goal_attempts_[*index].failures >= config_.failed_goal_max_retries;
}

void FrontierExplorerCore::blacklistGoal(const WorldPoint& goal_xy,
                                         const double now_sec) {
    const double expires_at = now_sec + config_.failed_goal_blacklist_sec;
    const auto existing = findGoalAttemptIndex(goal_xy);
    if (!existing) {
        goal_attempts_.push_back({goal_xy, 1, expires_at});
        return;
    }
    auto& record = goal_attempts_[*existing];
    record.xy = goal_xy;
    ++record.failures;
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

double FrontierExplorerCore::pathGoalYaw(const std::vector<Cell>& path,
                                         const WorldPoint& robot_xy,
                                         const WorldPoint& goal_xy) const {
    if (path.size() >= 2) {
        const auto prev_xy = mapToWorldUnchecked(path[path.size() - 2].first,
                                                 path[path.size() - 2].second);
        return std::atan2(goal_xy.second - prev_xy.second,
                          goal_xy.first - prev_xy.first);
    }
    return std::atan2(goal_xy.second - robot_xy.second,
                      goal_xy.first - robot_xy.first);
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
