#include <algorithm>
#include <bitset>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"
#include "yahboom_alc_bringup/math_util.hpp"

namespace yahboom_alc_bringup
{

double FootprintCheck::unknownRatio() const {
    return total <= 0 ? 1.0 : static_cast<double>(unknown) / total;
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

bool isLethalCostmapCell(const nav_msgs::msg::OccupancyGrid& costmap,
                         const WorldPoint& xy) {
    const double resolution = costmap.info.resolution;
    if (resolution <= 0.0 || costmap.data.empty()) {
        return false;
    }
    const double mx = (xy.first - costmap.info.origin.position.x) / resolution;
    const double my = (xy.second - costmap.info.origin.position.y) / resolution;
    if (mx < 0.0 || my < 0.0 || mx >= costmap.info.width ||
        my >= costmap.info.height) {
        return false;
    }
    const auto index = static_cast<std::size_t>(my) * costmap.info.width +
                       static_cast<std::size_t>(mx);
    constexpr std::int8_t kLethal = 100;
    return index < costmap.data.size() && costmap.data[index] >= kLethal;
}

bool isFootprintBlockedInCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                 const FootprintGeometry& footprint,
                                 const WorldPoint& xy, const double yaw) {
    const double resolution = costmap.info.resolution;
    const int width = static_cast<int>(costmap.info.width);
    const int height = static_cast<int>(costmap.info.height);
    if (resolution <= 0.0 ||
        costmap.data.size() != static_cast<std::size_t>(width) * height) {
        return false;
    }
    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;
    const int cx =
        static_cast<int>(std::floor((xy.first - origin_x) / resolution));
    const int cy =
        static_cast<int>(std::floor((xy.second - origin_y) / resolution));
    if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
        return false;
    }
    constexpr std::int8_t kInscribed = 99;
    constexpr std::int8_t kLethal = 100;
    if (costmap.data[static_cast<std::size_t>(cy) * width + cx] >= kInscribed) {
        return true;
    }
    const int radius =
        static_cast<int>(std::ceil(
            std::max({footprint.front, footprint.rear, footprint.half_width}) /
            resolution)) +
        1;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (int y = std::max(0, cy - radius);
         y <= std::min(height - 1, cy + radius); ++y) {
        for (int x = std::max(0, cx - radius);
             x <= std::min(width - 1, cx + radius); ++x) {
            const double rel_x = origin_x + (x + 0.5) * resolution - xy.first;
            const double rel_y = origin_y + (y + 0.5) * resolution - xy.second;
            const double body_x = cos_yaw * rel_x + sin_yaw * rel_y;
            const double body_y = -sin_yaw * rel_x + cos_yaw * rel_y;
            if (body_x < -footprint.rear || body_x > footprint.front ||
                std::abs(body_y) > footprint.half_width) {
                continue;
            }
            if (costmap.data[static_cast<std::size_t>(y) * width + x] >=
                kLethal) {
                return true;
            }
        }
    }
    return false;
}

std::optional<double> unknownBearing(const nav_msgs::msg::OccupancyGrid& map,
                                     const WorldPoint& xy,
                                     const double robot_yaw,
                                     const double view_half_angle_rad,
                                     const double radius_m, const int min_cells,
                                     const double min_concentration) {
    const double resolution = map.info.resolution;
    if (resolution <= 0.0 || radius_m <= 0.0) {
        return std::nullopt;
    }
    const auto width = static_cast<int>(map.info.width);
    const auto height = static_cast<int>(map.info.height);
    const int r = static_cast<int>(std::ceil(radius_m / resolution));
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;
    // Walk a robot-centred window, not the map: RTAB-Map's /map only spans
    // what has been observed, so space never seen (e.g. right behind a robot
    // that has not moved yet) lies outside the grid and counts as unknown.
    for (int iy = -r; iy <= r; ++iy) {
        for (int ix = -r; ix <= r; ++ix) {
            const double dx = ix * resolution;
            const double dy = iy * resolution;
            const double d = std::hypot(dx, dy);
            if (d > radius_m || d < 1e-6 ||
                std::abs(std::remainder(std::atan2(dy, dx) - robot_yaw,
                                        2.0 * kPi)) <= view_half_angle_rad) {
                continue;
            }
            const int mx = static_cast<int>(std::floor(
                (xy.first + dx - map.info.origin.position.x) / resolution));
            const int my = static_cast<int>(std::floor(
                (xy.second + dy - map.info.origin.position.y) / resolution));
            const bool in_bounds =
                mx >= 0 && my >= 0 && mx < width && my < height &&
                static_cast<std::size_t>(my) * width + mx < map.data.size();
            if (in_bounds &&
                map.data[static_cast<std::size_t>(my) * width + mx] >= 0) {
                continue;
            }
            sum_x += dx / d;
            sum_y += dy / d;
            ++count;
        }
    }
    if (count < min_cells ||
        std::hypot(sum_x, sum_y) < min_concentration * count) {
        return std::nullopt;
    }
    return std::atan2(sum_y, sum_x);
}

}  // namespace yahboom_alc_bringup
