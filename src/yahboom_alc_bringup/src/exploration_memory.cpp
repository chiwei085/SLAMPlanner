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
namespace
{
constexpr double kHazardBoundaryEpsilonM = 1e-9;
// A traversal only counts as "safe" once the robot got this close (relative
// to the region radius) to the recorded hazard point -- otherwise skirting
// the boundary back and forth could rack up the same crossing distance
// without ever revisiting the spot that actually failed.
constexpr double kHazardSafeTraversalCenterProximityFactor = 0.5;
}  // namespace

namespace
{
long long visitKey(const long qx, const long qy) {
    // qx/qy are world metres / kVisitQuantM, bounded to a small map, so this
    // linear combine is collision-free for the relevant coordinate range.
    return qx * 1000000LL + qy;
}
}  // namespace

void FrontierExplorerCore::noteRobotVisited(const WorldPoint& robot_xy) {
    const long qx = std::lround(robot_xy.first / kVisitQuantM);
    const long qy = std::lround(robot_xy.second / kVisitQuantM);
    visited_keys_.insert(visitKey(qx, qy));
}

bool FrontierExplorerCore::isUnderObserved(const WorldPoint& xy) const {
    if (visited_keys_.empty()) {
        return true;
    }
    const long qx = std::lround(xy.first / kVisitQuantM);
    const long qy = std::lround(xy.second / kVisitQuantM);
    const int radius =
        std::max(1, static_cast<int>(std::ceil(config_.coverage_proximity_m /
                                               kVisitQuantM)));
    const double proximity_sq =
        config_.coverage_proximity_m * config_.coverage_proximity_m;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (!visited_keys_.contains(visitKey(qx + dx, qy + dy))) {
                continue;
            }
            const double wx = static_cast<double>(qx + dx) * kVisitQuantM;
            const double wy = static_cast<double>(qy + dy) * kVisitQuantM;
            const double dist_sq = (wx - xy.first) * (wx - xy.first) +
                                   (wy - xy.second) * (wy - xy.second);
            if (dist_sq <= proximity_sq) {
                return false;
            }
        }
    }
    return true;
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
                                             const double now_sec,
                                             const double path_length) {
    last_goal_ = goal_xy;
    last_goal_yaw_ = goal_yaw;
    last_goal_time_sec_ = now_sec;
    if (config_.intention_distance_threshold_m > 0.0 &&
        path_length >= config_.intention_distance_threshold_m) {
        intention_ = IntentionState{goal_xy, now_sec};
    }
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
    std::erase_if(goal_attempts_, [now_sec](const auto& record) {
        return record.expires_at_sec <= now_sec;
    });
}

bool FrontierExplorerCore::isInHazardRegion(const WorldPoint& xy) const {
    if (!config_.hazard_region_enabled) {
        return false;
    }
    return std::any_of(hazard_regions_.begin(), hazard_regions_.end(),
                       [&xy](const auto& region) {
                           return std::hypot(xy.first - region.xy.first,
                                             xy.second - region.xy.second) <=
                                  region.radius + kHazardBoundaryEpsilonM;
                       });
}

double FrontierExplorerCore::hazardPathPenalty(
    const std::vector<Cell>& path) const {
    if (!config_.hazard_region_enabled || path.empty()) {
        return 0.0;
    }
    double penalty = 0.0;
    for (const auto& region : hazard_regions_) {
        const bool intersects =
            std::any_of(path.begin(), path.end(), [&](const Cell& cell) {
                const auto xy = mapToWorldUnchecked(cell.first, cell.second);
                return std::hypot(xy.first - region.xy.first,
                                  xy.second - region.xy.second) <=
                       region.radius + kHazardBoundaryEpsilonM;
            });
        if (intersects) {
            penalty += config_.hazard_region_path_penalty * region.severity;
        }
    }
    return penalty;
}

std::optional<HazardRegionUpdate> FrontierExplorerCore::recordHazardRegion(
    const WorldPoint& robot_xy, const double now_sec, const int severity) {
    if (!config_.hazard_region_enabled) {
        return std::nullopt;
    }
    const int clamped_severity = std::clamp(severity, 1, 2);
    const auto radius = [this](const int value) {
        return value >= 2 ? config_.hazard_region_severity2_radius_m
                          : config_.hazard_region_severity1_radius_m;
    };
    auto existing = std::find_if(
        hazard_regions_.begin(), hazard_regions_.end(),
        [&robot_xy](const auto& region) {
            return std::hypot(robot_xy.first - region.xy.first,
                              robot_xy.second - region.xy.second) <=
                   region.radius + kHazardBoundaryEpsilonM;
        });
    if (existing == hazard_regions_.end()) {
        HazardRegion region;
        region.xy = robot_xy;
        region.severity = clamped_severity;
        region.radius = radius(clamped_severity);
        region.expires_at_sec = now_sec + config_.hazard_region_ttl_sec;
        hazard_regions_.push_back(region);
        return HazardRegionUpdate{region, true};
    }
    existing->severity = std::max(existing->severity, clamped_severity);
    existing->radius = radius(existing->severity);
    existing->expires_at_sec = now_sec + config_.hazard_region_ttl_sec;
    existing->traversal_entry_xy.reset();
    return HazardRegionUpdate{*existing, false};
}

std::vector<HazardRegion> FrontierExplorerCore::noteHazardTraversal(
    const WorldPoint& robot_xy) {
    std::vector<HazardRegion> cleared;
    if (!config_.hazard_region_enabled ||
        !config_.hazard_region_safe_traversal_clear_enabled) {
        return cleared;
    }
    for (auto& region : hazard_regions_) {
        const double distance_to_center =
            std::hypot(robot_xy.first - region.xy.first,
                       robot_xy.second - region.xy.second);
        const bool inside =
            distance_to_center <= region.radius + kHazardBoundaryEpsilonM;
        if (!inside) {
            region.traversal_entry_xy.reset();
            continue;
        }
        if (!region.traversal_entry_xy) {
            region.traversal_entry_xy = robot_xy;
            region.traversal_min_distance_to_center_m = distance_to_center;
        }
        else {
            region.traversal_min_distance_to_center_m = std::min(
                region.traversal_min_distance_to_center_m, distance_to_center);
        }
    }
    std::erase_if(hazard_regions_, [&cleared, &robot_xy](const auto& region) {
        if (!region.traversal_entry_xy) {
            return false;
        }
        // Crossed roughly the full diameter since first entering...
        const double chord =
            std::hypot(robot_xy.first - region.traversal_entry_xy->first,
                       robot_xy.second - region.traversal_entry_xy->second);
        const bool crossed_diameter =
            chord >= 2.0 * region.radius - kHazardBoundaryEpsilonM;
        // ...and actually passed near the point that failed, not
        // just along the boundary the whole time.
        const bool passed_near_center =
            region.traversal_min_distance_to_center_m <=
            region.radius * kHazardSafeTraversalCenterProximityFactor;
        if (!crossed_diameter || !passed_near_center) {
            return false;
        }
        cleared.push_back(region);
        return true;
    });
    return cleared;
}

std::vector<HazardRegion> FrontierExplorerCore::pruneHazardRegions(
    const double now_sec) {
    std::vector<HazardRegion> expired;
    std::erase_if(hazard_regions_, [now_sec, &expired](const auto& region) {
        if (region.expires_at_sec > now_sec) {
            return false;
        }
        expired.push_back(region);
        return true;
    });
    return expired;
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

}  // namespace yahboom_alc_bringup
