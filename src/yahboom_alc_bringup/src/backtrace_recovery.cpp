#include "yahboom_alc_bringup/backtrace_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace yahboom_alc_bringup
{

double distanceBetween(const WorldPoint& a, const WorldPoint& b) {
    return std::hypot(a.first - b.first, a.second - b.second);
}

bool withinDistanceTolerance(const WorldPoint& a, const WorldPoint& b,
                             const double tolerance_m) {
    return distanceBetween(a, b) <= tolerance_m;
}

BreadcrumbTrail::BreadcrumbTrail(BacktraceRecoveryConfig config)
    : config_(std::move(config)) {}

void BreadcrumbTrail::setConfig(const BacktraceRecoveryConfig& config) {
    config_ = config;
    prune();
}

void BreadcrumbTrail::clear() {
    poses_.clear();
}

void BreadcrumbTrail::add(const BreadcrumbPose& pose) {
    if (!poses_.empty() && distanceBetween(poses_.back().xy, pose.xy) < 0.01) {
        poses_.back() = pose;
    }
    else {
        poses_.push_back(pose);
    }
    prune();
}

void BreadcrumbTrail::prune() {
    if (poses_.empty()) {
        return;
    }
    const double newest_time = poses_.back().time_sec;
    while (poses_.size() > 1 && newest_time - poses_.front().time_sec >
                                    config_.breadcrumb_max_age_sec) {
        poses_.erase(poses_.begin());
    }

    double retained_distance = 0.0;
    std::size_t keep_from = poses_.size() - 1;
    for (std::size_t i = poses_.size() - 1; i > 0; --i) {
        retained_distance += distanceBetween(poses_[i].xy, poses_[i - 1].xy);
        if (retained_distance > config_.breadcrumb_max_distance_m) {
            break;
        }
        keep_from = i - 1;
    }
    if (keep_from > 0) {
        poses_.erase(poses_.begin(), poses_.begin() + keep_from);
    }
}

BacktraceRecoveryPlanner::BacktraceRecoveryPlanner(
    BacktraceRecoveryConfig config)
    : config_(std::move(config)) {}

void BacktraceRecoveryPlanner::setConfig(
    const BacktraceRecoveryConfig& config) {
    config_ = config;
}

void BacktraceRecoveryPlanner::reset() {
    state_ = {};
}

bool BacktraceRecoveryPlanner::canStart(const BreadcrumbTrail& trail) const {
    return config_.enabled && trail.poses().size() >= 2 &&
           config_.max_segments > 0 && config_.backup_step_m > 0.0 &&
           config_.backup_speed_mps > 0.0;
}

std::optional<double> BacktraceRecoveryPlanner::nextBackupDistance(
    const BreadcrumbTrail& trail, const WorldPoint& robot_xy) const {
    if (!canStart(trail) || state_.segments_attempted >= config_.max_segments ||
        state_.distance_commanded_m >=
            config_.backup_step_m * config_.max_segments) {
        return std::nullopt;
    }

    const auto& poses = trail.poses();
    double available = 0.0;
    WorldPoint cursor = robot_xy;
    for (auto it = poses.rbegin(); it != poses.rend(); ++it) {
        available += distanceBetween(cursor, it->xy);
        cursor = it->xy;
        if (available >= config_.backup_step_m) {
            break;
        }
    }
    if (available < config_.min_actual_displacement_m) {
        return std::nullopt;
    }

    const double remaining = config_.backup_step_m * config_.max_segments -
                             state_.distance_commanded_m;
    return std::min({config_.backup_step_m, remaining, available});
}

bool BacktraceRecoveryPlanner::noteSegmentStarted(
    const WorldPoint& robot_xy, const double actual_distance) {
    if (state_.segments_attempted >= config_.max_segments) {
        return false;
    }
    state_.segment_start_xy = robot_xy;
    state_.last_segment_commanded_m = actual_distance;
    ++state_.segments_attempted;
    state_.distance_commanded_m += actual_distance;
    return true;
}

BacktraceSegmentResult BacktraceRecoveryPlanner::noteSegmentFinished(
    const WorldPoint& robot_xy) {
    BacktraceSegmentResult result;
    result.commanded_m = state_.last_segment_commanded_m;
    if (!state_.segment_start_xy) {
        return result;
    }
    result.actual_displacement_m =
        distanceBetween(*state_.segment_start_xy, robot_xy);
    result.moved_enough =
        result.actual_displacement_m >= config_.min_actual_displacement_m;
    if (result.commanded_m > 0.0) {
        result.ratio = result.actual_displacement_m / result.commanded_m;
    }
    state_.segment_start_xy.reset();
    state_.last_segment_commanded_m = 0.0;
    return result;
}

}  // namespace yahboom_alc_bringup
