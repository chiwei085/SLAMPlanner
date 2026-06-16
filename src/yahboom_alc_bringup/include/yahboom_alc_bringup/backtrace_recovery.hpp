#pragma once

#include <optional>
#include <string>
#include <vector>

#include "yahboom_alc_bringup/frontier_explorer_core.hpp"

namespace yahboom_alc_bringup
{

struct BacktraceRecoveryConfig
{
    bool enabled = true;
    double breadcrumb_max_distance_m = 2.0;
    double breadcrumb_max_age_sec = 90.0;
    double backup_step_m = 0.10;
    double backup_speed_mps = 0.04;
    double backup_time_allowance_sec = 5.0;
    int max_segments = 4;
    double min_actual_displacement_m = 0.03;
    double cancel_timeout_sec = 1.0;
    double backup_stall_check_sec = 1.5;
    std::string backup_action = "/backup";
};

struct BreadcrumbPose
{
    WorldPoint xy;
    double yaw = 0.0;
    double time_sec = 0.0;
};

class BreadcrumbTrail
{
public:
    explicit BreadcrumbTrail(BacktraceRecoveryConfig config = {});

    void setConfig(const BacktraceRecoveryConfig& config);
    void clear();
    void add(const BreadcrumbPose& pose);
    const std::vector<BreadcrumbPose>& poses() const { return poses_; }

private:
    void prune();

    BacktraceRecoveryConfig config_;
    std::vector<BreadcrumbPose> poses_;
};

struct BacktraceRecoveryState
{
    int segments_attempted = 0;
    double distance_commanded_m = 0.0;
    std::optional<WorldPoint> segment_start_xy;
};

class BacktraceRecoveryPlanner
{
public:
    explicit BacktraceRecoveryPlanner(BacktraceRecoveryConfig config = {});

    void setConfig(const BacktraceRecoveryConfig& config);
    void reset();
    bool canStart(const BreadcrumbTrail& trail) const;
    std::optional<double> nextBackupDistance(const BreadcrumbTrail& trail,
                                             const WorldPoint& robot_xy) const;
    bool noteSegmentStarted(const WorldPoint& robot_xy, double actual_distance);
    bool noteSegmentFinished(const WorldPoint& robot_xy);
    const BacktraceRecoveryState& state() const { return state_; }

private:
    BacktraceRecoveryConfig config_;
    BacktraceRecoveryState state_;
};

double distanceBetween(const WorldPoint& a, const WorldPoint& b);
bool withinDistanceTolerance(const WorldPoint& a, const WorldPoint& b,
                             double tolerance_m);

}  // namespace yahboom_alc_bringup
