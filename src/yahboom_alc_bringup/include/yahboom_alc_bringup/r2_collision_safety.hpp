#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rik_sd/struct_descriptor.hpp>

#include <deque>
#include <string>

namespace yahboom_alc_bringup
{

struct R2CollisionSafetyConfig
{
    std::string input_topic = "/alc/nav_cmd_vel";
    std::string imu_topic = "/imu/data";
    std::string planner_event_topic = "/planner/events";
    std::string safety_stop_topic = "/collision_safety_stop_active";
    double command_vx_threshold_mps = 0.01;
    double imu_stale_timeout_sec = 0.5;
    int imu_baseline_samples = 20;
    double imu_impact_decel_threshold_mps2 = 1.5;
    double impact_stop_hold_sec = 0.5;
    double impact_cooldown_sec = 3.0;
};

// IMU-only collision reflex. impact_triggered is the rising edge (a fresh
// impact just registered, subject to cooldown); stop_hold_active stays true for
// the post-impact hold window during which forward motion must be suppressed.
struct ImpactDecision
{
    bool impact_triggered = false;
    bool stop_hold_active = false;
};

class CollisionSafetyCore
{
public:
    explicit CollisionSafetyCore(R2CollisionSafetyConfig config = {});

    void setConfig(const R2CollisionSafetyConfig& config);
    void reset();

    // Detects a forward-deceleration impact from the IMU and manages the
    // post-impact stop-hold window. Smooth obstacle braking is handled
    // upstream by the Nav2 collision monitor, not here.
    ImpactDecision detectImpact(double time_sec,
                                const geometry_msgs::msg::Twist& command,
                                bool imu_valid, double imu_forward_accel);

private:
    R2CollisionSafetyConfig config_;
    double impact_hold_until_sec_ = -1.0e9;
    double last_impact_sec_ = -1.0e9;
    std::deque<double> imu_baseline_;
};

}  // namespace yahboom_alc_bringup

namespace rik_sd
{

template <>
struct descriptor<yahboom_alc_bringup::R2CollisionSafetyConfig>
{
    static constexpr auto fields() {
        using yahboom_alc_bringup::R2CollisionSafetyConfig;
        return make_descriptor(
            named(&R2CollisionSafetyConfig::input_topic, "input_topic"),
            named(&R2CollisionSafetyConfig::imu_topic, "imu_topic"),
            named(&R2CollisionSafetyConfig::planner_event_topic,
                  "planner_event_topic"),
            named(&R2CollisionSafetyConfig::safety_stop_topic,
                  "safety_stop_topic"),
            named(&R2CollisionSafetyConfig::command_vx_threshold_mps,
                  "command_vx_threshold_mps"),
            named(&R2CollisionSafetyConfig::imu_stale_timeout_sec,
                  "imu_stale_timeout_sec"),
            named(&R2CollisionSafetyConfig::imu_baseline_samples,
                  "imu_baseline_samples"),
            named(&R2CollisionSafetyConfig::imu_impact_decel_threshold_mps2,
                  "imu_impact_decel_threshold_mps2"),
            named(&R2CollisionSafetyConfig::impact_stop_hold_sec,
                  "impact_stop_hold_sec"),
            named(&R2CollisionSafetyConfig::impact_cooldown_sec,
                  "impact_cooldown_sec"));
    }
};

}  // namespace rik_sd
