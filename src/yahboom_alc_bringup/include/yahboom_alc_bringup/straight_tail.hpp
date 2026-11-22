#pragma once

#include <optional>
#include <vector>

namespace yahboom_alc_bringup
{

// A view goal (x, y, yaw) is reached along a straight tail that ends at the
// goal and points along its yaw. Pure pursuit (RPP) converges onto a straight
// line and the vehicle heading on that line is the line direction, so a stop
// anywhere on the tail faces the goal yaw. The R2 cannot hold a heading any
// other way: it cannot turn in place and RPP has no heading control at the
// path end.
struct TailPose
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// Steering-feasible bearing of the pure pursuit carrot. Pure pursuit commands
// curvature 2 sin(alpha) / L_d for a carrot at bearing alpha and distance L_d,
// so the command stays within the vehicle limit iff
// |sin(alpha)| <= max_curvature * L_d / 2.
double maxCarrotBearing(double lookahead_m, double max_curvature);

// Distance along the goal yaw from the goal back to the robot's foot point on
// the tail line (<= 0 before the goal), when pure pursuit is already captured
// by the line: the carrot (the line point at L_d from the robot, ahead of the
// foot point) exists, lies at or before the goal, and is within
// maxCarrotBearing of the robot heading. Otherwise nullopt.
std::optional<double> tailCaptureAlong(const TailPose& robot,
                                       const TailPose& goal, double lookahead_m,
                                       double max_curvature);

// Poses from `along_from` to `along_to` (signed distances along the goal yaw,
// goal at 0), both ends included, spaced at most `step_m`, all with the goal
// yaw.
std::vector<TailPose> tailPoses(const TailPose& goal, double along_from,
                                double along_to, double step_m);

}  // namespace yahboom_alc_bringup
