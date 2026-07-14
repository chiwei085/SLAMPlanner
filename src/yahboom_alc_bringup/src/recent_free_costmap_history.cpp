#include "yahboom_alc_bringup/recent_free_costmap_history.hpp"

namespace yahboom_alc_bringup
{

namespace
{

std::int64_t floorDiv(const double value, const double resolution) {
    return static_cast<std::int64_t>(std::floor(value / resolution));
}

// Entries older than this are worthless to any plausible caller-supplied
// grace period; drop them opportunistically so a long-running session
// doesn't grow the map without bound.
constexpr double kPruneHorizonSec = 30.0;

}  // namespace

void RecentFreeCostmapHistory::observe(
    const nav_msgs::msg::OccupancyGrid& costmap, const double now_sec,
    const int free_cost_max) {
    const double resolution = costmap.info.resolution;
    if (resolution <= 0.0 || costmap.data.empty()) {
        return;
    }
    resolution_ = resolution;
    const double origin_x = costmap.info.origin.position.x;
    const double origin_y = costmap.info.origin.position.y;
    const auto width = costmap.info.width;
    const auto height = costmap.info.height;
    for (std::uint32_t my = 0; my < height; ++my) {
        for (std::uint32_t mx = 0; mx < width; ++mx) {
            const auto index = static_cast<std::size_t>(my) * width + mx;
            const int value = costmap.data[index];
            if (value < 0 || value > free_cost_max) {
                continue;
            }
            const double world_x = origin_x + (mx + 0.5) * resolution;
            const double world_y = origin_y + (my + 0.5) * resolution;
            last_free_sec_[{floorDiv(world_x, resolution),
                            floorDiv(world_y, resolution)}] = now_sec;
        }
    }
    for (auto it = last_free_sec_.begin(); it != last_free_sec_.end();) {
        if (now_sec - it->second > kPruneHorizonSec) {
            it = last_free_sec_.erase(it);
        }
        else {
            ++it;
        }
    }
}

bool RecentFreeCostmapHistory::wasRecentlyFree(const double world_x,
                                               const double world_y,
                                               const double now_sec,
                                               const double grace_sec) const {
    if (resolution_ <= 0.0 || grace_sec <= 0.0) {
        return false;
    }
    const auto it = last_free_sec_.find(
        {floorDiv(world_x, resolution_), floorDiv(world_y, resolution_)});
    if (it == last_free_sec_.end()) {
        return false;
    }
    return now_sec - it->second <= grace_sec;
}

}  // namespace yahboom_alc_bringup
