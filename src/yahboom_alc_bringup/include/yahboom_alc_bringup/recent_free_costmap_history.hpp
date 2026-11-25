#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>

namespace yahboom_alc_bringup
{

// A rolling local costmap reclassifies a cell as unknown the moment it falls
// out of the active observation window (outside the depth camera's current
// FOV, or past the obstacle layer's persistence horizon) even though it was
// confirmed clear moments ago. Safety checks that fail closed on unknown
// cells (e.g. blind-reverse) can't otherwise tell that apart from a cell
// that has genuinely never been observed. This tracker remembers, per world
// cell, the last time it was seen clearly free, so a caller can distinguish
// the two.
class RecentFreeCostmapHistory
{
public:
    // Scans every free cell in costmap and records now_sec against it.
    // free_cost_max mirrors kFootprintFreeCostMax: a
    // cell counts as free when 0 <= value <= free_cost_max. Returns false and
    // records nothing for an empty or malformed grid (data length not
    // width * height, or non-positive resolution).
    bool observe(const nav_msgs::msg::OccupancyGrid& costmap, double now_sec,
                 int free_cost_max);

    // True when the world cell at (world_x, world_y) was last observed free
    // within grace_sec of now_sec. False (fail closed, matching the default
    // unknown-is-blocked behavior) if it was never observed free at all, or
    // if grace_sec <= 0.
    bool wasRecentlyFree(double world_x, double world_y, double now_sec,
                         double grace_sec) const;

private:
    using CellKey = std::pair<std::int64_t, std::int64_t>;
    struct CellKeyHash
    {
        std::size_t operator()(const CellKey& key) const {
            return std::hash<std::int64_t>()(key.first) ^
                   (std::hash<std::int64_t>()(key.second) << 1);
        }
    };

    double resolution_ = 0.0;
    std::unordered_map<CellKey, double, CellKeyHash> last_free_sec_;
};

}  // namespace yahboom_alc_bringup
