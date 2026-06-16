#pragma once

#include <cmath>

namespace yahboom_alc_bringup
{

inline double rounded(double value, int precision = 3) {
    const double scale = std::pow(10.0, precision);
    return std::round(value * scale) / scale;
}

}  // namespace yahboom_alc_bringup
