#pragma once

#include <cmath>

namespace yahboom_alc_bringup
{

struct DecimalPlaces
{
    explicit DecimalPlaces(const int value_in) : value(value_in) {}
    int value;
};

inline double rounded(const double value, const DecimalPlaces precision) {
    const double scale = std::pow(10.0, precision.value);
    return std::round(value * scale) / scale;
}

inline double rounded(const double value) {
    return rounded(value, DecimalPlaces{3});
}

}  // namespace yahboom_alc_bringup
