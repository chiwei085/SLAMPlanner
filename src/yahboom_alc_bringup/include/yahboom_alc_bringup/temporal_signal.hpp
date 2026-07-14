#pragma once

#include <optional>

namespace yahboom_alc_bringup
{

// State is deliberately owned by each subscriber.  This header only performs
// the arithmetic so no node becomes a central source of timing truth.
struct TemporalSignalState
{
    std::optional<double> last_receive_sec;
};

struct TemporalSignalObservation
{
    std::optional<double> producer_lag_sec;
    std::optional<double> consume_gap_sec;
    std::optional<double> receive_silence_sec;
};

inline TemporalSignalObservation observeTemporalSignal(
    const std::optional<double> source_sec, const double receive_sec,
    TemporalSignalState& state) {
    TemporalSignalObservation result;
    if (source_sec && *source_sec != 0.0) {
        result.producer_lag_sec = receive_sec - *source_sec;
    }
    if (state.last_receive_sec) {
        result.consume_gap_sec = receive_sec - *state.last_receive_sec;
    }
    state.last_receive_sec = receive_sec;
    result.receive_silence_sec = 0.0;
    return result;
}

inline TemporalSignalObservation temporalSignalAt(
    const TemporalSignalState& state, const double now_sec) {
    TemporalSignalObservation result;
    if (state.last_receive_sec) {
        result.receive_silence_sec = now_sec - *state.last_receive_sec;
    }
    return result;
}

}  // namespace yahboom_alc_bringup
