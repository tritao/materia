#include "clock_estimator6.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robotkit {

ClockEstimator6::ClockEstimator6(std::uint64_t device_tick_hz,
                                 std::uint64_t uncertainty_bound_ns)
    : nominal_rate_(static_cast<double>(device_tick_hz) / 1e9),
      rate_(nominal_rate_), bound_ns_(uncertainty_bound_ns) {}

bool ClockEstimator6::sync_due(std::uint64_t host_now_ns, std::uint64_t period_ns) const {
    return !sent_ || host_now_ns < last_send_ns_ ||
           host_now_ns - last_send_ns_ >= period_ns;
}

void ClockEstimator6::note_sync_request(std::uint64_t host_send_ns) {
    sent_ = true;
    last_send_ns_ = host_send_ns;
}

void ClockEstimator6::observe(std::uint64_t host_send_ns, std::uint64_t host_receive_ns,
                              std::uint64_t device_receive_ticks,
                              std::uint64_t device_send_ticks) {
    if (!nominal_rate_ || host_receive_ns <= host_send_ns ||
        device_send_ticks < device_receive_ticks) return;
    const auto processing_ns = (device_send_ticks - device_receive_ticks) / nominal_rate_;
    const auto rtt = static_cast<double>(host_receive_ns - host_send_ns) - processing_ns;
    if (rtt < 0 || rtt > 2.0 * static_cast<double>(bound_ns_)) return;
    const Sample sample{(static_cast<double>(host_send_ns) + host_receive_ns) * .5,
                        (static_cast<double>(device_receive_ticks) + device_send_ticks) * .5,
                        rtt};
    if (ready_ && std::abs((sample.device_ticks - (offset_ + rate_ * sample.host_ns)) / rate_) >
                      static_cast<double>(bound_ns_)) {
        if (++bad_samples_ >= 3) {
            sync_lost_ = true;
            ready_ = false;
            count_ = next_ = 0;
            bad_samples_ = 0;
        }
        return;
    }
    bad_samples_ = 0;
    samples_[next_] = sample;
    next_ = (next_ + 1) % samples_.size();
    count_ = std::min(count_ + 1, samples_.size());
    fit();
    if (ready_) sync_lost_ = uncertainty_ns_ > bound_ns_;
}

void ClockEstimator6::fit() {
    if (count_ < 2) return;
    std::array<Sample, 16> sorted = samples_;
    std::sort(sorted.begin(), sorted.begin() + count_,
              [](const Sample &a, const Sample &b) { return a.round_trip_ns < b.round_trip_ns; });
    const auto selected = std::min<std::size_t>(count_, 8);
    double mean_x = 0, mean_y = 0;
    for (std::size_t i = 0; i < selected; ++i) {
        mean_x += sorted[i].host_ns;
        mean_y += sorted[i].device_ticks;
    }
    mean_x /= selected;
    mean_y /= selected;
    double covariance = 0, variance = 0;
    for (std::size_t i = 0; i < selected; ++i) {
        const auto dx = sorted[i].host_ns - mean_x;
        covariance += dx * (sorted[i].device_ticks - mean_y);
        variance += dx * dx;
    }
    if (variance <= 0) return;
    const auto fitted_rate = covariance / variance;
    if (!std::isfinite(fitted_rate) || fitted_rate <= 0) return;
    // Two early serial samples can differ by a full device update tick. Keep
    // that quantization from turning into an implausible clock-rate estimate.
    rate_ = std::clamp(fitted_rate, nominal_rate_ * 0.995, nominal_rate_ * 1.005);
    offset_ = mean_y - rate_ * mean_x;
    double worst_residual_ns = 0;
    for (std::size_t i = 0; i < selected; ++i)
        worst_residual_ns = std::max(worst_residual_ns,
            std::abs(sorted[i].device_ticks - (offset_ + rate_ * sorted[i].host_ns)) / rate_);
    uncertainty_ns_ = static_cast<std::uint64_t>(
        std::ceil(sorted[0].round_trip_ns * .5 + worst_residual_ns));
    ready_ = true;
}

std::uint64_t ClockEstimator6::map_host_ns(std::uint64_t host_ns) const {
    if (!ready_) return 0;
    const auto ticks = offset_ + rate_ * static_cast<double>(host_ns);
    if (ticks <= 0) return 0;
    if (ticks >= static_cast<double>(std::numeric_limits<std::uint64_t>::max()))
        return std::numeric_limits<std::uint64_t>::max();
    return static_cast<std::uint64_t>(std::llround(ticks));
}

std::uint64_t ClockEstimator6::committed_horizon_extra_ns(std::uint64_t link_latency_ns) const {
    const auto margin = uncertainty_ns_ > (UINT64_MAX - link_latency_ns) / 2
        ? UINT64_MAX : link_latency_ns + 2 * uncertainty_ns_;
    return margin;
}

} // namespace robotkit
