#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotkit {

/// Maps host monotonic nanoseconds to device ticks using low-RTT sync samples.
class ClockEstimator6 {
public:
    explicit ClockEstimator6(std::uint64_t device_tick_hz,
                             std::uint64_t uncertainty_bound_ns);
    void observe(std::uint64_t host_send_ns, std::uint64_t host_receive_ns,
                 std::uint64_t device_receive_ticks, std::uint64_t device_send_ticks);
    bool ready() const { return ready_; }
    bool may_commit() const { return ready_ && !sync_lost_; }
    bool clock_sync_lost() const { return sync_lost_; }
    const char *fault_reason() const { return sync_lost_ ? "clock_sync_lost" : nullptr; }
    bool sync_due(std::uint64_t host_now_ns, std::uint64_t period_ns) const;
    void note_sync_request(std::uint64_t host_send_ns);
    std::uint64_t uncertainty_ns() const { return uncertainty_ns_; }
    std::uint64_t map_host_ns(std::uint64_t host_ns) const;
    std::uint64_t committed_horizon_extra_ns(std::uint64_t link_latency_ns) const;

private:
    struct Sample { double host_ns; double device_ticks; double round_trip_ns; };
    std::array<Sample, 16> samples_{};
    std::size_t count_ = 0;
    std::size_t next_ = 0;
    double nominal_rate_;
    double rate_;
    double offset_ = 0;
    std::uint64_t bound_ns_;
    std::uint64_t uncertainty_ns_ = 0;
    bool ready_ = false;
    bool sync_lost_ = false;
    bool sent_ = false;
    std::uint8_t bad_samples_ = 0;
    std::uint64_t last_send_ns_ = 0;
    void fit();
};

} // namespace robotkit
