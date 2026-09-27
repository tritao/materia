#include "clock_estimator6.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>

using robotkit::ClockEstimator6;

static void sample(ClockEstimator6 &clock, std::uint64_t send_ns,
                   std::int64_t offset_ticks, double drift_ppm,
                   std::uint64_t one_way_ns) {
    constexpr double hz = 1'000'000.0;
    const auto ticks = [&](std::uint64_t ns) {
        return static_cast<std::uint64_t>(std::llround(
            offset_ticks + ns * hz * (1.0 + drift_ppm / 1'000'000.0) / 1e9));
    };
    clock.observe(send_ns, send_ns + 2 * one_way_ns,
                  ticks(send_ns + one_way_ns), ticks(send_ns + one_way_ns));
}

int main() {
    ClockEstimator6 clock(1'000'000, 500'000);
    assert(clock.sync_due(1'000'000'000, 100'000'000));
    clock.note_sync_request(1'000'000'000);
    assert(!clock.sync_due(1'050'000'000, 100'000'000));
    assert(clock.sync_due(1'100'000'000, 100'000'000));
    for (int i = 0; i < 20; ++i)
        sample(clock, 1'000'000'000ULL + i * 100'000'000ULL,
               50'000, 1'000.0, 100'000 + (i % 5) * 20'000);
    assert(clock.ready());
    assert(clock.may_commit());
    const auto mapped = clock.map_host_ns(4'000'000'000ULL);
    assert(std::abs(static_cast<std::int64_t>(mapped) - 4'054'000) < 250);
    assert(clock.uncertainty_ns() <= 500'000);
    assert(clock.committed_horizon_extra_ns(200'000) ==
           200'000 + 2 * clock.uncertainty_ns());
    // A late serial reply is an RTT outlier, not evidence that the clocks moved.
    clock.observe(3'100'000'000ULL, 3'150'000'000ULL,
                  3'153'100, 3'153'100);
    assert(clock.may_commit());
    sample(clock, 4'100'000'000ULL, 60'000, 1'000.0, 100'000);
    assert(clock.may_commit());
    sample(clock, 4'200'000'000ULL, 60'000, 1'000.0, 100'000);
    sample(clock, 4'300'000'000ULL, 60'000, 1'000.0, 100'000);
    assert(!clock.may_commit());
    assert(clock.clock_sync_lost());
    assert(std::string(clock.fault_reason()) == "clock_sync_lost");
    assert(clock.sync_due(4'400'000'000ULL, 100'000'000ULL));
    for (int i = 0; i < 8; ++i)
        sample(clock, 4'400'000'000ULL + i * 100'000'000ULL,
               50'000, 1'000.0, 100'000);
    assert(clock.may_commit());

    ClockEstimator6 jittered(1'000'000, 500'000);
    for (int i = 0; i < 20; ++i) {
        const auto send = 1'000'000'000ULL + i * 100'000'000ULL;
        const auto outbound = 100'000ULL + (i % 5) * 50'000ULL;
        const auto inbound = 100'000ULL + ((i + 2) % 5) * 50'000ULL;
        const auto device = static_cast<std::uint64_t>(std::llround(
            50'000 + (send + outbound) * 0.001001));
        jittered.observe(send, send + outbound + inbound, device, device);
    }
    assert(jittered.may_commit());
    assert(jittered.uncertainty_ns() <= 500'000);
    assert(std::abs(static_cast<std::int64_t>(jittered.map_host_ns(4'000'000'000ULL))
                    - 4'054'000) <= 500);
}
