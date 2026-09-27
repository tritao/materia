#include "device_compiler6.hpp"
#include <cassert>
#include <cmath>

int main() {
    robotkit::ClockEstimator6 clock(1'000'000, 500'000);
    for (int i = 0; i < 8; ++i) {
        const auto host = 1'000'000'000ULL + i * 100'000'000ULL;
        const auto ticks = 50'000ULL + host / 1'000;
        clock.observe(host, host + 200'000, ticks + 100, ticks + 100);
    }
    assert(clock.may_commit());
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 10;
    blueprint.joints[0].max_acceleration = 10;
    rk_trajectory_segment segment{};
    segment.time_from_start_ns = 0;
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 1.0 / 3.0;
    auto accepted = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(accepted.ok && accepted.segments.size() == 1);
    assert(accepted.segments[0].header.t0_ticks == 1'050'000);
    assert(accepted.segments[0].header.duration_ticks == 1'000'000);
    auto rejected = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 0.0);
    assert(!rejected.ok);
    segment.degree = 6;
    auto degree = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!degree.ok);
    segment.degree = 1;
    segment.coefficients[0].value[0] = 0.01;
    segment.coefficients[0].value[1] = 0.002;
    robotkit::DeviceActuator6 first{0, 2.0, 0.005, 400'000.0, 0.01};
    robotkit::DeviceActuator6 second{0, 4.0, 0.005, 400'000.0, 0.02};
    const robotkit::DeviceActuator6 layout[] = {first, second};
    auto transmitted = robotkit::compile_device_segments6(
        std::span(&segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, layout);
    assert(transmitted.ok && transmitted.segments[0].coefficients.size() == 2);
    assert(std::abs(transmitted.segments[0].coefficients[0].c0 - 0.01f) < 1e-7);
    assert(std::abs(transmitted.segments[0].coefficients[1].c0 - 0.02f) < 1e-7);
    assert(std::abs(transmitted.segments[0].coefficients[1].c1 - 0.008f) < 1e-7);
    second.max_rate = 0.001;
    const robotkit::DeviceActuator6 slow[] = {first, second};
    auto rate_rejected = robotkit::compile_device_segments6(
        std::span(&segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, slow);
    assert(!rate_rejected.ok);
    blueprint.owner_period_ns = 10'000'000;
    rk_trajectory_segment curved{};
    curved.duration_ns = 100'000'000;
    curved.degree = 2;
    curved.joint_count = 1;
    curved.coefficients[0].value[2] = 0.5;
    auto lowered = robotkit::compile_device_segments6(std::span(&curved, 1), 9, true,
        1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000, 1, 1e-4);
    assert(lowered.ok && lowered.segments.size() == 10);
    for (const auto &piece : lowered.segments) assert(piece.header.degree == 1);
}
