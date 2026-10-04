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
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 10);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    robotkit::TrajectorySegment segment{};
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
    auto minimal_line = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 1, 1e-6);
    assert(minimal_line.ok && minimal_line.segments.size() == 10);
    assert(minimal_line.segments[0].header.duration_ticks == 100'000);
    assert(!minimal_line.segments[0].header.ends_at_rest);
    assert(minimal_line.segments.back().header.ends_at_rest);
    auto rejected = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 0.0);
    assert(!rejected.ok);
    // The switch is inside physical guide room beyond the normal soft limit.
    auto home_segment = segment;
    home_segment.coefficients[0].value[0] = 10.05;
    home_segment.coefficients[0].value[1] = 0.0;
    blueprint.joint_overtravel[0] = 0.1;
    auto ordinary_overtravel = robotkit::compile_device_segments6(
        std::span(&home_segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!ordinary_overtravel.ok);
    auto home_overtravel = robotkit::compile_device_segments6(
        std::span(&home_segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, {}, 0, true);
    assert(home_overtravel.ok);
    home_segment.coefficients[0].value[0] = 10.15;
    auto beyond_stop = robotkit::compile_device_segments6(
        std::span(&home_segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, {}, 0, true);
    assert(!beyond_stop.ok);
    blueprint.joint_overtravel[0] = 0.0;
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
    blueprint.joint_count = 2;
    blueprint.joints[1].lower_limit = -10;
    blueprint.joints[1].upper_limit = 10;
    blueprint.joints[1].max_velocity = (blueprint.joints[1].limit_flags |= RK_LIMIT_VELOCITY, 10);
    blueprint.joints[1].max_acceleration = (blueprint.joints[1].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    blueprint.coupling_count = 1;
    blueprint.couplings[0] = {0, 1, 2.0, 0.1};
    segment.joint_count = 2;
    segment.coefficients[1].value[0] = 0.12;
    segment.coefficients[1].value[1] = 0.004;
    auto coupled = robotkit::compile_device_segments6(
        std::span(&segment, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(coupled.ok);
    segment.coefficients[1].value[0] = 0.1200005;
    auto derived = robotkit::compile_device_segments6(
        std::span(&segment, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(derived.ok);
    assert(derived.segments[0].coefficients[1].c0 ==
        coupled.segments[0].coefficients[1].c0);
    segment.coefficients[1].value[0] = 0.13;
    auto inconsistent = robotkit::compile_device_segments6(
        std::span(&segment, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!inconsistent.ok && inconsistent.error == "coupled follower trajectory mismatch");
    // A follower with two leaders: 2 a - 3 b + 0.1 + 0.05.
    blueprint.joint_count = 3;
    blueprint.joints[2] = blueprint.joints[1];
    blueprint.coupling_count = 2;
    blueprint.couplings[0] = {0, 2, 2.0, 0.1};
    blueprint.couplings[1] = {1, 2, -3.0, 0.05};
    segment.joint_count = 3;
    segment.coefficients[1].value[0] = -0.02;
    segment.coefficients[1].value[1] = 0.001;
    segment.coefficients[2].value[0] = 2.0 * 0.01 + 0.1 + -3.0 * -0.02 + 0.05;
    segment.coefficients[2].value[1] = 2.0 * 0.002 + -3.0 * 0.001;
    auto summed = robotkit::compile_device_segments6(
        std::span(&segment, 1), 10, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(summed.ok);
    segment.coefficients[2].value[1] += 0.001;
    auto unsummed = robotkit::compile_device_segments6(
        std::span(&segment, 1), 10, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!unsummed.ok && unsummed.error == "coupled follower trajectory mismatch");
    blueprint.coupling_count = 0;
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    robotkit::TrajectorySegment curved{};
    curved.duration_ns = 100'000'000;
    curved.degree = 2;
    curved.joint_count = 1;
    curved.coefficients[0].value[2] = 0.5;
    auto lowered = robotkit::compile_device_segments6(std::span(&curved, 1), 9, true,
        1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000, 1, 1e-4);
    assert(lowered.ok && lowered.segments.size() == 10);
    for (const auto &piece : lowered.segments) assert(piece.header.degree == 1);
    // Float coefficients can separate a continuous motor seam by over 1 microradian,
    // while both pieces remain within the deployment's declared conversion error.
    blueprint.joints[0].lower_limit = -30;
    blueprint.joints[0].upper_limit = 30;
    robotkit::TrajectorySegment joined[2]{};
    for (auto &piece : joined) {
        piece.joint_count = 1; piece.degree = 1; piece.duration_ns = 1'000'000'000;
        piece.coefficients[0].value[1] = 0.1;
    }
    joined[0].coefficients[0].value[0] = 20.000001;
    joined[1].time_from_start_ns = 1'000'000'000;
    joined[1].coefficients[0].value[0] = 20.100001;
    const double rounded_gap = std::abs(static_cast<double>(static_cast<float>(20.000001)) +
        static_cast<float>(0.1) - static_cast<float>(20.100001));
    assert(rounded_gap > 1e-6 && rounded_gap < 2e-5);
    auto rounded = robotkit::compile_device_segments6(joined, 11, false,
        1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000, 5, 1e-5);
    assert(rounded.ok && rounded.worst_position_error <= 1e-5);
    auto too_precise = robotkit::compile_device_segments6(joined, 11, false,
        1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000, 5, 1e-7);
    assert(!too_precise.ok);

}
