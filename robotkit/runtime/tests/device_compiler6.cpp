#include "device_compiler6.hpp"
#include "../../../trajectorykit/native/tests/merge_cases.hpp"
#include <cassert>
#include <cmath>
#include <vector>

int main() {
    merge_cases([](const std::vector<mk_segment> &segments, const motionkit::MergeOptions &options) {
        std::vector<robotkit::TrajectorySegment> host;
        for (const auto &s : segments) {
            robotkit::TrajectorySegment out{};
            out.time_from_start_ns = s.t0_ns; out.duration_ns = s.duration_ns;
            out.degree = s.degree; out.joint_count = s.joint_count;
            for (uint32_t j = 0; j < s.joint_count; ++j)
                std::copy_n(s.coefficients[j].value, s.degree + 1, out.coefficients[j].value);
            host.push_back(out);
        }
        motionkit::PolynomialSource source{host.data(), static_cast<uint32_t>(host.size()), 1,
            [](const void *context, uint32_t i) {
                const auto &s = static_cast<const robotkit::TrajectorySegment *>(context)[i];
                return motionkit::PolynomialSegmentView{static_cast<int64_t>(s.time_from_start_ns),
                    static_cast<int64_t>(s.duration_ns), s.degree, s.joint_count,
                    s.coefficients, sizeof(robotkit::SegmentCoefficients)};
            }};
        std::vector<mk_segment> out;
        assert(motionkit::merge_polynomials(source, options, {&out,
            [](void *context, const mk_segment &s, uint32_t, uint32_t) -> mk_result {
                static_cast<std::vector<mk_segment> *>(context)->push_back(s); return MK_OK;
            }}) == MK_OK);
        return out;
    });
    // 100 bytes at 1 Mbaud is 1 ms on the wire; the stated 500 Hz segment service is limiting.
    const robotkit::DeviceLink6 link{100, 1000000, 128, 500, 0};
    const auto window = robotkit::device_merge_window6(link);
    assert(window.below_ns == 2015749 && window.up_to_ns == 4031498);
    auto slow_link = link; slow_link.baud /= 4;
    assert(robotkit::device_merge_window6(slow_link).below_ns > window.below_ns);
    auto shallow_link = link; shallow_link.queue_capacity = 2;
    assert(robotkit::device_merge_window6(shallow_link).below_ns == 4000000);
    assert(robotkit::device_merge_window6({}).below_ns == 0);

    robotkit::ClockEstimator6 clock(1'000'000, 500'000);
    for (int i = 0; i < 8; ++i) {
        const auto host = 1'000'000'000ULL + i * 100'000'000ULL;
        const auto ticks = 50'000ULL + host / 1'000;
        clock.observe(host, host + 200'000, ticks + 100, ticks + 100);
    }
    assert(clock.may_commit());
    robotkit::ClockEstimator6 unsynced(1000000, 500000);

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
    // A positive sub-tick segment may have equal mapped endpoints. Preserve it
    // for one clock tick, without shortening it or smoothing away its polynomial.
    auto tiny = segment;
    tiny.duration_ns = 100;
    tiny.coefficients[0].value[1] = 0.025;
    auto next_tiny = tiny;
    next_tiny.time_from_start_ns = tiny.duration_ns;
    next_tiny.coefficients[0].value[0] = 0.025 * tiny.duration_ns / 1e9;
    const robotkit::TrajectorySegment sub_ticks[] = {tiny, next_tiny};
    auto sub_tick_plan = robotkit::compile_device_segments6(
        sub_ticks, 8, false, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(sub_tick_plan.ok && sub_tick_plan.segments.size() == 2);
    assert(sub_tick_plan.segments[0].header.duration_ticks == 1);
    assert(sub_tick_plan.segments[1].header.duration_ticks == 1);
    assert(sub_tick_plan.segments[1].header.t0_ticks == sub_tick_plan.segments[0].header.t0_ticks + 1);
    assert(sub_tick_plan.segments[0].host_duration_ns == 100);

    // A non-integral duration must round up, including continuations, so a
    // trajectory at the speed ceiling cannot be accelerated by tick rounding.
    auto ceiling_blueprint = blueprint;
    ceiling_blueprint.joints[0].max_velocity = 0.025;
    auto ceiling_segment = segment;
    ceiling_segment.duration_ns = 20'000'400;
    ceiling_segment.coefficients[0].value[1] = 0.025;
    auto continuation = ceiling_segment;
    continuation.time_from_start_ns = ceiling_segment.duration_ns;
    continuation.coefficients[0].value[0] = 0.025 * ceiling_segment.duration_ns / 1e9;
    const robotkit::TrajectorySegment ceiling_segments[] = {ceiling_segment, continuation};
    auto ceiling_plan = robotkit::compile_device_segments6(
        ceiling_segments, 8, false, 1'000'000'000ULL,
        clock, ceiling_blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(ceiling_plan.ok && ceiling_plan.segments.size() == 2);
    assert(ceiling_plan.segments[0].header.duration_ticks == 20'001);
    assert(ceiling_plan.segments[1].header.t0_ticks ==
        ceiling_plan.segments[0].header.t0_ticks + ceiling_plan.segments[0].header.duration_ticks);
    assert(ceiling_plan.segments[1].coefficients[0].c1 <= 0.025);
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
    // A stepper homed onto its limit rests just beyond it: a plan may start there within its start
    // tolerance, with the bound reaching the start and no further.
    auto resting = segment;
    resting.coefficients[0].value[0] = 10.002;
    resting.coefficients[0].value[1] = 0.0;
    auto strict = robotkit::compile_device_segments6(
        std::span(&resting, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!strict.ok);
    const double start_tolerance[robotkit::device_wire6::MAX_ACTUATORS] = {0.005};
    auto admitted = robotkit::compile_device_segments6(
        std::span(&resting, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, {}, 0, false, start_tolerance);
    assert(admitted.ok);
    resting.coefficients[0].value[0] = 10.02;
    auto too_far = robotkit::compile_device_segments6(
        std::span(&resting, 1), 9, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, {}, 0, false, start_tolerance);
    assert(!too_far.ok);
    segment.degree = 6;
    auto degree = robotkit::compile_device_segments6(
        std::span(&segment, 1), 7, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6);
    assert(!degree.ok);
    segment.degree = 1;
    segment.coefficients[0].value[0] = 0.01;
    segment.coefficients[0].value[1] = 0.002;
    robotkit::DeviceActuator6 first{0, 2.0, 0.005, 400'000.0};
    robotkit::DeviceActuator6 second{0, 4.0, 0.005, 400'000.0};
    const robotkit::DeviceActuator6 layout[] = {first, second};
    auto transmitted = robotkit::compile_device_segments6(
        std::span(&segment, 1), 8, true, 1'000'000'000ULL,
        clock, blueprint, 1'000'000, 40'000, 5, 1e-6, layout);
    assert(transmitted.ok && transmitted.segments[0].coefficients.size() == 2);
    // The whole steps go as an integer base: 0.01 m at 400,000 steps a metre is 4,000 steps, and the float keeps the rest.
    assert(transmitted.segments[0].coefficients[0].base_steps == 4'000 &&
        std::abs(transmitted.segments[0].coefficients[0].c0) < 1e-7f);
    assert(std::abs(robotkit::wire_position(transmitted.segments[0].coefficients[0], 1, 0.0f, 400'000.0) - 0.01) < 1e-12);
    assert(std::abs(robotkit::wire_position(transmitted.segments[0].coefficients[1], 1, 0.0f, 400'000.0) - 0.02) < 1e-12);
    assert(std::abs(transmitted.segments[0].coefficients[1].c1 - 0.008f) < 1e-7);
    second.min_step_ticks = 1'000;
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
    assert(inconsistent.failure == robotkit::DeviceCompileFailure6::Coupling && !inconsistent.retried_without_merge);
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

    // A sliced curve plans to hundreds of segments a second, more than a serial link can refill: runs of them that one
    // quintic follows to a tenth of a step go as one. A ramp of 400 pieces of 0.1 ms (constant acceleration, exact in a
    // quadratic each) becomes a few segments no longer than 4 ms, the same motion to the step.
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].max_velocity = 1.0;
    blueprint.joints[0].max_acceleration = 100.0;
    robotkit::DeviceActuator6 motor{0, 1.0, 0.0, 10'000.0, 1};
    const robotkit::DeviceActuator6 motors[] = {motor};
    std::vector<robotkit::TrajectorySegment> ramp(400);
    double position = 0.0, speed = 0.0;
    for (std::size_t i = 0; i < ramp.size(); ++i) {
        // Accelerate at 10 m/s^2 for the first half, brake for the second.
        const double acceleration = i < 200 ? 10.0 : -10.0, dt = 1e-4;
        ramp[i].time_from_start_ns = i * 100'000ULL; ramp[i].duration_ns = 100'000; ramp[i].degree = 2; ramp[i].joint_count = 1;
        ramp[i].coefficients[0].value[0] = position;
        ramp[i].coefficients[0].value[1] = speed;
        ramp[i].coefficients[0].value[2] = 0.5 * acceleration;
        position += speed * dt + 0.5 * acceleration * dt * dt;
        speed += acceleration * dt;
    }
    auto many = robotkit::compile_device_segments6(ramp, 12, true, 1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000,
        5, 1e-5, motors, 0, false, nullptr, link);
    auto no_clock = robotkit::compile_device_segments6(ramp, 12, true, 1000000000, unsynced, blueprint,
        1000000, 40000, 5, 1e-5, motors, 0, false, nullptr, link);
    assert(!no_clock.ok && no_clock.failure == robotkit::DeviceCompileFailure6::Clock && !no_clock.retried_without_merge);
    assert(many.ok);
    assert(many.segments.size() <= 14);
    std::uint64_t covered = 0;
    for (const auto &piece : many.segments) {
        assert(piece.host_duration_ns <= window.up_to_ns);
        assert(piece.host_time_from_start_ns == covered);
        covered += piece.host_duration_ns;
    }
    assert(covered == 40'000'000);
    assert(many.segments.back().header.ends_at_rest);
    // The pieces meet where the motion does: the last one ends where the ramp does (position 0.4 m... 2 x 0.5 x 10 x 0.02^2).
    const auto &last = many.segments.back();
    const double tau = static_cast<double>(last.header.duration_ticks) / 1e6;
    const double end = static_cast<double>(last.coefficients[0].base_steps) / 10'000.0 +
        last.coefficients[0].c0 + last.coefficients[0].c1 * tau + last.coefficients[0].c2 * tau * tau +
        last.coefficients[0].c3 * tau * tau * tau + last.coefficients[0].c4 * std::pow(tau, 4) + last.coefficients[0].c5 * std::pow(tau, 5);
    assert(std::abs(end - position) < 1e-6);
    // A limit violation in an untouched long segment is unrelated to earlier successful merges.
    {
        auto invalid = ramp;
        robotkit::TrajectorySegment outside{};
        outside.time_from_start_ns = 40000000; outside.duration_ns = 20000000000;
        outside.joint_count = 1; outside.degree = 1; outside.coefficients[0].value[0] = position;
        outside.coefficients[0].value[1] = 2;
        invalid.push_back(outside);
        auto refused = robotkit::compile_device_segments6(invalid, 15, true, 1000000000, clock, blueprint,
            1000000, 40000, 5, 1e-5, motors, 0, false, nullptr, link);
        assert(!refused.ok && refused.failure == robotkit::DeviceCompileFailure6::Limits);
        assert(!refused.retried_without_merge);
    }
    // Coupled quintics retain the follower check despite high inverse-duration coefficients.
    // Validate original evidence first so an inconsistent short piece cannot disappear in a merged run.
    {
        auto coupled_limits = blueprint;
        coupled_limits.joint_count = 2; coupled_limits.joints[1] = blueprint.joints[0];
        coupled_limits.coupling_count = 1; coupled_limits.couplings[0] = {0, 1, 2, .1};
        auto coupled_ramp = ramp;
        for (auto &s : coupled_ramp) {
            s.joint_count = 2;
            for (unsigned k = 0; k <= s.degree; ++k)
                s.coefficients[1].value[k] = 2 * s.coefficients[0].value[k] + (k == 0 ? .1 : 0);
        }
        auto valid = robotkit::compile_device_segments6(coupled_ramp, 16, true, 1000000000, clock, coupled_limits,
            1000000, 40000, 5, 1e-5, motors, 0, false, nullptr, link);
        assert(valid.ok && valid.segments.size() < coupled_ramp.size());
        coupled_ramp[17].coefficients[1].value[0] += .01;
        auto bad = robotkit::compile_device_segments6(coupled_ramp, 17, true, 1000000000, clock, coupled_limits,
            1000000, 40000, 5, 1e-5, motors, 0, false, nullptr, link);
        assert(!bad.ok && bad.failure == robotkit::DeviceCompileFailure6::Coupling && !bad.retried_without_merge);
    }
    // A path that turns inside a piece is not merged across the turn: a speed reversal every 0.1 ms.
    for (std::size_t i = 0; i < ramp.size(); ++i) {
        ramp[i].degree = 1; ramp[i].coefficients[0].value[0] = (i % 2 == 0 ? 0.0 : 0.0005);
        ramp[i].coefficients[0].value[1] = (i % 2 == 0 ? 5.0 : -5.0); ramp[i].coefficients[0].value[2] = 0.0;
    }
    auto zigzag = robotkit::compile_device_segments6(ramp, 13, true, 1'000'000'000ULL, clock, blueprint, 1'000'000, 40'000,
        5, 1e-5, motors, 0, false, nullptr, link);
    assert(!zigzag.ok || zigzag.segments.size() == ramp.size());
    // A narrow original excursion between the merge's fit samples is caught by the dense wire check.
    // Only this merge-caused precision failure retries; the source itself remains valid.
    {
        auto limits = blueprint;
        limits.joints[0].max_velocity = 1; limits.joints[0].max_acceleration = 100;
        std::vector<robotkit::TrajectorySegment> pulse(40);
        for (std::size_t i = 0; i < pulse.size(); ++i) {
            pulse[i].time_from_start_ns = i * 100000; pulse[i].duration_ns = 100000;
            pulse[i].joint_count = 1; pulse[i].degree = 5;
        }
        // u^3 (1-u)^2 peaks at u=.6. Its start position, velocity and acceleration are zero.
        const double amplitude = 2e-9 / .03456, dt = 1e-4;
        pulse[17].coefficients[0].value[3] = amplitude / std::pow(dt, 3);
        pulse[17].coefficients[0].value[4] = -2 * amplitude / std::pow(dt, 4);
        pulse[17].coefficients[0].value[5] = amplitude / std::pow(dt, 5);
        const robotkit::DeviceActuator6 fine_motor{0, 1, 0, 1e8, 1};
        auto retried = robotkit::compile_device_segments6(pulse, 14, true, 1000000000, clock, limits,
            1000000, 40000, 5, 1e-12, std::span(&fine_motor, 1), 0, false, nullptr, link);
        assert(retried.ok && retried.retried_without_merge);
        assert(retried.merge_cause == robotkit::DeviceCompileFailure6::Precision);
        assert(retried.segments.size() == pulse.size());
    }
    // An extruder that has fed 50 m of filament has turned some 40,000 rad: past 4,096 rad half a float's step (2.4e-4 rad
    // there, 2.4e-3 at 40,000) is more than a tenth of a step of the motor (1.95e-4 rad at 512 steps a radian), so a
    // position held as one float is refused or jitters. The wire carries the whole steps as an integer base.
    {
        auto extruder_blueprint = blueprint;
        extruder_blueprint.joint_count = 1;
        extruder_blueprint.coupling_count = 0;
        extruder_blueprint.owner_period_ns = 10'000'000;
        extruder_blueprint.joints[0].lower_limit = -1.0e6;
        extruder_blueprint.joints[0].upper_limit = 1.0e6;
        extruder_blueprint.joints[0].max_velocity = (extruder_blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 100.0);
        extruder_blueprint.joints[0].max_acceleration = (extruder_blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 1000.0);
        const robotkit::DeviceActuator6 extruder_motor{0, 1.0, 0.0, 512.0, 1};
        const robotkit::DeviceActuator6 extruder[] = {extruder_motor};
        // 40,000 rad to 40,010 rad in ten 100 ms quintics: a smooth rise and fall of speed, rest to rest.
        std::vector<robotkit::TrajectorySegment> feed(10);
        const double start = 40'000.0;
        for (std::size_t i = 0; i < feed.size(); ++i) {
            auto &f = feed[i];
            f.time_from_start_ns = i * 100'000'000ULL; f.duration_ns = 100'000'000; f.degree = 5; f.joint_count = 1;
            // Position from s(u) = 10 (10u^3 - 15u^4 + 6u^5) over u in [0, 1] of the second, in each tenth's own time.
            const double u0 = i / 10.0, T = 0.1;
            auto s = [](double u) { return 10.0 * (10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u); };
            auto ds = [](double u) { return 10.0 * (30 * u * u - 60 * u * u * u + 30 * u * u * u * u); };
            auto d2s = [](double u) { return 10.0 * (60 * u - 180 * u * u + 120 * u * u * u); };
            auto d3s = [](double u) { return 10.0 * (60 - 360 * u + 360 * u * u); };
            auto d4s = [](double u) { return 10.0 * (-360 + 720 * u); };
            auto d5s = [](double) { return 10.0 * 720.0; };
            f.coefficients[0].value[0] = start + s(u0);
            f.coefficients[0].value[1] = ds(u0);
            f.coefficients[0].value[2] = d2s(u0) / 2;
            f.coefficients[0].value[3] = d3s(u0) / 6;
            f.coefficients[0].value[4] = d4s(u0) / 24;
            f.coefficients[0].value[5] = d5s(u0) / 120;
            (void)T;
        }
        auto fed = robotkit::compile_device_segments6(feed, 20, true, 1'000'000'000ULL, clock, extruder_blueprint,
            1'000'000, 40'000, 5, 1e-6, extruder);
        assert(fed.ok);
        // The device's own arithmetic replays the plan to a tenth of a step (1.95e-4 rad) everywhere.
        for (const auto &piece : fed.segments) {
            const double seconds = static_cast<double>(piece.header.duration_ticks) / 1e6;
            for (int k = 0; k <= 20; ++k) {
                const double tau = seconds * k / 20.0;
                const double got = robotkit::wire_position(piece.coefficients[0], piece.header.degree,
                    static_cast<float>(tau), 512.0);
                const double at = static_cast<double>(piece.host_time_from_start_ns) * 1e-9 + tau;
                const double u = at;
                const double want = start + 10.0 * (10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u);
                assert(std::abs(got - want) < 0.1 / 512.0);
            }
        }
    }
    // The start rule's tolerance is the drive's resolution (one step of the finest motor on the joint), not the planner's
    // start mismatch: a plan may start half a step past the joint's limit, three steps past it is refused, whatever
    // start tolerance the plan carries.
    {
        auto limited = blueprint;
        limited.joint_count = 1;
        limited.coupling_count = 0;
        limited.joints[0].lower_limit = -10.0;
        limited.joints[0].upper_limit = 10.0;
        const robotkit::DeviceActuator6 stepper{0, 1.0, 0.0, 10'000.0, 1};
        const robotkit::DeviceActuator6 stepper_layout[] = {stepper};
        const double loose[robotkit::device_wire6::MAX_ACTUATORS] = {1e-3};
        auto start_at = [&](double position) {
            robotkit::TrajectorySegment rest{};
            rest.duration_ns = 100'000'000; rest.degree = 1; rest.joint_count = 1;
            rest.coefficients[0].value[0] = position;
            return robotkit::compile_device_segments6(std::span(&rest, 1), 30, true, 1'000'000'000ULL, clock, limited,
                1'000'000, 40'000, 5, 1e-6, stepper_layout, 0, false, loose);
        };
        assert(start_at(10.00005).ok);
        assert(!start_at(10.0003).ok);
    }
}
