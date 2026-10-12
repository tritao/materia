#include "motionkit.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <vector>

int main() {
    mk_path_sample samples[3]{};
    for (int i = 0; i < 3; ++i) {
        samples[i].struct_size = sizeof(mk_path_sample);
        samples[i].s = static_cast<double>(i);
        samples[i].joint_count = 2;
        // A sampled parabolic path; the second joint stays linear.
        samples[i].position[0] = samples[i].s * samples[i].s;
        samples[i].first[0] = 2.0 * samples[i].s;
        samples[i].second[0] = 2.0;
        samples[i].second_before[0] = 2.0;
        samples[i].position[1] = samples[i].s;
        samples[i].first[1] = 1.0;
    }
    mk_path_handle path{};
    assert(mk_path_create(samples, 3, &path) == MK_OK);

    mk_time_stage stages[2]{};
    for (auto &stage : stages) stage.struct_size = sizeof(mk_time_stage);
    stages[0].start_ns = 0;
    stages[0].duration_ns = 1'000'000'000;
    stages[0].start_s = 0.0;
    stages[0].speed = 0.5;
    stages[0].acceleration = 1.0;
    stages[1].start_ns = 1'000'000'000;
    stages[1].duration_ns = 1'000'000'000;
    stages[1].start_s = 1.0;
    stages[1].speed = 1.5;
    stages[1].acceleration = -1.0;
    mk_time_law_handle law{};
    assert(mk_time_law_create(stages, 2, &law) == MK_OK);
    double time = -1;
    assert(mk_path_distance_to_time(law, 1.0, &time) == MK_OK);
    assert(time == 1.0);
    assert(mk_path_distance_to_time(law, 2.0, &time) == MK_OK);
    assert(time == 2.0);

    // The closed-form inverse: s(0.5) = 0.5*0.5 + 0.5*0.25, s(1.5) = 1 + 1.5*0.5 - 0.5*0.25; times outside clamp.
    const double query[7] = {0.0, 0.5, 1.0, 1.5, 2.0, 5.0, -1.0};
    const double wanted[7] = {0.0, 0.375, 1.0, 1.625, 2.0, 2.0, 0.0};
    double reached[7] = {};
    assert(mk_path_times_to_distances(law, query, 7, reached) == MK_OK);
    for (int i = 0; i < 7; ++i) assert(std::abs(reached[i] - wanted[i]) < 1e-12);
    for (int i = 0; i < 5; ++i) {
        assert(mk_path_distance_to_time(law, reached[i], &time) == MK_OK);
        assert(std::abs(time - query[i]) < 1e-9); // round trip through the forward map
    }
    const double not_finite[1] = {std::nan("")};
    assert(mk_path_times_to_distances(law, not_finite, 1, reached) == MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_times_to_distances(law, nullptr, 1, reached) == MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_times_to_distances(law, query, 0, nullptr) == MK_OK);
    mk_time_law_handle missing{};
    missing.id = 0xffffff;
    assert(mk_path_times_to_distances(missing, query, 7, reached) == MK_ERROR_INVALID_HANDLE);

    mk_trajectory_handle tight{}, loose{};
    assert(mk_path_lower(path, law, 1e-6, &tight) == MK_OK);
    assert(mk_path_lower(path, law, 1e-2, &loose) == MK_OK);
    uint32_t tight_count = 0, loose_count = 0;
    assert(mk_trajectory_segment_count(tight, &tight_count) == MK_OK);
    assert(mk_trajectory_segment_count(loose, &loose_count) == MK_OK);
    assert(tight_count >= loose_count);
    mk_trajectory_state state{};
    state.struct_size = sizeof(state);
    for (int i = 0; i <= 200; ++i) {
        const int64_t ns = i * 10'000'000LL;
        const double t = static_cast<double>(ns) * 1e-9;
        const double s = t <= 1.0 ? 0.5 * t + 0.5 * t * t :
            1.0 + 1.5 * (t - 1.0) - 0.5 * (t - 1.0) * (t - 1.0);
        assert(mk_trajectory_evaluate(tight, ns, &state) == MK_OK);
        assert(std::abs(state.position[0] - s * s) <= 1e-6);
        assert(std::abs(state.position[1] - s) <= 1e-6);
    }
    mk_trajectory_destroy(tight);
    mk_trajectory_destroy(loose);
    mk_time_law_destroy(law);
    mk_path_destroy(path);

    // Rounded TOPP-RA stage durations can leave a few nanometres per second
    // of negative endpoint speed even when the requested endpoint is zero.
    mk_time_stage rounded_stop{};
    rounded_stop.struct_size = sizeof(rounded_stop);
    rounded_stop.duration_ns = 1'000'000'000;
    rounded_stop.speed = 0.1;
    rounded_stop.acceleration = -0.100000004;
    assert(mk_time_law_create(&rounded_stop, 1, &law) == MK_OK);
    mk_time_law_destroy(law);

    mk_path_sample straight[2]{};
    for (int i = 0; i < 2; ++i) {
        straight[i].struct_size = sizeof(straight[i]);
        straight[i].joint_count = 1;
        straight[i].s = static_cast<double>(i);
        straight[i].position[0] = straight[i].s;
        straight[i].first[0] = 1.0;
    }
    assert(mk_path_create(straight, 2, &path) == MK_OK);
    const double velocity_limit[1] = {0.4};
    const double acceleration_limit[1] = {1.0};
    mk_trajectory_handle timed_trajectory{};
    assert(mk_time_path(path, velocity_limit, acceleration_limit, 1,
        nullptr, 0, 0.0, 0.0, 1e-6, 0, nullptr, &law, &timed_trajectory) == MK_OK);
    assert(timed_trajectory.id != 0);
    mk_trajectory_state timed_state{};
    timed_state.struct_size = sizeof(timed_state);
    assert(mk_trajectory_evaluate(timed_trajectory, 0, &timed_state) == MK_OK);
    assert(std::abs(timed_state.position[0]) < 1e-9);
    assert(mk_path_distance_to_time(law, 1.0, &time) == MK_OK);
    assert(time > 2.8 && time < 3.1); // 0.4 m/s cruise, 1 m/s² ramps.
    uint32_t binding_count = 0;
    assert(mk_time_law_binding_count(law, &binding_count) == MK_OK);
    assert(binding_count > 0);
    bool velocity_bound = false;
    for (uint32_t i = 0; i < binding_count; ++i) {
        mk_timing_binding binding{};
        binding.struct_size = sizeof(binding);
        assert(mk_time_law_get_binding(law, i, &binding) == MK_OK);
        if (binding.kind == MK_TIMING_BINDING_JOINT_VELOCITY && binding.joint == 0)
            velocity_bound = true;
    }
    assert(velocity_bound);
    mk_trajectory_destroy(timed_trajectory);
    mk_time_law_destroy(law);
    const double boundary_velocity_limit[1] = {0.2};
    mk_timing_report strict_report{};
    assert(mk_time_path(path, boundary_velocity_limit, acceleration_limit, 1,
        nullptr, 0, 0.15, 0.15, 1e-8, 1, &strict_report, &law, &tight) == MK_OK);
    assert(strict_report.adjustments==0 && strict_report.derivative==0);
    mk_trajectory_state moving_boundary{};
    moving_boundary.struct_size = sizeof(moving_boundary);
    assert(mk_trajectory_evaluate(tight, 0, &moving_boundary) == MK_OK);
    assert(std::abs(moving_boundary.velocity[0] - 0.15) < 1e-7);
    assert(mk_path_distance_to_time(law, 1.0, &time) == MK_OK);
    assert(mk_trajectory_evaluate(tight,
        static_cast<int64_t>(std::llround(time * 1e9)), &moving_boundary) == MK_OK);
    assert(std::abs(moving_boundary.velocity[0] - 0.15) < 1e-7);
    mk_trajectory_destroy(tight);
    mk_time_law_destroy(law);
    mk_path_destroy(path);

    // A quarter circle is represented by a C2 Hermite span. A quadratic
    // time law raises its degree above five, exercising adaptive lowering.
    mk_path_sample circle[2]{};
    const double half_pi = std::acos(-1.0) * 0.5;
    for (int i = 0; i < 2; ++i) {
        auto &point = circle[i];
        point.struct_size = sizeof(point);
        point.joint_count = 2;
        point.s = static_cast<double>(i);
        const double angle = half_pi * point.s;
        point.position[0] = std::cos(angle);
        point.position[1] = std::sin(angle);
        point.first[0] = -half_pi * std::sin(angle);
        point.first[1] = half_pi * std::cos(angle);
        point.second[0] = -half_pi * half_pi * std::cos(angle);
        point.second[1] = -half_pi * half_pi * std::sin(angle);
        point.second_before[0] = point.second[0];
        point.second_before[1] = point.second[1];
    }
    assert(mk_path_create(circle, 2, &path) == MK_OK);
    mk_time_stage ramp{};
    ramp.struct_size = sizeof(ramp);
    ramp.duration_ns = 1'000'000'000;
    ramp.speed = 0.5;
    ramp.acceleration = 1.0;
    assert(mk_time_law_create(&ramp, 1, &law) == MK_OK);
    assert(mk_path_lower(path, law, 1e-10, &tight) == MK_OK);
    assert(mk_path_lower(path, law, 1e-3, &loose) == MK_OK);
    assert(mk_trajectory_segment_count(tight, &tight_count) == MK_OK);
    assert(mk_trajectory_segment_count(loose, &loose_count) == MK_OK);
    assert(tight_count > loose_count);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 2;
    for (int joint = 0; joint < 2; ++joint) {
        limits.max_velocity[joint] = (limits.derivative_claimed[joint] |= 1, 10.0);
        limits.max_acceleration[joint] = (limits.derivative_claimed[joint] |= 2, 20.0);
    }
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(tight, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_ACCELERATION].status == MK_CHECK_PASSED);
    for (int i = 0; i <= 100; ++i) {
        const int64_t ns = i * 10'000'000LL;
        const double t = static_cast<double>(ns) * 1e-9;
        const double angle = half_pi * (0.5 * t + 0.5 * t * t);
        assert(mk_trajectory_evaluate(tight, ns, &state) == MK_OK);
        // Analytic circle error includes the authored Hermite approximation.
        assert(std::abs(state.position[0] - std::cos(angle)) < 3e-4);
        assert(std::abs(state.position[1] - std::sin(angle)) < 3e-4);
    }
    mk_trajectory_destroy(tight);
    mk_trajectory_destroy(loose);
    mk_time_law_destroy(law);
    ramp.speed = 1.0;
    ramp.acceleration = 0.0;
    assert(mk_time_law_create(&ramp, 1, &law) == MK_OK);
    assert(mk_path_lower(path, law, 1e-10, &tight) == MK_OK);
    assert(mk_trajectory_segment_count(tight, &tight_count) == MK_OK);
    assert(tight_count == 1); // A quintic q(s) under linear s(t) is exact.
    mk_trajectory_destroy(tight);
    mk_time_law_destroy(law);
    mk_path_destroy(path);

    // A line that turns into a parabola at s = 1: the second derivative jumps
    // there, and each span keeps its own, so both stay exact.
    mk_path_sample kinked[3]{};
    const double kink_position[3] = {0.0, 1.0, 3.0}, kink_first[3] = {1.0, 1.0, 3.0};
    const double kink_before[3] = {0.0, 0.0, 2.0}, kink_after[3] = {0.0, 2.0, 2.0};
    for (int i = 0; i < 3; ++i) {
        kinked[i].struct_size = sizeof(mk_path_sample);
        kinked[i].joint_count = 1;
        kinked[i].s = static_cast<double>(i);
        kinked[i].position[0] = kink_position[i];
        kinked[i].first[0] = kink_first[i];
        kinked[i].second[0] = kink_after[i];
        kinked[i].second_before[0] = kink_before[i];
    }
    assert(mk_path_create(kinked, 3, &path) == MK_OK);
    const double kink_velocity[1] = {1.0}, kink_acceleration[1] = {2.0};
    mk_trajectory_handle kink_trajectory{};
    assert(mk_time_path(path,kink_velocity,kink_acceleration,1,nullptr,0,
        0,0,1e-9,1,&strict_report,&law,&kink_trajectory)==MK_ERROR_GENERATION);
    assert(!law.id && !kink_trajectory.id && strict_report.adjustments==0);
    assert(strict_report.joint==0 && strict_report.derivative==1 && strict_report.value>strict_report.limit);
    assert(strict_report.sustainable_scale>0 && strict_report.sustainable_scale<1);
    assert(mk_time_path(path, kink_velocity, kink_acceleration, 1, nullptr, 0,
        0.0, 0.0, 1e-9, 0, nullptr, &law, &kink_trajectory) == MK_OK);
    for (double at : {0.5, 1.5, 1.9}) {
        double seconds = 0.0;
        assert(mk_path_distance_to_time(law, at, &seconds) == MK_OK);
        mk_trajectory_state kink_state{};
        kink_state.struct_size = sizeof(kink_state);
        assert(mk_trajectory_evaluate(kink_trajectory,
            static_cast<int64_t>(std::llround(seconds * 1e9)), &kink_state) == MK_OK);
        const double expected = at <= 1.0 ? at : at + (at - 1.0) * (at - 1.0);
        assert(std::abs(kink_state.position[0] - expected) < 1e-6);
    }
    mk_trajectory_destroy(kink_trajectory);
    mk_time_law_destroy(law);
    mk_path_destroy(path);

    // A long straight rapid sampled every 2 mm, as a CNC Z move is: the knots
    // accumulate rounding, and the timed law must still end where the path does.
    std::vector<mk_path_sample> rapid(101);
    double distance = 0.0;
    for (size_t i = 0; i < rapid.size(); ++i) {
        rapid[i] = mk_path_sample{};
        rapid[i].struct_size = sizeof(mk_path_sample);
        rapid[i].joint_count = 3;
        rapid[i].s = i + 1 == rapid.size() ? 0.2 : distance;
        rapid[i].position[2] = -rapid[i].s;
        rapid[i].first[2] = -1.0;
        distance += 0.002;
    }
    assert(mk_path_create(rapid.data(), static_cast<uint32_t>(rapid.size()), &path) == MK_OK);
    const double axis_velocity[3] = {0.08, 0.08, 0.04};
    const double axis_acceleration[3] = {0.5, 0.4, 0.3};
    std::vector<double> rapid_caps(rapid.size() - 1, 0.08);
    mk_trajectory_handle rapid_trajectory{};
    assert(mk_time_path(path, axis_velocity, axis_acceleration, 3, rapid_caps.data(),
        static_cast<uint32_t>(rapid_caps.size()), 0.0, 0.0, 1e-6, 0, nullptr, &law,
        &rapid_trajectory) == MK_OK);
    mk_trajectory_destroy(rapid_trajectory);
    mk_time_law_destroy(law);
    mk_path_destroy(path);

    // A slow 1 mm-radius arc sampled every 1.5 um, as a printer's corner arcs
    // are: TOPP-RA's stages on it last ~2 us. Deriving each stage's
    // acceleration from its nanosecond-rounded duration made errors of several
    // m/s^2 there (AM5B), and no slowdown removes them; each stage must keep
    // TOPP-RA's, and the law must pass at once.
    {
        const double spacing = 1.5e-6, radius = 1e-3;
        std::vector<mk_path_sample> fine(401);
        for (size_t i = 0; i < fine.size(); ++i) {
            auto &q = fine[i];
            q = mk_path_sample{};
            q.struct_size = sizeof(q);
            q.joint_count = 2;
            q.s = spacing * static_cast<double>(i);
            const double phi = q.s / radius;
            q.position[0] = radius * std::sin(phi);
            q.position[1] = radius * (1.0 - std::cos(phi));
            q.first[0] = std::cos(phi);
            q.first[1] = std::sin(phi);
            q.second[0] = q.second_before[0] = -std::sin(phi) / radius;
            q.second[1] = q.second_before[1] = std::cos(phi) / radius;
        }
        assert(mk_path_create(fine.data(), static_cast<uint32_t>(fine.size()), &path) == MK_OK);
        const double xy_velocity[2] = {0.25, 0.25}, xy_acceleration[2] = {1.0, 1.0};
        const std::vector<double> feed(fine.size() - 1, 0.02);
        mk_timing_report fine_report{};
        mk_trajectory_handle fine_trajectory{};
        assert(mk_time_path(path, xy_velocity, xy_acceleration, 2, feed.data(),
            static_cast<uint32_t>(feed.size()), 0.0, 0.0, 1e-9, 1, &fine_report, &law,
            &fine_trajectory) == MK_OK);
        assert(fine_report.adjustments == 0);
        mk_limits xy_limits{};
        xy_limits.struct_size = sizeof(xy_limits);
        xy_limits.joint_count = 2;
        for (int j = 0; j < 2; ++j) {
            xy_limits.derivative_claimed[j] = 3u;
            xy_limits.max_velocity[j] = xy_velocity[j];
            xy_limits.max_acceleration[j] = xy_acceleration[j];
        }
        mk_validation_report fine_validation{};
        fine_validation.struct_size = sizeof(fine_validation);
        assert(mk_validate(fine_trajectory, &xy_limits, &fine_validation) == MK_OK);
        // Collocation plans a curved path to 0.8 of the limit; the law keeps it.
        assert(fine_validation.checks[MK_CHECK_ACCELERATION].value < 0.85);
        // Every sample is passed at a knot of the law.
        for (const auto &q : fine) {
            assert(mk_path_distance_to_time(law, q.s, &time) == MK_OK);
            assert(std::abs(time * 1e9 - std::round(time * 1e9)) < 1e-3);
        }
        mk_trajectory_destroy(fine_trajectory);
        mk_time_law_destroy(law);
        mk_path_destroy(path);
    }

    // A step move timed to cruise at the speed limit, then merged into longer quintics as a compiler does: a
    // merged run may not lift the plateau past the limit between the points it is checked at, or the plan
    // validator refuses the merged form.
    {
        std::vector<mk_path_sample> step(51);
        for (size_t i = 0; i < step.size(); ++i) {
            auto &q = step[i];
            q = mk_path_sample{};
            q.struct_size = sizeof(q);
            q.joint_count = 2;
            q.s = 0.002 * static_cast<double>(i);
            q.position[0] = q.s;
            q.first[0] = 1.0;
        }
        assert(mk_path_create(step.data(), static_cast<uint32_t>(step.size()), &path) == MK_OK);
        const double step_velocity[2] = {0.25, 0.25}, step_acceleration[2] = {12.5627130716, 24.09};
        mk_trajectory_handle step_trajectory{}, step_merged{};
        assert(mk_time_path(path, step_velocity, step_acceleration, 2, nullptr, 0, 0.0, 0.0, 1e-6, 0,
            nullptr, &law, &step_trajectory) == MK_OK);
        double position_tolerance[2], velocity_tolerance[2], acceleration_tolerance[2];
        const double lower[2] = {-1e9, -1e9}, upper[2] = {1e9, 1e9};
        for (int j = 0; j < 2; ++j) {
            position_tolerance[j] = 10e-6 * step_velocity[j];
            velocity_tolerance[j] = 0.01 * step_velocity[j];
            acceleration_tolerance[j] = 0.1 * step_acceleration[j];
        }
        assert(mk_merge_segments(step_trajectory, position_tolerance, velocity_tolerance,
            acceleration_tolerance, step_velocity, step_acceleration, lower, upper, 2, 0.01,
            &step_merged) == MK_OK);
        uint32_t lowered_count = 0, merged_count = 0;
        assert(mk_trajectory_segment_count(step_trajectory, &lowered_count) == MK_OK);
        assert(mk_trajectory_segment_count(step_merged, &merged_count) == MK_OK);
        assert(merged_count * 10 < lowered_count);
        mk_limits step_limits{};
        step_limits.struct_size = sizeof(step_limits);
        step_limits.joint_count = 2;
        for (int j = 0; j < 2; ++j) {
            step_limits.derivative_claimed[j] = 3u;
            step_limits.max_velocity[j] = step_velocity[j];
            step_limits.max_acceleration[j] = step_acceleration[j];
        }
        mk_validation_report step_report{};
        step_report.struct_size = sizeof(step_report);
        assert(mk_validate(step_merged, &step_limits, &step_report) == MK_OK);
        assert(step_report.checks[MK_CHECK_VELOCITY].status != MK_CHECK_FAILED);
        assert(step_report.checks[MK_CHECK_ACCELERATION].status != MK_CHECK_FAILED);
        mk_trajectory_destroy(step_merged);
        mk_trajectory_destroy(step_trajectory);
        mk_time_law_destroy(law);
        mk_path_destroy(path);
    }

    // An input shaper convolves a move with delayed copies of itself: a quadratic run (accelerating at 8 m/s^2 for 0.1 s)
    // shaped by two equal impulses 20 ms apart, and a second joint that is only delayed by 10 ms.
    {
        mk_segment run{};
        run.struct_size = sizeof(run);
        run.t0_ns = 0;
        run.duration_ns = 100'000'000;
        run.degree = 2;
        run.joint_count = 2;
        run.coefficients[0].value[2] = 4.0;
        run.coefficients[1].value[2] = 4.0;
        mk_trajectory_handle original{}, shaped{};
        assert(mk_trajectory_create(2, &original) == MK_OK);
        assert(mk_trajectory_append_segment(original, &run) == MK_OK);
        uint32_t counts[2] = {2, 1};
        double delays[2 * MK_MAX_SHAPER_IMPULSES] = {}, amplitudes[2 * MK_MAX_SHAPER_IMPULSES] = {};
        delays[1] = 0.02; amplitudes[0] = amplitudes[1] = 0.5;
        delays[MK_MAX_SHAPER_IMPULSES] = 0.01; amplitudes[MK_MAX_SHAPER_IMPULSES] = 1.0;
        assert(mk_shape_trajectory(original, counts, delays, amplitudes, 2, 2 * MK_MAX_SHAPER_IMPULSES, &shaped) == MK_OK);
        auto at = [&](mk_trajectory_handle h, double seconds, int joint) {
            mk_trajectory_state state{};
            state.struct_size = sizeof(state);
            assert(mk_trajectory_evaluate(h, static_cast<int64_t>(std::llround(seconds * 1e9)), &state) == MK_OK);
            return state.position[joint];
        };
        auto held = [&](double seconds, int joint) {
            if (seconds <= 0.0) return 0.0;
            return at(original, std::min(seconds, 0.1), joint);
        };
        // The longest delay (20 ms) lengthens the move; the joint with one impulse is the original 10 ms later.
        for (double time : {0.005, 0.015, 0.03, 0.07, 0.1, 0.11, 0.119}) {
            assert(std::abs(at(shaped, time, 0) - 0.5 * (held(time, 0) + held(time - 0.02, 0))) < 1e-12);
            assert(std::abs(at(shaped, time, 1) - held(time - 0.01, 1)) < 1e-12);
        }
        assert(std::abs(at(shaped, 0.12, 0) - 0.04) < 1e-12);
        uint32_t pieces = 0;
        assert(mk_trajectory_segment_count(shaped, &pieces) == MK_OK && pieces >= 3);
        // Amplitudes that do not add up to one, a negative delay and no impulses are refused.
        mk_trajectory_handle none{};
        amplitudes[1] = 0.6;
        assert(mk_shape_trajectory(original, counts, delays, amplitudes, 2, 2 * MK_MAX_SHAPER_IMPULSES, &none) == MK_ERROR_INVALID_ARGUMENT);
        amplitudes[1] = 0.5; delays[1] = -0.02;
        assert(mk_shape_trajectory(original, counts, delays, amplitudes, 2, 2 * MK_MAX_SHAPER_IMPULSES, &none) == MK_ERROR_INVALID_ARGUMENT);
        delays[1] = 0.02; counts[0] = 0;
        assert(mk_shape_trajectory(original, counts, delays, amplitudes, 2, 2 * MK_MAX_SHAPER_IMPULSES, &none) == MK_ERROR_INVALID_ARGUMENT);
        mk_trajectory_destroy(shaped);
        mk_trajectory_destroy(original);
    }
}
