#include "motionkit.h"

#include <cassert>
#include <cmath>
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
        nullptr, 0, 0.0, 0.0, 1e-6, &law, &timed_trajectory) == MK_OK);
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
    assert(mk_time_path(path, boundary_velocity_limit, acceleration_limit, 1,
        nullptr, 0, 0.15, 0.15, 1e-8, &law, &tight) == MK_OK);
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
        limits.max_velocity[joint] = 10.0;
        limits.max_acceleration[joint] = 20.0;
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
        static_cast<uint32_t>(rapid_caps.size()), 0.0, 0.0, 1e-6, &law,
        &rapid_trajectory) == MK_OK);
    mk_trajectory_destroy(rapid_trajectory);
    mk_time_law_destroy(law);
    mk_path_destroy(path);
}
