#include "motionkit.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include <thread>

struct Result {
    mk_time_law_handle law{};
    mk_trajectory_handle trajectory{};
    ~Result() { mk_time_law_destroy(law); mk_trajectory_destroy(trajectory); }
};
static void equivalent(const Result& a, const Result& b, double length) {
    int64_t duration_a = 0, duration_b = 0;
    assert(mk_trajectory_duration_ns(a.trajectory, &duration_a) == MK_OK);
    assert(mk_trajectory_duration_ns(b.trajectory, &duration_b) == MK_OK);
    assert(std::abs(duration_a - duration_b) <= 2);
    for (int i = 0; i <= 50; ++i) {
        mk_trajectory_state x{}, y{};
        x.struct_size = sizeof(x); y.struct_size = sizeof(y);
        assert(mk_trajectory_evaluate(a.trajectory, duration_a * i / 50, &x) == MK_OK);
        assert(mk_trajectory_evaluate(b.trajectory, duration_b * i / 50, &y) == MK_OK);
        assert(std::abs(x.position[0] - y.position[0]) < 1e-8);
        assert(std::abs(x.velocity[0] - y.velocity[0]) < 1e-8);
        assert(std::abs(x.acceleration[0] - y.acceleration[0]) < 1e-7);
        double ta = 0, tb = 0;
        assert(mk_path_distance_to_time(a.law, length * i / 50, &ta) == MK_OK);
        assert(mk_path_distance_to_time(b.law, length * i / 50, &tb) == MK_OK);
        assert(std::abs(ta - tb) < 1e-8);
    }
}
static void entry_intervals() {
    mk_path_sample samples[2]{};
    for (int i = 0; i < 2; ++i) {
        samples[i].struct_size = sizeof(mk_path_sample); samples[i].joint_count = 1;
        samples[i].s = samples[i].position[0] = 0.1 * i;
        samples[i].first[0] = 1;
    }
    const double velocity = 2, acceleration = 1;
    mk_path_handle path{};
    assert(mk_path_create(samples, 2, &path) == MK_OK);
    mk_path_timing_session_handle session{};
    assert(mk_path_timing_session_create(path, &velocity, &acceleration, 1, nullptr, 0,
        1e-7, 0, &session) == MK_OK);
    double lower = -1, upper = -1;
    assert(mk_path_timing_session_entry_interval(session, 0, 0, 1, &lower, &upper) == MK_OK);
    assert(lower == 0 && upper > 0.4 && upper < 0.5);
    const double braking_entry = upper;
    Result braking;
    assert(mk_path_timing_session_time(session, braking_entry, 0, nullptr,
        &braking.law, &braking.trajectory) == MK_OK);
    assert(mk_path_timing_session_entry_interval(session, 1, 1, 1, &lower, &upper) == MK_OK);
    assert(lower > 0.8 && upper > lower);
    Result moving;
    assert(mk_path_timing_session_time(session, (lower + upper) / 2, 1, nullptr,
        &moving.law, &moving.trajectory) == MK_OK);
    assert(mk_path_timing_session_entry_interval(session, 1, 1, 0, &lower, &upper) != MK_OK);
    assert(lower == 0 && upper == 0);
    assert(mk_path_timing_session_entry_interval(session, 0, 0, 1, &lower, &upper) == MK_OK);
    assert(lower == 0 && std::abs(upper - braking_entry) < 1e-12);
    assert(mk_path_timing_session_entry_interval(session, 0, 0, 0, &lower, &upper) == MK_OK);
    assert(lower == 0 && upper == 0);
    for (auto interval : {std::pair<double,double>{3,3}, {1,0}, {0,1e200}}) {
        lower = upper = 123;
        assert(mk_path_timing_session_entry_interval(session, interval.first, interval.second,
            1, &lower, &upper) != MK_OK);
        assert(lower == 0 && upper == 0);
    }
    assert(mk_path_timing_session_entry_interval(session, 0, 0, 1, &lower, &upper) == MK_OK);
    assert(lower == 0 && std::abs(upper - braking_entry) < 1e-12);
    mk_path_timing_session_destroy(session); mk_path_destroy(path);
    int64_t duration = 0;
    assert(mk_trajectory_duration_ns(braking.trajectory, &duration) == MK_OK && duration > 0);
}

static void terminal_intervals() {
    mk_path_sample samples[2]{};
    for (int i = 0; i < 2; ++i) {
        samples[i].struct_size = sizeof(mk_path_sample);
        samples[i].joint_count = 1;
        samples[i].s = samples[i].position[0] = 0.1 * i;
        samples[i].first[0] = 1;
    }
    mk_path_handle path{};
    assert(mk_path_create(samples, 2, &path) == MK_OK);
    const double velocity = 2, acceleration = 1;
    mk_path_timing_session_handle session{};
    assert(mk_path_timing_session_create(path, &velocity, &acceleration, 1, nullptr, 0,
        1e-7, 0, &session) == MK_OK);
    for (auto interval : {std::pair<double,double>{0,2}, {0.1,0.2}, {0,0.3}, {0.2,2}, {0,2}}) {
        Result result, fresh;
        double reached = -1;
        assert(mk_path_timing_session_time_to_interval(session, 0, interval.first, interval.second,
            &reached, nullptr, &result.law, &result.trajectory) == MK_OK);
        assert(reached >= interval.first && reached <= interval.second);
        // The wide interval's upper bound is physically unreachable from rest in 100 mm.
        assert(reached * reached <= 2 * acceleration * 0.1 + 1e-10);
        if (interval.second == 2) assert(reached > 0.4 && reached < 0.5);
        int64_t duration = 0;
        assert(mk_trajectory_duration_ns(result.trajectory, &duration) == MK_OK);
        mk_trajectory_state end{};
        end.struct_size = sizeof(end);
        assert(mk_trajectory_evaluate(result.trajectory, duration, &end) == MK_OK);
        assert(std::abs(end.velocity[0] - reached) < 1e-8);
        assert(mk_time_path(path, &velocity, &acceleration, 1, nullptr, 0, 0, reached,
            1e-7, 0, nullptr, &fresh.law, &fresh.trajectory) == MK_OK);
        equivalent(result, fresh, 0.1);
    }
    for (auto interval : {std::pair<double,double>{1,2}, {0.2,0.1}, {-1,2},
            {0,std::numeric_limits<double>::infinity()}, {0,1e200}}) {
        Result failed;
        failed.law.id = failed.trajectory.id = 123;
        double reached = 123;
        assert(mk_path_timing_session_time_to_interval(session, 0, interval.first, interval.second,
            &reached, nullptr, &failed.law, &failed.trajectory) != MK_OK);
        assert(reached == 0 && failed.law.id == 0 && failed.trajectory.id == 0);
    }
    Result stopped, fresh;
    assert(mk_path_timing_session_time(session, 0, 0, nullptr, &stopped.law, &stopped.trajectory) == MK_OK);
    assert(mk_time_path(path, &velocity, &acceleration, 1, nullptr, 0, 0, 0,
        1e-7, 0, nullptr, &fresh.law, &fresh.trajectory) == MK_OK);
    mk_path_timing_session_destroy(session);
    mk_path_destroy(path);
    equivalent(stopped, fresh, 0.1);
}

int main() {
    terminal_intervals();
    entry_intervals();
    for (int spans : {2, 9000}) {
        const double length = spans == 2 ? 1.0 : 0.009;
        std::vector<mk_path_sample> samples(spans + 1);
        for (int i = 0; i <= spans; ++i) {
            auto& sample = samples[i];
            sample.struct_size = sizeof(sample); sample.joint_count = 1;
            sample.s = length * i / spans;
            sample.position[0] = sample.s; sample.first[0] = 1;
        }
        std::vector<double> caps(spans, 0.2), original_caps = caps;
        double velocity = 0.4, acceleration = 1;
        mk_path_handle source{};
        assert(mk_path_create(samples.data(), samples.size(), &source) == MK_OK);
        mk_path_timing_session_handle session{123};
        assert(mk_path_timing_session_create({}, &velocity, &acceleration, 1, nullptr,
            0, 1e-6, 0, &session) == MK_ERROR_INVALID_HANDLE && session.id == 0);
        assert(mk_path_timing_session_create(source, nullptr, &acceleration, 1, nullptr,
            0, 1e-6, 0, &session) == MK_ERROR_INVALID_ARGUMENT && session.id == 0);
        assert(mk_path_timing_session_create(source, &velocity, &acceleration, 1, caps.data(),
            caps.size(), 1e-6, 0, &session) == MK_OK);
        mk_path_destroy(source);
        velocity = 0.001; acceleration = 0.001;
        for (auto& cap : caps) cap = 0.001;
        mk_path_handle fresh_path{};
        assert(mk_path_create(samples.data(), samples.size(), &fresh_path) == MK_OK);
        const double fresh_velocity = 0.4, fresh_acceleration = 1;
        Result preserved;
        assert(mk_path_timing_session_time(session, 0, 0, nullptr, &preserved.law, &preserved.trajectory) == MK_OK);
        for (auto endpoints : {std::pair<double,double>{0,0}, {0,0.01}, {0.02,0.01},
                {0,0.02}, {0,0.01}, {0,0.01}, {0,0}}) {
            Result reused, fresh;
            mk_timing_report report{};
            assert(mk_path_timing_session_time(session, endpoints.first, endpoints.second,
                &report, &reused.law, &reused.trajectory) == MK_OK);
            assert(report.windows >= 1 && report.boundary_lowerings == 0);
            if (spans == 9000) assert(report.windows > 1);
            assert(mk_time_path(fresh_path, &fresh_velocity, &fresh_acceleration, 1,
                original_caps.data(), original_caps.size(), endpoints.first, endpoints.second,
                1e-6, 0, nullptr, &fresh.law, &fresh.trajectory) == MK_OK);
            equivalent(reused, fresh, length);
        }
        if (spans == 2) {
            // Calls on one session serialize mutable solver state, while returned
            // results remain independently readable on their caller's thread.
            auto worker = [&](double end_speed) {
                for (int repeat = 0; repeat < 3; ++repeat) {
                    Result result;
                    assert(mk_path_timing_session_time(session, 0, end_speed, nullptr,
                        &result.law, &result.trajectory) == MK_OK);
                    int64_t duration = 0;
                    assert(mk_trajectory_duration_ns(result.trajectory, &duration) == MK_OK);
                    mk_trajectory_state end{};
                    end.struct_size = sizeof(end);
                    assert(mk_trajectory_evaluate(result.trajectory, duration, &end) == MK_OK);
                    assert(std::abs(end.velocity[0] - end_speed) < 1e-8);
                }
            };
            std::thread first(worker, 0.01), second(worker, 0.02);
            first.join(); second.join();
        }
        Result failed;
        failed.law.id = 123; failed.trajectory.id = 123;
        assert(mk_path_timing_session_time(session, 0, 10, nullptr, &failed.law, &failed.trajectory) != MK_OK);
        assert(failed.law.id == 0 && failed.trajectory.id == 0);
        assert(mk_path_timing_session_time(session, std::numeric_limits<double>::quiet_NaN(), 0,
            nullptr, &failed.law, &failed.trajectory) == MK_ERROR_INVALID_ARGUMENT);
        Result recovered;
        assert(mk_path_timing_session_time(session, 0, 0, nullptr, &recovered.law, &recovered.trajectory) == MK_OK);
        equivalent(recovered, preserved, length);
        mk_path_timing_session_destroy(session);
        mk_path_timing_session_destroy(session);
        assert(mk_path_timing_session_time(session, 0, 0, nullptr,
            &failed.law, &failed.trajectory) == MK_ERROR_INVALID_HANDLE);
        mk_path_destroy(fresh_path);
        equivalent(recovered, preserved, length);
        // Lazy binding diagnostics also survive every source/session handle.
        uint32_t count = 0;
        assert(mk_time_law_binding_count(preserved.law, &count) == MK_OK && count > 0);
        mk_timing_binding binding{};
        binding.struct_size = sizeof(binding);
        assert(mk_time_law_get_binding(preserved.law, count - 1, &binding) == MK_OK);
    }
}
