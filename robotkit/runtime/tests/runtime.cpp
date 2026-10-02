// This test uses assert for setup calls; keep them active in Release builds.
#include "robotkit_runtime.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace {

class FaultEndpoint final : public robotkit::RobotEndpoint {
public:
    bool fail_apply = false;
    bool fail_sample = false;
    int discard_count = 0;
    int emergency_stop_count = 0;
    int sample_count = 0;

    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            ++emergency_stop_count;
        return fail_apply ? RK_ERROR_BACKEND : RK_OK;
    }

    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        ++sample_count;
        if (fail_sample)
            return RK_ERROR_STALE_STATE;
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        state.received_timestamp_ns = timestamp_ns;
        state.joint_count = 2;
        return RK_OK;
    }

    void discard_pending() noexcept override { ++discard_count; }
};

class QueueExecutingEndpoint final : public robotkit::RobotEndpoint {
public:
    int plans = 0;
    int sampled_targets = 0;
    bool executes_trajectory_queue() const noexcept override { return true; }
    rk_result submit_device_plan(const robotkit::PlanRequest &, uint64_t,
        uint64_t, uint64_t, const rk_robot_runtime_blueprint &) override {
        ++plans;
        return RK_OK;
    }
    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_JOINT_TARGETS) ++sampled_targets;
        return RK_OK;
    }
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.joint_count = 2;
        state.source_timestamp_ns = timestamp_ns;
        state.active_plan_id = 99;
        state.committed_until_ns = 250'000'000;
        return RK_OK;
    }
};

class EchoEndpoint final : public robotkit::RobotEndpoint {
public:
    explicit EchoEndpoint(uint32_t joint_count, bool queue_support = true)
        : joint_count_(joint_count), queue_support_(queue_support) {}

    bool supports_trajectory_queue() const noexcept override { return queue_support_; }

    rk_result apply(const rk_robot_command &command) override {
        ++apply_count;
        last_command = command;
        if (command.kind == RK_COMMAND_STOP || command.kind == RK_COMMAND_EMERGENCY_STOP ||
            command.kind == RK_COMMAND_RESET_SAFETY) {
            if (command.kind == RK_COMMAND_EMERGENCY_STOP)
                stopped_ = true;
            else
                stopped_ = false;
            return RK_OK;
        }
        if (stopped_)
            return RK_ERROR_SAFETY_STOPPED;
        for (uint32_t index = 0; index < command.target_count; ++index) {
            const auto &target = command.targets[index];
            if (target.mode == RK_TARGET_POSITION)
                position_[target.joint] = target.target;
        }
        return RK_OK;
    }

    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        // Backend-reported modes must not clear runtime-owned safety state.
        state.mode = RK_ROBOT_MODE_IDLE;
        state.safety = RK_SAFETY_READY;
        state.joint_count = joint_count_;
        for (uint32_t index = 0; index < joint_count_; ++index)
            state.position[index] = position_[index];
        return RK_OK;
    }

    uint32_t apply_count = 0;
    rk_robot_command last_command{};

private:
    uint32_t joint_count_ = 0;
    bool queue_support_ = true;
    double position_[RK_MAX_JOINTS]{};
    bool stopped_ = false;
};

class OffsetEndpoint final : public robotkit::RobotEndpoint {
public:
    explicit OffsetEndpoint(double offset) : offset_(offset) {}
    rk_result apply(const rk_robot_command &command) override {
        for (uint32_t index = 0; index < command.target_count; ++index)
            if (command.targets[index].mode == RK_TARGET_POSITION)
                commanded_[command.targets[index].joint] = command.targets[index].target;
        return RK_OK;
    }
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        state.joint_count = 2;
        state.position[0] = commanded_[0] + offset_;
        state.position[1] = commanded_[1];
        return RK_OK;
    }
private:
    double offset_ = 0.0;
    double commanded_[2]{};
};

class StaleSampleEndpoint final : public robotkit::RobotEndpoint {
public:
    rk_result sample(uint64_t, rk_robot_state &state) override {
        if (++sample_count > 1)
            return RK_ERROR_STALE_STATE;
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = 42;
        state.joint_count = 2;
        return RK_OK;
    }

    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            emergency_stop_received = true;
        return RK_OK;
    }

    bool emergency_stop_received = false;
    uint32_t sample_count = 0;
};

class OutOfLimitEndpoint final : public robotkit::RobotEndpoint {
public:
    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            emergency_stop_received = true;
        return RK_OK;
    }

    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        state.joint_count = 2;
        state.position[0] = 1.5;
        return RK_OK;
    }

    bool emergency_stop_received = false;
};

class WrongSensorLayoutEndpoint final : public robotkit::RobotEndpoint {
public:
    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            emergency_stop_received = true;
        return RK_OK;
    }

    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        state.joint_count = 2;
        state.sensor_count = 1;
        state.sensors[0].sequence = 1;
        state.sensors[0].value_count = 1;
        state.sensors[0].values[0] = 1.0;
        return RK_OK;
    }

    bool emergency_stop_received = false;
};

void wait_for_sequence(robotkit::RobotRuntime &runtime, uint64_t sequence) {
    for (int attempt = 0; attempt != 100; ++attempt) {
        rk_robot_state state{};
        assert(runtime.snapshot(state) == RK_OK);
        if (state.sequence >= sequence)
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(false && "RobotRuntime worker did not publish a sample");
}

rk_robot_command velocity_batch(uint64_t sequence,
                                std::initializer_list<std::pair<uint32_t, double>> targets) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    for (const auto &[joint, velocity] : targets)
        value.targets[value.target_count++] = {joint, RK_TARGET_VELOCITY, velocity, 0.0, 0.0};
    return value;
}

rk_robot_command lifecycle_command(uint64_t sequence, rk_command_kind kind) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = kind;
    return value;
}

rk_robot_state apply_cycle(robotkit::RobotRuntime &runtime, uint64_t &timestamp) {
    assert(runtime.apply_pending_commands() == RK_OK);
    assert(runtime.publish_sample(timestamp += 100'000'000) == RK_OK);
    rk_robot_state state{};
    assert(runtime.snapshot(state) == RK_OK);
    return state;
}

rk_robot_command trajectory_command(uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    return value;
}

rk_robot_command segment_command(uint64_t sequence) {
    auto value = trajectory_command(sequence);
    value.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    return value;
}

robotkit::SegmentBatch linear_segment_chunk(double start, double slope,
                                                  uint64_t duration_ns, uint64_t tag = 0) {
    robotkit::SegmentBatch chunk{};
    chunk.segments.resize(1);
    chunk.tag = tag;
    auto &segment = chunk.segments[0];
    segment.time_from_start_ns = 0;
    segment.duration_ns = duration_ns;
    segment.degree = 1;
    segment.joint_count = 2;
    segment.coefficients[0].value[0] = start;
    segment.coefficients[0].value[1] = slope;
    segment.coefficients[1].value[0] = -start;
    segment.coefficients[1].value[1] = -slope;
    return chunk;
}

robotkit::SegmentBatch cubic_plan_chunk(uint64_t tag) {
    auto chunk = linear_segment_chunk(0.0, 0.0, 1'000'000'000, tag);
    chunk.segments[0].degree = 3;
    chunk.segments[0].coefficients[0].value[3] = 1.0;
    chunk.segments[0].coefficients[1].value[3] = -1.0;
    return chunk;
}

robotkit::SegmentBatch replacement_plan_chunk(uint64_t tag) {
    auto chunk = linear_segment_chunk(0.125, 0.75, 500'000'000, tag);
    chunk.segments[0].degree = 3;
    chunk.segments[0].coefficients[0].value[2] = 1.5;
    chunk.segments[0].coefficients[0].value[3] = -1.0;
    chunk.segments[0].coefficients[1].value[2] = -1.5;
    chunk.segments[0].coefficients[1].value[3] = 1.0;
    return chunk;
}

robotkit::SegmentBatch trajectory_batch(
    std::initializer_list<std::pair<uint64_t, double>> points, uint64_t tag = 0) {
    robotkit::SegmentBatch value{};
    value.tag = tag;
    auto previous = points.begin();
    assert(previous != points.end());
    for (auto next = previous + 1; next != points.end(); ++next, ++previous) {
        auto &segment = value.segments.emplace_back();
        segment.time_from_start_ns = previous->first;
        segment.duration_ns = next->first - previous->first;
        segment.degree = 1;
        segment.joint_count = 2;
        const double slope = (next->second - previous->second) /
            (static_cast<double>(segment.duration_ns) * 1e-9);
        segment.coefficients[0].value[0] = previous->second;
        segment.coefficients[0].value[1] = slope;
        segment.coefficients[1].value[0] = -previous->second;
        segment.coefficients[1].value[1] = -slope;
    }
    return value;
}

void unsupported_trajectory_queue_is_rejected(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count, false);
    robotkit::RobotRuntime runtime(blueprint, endpoint);
    assert(!runtime.supports_trajectory_queue());
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.1}})) == RK_ERROR_UNSUPPORTED);
    assert(runtime.apply_pending_commands() == RK_OK);
    rk_robot_state state{};
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.trajectory_queue_depth == 0);
    assert(state.trajectory_active == 0);
    assert(state.safety == RK_SAFETY_READY);
    assert(endpoint->apply_count == 0);
}

void timestamped_trajectory_interpolates_and_reports_progress(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.5},
                          {200'000'000, 1.0}})) == RK_OK);

    auto state = apply_cycle(runtime, timestamp);
    assert(state.position[0] == 0.0);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 2);
    assert(state.trajectory_time_ns == 0 && state.trajectory_duration_ns == 200'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.25) < 1e-9);
    assert(state.trajectory_active == 1 && state.trajectory_time_ns == 50'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.5) < 1e-9);
    assert(state.trajectory_queue_depth == 1 && state.trajectory_time_ns == 100'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.75) < 1e-6);
    assert(state.trajectory_queue_depth == 1 && state.trajectory_time_ns == 150'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 1.0) < 1e-9);
    assert(state.trajectory_active == 0 && state.trajectory_queue_depth == 0);
    assert(state.trajectory_time_ns == 0 && state.trajectory_duration_ns == 0);
}

void normal_stop_decelerates_active_trajectory(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {200'000'000, 1.0}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.position[0] == 0.0);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.25) < 1e-9);

    auto stop = lifecycle_command(2, RK_COMMAND_STOP);
    assert(runtime.submit(stop) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.5) < 1e-9);
    assert(state.mode == RK_ROBOT_MODE_STOPPING);
    assert(state.safety == RK_SAFETY_STOPPING);

    state = apply_cycle(runtime, timestamp);
    assert(state.position[0] > 0.5 && state.position[0] < 1.0);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.75) < 1e-6);
    assert(state.mode == RK_ROBOT_MODE_STOPPING);
}

void trajectory_stop_follows_path_and_reports_tag(
    const rk_robot_runtime_blueprint &blueprint) {
    auto slow_blueprint = blueprint;
    for (auto &joint : slow_blueprint.joints)
        joint.max_acceleration = 1.0;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(slow_blueprint.joint_count);
    robotkit::RobotRuntime runtime(slow_blueprint, endpoint, std::chrono::milliseconds(100));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {1'000'000'000, 1.0}}, 42)) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_tag == 42);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.1) < 1e-9);

    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    const auto stop_start = state.position[0];
    auto stop_state = apply_cycle(runtime, timestamp);
    assert(stop_state.trajectory_active == 1 && stop_state.trajectory_tag == 42);
    double previous_position = stop_state.position[0];
    int stop_cycles = 1;
    while (stop_state.trajectory_active) {
        stop_state = apply_cycle(runtime, timestamp);
        ++stop_cycles;
        assert(stop_state.position[0] + 1e-9 >= previous_position);
        assert(stop_state.position[0] <= 1.0 + 1e-9);
        previous_position = stop_state.position[0];
        assert(stop_cycles < 30);
    }
    assert(stop_cycles >= 8 && stop_cycles <= 14);
    assert(stop_state.trajectory_tag == 42);
    assert(stop_state.trajectory_tag_time_ns > 0);
    assert(std::abs(stop_state.position[0] -
        static_cast<double>(stop_state.trajectory_tag_time_ns) / 1'000'000'000.0) < 0.11);
    assert(stop_state.position[0] > stop_start);
}

/** Samples position(t) every step_ns over [0, duration_ns] into one chunk. */
template <typename Position>
robotkit::SegmentBatch sampled_batch(Position position, uint64_t duration_ns, uint64_t step_ns,
                                  uint64_t tag) {
    robotkit::SegmentBatch value{};
    value.tag = tag;
    for (uint64_t time = 0; time < duration_ns; time += step_ns) {
        auto &segment = value.segments.emplace_back();
        segment.time_from_start_ns = time;
        segment.duration_ns = std::min(step_ns, duration_ns - time);
        segment.degree = 1;
        segment.joint_count = 2;
        const double before = position(static_cast<double>(time) * 1e-9);
        const double after = position(static_cast<double>(time + segment.duration_ns) * 1e-9);
        const double slope = (after - before) / (static_cast<double>(segment.duration_ns) * 1e-9);
        segment.coefficients[0].value[0] = before;
        segment.coefficients[0].value[1] = slope;
        segment.coefficients[1].value[0] = -before;
        segment.coefficients[1].value[1] = -slope;
    }
    return value;
}

robotkit::SegmentBatch segments_from_samples(const robotkit::SegmentBatch &samples) {
    return samples;
}

robotkit::SegmentBatch segments_from_native(mk_trajectory_handle trajectory) {
    robotkit::SegmentBatch chunk{};
    uint32_t count = 0;
    assert(mk_trajectory_segment_count(trajectory, &count) == MK_OK);
    assert(count > 0 && count <= RK_MAX_TRAJECTORY_QUEUE_POINTS);
    for (uint32_t index = 0; index < count; ++index) {
        mk_segment source{};
        source.struct_size = sizeof(source);
        assert(mk_trajectory_get_segment(trajectory, index, &source) == MK_OK);
        auto &target = chunk.segments.emplace_back();
        target.time_from_start_ns = static_cast<uint64_t>(source.t0_ns);
        target.duration_ns = static_cast<uint64_t>(source.duration_ns);
        target.degree = source.degree;
        target.joint_count = source.joint_count;
        for (uint32_t joint = 0; joint < source.joint_count; ++joint)
            for (uint32_t degree = 0; degree <= source.degree; ++degree)
                target.coefficients[joint].value[degree] =
                    source.coefficients[joint].value[degree];
    }
    return chunk;
}

/** Largest |second difference| / dt^2 over consecutive observed positions. */
double peak_acceleration(const std::vector<double> &positions, double period_seconds) {
    double peak = 0.0;
    for (std::size_t index = 2; index < positions.size(); ++index)
        peak = std::max(peak, std::abs(positions[index] - 2.0 * positions[index - 1] +
            positions[index - 2]) / (period_seconds * period_seconds));
    return peak;
}

void trajectory_chunk_extends_running_stop(
    const rk_robot_runtime_blueprint &blueprint) {
    auto slow_blueprint = blueprint;
    for (auto &joint : slow_blueprint.joints)
        joint.max_acceleration = 1.0;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(slow_blueprint.joint_count);
    robotkit::RobotRuntime runtime(slow_blueprint, endpoint, std::chrono::milliseconds(100));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {1'000'000'000, 1.0}}, 7)) == RK_OK);
    apply_cycle(runtime, timestamp);
    auto state = apply_cycle(runtime, timestamp);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    double previous_position = state.position[0];
    double previous_step = 1.0;
    uint64_t sequence = 3;
    for (int cycle = 0; cycle < 40; ++cycle) {
        if (cycle == 2) {
            // More path arriving mid-stop, which would turn back towards 0,
            // must extend the stop without resuming full speed.
            assert(runtime.submit_segments(trajectory_command(sequence++),
                trajectory_batch({{0, 1.0}, {1'000'000'000, 0.0}}, 8)) == RK_OK);
        }
        if (cycle == 4) {
            // A repeated stop continues the running one instead of restarting it.
            assert(runtime.submit(lifecycle_command(sequence++, RK_COMMAND_STOP)) == RK_OK);
        }
        state = apply_cycle(runtime, timestamp);
        const double step = state.position[0] - previous_position;
        assert(step >= -1e-9);
        assert(step <= previous_step + 1e-9);
        previous_step = step;
        previous_position = state.position[0];
        if (!state.trajectory_active)
            break;
    }
    assert(state.trajectory_active == 0 && state.trajectory_queue_depth == 0);
    assert(state.trajectory_tag == 7);
    assert(state.position[0] < 1.0);
}

void trajectory_stop_counts_trajectory_braking(
    const rk_robot_runtime_blueprint &blueprint) {
    // The trajectory cruises at 0.5 for 0.2 s, then brakes at the joint limit.
    // A stop that lands in that braking must not add its own deceleration.
    constexpr double limit = 1.0;
    auto limited = blueprint;
    for (auto &joint : limited.joints)
        joint.max_acceleration = limit;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    const auto profile = [](double t) {
        if (t <= 0.2)
            return 0.5 * t;
        const double braking = std::min(t - 0.2, 0.5);
        return 0.1 + 0.5 * braking - 0.5 * braking * braking;
    };
    const double end = profile(0.7);
    assert(runtime.submit_segments(trajectory_command(1),
        sampled_batch(profile, 700'000'000, 10'000'000, 1)) == RK_OK);
    std::vector<double> positions;
    for (int cycle = 0; cycle < 22; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    for (int cycle = 0; cycle < 120; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(peak_acceleration(positions, 0.01) <= limit * 1.05);
    assert(positions.back() <= end + 1e-9);
    assert(std::abs(positions.back() - positions[positions.size() - 2]) < 1e-12);
}

void stop_beyond_queued_path_ramps_within_limits(
    const rk_robot_runtime_blueprint &blueprint) {
    // Only 0.1 s of path is queued at 1 m/s, but stopping at 1 m/s^2 needs
    // 1 s. The stop must finish on a limited ramp instead of stopping dead.
    constexpr double limit = 1.0;
    auto limited = blueprint;
    for (auto &joint : limited.joints)
        joint.max_acceleration = limit;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 1;
    plan.model_revision = limited.revision;
    plan.calibration_revision = limited.calibration_revision;
    plan.ends_at_rest = 0;
    plan.segments = sampled_batch([](double t) { return t; }, 100'000'000, 10'000'000, 1);
    assert(runtime.submit_plan(plan) == RK_OK);
    std::vector<double> positions;
    for (int cycle = 0; cycle < 6; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    for (int cycle = 0; cycle < 150; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(peak_acceleration(positions, 0.01) <= limit * 1.05);
    // Stopping distance from 1 m/s at 1 m/s^2 is 0.5 m.
    assert(positions.back() > 0.5 && positions.back() < 0.6);
    assert(std::abs(positions.back() - positions[positions.size() - 2]) < 1e-12);
    // With room to stop within the limit, the ramp is not a fault.
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety != RK_SAFETY_FAULT);
}

void stop_ramp_stays_within_travel(const rk_robot_runtime_blueprint &blueprint) {
    // At 1 m/s with 1 m/s^2 the stop needs 0.5 m, but the queued path ends
    // 0.15 m short of the upper travel limit at 1.0. The ramp must stop at
    // the limit rather than run into it, then report braking past its limit.
    constexpr double limit = 1.0;
    auto limited = blueprint;
    for (auto &joint : limited.joints)
        joint.max_acceleration = limit;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    auto initial = velocity_batch(1, {{0, 0.0}, {1, 0.0}});
    initial.targets[0].mode = RK_TARGET_POSITION;
    initial.targets[0].target = 0.8;
    initial.targets[1].mode = RK_TARGET_POSITION;
    initial.targets[1].target = -0.8;
    assert(runtime.submit(initial) == RK_OK);
    apply_cycle(runtime, timestamp);
    robotkit::PlanRequest plan{};
    plan.sequence = 2;
    plan.plan_id = 1;
    plan.model_revision = limited.revision;
    plan.calibration_revision = limited.calibration_revision;
    plan.start_position[0] = 0.8;
    plan.start_position[1] = -0.8;
    plan.ends_at_rest = 0;
    plan.segments = sampled_batch([](double t) { return 0.8 + t; }, 50'000'000, 10'000'000, 1);
    assert(runtime.submit_plan(plan) == RK_OK);
    std::vector<double> positions;
    for (int cycle = 0; cycle < 3; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(runtime.submit(lifecycle_command(3, RK_COMMAND_STOP)) == RK_OK);
    rk_result result = RK_OK;
    for (int cycle = 0; cycle < 100 && result == RK_OK; ++cycle) {
        result = runtime.apply_pending_commands();
        if (result == RK_OK)
            result = runtime.publish_sample(timestamp += 10'000'000);
        rk_robot_state state{};
        state.struct_size = sizeof(state);
        assert(runtime.snapshot(state) == RK_OK);
        positions.push_back(state.position[0]);
    }
    assert(result == RK_ERROR_LIMIT);
    double furthest = 0.0;
    for (double position : positions)
        furthest = std::max(furthest, position);
    assert(furthest <= 1.0 + 1e-12);
    assert(positions.back() > 0.95);
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    rk_robot_snapshot snapshot{};
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.fault_code == RK_FAULT_RAMP_LIMIT);
}

void faulted_batch_skips_commands_before_reset(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;

    auto invalid = velocity_batch(1, {{0, 0.2}});
    invalid.targets[0].mode = RK_TARGET_POSITION;
    invalid.targets[0].target = 2.0;
    assert(runtime.submit(invalid) == RK_OK);
    assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);

    assert(runtime.submit(velocity_batch(2, {{1, -0.7}})) == RK_OK);
    assert(runtime.submit(lifecycle_command(3, RK_COMMAND_RESET_SAFETY)) == RK_OK);
    assert(runtime.submit(velocity_batch(4, {{0, 0.25}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.safety == RK_SAFETY_READY);
    assert(state.velocity[0] == 0.25);
    assert(state.velocity[1] == 0.0);
}

void invalid_trajectory_chunk_is_atomic(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.2}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 1);

    auto invalid = trajectory_batch({{0, 0.2}, {100'000'000, 2.0}});
    assert(runtime.submit_segments(trajectory_command(2), invalid) == RK_OK);
    assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    assert(state.trajectory_active == 0 && state.trajectory_queue_depth == 0);
}

void trajectory_chunk_speed_is_limited(const rk_robot_runtime_blueprint &blueprint) {
    auto limited = blueprint;
    for (auto &joint : limited.joints)
        joint.max_velocity = 1.0;

    {
        // 0.1 m in 100 ms is 1 m/s: exactly at the limit.
        auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
        robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(50));
        uint64_t timestamp = 0;
        assert(runtime.submit_segments(trajectory_command(1),
            trajectory_batch({{0, 0.0}, {100'000'000, 0.1}})) == RK_OK);
        auto state = apply_cycle(runtime, timestamp);
        assert(state.safety == RK_SAFETY_READY && state.trajectory_active == 1);

        // Continuing from where the queue ends is accepted; jumping away is not.
        assert(runtime.submit_segments(trajectory_command(2),
            trajectory_batch({{0, 0.1}, {100'000'000, 0.2}})) == RK_OK);
        state = apply_cycle(runtime, timestamp);
        assert(state.safety == RK_SAFETY_READY && state.trajectory_queue_depth > 0);
        assert(runtime.submit_segments(trajectory_command(3),
            trajectory_batch({{0, 0.5}, {100'000'000, 0.5}})) == RK_OK);
        assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);
        assert(runtime.snapshot(state) == RK_OK);
        assert(state.safety == RK_SAFETY_FAULT);
        assert(state.trajectory_active == 0 && state.trajectory_queue_depth == 0);
    }

    {
        // 0.2 m in 100 ms is 2 m/s: twice the limit.
        auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
        robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(50));
        assert(runtime.submit_segments(trajectory_command(1),
            trajectory_batch({{0, 0.0}, {100'000'000, 0.2}})) == RK_OK);
        assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);
        rk_robot_state state{};
        state.struct_size = sizeof(state);
        assert(runtime.snapshot(state) == RK_OK);
        assert(state.safety == RK_SAFETY_FAULT && state.trajectory_queue_depth == 0);
    }
}

void trajectory_queue_is_bounded(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    constexpr uint32_t batch_size = 128;
    robotkit::SegmentBatch full{};
    for (uint32_t index = 0; index < batch_size; ++index) {
        auto &segment = full.segments.emplace_back();
        segment.time_from_start_ns = static_cast<uint64_t>(index) * 1'000'000;
        segment.duration_ns = 1'000'000;
        segment.degree = 1;
        segment.joint_count = 2;
    }
    const uint32_t chunks = RK_MAX_TRAJECTORY_QUEUE_POINTS / batch_size;
    uint64_t sequence = 1;
    for (uint32_t index = 0; index < chunks; ++index)
        assert(runtime.submit_segments(trajectory_command(sequence++), full) == RK_OK);
    // Pending mailbox chunks count towards the bound before the owner runs.
    assert(runtime.submit_segments(trajectory_command(sequence++), full) == RK_ERROR_QUEUE_FULL);
    assert(runtime.apply_pending_commands() == RK_OK);
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.trajectory_queue_depth <= RK_MAX_TRAJECTORY_QUEUE_POINTS);
    // Once queued, the published depth keeps the bound.
    assert(runtime.submit_segments(trajectory_command(sequence++), full) == RK_ERROR_QUEUE_FULL);
}

void mixed_queue_depth_counts_knots(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.1}})) == RK_OK);
    assert(runtime.submit_segments(segment_command(2),
        linear_segment_chunk(0.1, 1.0, 100'000'000)) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_queue_depth == 2);
    state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_queue_depth == 1);
}

void plan_submission_checks_and_replacement(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    blueprint.revision = 42;
    blueprint.calibration_revision = 7;
    blueprint.commit_lead_ns = 250'000'000;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
    auto first = cubic_plan_chunk(11);
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 11;
    plan.model_revision = 42;
    plan.calibration_revision = 7;
    plan.segments = first;
    plan.start_position[0] = 0.0;
    plan.start_position[1] = 0.0;
    assert(robotkit::validate_plan_for_blueprint(plan, blueprint) == RK_OK);
    plan.model_revision = 41;
    assert(runtime.submit_plan(plan) == RK_ERROR_MODEL_MISMATCH);
    rk_robot_snapshot rejected_snapshot{};
    assert(runtime.snapshot_full(rejected_snapshot) == RK_OK);
    assert(rejected_snapshot.trajectory_queue_depth == 0);
    plan.model_revision = 42;
    plan.calibration_revision = 6;
    assert(runtime.submit_plan(plan) == RK_ERROR_MODEL_MISMATCH);
    plan.calibration_revision = 7;
    plan.required_capabilities = 0x80000000u;
    assert(runtime.submit_plan(plan) == RK_ERROR_UNSUPPORTED);
    plan.required_capabilities = RK_PLAN_CAPABILITY_EVENTS;
    // An event-capable plan still needs a declared output channel.
    plan.events.resize(1);
    plan.events[0].time_ns = 0;
    std::strcpy(plan.events[0].channel, "sprayer.flow");
    plan.events[0].value.kind = RK_EVENT_DIGITAL;
    plan.events[0].value.digital = 1;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_ARGUMENT);
    plan.events.resize(0);
    plan.required_capabilities = 0;
    plan.start_position[0] = 0.1;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    plan.start_position[0] = 0.0;
    plan.required_capabilities = RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE;
    assert(runtime.submit_plan(plan) == RK_OK);
    rk_robot_snapshot snapshot{};
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.active_plan_id == 11 && snapshot.trajectory_queue_depth == 1);
    uint64_t timestamp = 0;
    apply_cycle(runtime, timestamp);
    apply_cycle(runtime, timestamp);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.committed_until_ns == 350'000'000);
    assert(snapshot.queue_end_time_ns == 1'000'000'000);
    assert(snapshot.session_state == RK_SESSION_EXECUTING);
    auto replacement = replacement_plan_chunk(12);
    plan.sequence = 2;
    plan.plan_id = 12;
    plan.segments = replacement;
    plan.replace_after_plan_id = 11;
    plan.replace_after_time_ns = 200'000'000;
    plan.start_position[0] = 0.008;
    plan.start_position[1] = -0.008;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.active_plan_id == 11 && snapshot.queue_end_time_ns == 1'000'000'000);
    plan.replace_after_time_ns = 500'000'000;
    plan.start_position[0] = 0.125;
    plan.start_position[1] = -0.125;
    plan.start_velocity[0] = 0.75;
    plan.start_velocity[1] = -0.75;
    plan.start_acceleration[0] = 3.0;
    plan.start_acceleration[1] = -3.0;
    assert(runtime.submit_plan(plan) == RK_OK);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.queue_end_time_ns == 1'000'000'000);
    auto state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.008) < 1e-12);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.027) < 1e-12);
}

void device_queue_endpoint_does_not_receive_sampled_targets(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<QueueExecutingEndpoint>();
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 99;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.segments = cubic_plan_chunk(99);
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    apply_cycle(runtime, timestamp);
    apply_cycle(runtime, timestamp);
    assert(endpoint->plans == 1);
    assert(endpoint->sampled_targets == 0);
    rk_robot_snapshot snapshot{};
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.active_plan_id == 99 && snapshot.committed_until_ns == 250'000'000);
}

void plan_events_follow_path_clock(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
        blueprint.joints[joint].max_acceleration = 100.0;
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.digital = 0;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 100;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.required_capabilities = RK_PLAN_CAPABILITY_EVENTS;
    plan.segments = cubic_plan_chunk(100);
    plan.events.resize(2);
    std::strcpy(plan.events[0].channel, "sprayer.flow");
    plan.events[0].time_ns = 0;
    plan.events[0].value.kind = RK_EVENT_DIGITAL;
    plan.events[0].value.digital = 1;
    plan.events[0].hold_policy = RK_EVENT_RESTORE_ON_RESUME;
    plan.events[1] = plan.events[0];
    plan.events[1].time_ns = 200'000'000;
    plan.events[1].value.digital = 0;
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    apply_cycle(runtime, timestamp);
    rk_event_record_batch records{};
    records.struct_size = sizeof(records);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].cause == RK_EVENT_SCHEDULED);
    assert(records.records[0].scheduled_time_ns == 0);
    for (int cycle = 0; cycle < 10; ++cycle) apply_cycle(runtime, timestamp);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_HOLD)) == RK_OK);
    apply_cycle(runtime, timestamp);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].cause == RK_EVENT_HOLD_SAFE);
    for (int cycle = 0; cycle < 40; ++cycle) apply_cycle(runtime, timestamp);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 0);
    assert(runtime.submit(lifecycle_command(3, RK_COMMAND_RESUME)) == RK_OK);
    apply_cycle(runtime, timestamp);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].cause == RK_EVENT_RESUME_RESTORE);
    for (int cycle = 0; cycle < 40; ++cycle) apply_cycle(runtime, timestamp);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].scheduled_time_ns == 200'000'000);
    assert(records.records[0].applied_owner_time_ns >= records.records[0].scheduled_time_ns);
}

void plan_event_records_report_overflow(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 101;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.required_capabilities = RK_PLAN_CAPABILITY_EVENTS;
    plan.segments = cubic_plan_chunk(101);
    plan.events.resize(RK_MAX_EVENT_RECORDS + 6);
    for (uint32_t index = 0; index < plan.events.size(); ++index) {
        std::strcpy(plan.events[index].channel, "sprayer.flow");
        plan.events[index].value.kind = RK_EVENT_DIGITAL;
        plan.events[index].value.digital = index % 2;
    }
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    apply_cycle(runtime, timestamp);
    rk_event_record_batch records{};
    records.struct_size = sizeof(records);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == RK_MAX_EVENT_RECORDS && records.overflow == 1);
    assert(records.records[0].value.digital == 0);
    assert(runtime.poll_events(records) == RK_OK);
    assert(records.count == 0 && records.overflow == 0);
}

void accepted_plan_keeps_committed_region_identical(
    const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    auto reference_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    auto replacement_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime reference(blueprint, reference_endpoint,
        std::chrono::milliseconds(1));
    robotkit::RobotRuntime replacement(blueprint, replacement_endpoint,
        std::chrono::milliseconds(1));
    robotkit::PlanRequest first{};
    first.sequence = 1;
    first.plan_id = 71;
    first.model_revision = blueprint.revision;
    first.calibration_revision = blueprint.calibration_revision;
    first.segments = cubic_plan_chunk(71);
    first.required_capabilities = RK_PLAN_CAPABILITY_EVENTS;
    first.events.resize(1);
    first.events[0].time_ns = 700'000'000;
    std::strcpy(first.events[0].channel, "sprayer.flow");
    first.events[0].value.kind = RK_EVENT_DIGITAL;
    first.events[0].value.digital = 1;
    assert(reference.submit_plan(first) == RK_OK);
    assert(replacement.submit_plan(first) == RK_OK);
    uint64_t ref_time = 0, replacement_time = 0;
    for (int tick = 0; tick < 100; ++tick) {
        apply_cycle(reference, ref_time);
        apply_cycle(replacement, replacement_time);
    }
    robotkit::PlanRequest next = first;
    next.sequence = 2;
    next.plan_id = 72;
    next.replace_after_plan_id = 71;
    next.replace_after_time_ns = 500'000'000;
    next.start_position[0] = 0.125;
    next.start_position[1] = -0.125;
    next.start_velocity[0] = 0.75;
    next.start_velocity[1] = -0.75;
    next.start_acceleration[0] = 3.0;
    next.start_acceleration[1] = -3.0;
    next.segments = replacement_plan_chunk(72);
    next.events[0].time_ns = 200'000'000;
    next.events[0].value.digital = 0;
    assert(replacement.submit_plan(next) == RK_OK);
    // One sample per millisecond across the entire committed part, not just
    // the replacement junction. The two queues must produce identical bytes.
    for (int tick = 100; tick < 500; ++tick) {
        const auto before = apply_cycle(reference, ref_time);
        const auto after = apply_cycle(replacement, replacement_time);
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
            assert(std::memcmp(&before.position[joint], &after.position[joint],
                sizeof(double)) == 0);
    }
    for (int tick = 500; tick < 800; ++tick) {
        apply_cycle(reference, ref_time);
        apply_cycle(replacement, replacement_time);
    }
    rk_event_record_batch records{};
    records.struct_size = sizeof(records);
    assert(reference.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].value.digital == 1);
    assert(replacement.poll_events(records) == RK_OK);
    assert(records.count == 1 && records.records[0].plan_id == 72 &&
        records.records[0].value.digital == 0);
}

void moving_degree_one_plan_cannot_retarget(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 81;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.segments = linear_segment_chunk(0.0, 1.0, 1'000'000'000, 81);
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    for (int tick = 0; tick < 10; ++tick) apply_cycle(runtime, timestamp);
    plan.sequence = 2;
    plan.plan_id = 82;
    plan.replace_after_plan_id = 81;
    plan.replace_after_time_ns = 500'000'000;
    plan.start_position[0] = 0.5;
    plan.start_position[1] = -0.5;
    plan.segments = linear_segment_chunk(0.5, 0.5, 500'000'000, 82);
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
}

void idle_plan_uses_commanded_anchor_and_following_error(
    const rk_robot_runtime_blueprint &source) {
    // Four runtimes and two plans exceed a comfortable stack; keep the
    // runtimes on the heap, as the C API does.
    auto blueprint = source;
    blueprint.following_error_bound[0] = 0.0002;
    auto accepted_endpoint = std::make_shared<OffsetEndpoint>(0.0001);
    auto accepted_owner = std::make_unique<robotkit::RobotRuntime>(blueprint, accepted_endpoint,
        std::chrono::milliseconds(10));
    auto &accepted = *accepted_owner;
    assert(accepted.publish_sample(1) == RK_OK);
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 91;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.segments = cubic_plan_chunk(91);
    const auto initial_plan = plan;
    assert(accepted.submit_plan(plan) == RK_OK);

    auto held_endpoint = std::make_shared<OffsetEndpoint>(0.0001);
    auto held_owner = std::make_unique<robotkit::RobotRuntime>(blueprint, held_endpoint, std::chrono::milliseconds(10));
    auto &held = *held_owner;
    uint64_t timestamp = 0;
    assert(held.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {1'000'000'000, 0.5}}, 90)) == RK_OK);
    for (int tick = 0; tick < 101; ++tick) apply_cycle(held, timestamp);
    rk_robot_state held_state{};
    assert(held.snapshot(held_state) == RK_OK);
    assert(held_state.trajectory_queue_depth == 0);
    assert(std::abs(held_state.position[0] - 0.5001) < 1e-12);
    assert(held.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    apply_cycle(held, timestamp);
    plan.sequence = 3;
    plan.start_position[0] = 0.5;
    plan.start_position[1] = -0.5;
    plan.segments.segments[0].coefficients[0].value[0] = 0.5;
    plan.segments.segments[0].coefficients[0].value[3] = 0.2;
    plan.segments.segments[0].coefficients[1].value[0] = -0.5;
    plan.segments.segments[0].coefficients[1].value[3] = -0.2;
    assert(held.submit_plan(plan) == RK_OK);

    auto rejected_endpoint = std::make_shared<OffsetEndpoint>(0.0003);
    auto rejected_owner = std::make_unique<robotkit::RobotRuntime>(blueprint, rejected_endpoint,
        std::chrono::milliseconds(10));
    auto &rejected = *rejected_owner;
    assert(rejected.publish_sample(1) == RK_OK);
    assert(rejected.submit_plan(plan) == RK_ERROR_FOLLOWING_ERROR);
    rk_robot_snapshot snapshot{};
    assert(rejected.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.trajectory_queue_depth == 0);

    auto unchecked_blueprint = source;
    auto unchecked_endpoint = std::make_shared<OffsetEndpoint>(0.0003);
    auto unchecked_owner = std::make_unique<robotkit::RobotRuntime>(unchecked_blueprint, unchecked_endpoint,
        std::chrono::milliseconds(10));
    auto &unchecked = *unchecked_owner;
    assert(unchecked.publish_sample(1) == RK_OK);
    assert(unchecked.submit_plan(initial_plan) == RK_OK);
}

void submitted_start_tolerances_control_acceptance(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 92;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.start_position[0] = 0.00005;
    plan.start_velocity[0] = 0.00002;
    plan.start_acceleration[0] = 0.00003;
    plan.segments = cubic_plan_chunk(92);
    plan.segments.segments[0].coefficients[0].value[0] = 0.00005;
    plan.segments.segments[0].coefficients[0].value[1] = 0.00002;
    plan.segments.segments[0].coefficients[0].value[2] = 0.000015;
    plan.segments.segments[0].coefficients[0].value[3] = 0.5;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    plan.position_tolerance[0] = 0.0001;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    plan.velocity_tolerance[0] = 0.0001;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    plan.acceleration_tolerance[0] = 0.0001;
    assert(runtime.submit_plan(plan) == RK_OK);
}

void degree_one_append_checks_chord_velocity(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {1'000'000'000, 0.5}}, 93)) == RK_OK);
    apply_cycle(runtime, timestamp);
    robotkit::PlanRequest plan{};
    plan.sequence = 2;
    plan.plan_id = 94;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.segments = linear_segment_chunk(0.5, 0.4, 100'000'000, 94);
    plan.segments.segments[0].degree = 3;
    plan.start_position[0] = 0.5;
    plan.start_position[1] = -0.5;
    plan.start_velocity[0] = 0.4;
    plan.start_velocity[1] = -0.4;
    assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_STATE);
    plan.start_velocity[0] = 0.5;
    plan.start_velocity[1] = -0.5;
    plan.segments.segments[0].coefficients[0].value[1] = 0.5;
    plan.segments.segments[0].coefficients[1].value[1] = -0.5;
    assert(runtime.submit_plan(plan) == RK_OK);
}

void ruckig_segments_match_motionkit_evaluation(
    const rk_robot_runtime_blueprint &blueprint) {
    auto limited = blueprint;
    for (auto &joint : limited.joints) {
        joint.max_velocity = 1.0;
        joint.max_acceleration = 2.0;
    }
    mk_state_to_state_request request{};
    request.struct_size = sizeof(request);
    request.joint_count = 2;
    request.target_position[0] = 0.5;
    request.target_position[1] = -0.5;
    for (uint32_t joint = 0; joint < 2; ++joint) {
        request.max_velocity[joint] = 1.0;
        request.max_acceleration[joint] = 2.0;
        request.max_jerk[joint] = 4.0;
    }
    mk_trajectory_handle trajectory{};
    int32_t result = 0;
    assert(mk_generate_state_to_state(&request, &trajectory, &result) == MK_OK);
    auto chunk = segments_from_native(trajectory);
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(segment_command(1), chunk) == RK_OK);
    int64_t duration = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration) == MK_OK);
    for (int64_t time = 0; time < duration; time += 10'000'000) {
        const auto state = apply_cycle(runtime, timestamp);
        mk_trajectory_state reference{};
        reference.struct_size = sizeof(reference);
        assert(mk_trajectory_evaluate(trajectory, time, &reference) == MK_OK);
        for (uint32_t joint = 0; joint < 2; ++joint)
            assert(std::abs(state.position[joint] - reference.position[joint]) < 1e-12);
    }
    mk_trajectory_destroy(trajectory);
}

void overacceleration_segment_is_rejected(const rk_robot_runtime_blueprint &blueprint) {
    auto limited = blueprint;
    for (auto &joint : limited.joints) joint.max_acceleration = 1.0;
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    auto chunk = linear_segment_chunk(0.0, 0.0, 100'000'000);
    chunk.segments[0].degree = 2;
    chunk.segments[0].coefficients[0].value[2] = 1.0;
    chunk.segments[0].coefficients[1].value[2] = -1.0;
    assert(runtime.submit_segments(segment_command(1), chunk) == RK_OK);
    assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
}

void segment_junction_jump_is_rejected(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    assert(runtime.submit_segments(segment_command(1),
        linear_segment_chunk(0.0, 1.0, 100'000'000)) == RK_OK);
    assert(runtime.apply_pending_commands() == RK_OK);
    assert(runtime.submit_segments(segment_command(2),
        linear_segment_chunk(0.3, 1.0, 100'000'000)) == RK_OK);
    assert(runtime.apply_pending_commands() == RK_ERROR_LIMIT);
}

void stop_braking_uses_segment_degree(const rk_robot_runtime_blueprint &blueprint) {
    auto limited = blueprint;
    for (auto &joint : limited.joints) joint.max_acceleration = 1.0;
    const auto profile = [](double t) {
        if (t <= 0.2) return 0.5 * t;
        const double braking = std::min(t - 0.2, 0.5);
        return 0.1 + 0.5 * braking - 0.5 * braking * braking;
    };
    auto samples = sampled_batch(profile, 700'000'000, 10'000'000, 1);
    auto degree_one = segments_from_samples(samples);
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(segment_command(1), degree_one) == RK_OK);
    std::vector<double> positions;
    for (int cycle = 0; cycle < 22; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    for (int cycle = 0; cycle < 120; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(peak_acceleration(positions, 0.01) <= 1.05);

    auto analytic = linear_segment_chunk(0.0, 0.5, 500'000'000);
    analytic.segments[0].degree = 2;
    analytic.segments[0].coefficients[0].value[2] = -0.5;
    analytic.segments[0].coefficients[1].value[2] = 0.5;
    auto analytic_endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime analytic_runtime(limited, analytic_endpoint,
        std::chrono::milliseconds(10));
    timestamp = 0;
    assert(analytic_runtime.submit_segments(segment_command(1), analytic) == RK_OK);
    positions.clear();
    for (int cycle = 0; cycle < 10; ++cycle)
        positions.push_back(apply_cycle(analytic_runtime, timestamp).position[0]);
    assert(analytic_runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    for (int cycle = 0; cycle < 60; ++cycle)
        positions.push_back(apply_cycle(analytic_runtime, timestamp).position[0]);
    assert(peak_acceleration(positions, 0.01) <= 1.05);
}

void ruckig_segment_stop_uses_analytic_braking(
    const rk_robot_runtime_blueprint &blueprint) {
    auto limited = blueprint;
    for (auto &joint : limited.joints) {
        joint.max_velocity = 1.0;
        joint.max_acceleration = 1.0;
    }
    mk_state_to_state_request request{};
    request.struct_size = sizeof(request);
    request.joint_count = 2;
    for (uint32_t joint = 0; joint < 2; ++joint) {
        const double sign = joint == 0 ? 1.0 : -1.0;
        request.current_velocity[joint] = sign * 0.5;
        request.target_position[joint] = sign * 0.25;
        request.max_velocity[joint] = 1.0;
        request.max_acceleration[joint] = 1.0;
        request.max_jerk[joint] = 4.0;
    }
    mk_trajectory_handle trajectory{};
    int32_t ruckig_result = 0;
    assert(mk_generate_state_to_state(&request, &trajectory, &ruckig_result) == MK_OK);
    int64_t duration = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration) == MK_OK);
    int64_t braking_time = -1;
    for (int64_t time = 0; time < duration; time += 10'000'000) {
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        assert(mk_trajectory_evaluate(trajectory, time, &state) == MK_OK);
        if (state.acceleration[0] < -0.2 && state.velocity[0] > 0.05) {
            braking_time = time;
            break;
        }
    }
    assert(braking_time >= 0);
    mk_trajectory_state analytic{};
    analytic.struct_size = sizeof(analytic);
    assert(mk_trajectory_evaluate(trajectory, braking_time, &analytic) == MK_OK);
    mk_path_derivative_estimate estimate{};
    estimate.struct_size = sizeof(estimate);
    assert(mk_trajectory_estimate_path_derivatives(trajectory, braking_time,
        10'000'000, &estimate) == MK_OK);
    assert(std::abs(estimate.velocity[0] - analytic.velocity[0]) < 1e-12);
    assert(std::abs(estimate.acceleration[0] - analytic.acceleration[0]) < 1e-12);
    auto chunk = segments_from_native(trajectory);
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    assert(runtime.submit_segments(segment_command(1), chunk) == RK_OK);
    std::vector<double> positions;
    for (int64_t time = 0; time <= braking_time; time += 10'000'000)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_STOP)) == RK_OK);
    for (int cycle = 0; cycle < 150; ++cycle)
        positions.push_back(apply_cycle(runtime, timestamp).position[0]);
    assert(peak_acceleration(positions, 0.01) <= 1.05);
    mk_trajectory_destroy(trajectory);
}

void same_cycle_target_batches_merge_per_joint(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
    uint64_t timestamp = 0;

    // Separate batches for different joints in one cycle both apply, like a
    // drive and an arm commanded independently.
    assert(runtime.submit(velocity_batch(1, {{0, 0.2}})) == RK_OK);
    assert(runtime.submit(velocity_batch(2, {{1, -0.3}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.2 && state.velocity[1] == -0.3);

    // For a joint both batches set, the newer target wins; a joint neither
    // batch names keeps its target.
    assert(runtime.submit(velocity_batch(3, {{0, 0.5}})) == RK_OK);
    assert(runtime.submit(velocity_batch(4, {{0, 0.1}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.1 && state.velocity[1] == -0.3);

    // A newest stop wins alone: the earlier target in the cycle never runs.
    assert(runtime.submit(velocity_batch(5, {{1, 0.4}})) == RK_OK);
    assert(runtime.submit(lifecycle_command(6, RK_COMMAND_STOP)) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.0 && state.velocity[1] == 0.0);

    // Only batches newer than the latest stop merge.
    assert(runtime.submit(velocity_batch(7, {{0, 0.3}, {1, 0.3}})) == RK_OK);
    assert(runtime.submit(lifecycle_command(8, RK_COMMAND_STOP)) == RK_OK);
    assert(runtime.submit(velocity_batch(9, {{0, 0.2}})) == RK_OK);
    assert(runtime.submit(velocity_batch(10, {{1, 0.25}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.2 && state.velocity[1] == 0.25);

    // An emergency stop anywhere in the cycle wins over every batch.
    assert(runtime.submit(velocity_batch(11, {{0, 0.4}})) == RK_OK);
    assert(runtime.submit(lifecycle_command(12, RK_COMMAND_EMERGENCY_STOP)) == RK_OK);
    assert(runtime.submit(velocity_batch(13, {{1, 0.4}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.safety == RK_SAFETY_EMERGENCY_STOP);
    assert(state.velocity[0] == 0.0 && state.velocity[1] == 0.0);
}

void partial_targets_and_ordered_trajectory_commands(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;

    // Partial target batches retain joints that the newer batch does not
    // name, including across owner cycles.
    assert(runtime.submit(velocity_batch(1, {{0, 0.2}, {1, -0.3}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.2 && state.velocity[1] == -0.3);
    assert(runtime.submit(velocity_batch(2, {{0, 0.4}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.velocity[0] == 0.4 && state.velocity[1] == -0.3);

    // Updating another joint must not reset an in-progress position
    // reference or silently drop its rate-limited motion.
    auto position_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime position_runtime(blueprint, position_endpoint,
                                            std::chrono::milliseconds(50));
    auto position = velocity_batch(1, {{1, 0.2}});
    position.targets[0] = {0, RK_TARGET_POSITION, 0.8, 1.0, 0.0};
    assert(position_runtime.submit(position) == RK_OK);
    uint64_t position_timestamp = 0;
    state = apply_cycle(position_runtime, position_timestamp);
    assert(std::abs(state.position[0] - 0.05) < 1e-9);
    assert(position_runtime.submit(velocity_batch(2, {{1, 0.3}})) == RK_OK);
    state = apply_cycle(position_runtime, position_timestamp);
    assert(std::abs(state.position[0] - 0.1) < 1e-9);

    // Multiple chunks in one mailbox drain append in sequence order instead
    // of the newest chunk replacing the earlier one.
    assert(runtime.submit_segments(trajectory_command(3),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.2}})) == RK_OK);
    assert(runtime.submit_segments(trajectory_command(4),
        trajectory_batch({{0, 0.2}, {100'000'000, 0.4}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 2);
    assert(state.trajectory_duration_ns == 200'000'000);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.1) < 1e-9);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.2) < 1e-9);
    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.3) < 1e-9);

    // A stop followed by a replacement chunk clears the old queue before the
    // new motion is accepted; the final owner output is the new trajectory.
    assert(runtime.submit(lifecycle_command(5, RK_COMMAND_STOP)) == RK_OK);
    assert(runtime.submit_segments(trajectory_command(6),
        trajectory_batch({{0, 0.1}, {100'000'000, 0.3}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 1);
    assert(std::abs(state.position[0] - 0.1) < 1e-9);
    assert(state.mode == RK_ROBOT_MODE_TRACKING);

    // Reset and a chunk in one owner cycle are also sequential. The reset is
    // delivered to the endpoint before the generated trajectory setpoint.
    auto reset_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime reset_runtime(blueprint, reset_endpoint,
                                         std::chrono::milliseconds(50));
    assert(reset_runtime.submit(lifecycle_command(1, RK_COMMAND_EMERGENCY_STOP)) == RK_OK);
    state = apply_cycle(reset_runtime, timestamp);
    assert(state.safety == RK_SAFETY_EMERGENCY_STOP);
    assert(reset_runtime.submit(lifecycle_command(2, RK_COMMAND_RESET_SAFETY)) == RK_OK);
    assert(reset_runtime.submit_segments(trajectory_command(3),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.25}})) == RK_OK);
    state = apply_cycle(reset_runtime, timestamp);
    assert(state.safety == RK_SAFETY_READY);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 1);
}

} // namespace

void declared_plan_completion_and_underflow(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        blueprint.joints[joint].max_acceleration = 1.0;
        blueprint.joints[joint].max_velocity = 1.0;
    }
    auto make_plan = [&](uint64_t id, bool ends_at_rest, bool smooth) {
        robotkit::PlanRequest plan{};
        plan.sequence = 1;
        plan.plan_id = id;
        plan.model_revision = blueprint.revision;
        plan.calibration_revision = blueprint.calibration_revision;
        plan.ends_at_rest = ends_at_rest ? 1u : 0u;
        plan.segments = linear_segment_chunk(0.0, 0.2, 1'000'000'000, id);
        if (smooth) {
            plan.segments.segments[0].degree = 4;
            for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
                const double sign = joint == 0 ? 1.0 : -1.0;
                plan.segments.segments[0].coefficients[joint].value[1] = 0.0;
                plan.segments.segments[0].coefficients[joint].value[2] = 0.0;
                plan.segments.segments[0].coefficients[joint].value[3] = sign * 0.2;
                plan.segments.segments[0].coefficients[joint].value[4] = sign * -0.1;
            }
        }
        return plan;
    };
    {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto plan = make_plan(201, true, false);
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        for (int step = 0; step < 14; ++step) apply_cycle(runtime, timestamp);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.trajectory_active == 0 && snapshot.fault_code == 0);
        assert(std::abs(snapshot.position[0] - 0.2) < 1e-9);
    }
    {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto plan = make_plan(202, true, true);
        assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_ARGUMENT);
    }
    {
        // Reach zero velocity with bounded but nonzero endpoint acceleration,
        // as a TOPP-RA path can do at its final knot.
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto plan = make_plan(205, true, false);
        plan.flags = RK_PLAN_JERK_UNCHECKED;
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const double sign = joint == 0 ? 1.0 : -1.0;
            auto &segment = plan.segments.segments[0];
            segment.degree = 4;
            segment.coefficients[joint].value[1] = 0.0;
            segment.coefficients[joint].value[2] = 0.0;
            segment.coefficients[joint].value[3] = sign * 0.08;
            segment.coefficients[joint].value[4] = sign * -0.06;
        }
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        for (int step = 0; step < 14; ++step) apply_cycle(runtime, timestamp);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.trajectory_active == 0 && snapshot.fault_code == 0);
    }
    {
        // A checked-jerk plan must finish with zero acceleration.
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto plan = make_plan(206, true, false);
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const double sign = joint == 0 ? 1.0 : -1.0;
            auto &segment = plan.segments.segments[0];
            segment.degree = 4;
            segment.coefficients[joint].value[1] = 0.0;
            segment.coefficients[joint].value[2] = 0.0;
            segment.coefficients[joint].value[3] = sign * 0.08;
            segment.coefficients[joint].value[4] = sign * -0.06;
        }
        assert(runtime.submit_plan(plan) == RK_ERROR_INVALID_ARGUMENT);
        plan.flags = RK_PLAN_JERK_UNCHECKED;
        assert(runtime.submit_plan(plan) == RK_OK);
    }
    for (bool unchecked : {false, true}) {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto first = make_plan(207, false, false);
        first.segments.segments[0].degree = 2;
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const double sign = joint == 0 ? 1.0 : -1.0;
            first.segments.segments[0].coefficients[joint].value[1] = 0.0;
            first.segments.segments[0].coefficients[joint].value[2] = sign * 0.1;
            first.start_acceleration[joint] = sign * 0.2;
            first.acceleration_tolerance[joint] = 0.5;
        }
        assert(runtime.submit_plan(first) == RK_OK);
        auto second = make_plan(208, false, false);
        second.sequence = 2;
        second.flags = unchecked ? RK_PLAN_JERK_UNCHECKED : 0;
        second.segments.segments[0].degree = 2;
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const double sign = joint == 0 ? 1.0 : -1.0;
            second.start_position[joint] = sign * 0.1;
            second.start_velocity[joint] = sign * 0.2;
            second.start_acceleration[joint] = sign * 0.4;
            second.acceleration_tolerance[joint] = 0.5;
            auto &coefficients = second.segments.segments[0].coefficients[joint].value;
            coefficients[0] = sign * 0.1;
            coefficients[1] = sign * 0.2;
            coefficients[2] = sign * 0.2;
        }
        assert(runtime.submit_plan(second) == (unchecked ? RK_OK : RK_ERROR_INVALID_STATE));
    }
    for (bool smooth : {false, true}) {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto plan = make_plan(smooth ? 204 : 203, false, smooth);
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        std::vector<double> positions;
        for (int step = 0; step < 16; ++step)
            positions.push_back(apply_cycle(runtime, timestamp).position[0]);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.fault_code == RK_FAULT_TRAJECTORY_UNDERFLOW);
        assert(snapshot.safety == RK_SAFETY_READY);
        assert(positions.back() > positions[11]);
        for (std::size_t step = 2; step < positions.size(); ++step)
            assert(std::abs(positions[step] - 2 * positions[step - 1] +
                positions[step - 2]) <= 0.010001);
    }
    {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto first = make_plan(205, false, false);
        assert(runtime.submit_plan(first) == RK_OK);
        uint64_t timestamp = 0;
        for (int step = 0; step < 4; ++step) apply_cycle(runtime, timestamp);
        auto refill = make_plan(206, true, false);
        refill.sequence = 2;
        refill.start_position[0] = 0.2;
        refill.start_position[1] = -0.2;
        refill.start_velocity[0] = 0.2;
        refill.start_velocity[1] = -0.2;
        refill.segments = linear_segment_chunk(0.2, 0.2, 1'000'000'000, 206);
        assert(runtime.submit_plan(refill) == RK_OK);
        for (int step = 0; step < 23; ++step) apply_cycle(runtime, timestamp);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.fault_code == 0 && snapshot.trajectory_active == 0);
        assert(std::abs(snapshot.position[0] - 0.4) < 1e-9);
    }
}

void plan_start_ignores_chunking(const rk_robot_runtime_blueprint &blueprint) {
    // A plan's first cycle commands its start, however many chunks were
    // queued before that cycle ran.
    auto run = [&](bool split) {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        auto chunk = [&](uint64_t sequence, double start, uint64_t duration_ns, bool ends_at_rest) {
            robotkit::PlanRequest plan{};
            plan.sequence = sequence;
            plan.plan_id = 301;
            plan.model_revision = blueprint.revision;
            plan.calibration_revision = blueprint.calibration_revision;
            plan.ends_at_rest = ends_at_rest ? 1u : 0u;
            plan.segments = linear_segment_chunk(start, 0.2, duration_ns, sequence);
            for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
                const double sign = joint == 0 ? 1.0 : -1.0;
                plan.start_position[joint] = sign * start;
                if (sequence > 1) plan.start_velocity[joint] = sign * 0.2;
            }
            return plan;
        };
        if (split) {
            assert(runtime.submit_plan(chunk(1, 0.0, 500'000'000, false)) == RK_OK);
            assert(runtime.submit_plan(chunk(2, 0.1, 500'000'000, true)) == RK_OK);
        } else {
            assert(runtime.submit_plan(chunk(1, 0.0, 1'000'000'000, true)) == RK_OK);
        }
        uint64_t timestamp = 0;
        std::vector<double> positions;
        for (int step = 0; step < 16; ++step) {
            const auto state = apply_cycle(runtime, timestamp);
            if (step == 0) assert(state.trajectory_time_ns == 0);
            positions.push_back(state.trajectory_active ? state.position[0] : -1.0);
        }
        return positions;
    };
    const auto whole = run(false), split = run(true);
    for (std::size_t step = 0; step < whole.size(); ++step)
        assert(std::abs(whole[step] - split[step]) < 1e-9);
}

void discarded_tick_restores_trajectory(const rk_robot_runtime_blueprint &blueprint) {
    // A discarded world tick leaves the runtime as if it never ran: the knots it
    // consumed, the path it appended and the stop that cleared it are all undone.
    auto make = [&]() {
        auto runtime = std::make_unique<robotkit::RobotRuntime>(blueprint,
            std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count),
            std::chrono::milliseconds(100));
        assert(runtime->submit_segments(trajectory_command(1), trajectory_batch(
            {{0, 0.0}, {100'000'000, 0.1}, {200'000'000, 0.2}, {300'000'000, 0.3},
             {400'000'000, 0.4}, {500'000'000, 0.5}})) == RK_OK);
        return runtime;
    };
    auto kept = make(), discarded = make();
    uint64_t kept_time = 0, discarded_time = 0;
    for (int step = 0; step < 2; ++step) {
        apply_cycle(*kept, kept_time);
        apply_cycle(*discarded, discarded_time);
    }
    assert(discarded->submit_segments(trajectory_command(2), trajectory_batch(
        {{0, 0.5}, {100'000'000, 0.6}})) == RK_OK);
    assert(discarded->apply_pending_commands() == RK_OK);
    discarded->discard_pending_commands();
    assert(discarded->submit(lifecycle_command(3, RK_COMMAND_STOP)) == RK_OK);
    assert(discarded->apply_pending_commands() == RK_OK);
    discarded->discard_pending_commands();
    for (int step = 0; step < 8; ++step) {
        const auto a = apply_cycle(*kept, kept_time);
        const auto b = apply_cycle(*discarded, discarded_time);
        assert(a.position[0] == b.position[0] && a.trajectory_active == b.trajectory_active &&
            a.trajectory_queue_depth == b.trajectory_queue_depth);
    }
}

void hold_with_unlimited_follower(const rk_robot_runtime_blueprint &source) {
    // A lead screw with no acceleration limit of its own turns within its axis's: a hold
    // brakes the axis, and the screw with it, instead of being refused.
    auto blueprint = source;
    blueprint.joints[0].max_acceleration = 1.0;
    blueprint.joints[0].max_velocity = 1.0;
    blueprint.joints[1].max_acceleration = 0.0;
    blueprint.joints[1].max_velocity = 0.0;
    blueprint.coupling_count = 1;
    blueprint.couplings[0] = {0, 1, -1.0, 0.0};
    auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
    assert(runtime.blueprint().joints[1].max_acceleration == 1.0);
    assert(runtime.blueprint().joints[1].max_velocity == 1.0);
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 212;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.ends_at_rest = 1;
    plan.segments = linear_segment_chunk(0.0, 0.2, 2'000'000'000, 212);
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    for (int step = 0; step < 3; ++step) apply_cycle(runtime, timestamp);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_HOLD)) == RK_OK);
    for (int step = 0; step < 7; ++step) apply_cycle(runtime, timestamp);
    rk_robot_snapshot snapshot{};
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.session_state == RK_SESSION_HELD && snapshot.fault_code == 0);
    assert(std::abs(snapshot.position[1] + snapshot.position[0]) < 1e-9);
}

void native_hold_resume_and_abort(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        blueprint.joints[joint].max_acceleration = 1.0;
        blueprint.joints[joint].max_velocity = 1.0;
    }
    auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 210;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.ends_at_rest = 1;
    plan.segments = linear_segment_chunk(0.0, 0.2, 2'000'000'000, 210);
    assert(runtime.submit_plan(plan) == RK_OK);
    uint64_t timestamp = 0;
    for (int step = 0; step < 3; ++step) apply_cycle(runtime, timestamp);
    assert(runtime.submit(lifecycle_command(2, RK_COMMAND_HOLD)) == RK_OK);
    for (int step = 0; step < 7; ++step) apply_cycle(runtime, timestamp);
    rk_robot_snapshot snapshot{};
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.session_state == RK_SESSION_HELD);
    assert(snapshot.trajectory_queue_depth > 0);
    const double held_position = snapshot.position[0];
    for (int step = 0; step < 3; ++step) apply_cycle(runtime, timestamp);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(std::abs(snapshot.position[0] - held_position) < 1e-9);
    assert(runtime.submit(lifecycle_command(3, RK_COMMAND_RESUME)) == RK_OK);
    apply_cycle(runtime, timestamp);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.session_state == RK_SESSION_EXECUTING);
    assert(snapshot.mode == RK_ROBOT_MODE_TRACKING);
    for (int step = 0; step < 30; ++step) apply_cycle(runtime, timestamp);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.session_state == RK_SESSION_IDLE);
    assert(std::abs(snapshot.position[0] - 0.4) < 1e-9);

    auto abort_endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
    robotkit::RobotRuntime abort_runtime(blueprint, abort_endpoint, std::chrono::milliseconds(100));
    plan.sequence = 1;
    plan.plan_id = 211;
    assert(abort_runtime.submit_plan(plan) == RK_OK);
    for (int step = 0; step < 4; ++step) apply_cycle(abort_runtime, timestamp);
    assert(abort_runtime.submit(lifecycle_command(2, RK_COMMAND_ABORT)) == RK_OK);
    std::vector<double> aborted_positions;
    for (int step = 0; step < 8; ++step)
        aborted_positions.push_back(apply_cycle(abort_runtime, timestamp).position[0]);
    for (std::size_t step = 2; step < aborted_positions.size(); ++step)
        assert(std::abs(aborted_positions[step] - 2 * aborted_positions[step - 1] +
            aborted_positions[step - 2]) <= 0.010001);
    assert(abort_runtime.snapshot_full(snapshot) == RK_OK);
    assert(snapshot.trajectory_queue_depth == 0);
    assert(snapshot.safety != RK_SAFETY_FAULT && snapshot.fault_code == 0);
}

void smooth_path_hold_respects_acceleration(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        blueprint.joints[joint].max_acceleration = 1.0;
        blueprint.joints[joint].max_velocity = 1.0;
    }
    for (int hold_after : {3, 14}) {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(100));
        robotkit::PlanRequest plan{};
        plan.sequence = 1;
        plan.plan_id = 220 + hold_after;
        plan.model_revision = blueprint.revision;
        plan.calibration_revision = blueprint.calibration_revision;
        plan.ends_at_rest = 1;
        plan.segments = linear_segment_chunk(0.0, 0.0, 2'000'000'000, plan.plan_id);
        auto &segment = plan.segments.segments[0];
        segment.degree = 5;
        for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
            const double sign = joint == 0 ? 1.0 : -1.0;
            segment.coefficients[joint].value[3] = sign * 0.5;
            segment.coefficients[joint].value[4] = sign * -0.375;
            segment.coefficients[joint].value[5] = sign * 0.075;
        }
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        std::vector<double> positions;
        for (int step = 0; step < hold_after; ++step)
            positions.push_back(apply_cycle(runtime, timestamp).position[0]);
        assert(runtime.submit(lifecycle_command(2, RK_COMMAND_HOLD)) == RK_OK);
        for (int step = 0; step < 15; ++step)
            positions.push_back(apply_cycle(runtime, timestamp).position[0]);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.session_state == RK_SESSION_HELD);
        for (std::size_t step = 2; step < positions.size(); ++step)
            assert(std::abs(positions[step] - 2 * positions[step - 1] +
                positions[step - 2]) <= 0.010001);
        const double held = snapshot.position[0];
        apply_cycle(runtime, timestamp);
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(std::abs(snapshot.position[0] - held) < 1e-9);
        assert(runtime.submit(lifecycle_command(3, RK_COMMAND_RESUME)) == RK_OK);
        positions.clear();
        positions.push_back(held);
        for (int step = 0; step < 35; ++step)
            positions.push_back(apply_cycle(runtime, timestamp).position[0]);
        for (std::size_t step = 2; step < positions.size(); ++step)
            assert(std::abs(positions[step] - 2 * positions[step - 1] +
                positions[step - 2]) <= 0.010001);
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.session_state == RK_SESSION_IDLE);
        assert(std::abs(snapshot.position[0] - 0.4) < 1e-8);
    }
}

void presampled_publication_does_not_sample_again(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<FaultEndpoint>();
    robotkit::RobotRuntime runtime(blueprint, endpoint);
    rk_robot_state sampled{};
    assert(endpoint->sample(100, sampled) == RK_OK);
    assert(runtime.publish_presampled(100, sampled) == RK_OK);
    assert(endpoint->sample_count == 1);
}

void plan_end_braking_stays_on_path(const rk_robot_runtime_blueprint &source) {
    auto blueprint = source;
    for (auto &joint : blueprint.joints) {
        joint.max_velocity = 1.0;
        joint.max_acceleration = 1.0;
    }
    mk_state_to_state_request request{};
    request.struct_size = sizeof(request);
    request.joint_count = blueprint.joint_count;
    for (uint32_t joint = 0; joint < blueprint.joint_count; ++joint) {
        const double sign = joint == 0 ? 1.0 : -1.0;
        request.target_position[joint] = sign * 0.25;
        request.max_velocity[joint] = 1.0;
        request.max_acceleration[joint] = 1.0;
        request.max_jerk[joint] = 4.0;
    }
    mk_trajectory_handle native{};
    int32_t result = 0;
    assert(mk_generate_state_to_state(&request, &native, &result) == MK_OK);
    auto ruckig = segments_from_native(native);
    int64_t ruckig_duration = 0;
    assert(mk_trajectory_duration_ns(native, &ruckig_duration) == MK_OK);
    mk_trajectory_destroy(native);
    for (bool smooth : {false, true}) {
        const auto duration_ns = smooth ? static_cast<uint64_t>(ruckig_duration) :
            1'000'000'000ULL;
        const double final_position = smooth ? 0.25 : 0.2;
        for (auto command : {RK_COMMAND_HOLD, RK_COMMAND_STOP, RK_COMMAND_ABORT}) {
            for (uint64_t remaining_ns : {300'000'000ULL, 200'000'000ULL,
                                          100'000'000ULL}) {
                auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
                robotkit::RobotRuntime runtime(blueprint, endpoint,
                    std::chrono::milliseconds(100));
                robotkit::PlanRequest plan{};
                plan.sequence = 1;
                plan.plan_id = 300;
                plan.model_revision = blueprint.revision;
                plan.calibration_revision = blueprint.calibration_revision;
                plan.ends_at_rest = 1;
                plan.segments = smooth ? ruckig :
                    linear_segment_chunk(0.0, 0.2, duration_ns, 300);
                assert(runtime.submit_plan(plan) == RK_OK);
                uint64_t timestamp = 0;
                while (true) {
                    rk_robot_snapshot snapshot{};
                    assert(runtime.snapshot_full(snapshot) == RK_OK);
                    if (snapshot.trajectory_active &&
                        snapshot.trajectory_time_ns + remaining_ns >= duration_ns)
                        break;
                    apply_cycle(runtime, timestamp);
                }
                assert(runtime.submit(lifecycle_command(2, command)) == RK_OK);
                for (int step = 0; step < 40; ++step) {
                    const auto state = apply_cycle(runtime, timestamp);
                    assert(state.position[0] <= final_position + 1e-9);
                    assert(state.position[1] >= -final_position - 1e-9);
                    assert(state.safety != RK_SAFETY_FAULT);
                }
            }
        }
    }
    for (auto command : {RK_COMMAND_HOLD, RK_COMMAND_STOP, RK_COMMAND_ABORT}) {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint,
            std::chrono::milliseconds(10));
        robotkit::PlanRequest plan{};
        plan.sequence = 1;
        plan.plan_id = 301;
        plan.model_revision = blueprint.revision;
        plan.calibration_revision = blueprint.calibration_revision;
        plan.ends_at_rest = 0;
        plan.segments = linear_segment_chunk(0.0, 0.2, 1'000'000'000, 301);
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        for (int step = 0; step < 99; ++step) apply_cycle(runtime, timestamp);
        assert(runtime.submit(lifecycle_command(2, command)) == RK_OK);
        double furthest = 0.0;
        for (int step = 0; step < 100; ++step)
            furthest = std::max(furthest, apply_cycle(runtime, timestamp).position[0]);
        assert(furthest > 0.2 + 1e-6);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.safety == RK_SAFETY_READY);
    }
    {
        auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
        robotkit::RobotRuntime runtime(blueprint, endpoint,
            std::chrono::milliseconds(10));
        robotkit::PlanRequest plan{};
        plan.sequence = 1;
        plan.plan_id = 302;
        plan.model_revision = blueprint.revision;
        plan.calibration_revision = blueprint.calibration_revision;
        plan.ends_at_rest = 1;
        plan.segments = linear_segment_chunk(0.0, 0.2, 1'000'000'000, 302);
        assert(runtime.submit_plan(plan) == RK_OK);
        uint64_t timestamp = 0;
        for (int step = 0; step < 95; ++step) apply_cycle(runtime, timestamp);
        assert(runtime.submit(lifecycle_command(2, RK_COMMAND_ABORT)) == RK_OK);
        for (int step = 0; step < 30; ++step)
            assert(apply_cycle(runtime, timestamp).position[0] <= 0.2 + 1e-9);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.safety == RK_SAFETY_READY);
        assert(std::abs(snapshot.position[0] - 0.2) < 1e-9);
    }
}

void plan_end_does_not_restore_earlier_targets(const rk_robot_runtime_blueprint &blueprint) {
    // Position targets place the robot, then a plan moves it on. When the
    // plan ends the robot must stay at the plan's end, not return to the
    // targets it held before.
    auto endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 0;
    auto placed = velocity_batch(1, {{0, 0.0}, {1, 0.0}});
    placed.targets[0].mode = RK_TARGET_POSITION;
    placed.targets[0].target = 0.2;
    placed.targets[1].mode = RK_TARGET_POSITION;
    placed.targets[1].target = -0.2;
    assert(runtime.submit(placed) == RK_OK);
    apply_cycle(runtime, timestamp);
    robotkit::PlanRequest plan{};
    plan.sequence = 2;
    plan.plan_id = 1;
    plan.model_revision = blueprint.revision;
    plan.calibration_revision = blueprint.calibration_revision;
    plan.start_position[0] = 0.2;
    plan.start_position[1] = -0.2;
    plan.ends_at_rest = 1;
    plan.segments = sampled_batch([](double t) { return 0.2 + 2.0 * t; }, 100'000'000, 10'000'000, 1);
    assert(runtime.submit_plan(plan) == RK_OK);
    rk_robot_state state{};
    for (int cycle = 0; cycle < 40; ++cycle) {
        assert(runtime.apply_pending_commands() == RK_OK);
        assert(runtime.publish_sample(timestamp += 10'000'000) == RK_OK);
        state.struct_size = sizeof(state);
        assert(runtime.snapshot(state) == RK_OK);
    }
    assert(std::abs(state.position[0] - 0.4) < 1e-9);
    assert(std::abs(state.position[1] + 0.4) < 1e-9);
    assert(state.trajectory_active == 0);
    // The snapshot names where the next plan must start.
    rk_robot_snapshot snapshot{};
    snapshot.struct_size = sizeof(snapshot);
    assert(runtime.snapshot_full(snapshot) == RK_OK);
    assert(std::abs(snapshot.setpoint_position[0] - 0.4) < 1e-9);
    assert(std::abs(snapshot.setpoint_position[1] + 0.4) < 1e-9);
}

// Reports joint 0 at its upper limit as a 32-bit float would carry it.
class SinglePrecisionEndpoint final : public robotkit::RobotEndpoint {
public:
    explicit SinglePrecisionEndpoint(double limit, bool declares) : limit_(limit), declares_(declares) {}
    rk_result apply(const rk_robot_command &) override { return RK_OK; }
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp_ns;
        state.joint_count = 2;
        state.position[0] = static_cast<double>(static_cast<float>(limit_));
        return RK_OK;
    }
    double observed_position_precision() const noexcept override {
        return declares_ ? std::numeric_limits<float>::epsilon() : 0.0;
    }
private:
    double limit_;
    bool declares_;
};

void single_precision_reading_at_a_limit_is_not_a_fault(const rk_robot_runtime_blueprint &blueprint) {
    // 1.2 as a float is 1.2000000477: past the limit unless the endpoint's precision is allowed for.
    auto limited = blueprint;
    limited.joints[0].upper_limit = 1.2;
    for (bool declares : {true, false}) {
        auto endpoint = std::make_shared<SinglePrecisionEndpoint>(1.2, declares);
        robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
        const auto result = runtime.publish_sample(10'000'000);
        assert(declares ? result == RK_OK : result == RK_ERROR_LIMIT);
    }
}

void expired_velocity_targets_brake_within_limits(const rk_robot_runtime_blueprint &blueprint) {
    // Joint 0 brakes at 2 rad/s^2; joint 1 has no acceleration limit.
    auto limited = blueprint;
    limited.joints[0].max_acceleration = 2.0;
    limited.joints[1].max_acceleration = 0.0;
    auto endpoint = std::make_shared<EchoEndpoint>(limited.joint_count);
    robotkit::RobotRuntime runtime(limited, endpoint, std::chrono::milliseconds(10));
    uint64_t timestamp = 10'000'000;
    assert(runtime.publish_sample(timestamp) == RK_OK);
    auto emitted = [&](uint32_t joint) {
        const auto &command = endpoint->last_command;
        for (uint32_t index = 0; index < command.target_count; ++index)
            if (command.targets[index].joint == joint) {
                assert(command.targets[index].mode == RK_TARGET_VELOCITY);
                return command.targets[index].target;
            }
        assert(false && "joint target missing");
        return 0.0;
    };
    auto cycle = [&] {
        assert(runtime.apply_pending_commands() == RK_OK);
        assert(runtime.publish_sample(timestamp += 10'000'000) == RK_OK);
    };
    auto fault_code = [&] {
        rk_robot_snapshot snapshot{};
        snapshot.struct_size = sizeof(snapshot);
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        assert(snapshot.safety == RK_SAFETY_READY);
        return snapshot.fault_code;
    };

    // Lapses 50 ms after the sample it was sent against, then brakes 0.02 per period.
    auto streamed = velocity_batch(1, {{0, 1.0}, {1, 0.5}});
    streamed.expires_at_ns = timestamp + 50'000'000;
    assert(runtime.submit(streamed) == RK_OK);
    int running = 0;
    for (;;) {
        cycle();
        if (emitted(0) != 1.0) break;
        assert(emitted(1) == 0.5 && fault_code() == 0);
        ++running;
        assert(running < 10);
    }
    assert(running == 5);
    assert(std::abs(emitted(0) - 0.98) < 1e-12 && emitted(1) == 0.0);
    assert(fault_code() == RK_FAULT_COMMAND_EXPIRED);
    double previous = emitted(0);
    int braking = 1;
    while (emitted(0) != 0.0) {
        cycle();
        assert(previous - emitted(0) <= 0.02 + 1e-12 && emitted(0) >= 0.0);
        previous = emitted(0);
        ++braking;
        assert(braking < 100);
    }
    assert(braking == 50);
    for (int hold = 0; hold < 20; ++hold) {
        cycle();
        assert(emitted(0) == 0.0 && emitted(1) == 0.0);
    }

    // A batch without a deadline runs on and clears the diagnostic.
    assert(runtime.submit(velocity_batch(2, {{0, -0.3}})) == RK_OK);
    for (int held = 0; held < 100; ++held) {
        cycle();
        assert(emitted(0) == -0.3);
    }
    assert(fault_code() == 0);

    // A deadline already past lapses at once; a later target replaces a lapsed one.
    auto late = velocity_batch(3, {{0, -0.8}});
    late.expires_at_ns = timestamp - 1;
    assert(runtime.submit(late) == RK_OK);
    cycle();
    assert(std::abs(emitted(0) + 0.78) < 1e-12);
    auto renewed = velocity_batch(4, {{0, 0.4}});
    renewed.expires_at_ns = timestamp + 1'000'000'000;
    assert(runtime.submit(renewed) == RK_OK);
    for (int held = 0; held < 20; ++held) {
        cycle();
        assert(emitted(0) == 0.4);
    }
    assert(fault_code() == 0);
}

int main() {
    static_assert(sizeof(rk_robot_command) < 20'000,
        "trajectory payload must not be embedded in the command mailbox value");
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
    blueprint.calibration_revision = 9;
    blueprint.joint_count = 2;
    blueprint.link_count = 3;
    blueprint.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < blueprint.link_count; ++i) {
        blueprint.links[i].mass = 1.0;
        blueprint.links[i].inertia_tensor[0] = blueprint.links[i].inertia_tensor[4] = blueprint.links[i].inertia_tensor[8] = 1.0;
    }
    blueprint.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -1.0, 1.0, 3.0};
    blueprint.joints[1] = {1, RK_RUNTIME_JOINT_REVOLUTE, 1, 2, -1.0, 1.0, 3.0};
    for (auto &joint : blueprint.joints) {
        joint.parent_frame_rotation[3] = joint.child_frame_rotation[3] = 1.0;
        joint.axis[2] = 1.0;
    }
    presampled_publication_does_not_sample_again(blueprint);
    same_cycle_target_batches_merge_per_joint(blueprint);
    partial_targets_and_ordered_trajectory_commands(blueprint);
    unsupported_trajectory_queue_is_rejected(blueprint);
    timestamped_trajectory_interpolates_and_reports_progress(blueprint);
    normal_stop_decelerates_active_trajectory(blueprint);
    trajectory_stop_follows_path_and_reports_tag(blueprint);
    trajectory_chunk_extends_running_stop(blueprint);
    trajectory_stop_counts_trajectory_braking(blueprint);
    stop_beyond_queued_path_ramps_within_limits(blueprint);
    stop_ramp_stays_within_travel(blueprint);
    expired_velocity_targets_brake_within_limits(blueprint);
    plan_end_does_not_restore_earlier_targets(blueprint);
    single_precision_reading_at_a_limit_is_not_a_fault(blueprint);
    faulted_batch_skips_commands_before_reset(blueprint);
    invalid_trajectory_chunk_is_atomic(blueprint);
    trajectory_chunk_speed_is_limited(blueprint);
    trajectory_queue_is_bounded(blueprint);
    mixed_queue_depth_counts_knots(blueprint);
    plan_submission_checks_and_replacement(blueprint);
    device_queue_endpoint_does_not_receive_sampled_targets(blueprint);
    plan_events_follow_path_clock(blueprint);
    plan_event_records_report_overflow(blueprint);
    accepted_plan_keeps_committed_region_identical(blueprint);
    moving_degree_one_plan_cannot_retarget(blueprint);
    idle_plan_uses_commanded_anchor_and_following_error(blueprint);
    submitted_start_tolerances_control_acceptance(blueprint);
    degree_one_append_checks_chord_velocity(blueprint);
    ruckig_segments_match_motionkit_evaluation(blueprint);
    overacceleration_segment_is_rejected(blueprint);
    segment_junction_jump_is_rejected(blueprint);
    stop_braking_uses_segment_degree(blueprint);
    declared_plan_completion_and_underflow(blueprint);
    plan_start_ignores_chunking(blueprint);
    discarded_tick_restores_trajectory(blueprint);
    native_hold_resume_and_abort(blueprint);
    hold_with_unlimited_follower(blueprint);
    plan_end_braking_stays_on_path(blueprint);
    smooth_path_hold_respects_acceleration(blueprint);
    ruckig_segment_stop_uses_analytic_braking(blueprint);

    // Zero is a valid source epoch, not a missing-timestamp sentinel.
    auto clock_endpoint = std::make_shared<FaultEndpoint>();
    robotkit::RobotRuntime clock_runtime(blueprint, clock_endpoint);
    const auto before_receive = std::chrono::steady_clock::now().time_since_epoch();
    assert(clock_runtime.publish_sample(0) == RK_OK);
    rk_robot_state clock_state{};
    assert(clock_runtime.snapshot(clock_state) == RK_OK);
    rk_robot_snapshot revision_snapshot{};
    assert(clock_runtime.snapshot_full(revision_snapshot) == RK_OK);
    assert(revision_snapshot.revision == 1);
    assert(revision_snapshot.calibration_revision == 9);
    const auto after_receive = std::chrono::steady_clock::now().time_since_epoch();
    assert(clock_state.source_timestamp_ns == 0);
    assert(clock_state.received_timestamp_ns >= static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(before_receive).count()));
    assert(clock_state.received_timestamp_ns <= static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(after_receive).count()));

    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, std::move(endpoint));

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 1.0, 1.0, 0.0};
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.submit(command) == RK_ERROR_STALE_COMMAND);
    assert(runtime.start() == RK_OK);
    wait_for_sequence(runtime, 1);
    assert(runtime.stop() == RK_OK);

    rk_robot_state state{};
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.sequence == 1);
    assert(state.mode == RK_ROBOT_MODE_TRACKING);
    assert(state.safety == RK_SAFETY_READY);
    assert(state.position[0] > 0.0 && state.position[0] < 0.1);

    // One runtime controller emits the same rate-bounded setpoints regardless
    // of whether its endpoint is the loopback adapter or a hardware-like peer.
    auto simulated_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    auto physical_endpoint = std::make_shared<EchoEndpoint>(blueprint.joint_count);
    robotkit::RobotRuntime simulated_control(blueprint, simulated_endpoint,
                                             std::chrono::milliseconds(100));
    robotkit::RobotRuntime physical_control(blueprint, physical_endpoint,
                                            std::chrono::milliseconds(100));
    rk_robot_command rate_limited{};
    rate_limited.struct_size = sizeof(rate_limited);
    rate_limited.sequence = 1;
    rate_limited.kind = RK_COMMAND_JOINT_TARGETS;
    rate_limited.target_count = 1;
    rate_limited.targets[0] = {0, RK_TARGET_POSITION, 0.8, 1.0, 0.0};
    assert(simulated_control.submit(rate_limited) == RK_OK);
    assert(physical_control.submit(rate_limited) == RK_OK);
    for (uint64_t tick = 1; tick <= 3; ++tick) {
        const auto timestamp = tick * 100'000'000;
        assert(simulated_control.apply_pending_commands() == RK_OK);
        assert(physical_control.apply_pending_commands() == RK_OK);
        assert(simulated_control.publish_sample(timestamp) == RK_OK);
        assert(physical_control.publish_sample(timestamp) == RK_OK);
        rk_robot_state simulated_state{};
        rk_robot_state physical_state{};
        assert(simulated_control.snapshot(simulated_state) == RK_OK);
        assert(physical_control.snapshot(physical_state) == RK_OK);
        const double expected = static_cast<double>(tick) * 0.1;
        assert(std::abs(simulated_state.position[0] - expected) < 1e-9);
        assert(std::abs(physical_state.position[0] - expected) < 1e-9);
        assert(physical_state.mode == RK_ROBOT_MODE_TRACKING);
        assert(physical_state.safety == RK_SAFETY_READY);
        assert(std::abs(physical_endpoint->last_command.targets[0].target - expected) < 1e-9);
    }
    assert(physical_endpoint->apply_count == 3);

    auto mixed_blueprint = blueprint;
    mixed_blueprint.joint_count = 3;
    mixed_blueprint.link_count = 4;
    mixed_blueprint.joints[2] = {2, RK_RUNTIME_JOINT_REVOLUTE, 2, 3, -1.0, 1.0, 3.0};
    mixed_blueprint.joints[2].parent_frame_rotation[3] = 1.0;
    mixed_blueprint.joints[2].child_frame_rotation[3] = 1.0;
    mixed_blueprint.joints[2].axis[2] = 1.0;
    mixed_blueprint.links[3].mass = 1.0;
    mixed_blueprint.links[3].inertia_tensor[0] = 1.0;
    mixed_blueprint.links[3].inertia_tensor[4] = 1.0;
    mixed_blueprint.links[3].inertia_tensor[8] = 1.0;
    auto mixed_endpoint = std::make_shared<robotkit::InMemoryRobot>(3);
    robotkit::RobotRuntime mixed_control(mixed_blueprint, mixed_endpoint,
                                         std::chrono::milliseconds(100));
    rk_robot_command mixed_targets{};
    mixed_targets.struct_size = sizeof(mixed_targets);
    mixed_targets.sequence = 1;
    mixed_targets.kind = RK_COMMAND_JOINT_TARGETS;
    mixed_targets.target_count = 3;
    mixed_targets.targets[0] = {0, RK_TARGET_POSITION, 0.8, 1.0, 3.0};
    mixed_targets.targets[1] = {1, RK_TARGET_VELOCITY, 2.0, 0.25, 3.0};
    mixed_targets.targets[2] = {2, RK_TARGET_EFFORT, -2.0, 0.0, 3.0};
    assert(mixed_control.submit(mixed_targets) == RK_OK);
    assert(mixed_control.apply_pending_commands() == RK_OK);
    assert(mixed_control.publish_sample(100'000'000) == RK_OK);
    rk_robot_state mixed_state{};
    assert(mixed_control.snapshot(mixed_state) == RK_OK);
    assert(std::abs(mixed_state.position[0] - 0.1) < 1e-9);
    assert(std::abs(mixed_state.velocity[1] - 0.25) < 1e-9);
    assert(std::abs(mixed_state.effort[2] + 2.0) < 1e-9);

    rk_robot_command normal_stop{};
    normal_stop.struct_size = sizeof(normal_stop);
    normal_stop.sequence = 2;
    normal_stop.kind = RK_COMMAND_STOP;
    assert(simulated_control.submit(normal_stop) == RK_OK);
    assert(physical_control.submit(normal_stop) == RK_OK);
    assert(simulated_control.apply_pending_commands() == RK_OK);
    assert(physical_control.apply_pending_commands() == RK_OK);
    assert(simulated_control.publish_sample(400'000'000) == RK_OK);
    assert(physical_control.publish_sample(400'000'000) == RK_OK);
    rk_robot_state stopped_state{};
    assert(simulated_control.snapshot(stopped_state) == RK_OK);
    assert(std::abs(stopped_state.position[0] - 0.3) < 1e-9);
    assert(stopped_state.mode == RK_ROBOT_MODE_STOPPING);
    assert(stopped_state.safety == RK_SAFETY_STOPPING);
    rk_robot_state physical_stopped_state{};
    assert(physical_control.snapshot(physical_stopped_state) == RK_OK);
    assert(physical_stopped_state.mode == RK_ROBOT_MODE_STOPPING);
    assert(physical_stopped_state.safety == RK_SAFETY_STOPPING);

    auto stale_sample_endpoint = std::make_shared<StaleSampleEndpoint>();
    robotkit::RobotRuntime stale_sample(blueprint, stale_sample_endpoint);
    assert(stale_sample.publish_sample(100) == RK_OK);
    assert(stale_sample.publish_sample(200) == RK_ERROR_STALE_STATE);
    assert(stale_sample.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    assert(stale_sample_endpoint->emergency_stop_received);

    auto out_of_limit_endpoint = std::make_shared<OutOfLimitEndpoint>();
    robotkit::RobotRuntime out_of_limit(blueprint, out_of_limit_endpoint);
    assert(out_of_limit.publish_sample(100) == RK_ERROR_LIMIT);
    assert(out_of_limit.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    assert(out_of_limit_endpoint->emergency_stop_received);

    auto wrong_sensor_endpoint = std::make_shared<WrongSensorLayoutEndpoint>();
    robotkit::RobotRuntime wrong_sensor_layout(blueprint, wrong_sensor_endpoint);
    assert(wrong_sensor_layout.publish_sample(100) == RK_ERROR_BACKEND);
    assert(wrong_sensor_layout.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    assert(wrong_sensor_endpoint->emergency_stop_received);

    command.sequence = 2;
    command.kind = RK_COMMAND_EMERGENCY_STOP;
    command.target_count = 0;
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.start() == RK_OK);
    wait_for_sequence(runtime, 2);
    assert(runtime.stop() == RK_OK);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_EMERGENCY_STOP);

    command.sequence = 3;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.start() == RK_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(runtime.stop() == RK_OK);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_EMERGENCY_STOP);

    command.sequence = 4;
    command.kind = RK_COMMAND_RESET_SAFETY;
    command.target_count = 0;
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.start() == RK_OK);
    wait_for_sequence(runtime, 3);
    assert(runtime.stop() == RK_OK);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_READY);

    command.sequence = 5;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0].target = 2.0;
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.start() == RK_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(runtime.stop() == RK_OK);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);

    auto threaded_endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime threaded(blueprint, std::move(threaded_endpoint),
                               std::chrono::milliseconds(1));
    assert(threaded.start() == RK_OK);
    assert(threaded.running());
    assert(threaded.stop() == RK_OK);
    assert(!threaded.running());

    auto apply_failure_endpoint = std::make_shared<FaultEndpoint>();
    apply_failure_endpoint->fail_apply = true;
    robotkit::RobotRuntime apply_failure(blueprint, apply_failure_endpoint);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.25, 0.0, 0.0};
    assert(apply_failure.submit(command) == RK_OK);
    assert(apply_failure.apply_pending_commands() == RK_ERROR_BACKEND);
    assert(apply_failure_endpoint->emergency_stop_count == 1);
    assert(apply_failure_endpoint->discard_count == 0);
    apply_failure.discard_pending_commands();
    assert(apply_failure_endpoint->discard_count == 1);
    assert(apply_failure.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);

    auto sample_failure_endpoint = std::make_shared<FaultEndpoint>();
    sample_failure_endpoint->fail_sample = true;
    robotkit::RobotRuntime sample_failure(blueprint, sample_failure_endpoint);
    assert(sample_failure.publish_sample(100) == RK_ERROR_STALE_STATE);
    assert(sample_failure_endpoint->emergency_stop_count == 1);
    assert(sample_failure.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    return 0;
}
