#include "robotkit_runtime.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
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

    rk_result apply(const rk_robot_command &command) override {
        if (command.kind == RK_COMMAND_EMERGENCY_STOP)
            ++emergency_stop_count;
        return fail_apply ? RK_ERROR_BACKEND : RK_OK;
    }

    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override {
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

class EchoEndpoint final : public robotkit::RobotEndpoint {
public:
    explicit EchoEndpoint(uint32_t joint_count) : joint_count_(joint_count) {}

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
    double position_[RK_MAX_JOINTS]{};
    bool stopped_ = false;
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
    value.kind = RK_COMMAND_TRAJECTORY_CHUNK;
    return value;
}

rk_trajectory_chunk trajectory_batch(
    std::initializer_list<std::pair<uint64_t, double>> points, uint64_t tag = 0) {
    rk_trajectory_chunk value{};
    value.struct_size = sizeof(value);
    value.tag = tag;
    for (const auto &[time, position] : points) {
        auto &point = value.points[value.point_count++];
        point.time_from_start_ns = time;
        point.joint_count = 2;
        point.positions[0] = position;
        point.positions[1] = -position;
    }
    return value;
}

void timestamped_trajectory_interpolates_and_reports_progress(
    const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;
    assert(runtime.submit_trajectory(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.5},
                          {200'000'000, 1.0}})) == RK_OK);

    auto state = apply_cycle(runtime, timestamp);
    assert(state.position[0] == 0.0);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 3);
    assert(state.trajectory_time_ns == 0 && state.trajectory_duration_ns == 200'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.25) < 1e-9);
    assert(state.trajectory_active == 1 && state.trajectory_time_ns == 50'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.5) < 1e-9);
    assert(state.trajectory_queue_depth == 2 && state.trajectory_time_ns == 100'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 0.75) < 1e-6);
    assert(state.trajectory_queue_depth == 2 && state.trajectory_time_ns == 150'000'000);

    state = apply_cycle(runtime, timestamp);
    assert(std::abs(state.position[0] - 1.0) < 1e-9);
    assert(state.trajectory_active == 0 && state.trajectory_queue_depth == 0);
    assert(state.trajectory_time_ns == 0 && state.trajectory_duration_ns == 0);
}

void normal_stop_decelerates_active_trajectory(const rk_robot_runtime_blueprint &blueprint) {
    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(blueprint.joint_count);
    robotkit::RobotRuntime runtime(blueprint, endpoint, std::chrono::milliseconds(50));
    uint64_t timestamp = 0;
    assert(runtime.submit_trajectory(trajectory_command(1),
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
    assert(runtime.submit_trajectory(trajectory_command(1),
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
rk_trajectory_chunk sampled_batch(Position position, uint64_t duration_ns, uint64_t step_ns,
                                  uint64_t tag) {
    rk_trajectory_chunk value{};
    value.struct_size = sizeof(value);
    value.tag = tag;
    for (uint64_t time = 0; time <= duration_ns; time += step_ns) {
        auto &point = value.points[value.point_count++];
        point.time_from_start_ns = time;
        point.joint_count = 2;
        point.positions[0] = position(static_cast<double>(time) / 1'000'000'000.0);
        point.positions[1] = -point.positions[0];
    }
    return value;
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
    assert(runtime.submit_trajectory(trajectory_command(1),
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
            assert(runtime.submit_trajectory(trajectory_command(sequence++),
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
    assert(runtime.submit_trajectory(trajectory_command(1),
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
    assert(runtime.submit_trajectory(trajectory_command(1),
        sampled_batch([](double t) { return t; }, 100'000'000, 10'000'000, 1)) == RK_OK);
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
    assert(runtime.submit_trajectory(trajectory_command(1),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.2}})) == RK_OK);
    auto state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 2);

    auto invalid = trajectory_batch({{0, 0.2}, {100'000'000, 2.0}});
    assert(runtime.submit_trajectory(trajectory_command(2), invalid) == RK_OK);
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
        assert(runtime.submit_trajectory(trajectory_command(1),
            trajectory_batch({{0, 0.0}, {100'000'000, 0.1}})) == RK_OK);
        auto state = apply_cycle(runtime, timestamp);
        assert(state.safety == RK_SAFETY_READY && state.trajectory_active == 1);

        // Continuing from where the queue ends is accepted; jumping away is not.
        assert(runtime.submit_trajectory(trajectory_command(2),
            trajectory_batch({{0, 0.1}, {100'000'000, 0.2}})) == RK_OK);
        state = apply_cycle(runtime, timestamp);
        assert(state.safety == RK_SAFETY_READY && state.trajectory_queue_depth > 0);
        assert(runtime.submit_trajectory(trajectory_command(3),
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
        assert(runtime.submit_trajectory(trajectory_command(1),
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
    rk_trajectory_chunk full{};
    full.struct_size = sizeof(full);
    for (uint32_t index = 0; index < RK_MAX_TRAJECTORY_POINTS; ++index) {
        auto &point = full.points[full.point_count++];
        point.time_from_start_ns = static_cast<uint64_t>(index) * 1'000'000;
        point.joint_count = 2;
    }
    const uint32_t chunks = RK_MAX_TRAJECTORY_QUEUE_POINTS / RK_MAX_TRAJECTORY_POINTS;
    uint64_t sequence = 1;
    for (uint32_t index = 0; index < chunks; ++index)
        assert(runtime.submit_trajectory(trajectory_command(sequence++), full) == RK_OK);
    // Pending mailbox chunks count towards the bound before the owner runs.
    assert(runtime.submit_trajectory(trajectory_command(sequence++), full) == RK_ERROR_QUEUE_FULL);
    assert(runtime.apply_pending_commands() == RK_OK);
    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.trajectory_queue_depth <= RK_MAX_TRAJECTORY_QUEUE_POINTS);
    // Once queued, the published depth keeps the bound.
    assert(runtime.submit_trajectory(trajectory_command(sequence++), full) == RK_ERROR_QUEUE_FULL);
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
    assert(runtime.submit_trajectory(trajectory_command(3),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.2}})) == RK_OK);
    assert(runtime.submit_trajectory(trajectory_command(4),
        trajectory_batch({{0, 0.2}, {100'000'000, 0.4}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 4);
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
    assert(runtime.submit_trajectory(trajectory_command(6),
        trajectory_batch({{0, 0.1}, {100'000'000, 0.3}})) == RK_OK);
    state = apply_cycle(runtime, timestamp);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 2);
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
    assert(reset_runtime.submit_trajectory(trajectory_command(3),
        trajectory_batch({{0, 0.0}, {100'000'000, 0.25}})) == RK_OK);
    state = apply_cycle(reset_runtime, timestamp);
    assert(state.safety == RK_SAFETY_READY);
    assert(state.trajectory_active == 1 && state.trajectory_queue_depth == 2);
}

} // namespace

int main() {
    static_assert(sizeof(rk_robot_command) < 20'000,
        "trajectory payload must not be embedded in the command mailbox value");
    static_assert(sizeof(rk_trajectory_chunk) > 130'000,
        "trajectory payload remains explicitly bounded and independently allocated");
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
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
    same_cycle_target_batches_merge_per_joint(blueprint);
    partial_targets_and_ordered_trajectory_commands(blueprint);
    timestamped_trajectory_interpolates_and_reports_progress(blueprint);
    normal_stop_decelerates_active_trajectory(blueprint);
    trajectory_stop_follows_path_and_reports_tag(blueprint);
    trajectory_chunk_extends_running_stop(blueprint);
    trajectory_stop_counts_trajectory_braking(blueprint);
    stop_beyond_queued_path_ramps_within_limits(blueprint);
    faulted_batch_skips_commands_before_reset(blueprint);
    invalid_trajectory_chunk_is_atomic(blueprint);
    trajectory_chunk_speed_is_limited(blueprint);
    trajectory_queue_is_bounded(blueprint);

    // Zero is a valid source epoch, not a missing-timestamp sentinel.
    auto clock_endpoint = std::make_shared<FaultEndpoint>();
    robotkit::RobotRuntime clock_runtime(blueprint, clock_endpoint);
    const auto before_receive = std::chrono::steady_clock::now().time_since_epoch();
    assert(clock_runtime.publish_sample(0) == RK_OK);
    rk_robot_state clock_state{};
    assert(clock_runtime.snapshot(clock_state) == RK_OK);
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
