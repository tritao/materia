#include "robotkit_runtime.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

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

} // namespace

int main() {
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
