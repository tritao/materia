#include "robotkit_runtime.hpp"

#include <cassert>
#include <chrono>
#include <memory>
#include <thread>

namespace {

class FaultEndpoint final : public robotkit::RobotEndpoint {
public:
    bool fail_apply = false;
    bool fail_sample = false;
    int discard_count = 0;

    rk_result apply(const rk_robot_command &) override {
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
    blueprint.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -1.0, 1.0, 3.0};
    blueprint.joints[1] = {1, RK_RUNTIME_JOINT_REVOLUTE, 1, 2, -1.0, 1.0, 3.0};

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
    assert(apply_failure_endpoint->discard_count == 0);
    apply_failure.discard_pending_commands();
    assert(apply_failure_endpoint->discard_count == 1);
    assert(apply_failure.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);

    auto sample_failure_endpoint = std::make_shared<FaultEndpoint>();
    sample_failure_endpoint->fail_sample = true;
    robotkit::RobotRuntime sample_failure(blueprint, sample_failure_endpoint);
    assert(sample_failure.publish_sample(100) == RK_ERROR_STALE_STATE);
    assert(sample_failure.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    return 0;
}
