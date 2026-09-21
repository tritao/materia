#include "robotkit_runtime.hpp"

#include <cassert>
#include <chrono>
#include <memory>
#include <thread>

namespace {

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
    rk_robot_runtime_layout layout{};
    layout.struct_size = sizeof(layout);
    layout.revision = 1;
    layout.joint_count = 2;

    auto endpoint = std::make_shared<robotkit::InMemoryRobot>(layout.joint_count);
    robotkit::RobotRuntime runtime(layout, std::move(endpoint));

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 1.0, 0.0, 0.0};
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.start() == RK_OK);
    wait_for_sequence(runtime, 1);
    assert(runtime.stop() == RK_OK);

    rk_robot_state state{};
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.sequence == 1);
    assert(state.mode == RK_ROBOT_MODE_TRACKING);
    assert(state.safety == RK_SAFETY_READY);
    assert(state.position[0] > 0.0 && state.position[0] < 1.0);

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
    assert(state.safety == RK_SAFETY_FAULT);

    auto threaded_endpoint = std::make_shared<robotkit::InMemoryRobot>(layout.joint_count);
    robotkit::RobotRuntime threaded(layout, std::move(threaded_endpoint),
                               std::chrono::milliseconds(1));
    assert(threaded.start() == RK_OK);
    assert(threaded.running());
    assert(threaded.stop() == RK_OK);
    assert(!threaded.running());
    return 0;
}
