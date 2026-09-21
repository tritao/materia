#include "robotkit_runtime.hpp"

#include <cassert>
#include <chrono>
#include <memory>

int main() {
    rk_runtime_layout layout{};
    layout.struct_size = sizeof(layout);
    layout.revision = 1;
    layout.joint_count = 2;

    auto endpoint = std::make_unique<robotkit::InMemoryEndpoint>(layout.joint_count);
    robotkit::Runtime runtime(layout, std::move(endpoint));

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 1.0, 0.0, 0.0};
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.step_once(100) == RK_OK);

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
    assert(runtime.step_once(200) == RK_OK);
    assert(runtime.snapshot(state) == RK_OK);
    assert(state.safety == RK_SAFETY_EMERGENCY_STOP);

    command.sequence = 3;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    assert(runtime.submit(command) == RK_OK);
    assert(runtime.step_once(300) == RK_ERROR_SAFETY_STOPPED);

    auto threaded_endpoint = std::make_unique<robotkit::InMemoryEndpoint>(layout.joint_count);
    robotkit::Runtime threaded(layout, std::move(threaded_endpoint),
                               std::chrono::milliseconds(1));
    assert(threaded.start() == RK_OK);
    assert(threaded.running());
    assert(threaded.stop() == RK_OK);
    assert(!threaded.running());
    return 0;
}
