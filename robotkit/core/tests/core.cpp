#include "robotkit_core.h"

#include <cassert>
#include <cmath>

int main() {
    rk_runtime_layout layout{};
    layout.struct_size = sizeof(layout);
    layout.revision = 7;
    layout.joint_count = 2;
    assert(rk_runtime_layout_validate(&layout) == RK_OK);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 2;
    command.targets[0] = {0, RK_TARGET_POSITION, 1.0, 2.0, 3.0};
    command.targets[1] = {1, RK_TARGET_POSITION, -0.5, 2.0, 3.0};
    assert(rk_robot_command_validate_for_layout(&command, &layout) == RK_OK);

    command.targets[1].joint = 2;
    assert(rk_robot_command_validate_for_layout(&command, &layout) == RK_ERROR_INVALID_ARGUMENT);
    command.targets[1].joint = 1;
    command.targets[1].target = NAN;
    assert(rk_robot_command_validate(&command) == RK_ERROR_INVALID_ARGUMENT);

    rk_robot_state state{};
    state.struct_size = sizeof(state);
    state.joint_count = 2;
    state.mode = RK_ROBOT_MODE_TRACKING;
    state.safety = RK_SAFETY_READY;
    state.position[0] = 1.0;
    state.position[1] = -0.5;
    assert(rk_robot_state_validate(&state) == RK_OK);

    rk_robot_capabilities capabilities{};
    capabilities.struct_size = sizeof(capabilities);
    capabilities.joint_count = 2;
    capabilities.supports_position_targets = 1;
    capabilities.supports_prediction = 1;
    assert(rk_robot_capabilities_validate(&capabilities) == RK_OK);
    return 0;
}
