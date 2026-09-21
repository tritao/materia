#include "robotkit_simkit.h"

#include <cassert>

int main() {
    rk_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 7;
    blueprint.joint_count = 1;
    blueprint.link_count = 2;
    blueprint.joints[0] = {
        0,
        RK_RUNTIME_JOINT_REVOLUTE,
        0,
        1,
        -3.14,
        3.14,
        100.0,
    };
    assert(rk_runtime_blueprint_validate(&blueprint) == RK_OK);

    rk_runtime runtime = RK_INVALID_RUNTIME;
    assert(rk_runtime_create_sim(&blueprint, &runtime) == RK_OK);
    assert(runtime != RK_INVALID_RUNTIME);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.5, 0.0, 0.0};
    assert(rk_runtime_submit(runtime, &command) == RK_OK);
    assert(rk_runtime_step(runtime, 100) == RK_OK);

    rk_robot_state state{};
    state.struct_size = sizeof(state);
    assert(rk_runtime_snapshot(runtime, &state) == RK_OK);
    assert(state.joint_count == 1);
    assert(state.position[0] == 0.5);

    rk_runtime_destroy(runtime);
    return 0;
}
