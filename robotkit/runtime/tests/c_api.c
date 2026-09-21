#include "robotkit_runtime.h"

#include <assert.h>
#include <time.h>

int main(void) {
    rk_robot_runtime_blueprint blueprint = {0};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
    blueprint.joint_count = 1;
    blueprint.link_count = 2;
    blueprint.joints[0].joint = 0;
    blueprint.joints[0].type = RK_RUNTIME_JOINT_REVOLUTE;
    blueprint.joints[0].parent_link = 0;
    blueprint.joints[0].child_link = 1;
    blueprint.joints[0].lower_limit = -1.0;
    blueprint.joints[0].upper_limit = 1.0;
    blueprint.joints[0].max_effort = 3.0;

    rk_robot_runtime runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &runtime) == RK_OK);
    assert(runtime != RK_INVALID_ROBOT_RUNTIME);

    rk_robot_command command = {0};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0].joint = 0;
    command.targets[0].mode = RK_TARGET_POSITION;
    command.targets[0].target = 1.0;
    assert(rk_robot_runtime_submit(runtime, &command) == RK_OK);

    /* Standalone runtimes advance through their worker lifecycle. */
    assert(rk_robot_runtime_start(runtime) == RK_OK);
    struct timespec delay = {0, 5 * 1000 * 1000};
    nanosleep(&delay, NULL);
    assert(rk_robot_runtime_stop(runtime) == RK_OK);

    rk_robot_state state = {0};
    state.struct_size = sizeof(state);
    assert(rk_robot_runtime_snapshot(runtime, &state) == RK_OK);
    assert(state.sequence == 1);
    assert(state.joint_count == 1);
    assert(state.position[0] > 0.0 && state.position[0] < 1.0);

    rk_robot_capabilities capabilities = {0};
    capabilities.struct_size = sizeof(capabilities);
    assert(rk_robot_runtime_capabilities(runtime, &capabilities) == RK_OK);
    assert(capabilities.joint_count == 1);
    assert(capabilities.supports_position_targets != 0);

    rk_robot_runtime_destroy(runtime);
    assert(rk_robot_runtime_snapshot(runtime, &state) == RK_ERROR_INVALID_HANDLE);
    return 0;
}
