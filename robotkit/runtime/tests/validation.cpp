#include "robotkit_runtime.h"

#include <cassert>
#include <cmath>

int main() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 7;
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
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_OK);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 2;
    command.targets[0] = {0, RK_TARGET_POSITION, 1.0, 2.0, 3.0};
    command.targets[1] = {1, RK_TARGET_POSITION, -0.5, 2.0, 3.0};
    assert(rk_robot_command_validate_for_blueprint(&command, &blueprint) == RK_OK);

    command.targets[1].joint = 0;
    assert(rk_robot_command_validate(&command) == RK_ERROR_INVALID_ARGUMENT);
    command.targets[1].joint = 1;

    command.targets[1].joint = 2;
    assert(rk_robot_command_validate_for_blueprint(&command, &blueprint) == RK_ERROR_INVALID_ARGUMENT);
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
    state.sensor_count = RK_MAX_SENSORS + 1;
    assert(rk_robot_state_validate(&state) == RK_ERROR_INVALID_ARGUMENT);
    state.sensor_count = 1;
    state.sensors[0].sequence = 1;
    state.sensors[0].value_count = 6;
    state.sensors[0].values[0] = NAN;
    assert(rk_robot_state_validate(&state) == RK_ERROR_INVALID_ARGUMENT);
    state.sensors[0].values[0] = 0.0;
    state.sensors[0].value_count = RK_MAX_SENSOR_VALUES + 1;
    assert(rk_robot_state_validate(&state) == RK_ERROR_INVALID_ARGUMENT);
    state.sensors[0].sequence = 0;
    state.sensors[0].value_count = 1;
    assert(rk_robot_state_validate(&state) == RK_ERROR_INVALID_ARGUMENT);
    state.sensors[0].value_count = 0;
    assert(rk_robot_state_validate(&state) == RK_OK);

    rk_robot_capabilities capabilities{};
    blueprint.sensor_count = 1;
    auto &sensor = blueprint.sensors[0];
    sensor.kind = RK_SENSOR_LIDAR;
    sensor.rotation[3] = 1.0;
    sensor.ray_count = 16;
    sensor.max_range = 20.0;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_OK);
    sensor.update_rate = NAN;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.update_rate = 10.0;
    sensor.ray_count = RK_MAX_SENSOR_VALUES + 1;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.ray_count = 16;
    sensor.rotation[3] = 0.0;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.rotation[3] = 1.0;
    sensor.field_of_view = 6.283185307179586 + 0.01;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.field_of_view = 6.283185307179586;
    sensor.start_angle = NAN;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.start_angle = 0.0;
    sensor.noise_stddev = -1.0;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    sensor.noise_stddev = 0.0;
    sensor.link = blueprint.link_count;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    capabilities.struct_size = sizeof(capabilities);
    capabilities.joint_count = 2;
    capabilities.supports_position_targets = 1;
    capabilities.supports_prediction = 1;
    assert(rk_robot_capabilities_validate(&capabilities) == RK_OK);
    return 0;
}
