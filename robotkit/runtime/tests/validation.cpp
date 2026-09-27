#include "robotkit_runtime.h"

#include <cassert>
#include <cstddef>
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
    blueprint.owner_period_ns = static_cast<uint64_t>(INT64_MAX) + 1;
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_ERROR_INVALID_ARGUMENT);
    blueprint.struct_size = offsetof(rk_robot_runtime_blueprint, owner_period_ns);
    assert(rk_robot_runtime_blueprint_validate(&blueprint) == RK_OK);
    blueprint.struct_size = sizeof(blueprint);
    blueprint.owner_period_ns = 20'000'000;
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

    rk_robot_command trajectory_command{};
    trajectory_command.struct_size = sizeof(trajectory_command);
    trajectory_command.sequence = 2;
    trajectory_command.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    assert(rk_robot_command_validate_for_blueprint(&trajectory_command, &blueprint) == RK_OK);
    trajectory_command.kind = 5;
    assert(rk_robot_command_validate(&trajectory_command) == RK_ERROR_INVALID_ARGUMENT);
    trajectory_command.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    rk_trajectory_segment_chunk trajectory{};
    trajectory.struct_size = sizeof(trajectory);
    trajectory.segment_count = 1;
    trajectory.segments[0].duration_ns = 10'000'000;
    trajectory.segments[0].degree = 1;
    trajectory.segments[0].joint_count = 2;
    trajectory.segments[0].coefficients[0].value[1] = 10.0;
    trajectory.segments[0].coefficients[1].value[1] = -10.0;
    assert(rk_trajectory_segment_chunk_validate_for_blueprint(&trajectory, &blueprint) == RK_OK);
    trajectory.segments[0].time_from_start_ns = 1;
    assert(rk_trajectory_segment_chunk_validate(&trajectory) == RK_ERROR_INVALID_ARGUMENT);
    trajectory.segments[0].time_from_start_ns = 0;
    trajectory.segments[0].joint_count = 1;
    assert(rk_trajectory_segment_chunk_validate_for_blueprint(&trajectory, &blueprint) == RK_ERROR_INVALID_ARGUMENT);

    rk_trajectory_segment_chunk segments{};
    segments.struct_size = sizeof(segments);
    segments.segment_count = 1;
    segments.segments[0].duration_ns = 10000000;
    segments.segments[0].degree = 1;
    segments.segments[0].joint_count = 2;
    assert(rk_trajectory_segment_chunk_validate_for_blueprint(&segments, &blueprint) == RK_OK);
    segments.segments[0].duration_ns = 0;
    assert(rk_trajectory_segment_chunk_validate(&segments) == RK_ERROR_INVALID_ARGUMENT);
    segments.segments[0].duration_ns = 10000000;
    segments.segments[0].coefficients[0].value[1] = NAN;
    assert(rk_trajectory_segment_chunk_validate(&segments) == RK_ERROR_INVALID_ARGUMENT);
    segments.segments[0].coefficients[0].value[1] = 0.0;
    segments.segment_count = 2;
    segments.segments[1] = segments.segments[0];
    assert(rk_trajectory_segment_chunk_validate(&segments) == RK_ERROR_INVALID_ARGUMENT);
    segments.segments[1].time_from_start_ns = 10000000;
    assert(rk_trajectory_segment_chunk_validate_for_blueprint(&segments, &blueprint) == RK_OK);
    segments.segment_count = 11;
    for (uint32_t index = 0; index < segments.segment_count; ++index) {
        auto &segment = segments.segments[index];
        segment.time_from_start_ns = index;
        segment.duration_ns = 1;
        segment.degree = 5;
        segment.joint_count = 64;
    }
    assert(rk_trajectory_segment_chunk_validate(&segments) == RK_ERROR_INVALID_ARGUMENT);

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
    capabilities.supports_trajectory_queue = 1;
    capabilities.supports_prediction = 1;
    assert(rk_robot_capabilities_validate(&capabilities) == RK_OK);
    return 0;
}
