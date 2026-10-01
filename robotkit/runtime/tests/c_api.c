#include "robotkit_runtime.h"

#include <assert.h>
#include <stddef.h>
#include <time.h>

int main(void) {
    rk_robot_runtime_blueprint blueprint = {0};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
    blueprint.joint_count = 1;
    blueprint.link_count = 2;
    blueprint.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t link = 0; link < blueprint.link_count; ++link) {
        blueprint.links[link].mass = 1.0;
        blueprint.links[link].inertia_tensor[0] = 1.0;
        blueprint.links[link].inertia_tensor[4] = 1.0;
        blueprint.links[link].inertia_tensor[8] = 1.0;
    }
    blueprint.joints[0].joint = 0;
    blueprint.joints[0].type = RK_RUNTIME_JOINT_REVOLUTE;
    blueprint.joints[0].parent_link = 0;
    blueprint.joints[0].child_link = 1;
    blueprint.joints[0].lower_limit = -1.0;
    blueprint.joints[0].upper_limit = 1.0;
    blueprint.joints[0].max_effort = 3.0;
    blueprint.joints[0].parent_frame_rotation[3] = 1.0;
    blueprint.joints[0].child_frame_rotation[3] = 1.0;
    blueprint.joints[0].axis[2] = 1.0;

    rk_robot_runtime runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &runtime) == RK_OK);
    assert(runtime != RK_INVALID_ROBOT_RUNTIME);
    rk_robot_snapshot full_snapshot = {0};
    full_snapshot.struct_size = sizeof(full_snapshot);
    assert(rk_robot_runtime_snapshot_full(runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.calibration_revision == 0);
    full_snapshot.struct_size = offsetof(rk_robot_snapshot, calibration_revision);
    full_snapshot.calibration_revision = 0x1234;
    assert(rk_robot_runtime_snapshot_full(runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.calibration_revision == 0x1234);

    rk_robot_runtime_blueprint old_blueprint = blueprint;
    old_blueprint.struct_size = offsetof(rk_robot_runtime_blueprint, calibration_revision);
    old_blueprint.calibration_revision = 77;
    rk_robot_runtime old_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&old_blueprint, &old_runtime) == RK_OK);
    full_snapshot.struct_size = sizeof(full_snapshot);
    assert(rk_robot_runtime_snapshot_full(old_runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.calibration_revision == 0);
    rk_robot_runtime_destroy(old_runtime);

    rk_robot_runtime segment_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &segment_runtime) == RK_OK);
    rk_robot_command segment_command = {0};
    segment_command.struct_size = sizeof(segment_command);
    segment_command.sequence = 1;
    segment_command.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    rk_trajectory_segment_chunk segment_chunk = {0};
    segment_chunk.struct_size = sizeof(segment_chunk);
    segment_chunk.segment_count = 1;
    segment_chunk.segments[0].duration_ns = 100000000;
    segment_chunk.segments[0].degree = 1;
    segment_chunk.segments[0].joint_count = 1;
    segment_chunk.segments[0].coefficients[0].value[1] = 0.5;
    assert(rk_robot_runtime_submit_segments(segment_runtime, &segment_command,
        &segment_chunk) == RK_OK);
    rk_robot_runtime_destroy(segment_runtime);

    rk_robot_runtime plan_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &plan_runtime) == RK_OK);
    rk_plan_submission plan = {0};
    plan.struct_size = sizeof(plan);
    plan.sequence = 1;
    plan.plan_id = 99;
    plan.model_revision = blueprint.revision;
    plan.required_capabilities = RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE;
    plan.segments.struct_size = sizeof(plan.segments);
    plan.segments.segment_count = 1;
    plan.segments.segments[0].duration_ns = 1000000000;
    plan.segments.segments[0].degree = 3;
    plan.segments.segments[0].joint_count = 1;
    plan.segments.segments[0].coefficients[0].value[3] = 0.1;
    assert(rk_plan_submission_validate_for_blueprint(&plan, &blueprint) == RK_OK);
    assert(rk_robot_runtime_submit_plan(plan_runtime, &plan) == RK_OK);
    full_snapshot.struct_size = sizeof(full_snapshot);
    assert(rk_robot_runtime_snapshot_full(plan_runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.active_plan_id == 99);
    assert(full_snapshot.session_state == RK_SESSION_EXECUTING);
    assert(full_snapshot.queue_end_time_ns == 1000000000);
    rk_robot_runtime_destroy(plan_runtime);

    /* The same plan from segment arrays, as a MotionKit plan holds them, starting mid-plan. */
    rk_robot_runtime array_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &array_runtime) == RK_OK);
    static rk_plan_header header;
    header.struct_size = sizeof(header);
    header.sequence = 1;
    header.plan_id = 100;
    header.model_revision = blueprint.revision;
    header.required_capabilities = RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE;
    header.tag = 7;
    header.ends_at_rest = 0;
    const int64_t starts[] = {5000000000};
    const int64_t durations[] = {1000000000};
    const int32_t degrees[] = {3};
    const double coefficients[] = {0.0, 0.0, 0.0, 0.1, 0.0, 0.0};
    const int32_t joint_map[] = {0};
    const int32_t bad_map[] = {1};
    assert(rk_robot_runtime_submit_plan_arrays(array_runtime, &header, starts, durations, degrees,
        1, coefficients, 5, joint_map, 1) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_plan_arrays(array_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, bad_map, 1) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_plan_arrays(array_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, joint_map, 1) == RK_OK);
    assert(rk_robot_runtime_snapshot_full(array_runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.active_plan_id == 100);
    assert(full_snapshot.queue_end_time_ns == 1000000000);
    rk_robot_runtime_destroy(array_runtime);

    rk_robot_command command = {0};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0].joint = 0;
    command.targets[0].mode = RK_TARGET_POSITION;
    command.targets[0].target = 1.0;
    command.targets[0].max_rate = 10.0;
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
    assert(capabilities.supports_trajectory_queue != 0);
    assert(capabilities.supports_execution_plans != 0);

    rk_robot_command trajectory_command = {0};
    trajectory_command.struct_size = sizeof(trajectory_command);
    trajectory_command.sequence = 2;
    trajectory_command.kind = RK_COMMAND_TRAJECTORY_SEGMENTS;
    rk_trajectory_segment_chunk trajectory = {0};
    trajectory.struct_size = sizeof(trajectory);
    trajectory.segment_count = 1;
    trajectory.segments[0].duration_ns = 100000000;
    trajectory.segments[0].degree = 1;
    trajectory.segments[0].joint_count = 1;
    trajectory.segments[0].coefficients[0].value[0] = state.position[0];
    trajectory.segments[0].coefficients[0].value[1] = (0.5 - state.position[0]) * 10.0;
    assert(rk_trajectory_segment_chunk_validate_for_blueprint(&trajectory, &blueprint) == RK_OK);
    assert(rk_robot_runtime_submit(runtime, &trajectory_command) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_segments(runtime, &trajectory_command, &trajectory) == RK_OK);
    assert(rk_robot_runtime_start(runtime) == RK_OK);
    nanosleep(&delay, NULL);
    assert(rk_robot_runtime_stop(runtime) == RK_OK);

    rk_robot_runtime_destroy(runtime);
    assert(rk_robot_runtime_snapshot(runtime, &state) == RK_ERROR_INVALID_HANDLE);
    return 0;
}
