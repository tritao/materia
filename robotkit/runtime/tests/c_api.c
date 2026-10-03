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
    blueprint.joints[0].max_effort = (blueprint.joints[0].limit_flags |= RK_LIMIT_EFFORT, 3.0);
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
    const int64_t one_start[] = {0};
    const int64_t short_duration[] = {100000000};
    const int32_t linear[] = {1};
    const double ramp[] = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0};
    assert(rk_robot_runtime_submit_segments(segment_runtime, &segment_command, 7, one_start,
        short_duration, linear, 1, ramp, 5) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_segments(segment_runtime, &segment_command, 7, one_start,
        short_duration, linear, 1, ramp, 6) == RK_OK);
    rk_robot_runtime_destroy(segment_runtime);

    /* A plan from segment arrays, as a MotionKit plan holds them, starting mid-plan. */
    rk_robot_runtime plan_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&blueprint, &plan_runtime) == RK_OK);
    static rk_plan_header header;
    header.struct_size = sizeof(header);
    header.sequence = 1;
    header.plan_id = 99;
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
    assert(rk_robot_runtime_submit_plan(plan_runtime, &header, starts, durations, degrees,
        1, coefficients, 5, joint_map, 1, NULL, 0) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_plan(plan_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, bad_map, 1, NULL, 0) == RK_ERROR_INVALID_ARGUMENT);
    header.ends_at_rest = 1; /* The cubic is still moving at its end. */
    assert(rk_robot_runtime_submit_plan(plan_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, joint_map, 1, NULL, 0) == RK_ERROR_INVALID_ARGUMENT);
    header.ends_at_rest = 0;
    assert(rk_robot_runtime_submit_plan(plan_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, joint_map, 1, NULL, 0) == RK_OK);
    full_snapshot.struct_size = sizeof(full_snapshot);
    assert(rk_robot_runtime_snapshot_full(plan_runtime, &full_snapshot) == RK_OK);
    assert(full_snapshot.active_plan_id == 99);
    assert(full_snapshot.session_state == RK_SESSION_EXECUTING);
    assert(full_snapshot.queue_end_time_ns == 1000000000);
    rk_robot_runtime_destroy(plan_runtime);

    /* A joint the plan leaves out follows its coupling's leader: a screw turning with its axis. */
    static rk_robot_runtime_blueprint coupled;
    coupled = blueprint;
    coupled.joint_count = 2;
    coupled.link_count = 3;
    coupled.links[2] = coupled.links[1];
    coupled.joints[1] = coupled.joints[0];
    coupled.joints[1].joint = 1;
    coupled.joints[1].parent_link = 1;
    coupled.joints[1].child_link = 2;
    coupled.joints[1].lower_limit = -10.0;
    coupled.joints[1].upper_limit = 10.0;
    coupled.coupling_count = 1;
    coupled.couplings[0].leader = 0;
    coupled.couplings[0].follower = 1;
    coupled.couplings[0].ratio = 3.0;
    rk_robot_runtime coupled_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&coupled, &coupled_runtime) == RK_OK);
    /* Held still, the follower would break the coupling as soon as its leader moved. */
    assert(rk_robot_runtime_submit_plan(coupled_runtime, &header, starts, durations, degrees,
        1, coefficients, 6, joint_map, 1, NULL, 0) == RK_OK);
    rk_robot_runtime_destroy(coupled_runtime);

    /* A joint with two couplings follows the sum of their leaders: 3 a - 2 b. */
    static rk_robot_runtime_blueprint summed;
    summed = coupled;
    summed.joint_count = 3;
    summed.link_count = 4;
    summed.links[3] = summed.links[2];
    summed.joints[2] = summed.joints[1];
    summed.joints[2].joint = 2;
    summed.joints[2].parent_link = 2;
    summed.joints[2].child_link = 3;
    summed.coupling_count = 2;
    summed.couplings[0].leader = 0;
    summed.couplings[0].follower = 2;
    summed.couplings[0].ratio = 3.0;
    summed.couplings[1].leader = 1;
    summed.couplings[1].follower = 2;
    summed.couplings[1].ratio = -2.0;
    rk_robot_runtime summed_runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_robot_runtime_create(&summed, &summed_runtime) == RK_OK);
    const double two_leaders[] = {0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0};
    const int32_t leader_map[] = {0, 1};
    assert(rk_robot_runtime_submit_plan(summed_runtime, &header, starts, durations, degrees,
        1, two_leaders, 12, leader_map, 2, NULL, 0) == RK_OK);
    rk_robot_runtime_destroy(summed_runtime);
    /* A cycle through the sum is refused when the runtime is created. */
    summed.couplings[summed.coupling_count].leader = 2;
    summed.couplings[summed.coupling_count].follower = 0;
    summed.couplings[summed.coupling_count].ratio = 1.0;
    summed.coupling_count++;
    assert(rk_robot_runtime_create(&summed, &summed_runtime) != RK_OK);

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
    const double toward_half[] = {state.position[0], (0.5 - state.position[0]) * 10.0,
        0.0, 0.0, 0.0, 0.0};
    assert(rk_robot_runtime_submit(runtime, &trajectory_command) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_robot_runtime_submit_segments(runtime, &trajectory_command, 0, one_start,
        short_duration, linear, 1, toward_half, 6) == RK_OK);
    assert(rk_robot_runtime_start(runtime) == RK_OK);
    nanosleep(&delay, NULL);
    assert(rk_robot_runtime_stop(runtime) == RK_OK);

    rk_robot_runtime_destroy(runtime);
    assert(rk_robot_runtime_snapshot(runtime, &state) == RK_ERROR_INVALID_HANDLE);
    return 0;
}
