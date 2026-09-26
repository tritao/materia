#include "robotkit_simkit.h"
#include "../src/sensor_math.hpp"

#include <cassert>
#include <cmath>
#include <chrono>
#include <thread>

namespace {

rk_robot_runtime_blueprint blueprint(uint64_t revision) {
    rk_robot_runtime_blueprint value{};
    value.struct_size = sizeof(value);
    value.revision = revision;
    value.joint_count = 1;
    value.link_count = 2;
    value.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < value.link_count; ++i) {
        value.links[i].mass = 1.0;
        value.links[i].inertia_tensor[0] = value.links[i].inertia_tensor[4] = value.links[i].inertia_tensor[8] = 1.0;
    }
    value.joints[0] = {
        0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0,
    };
    value.joints[0].parent_frame_rotation[3] = 1.0;
    value.joints[0].child_frame_rotation[3] = 1.0;
    value.joints[0].axis[2] = 1.0;
    assert(rk_robot_runtime_blueprint_validate(&value) == RK_OK);
    return value;
}

rk_robot_command target(double position, uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    value.target_count = 1;
    value.targets[0] = {0, RK_TARGET_POSITION, position, 0.0, 0.0};
    return value;
}

rk_robot_command velocity_target(double velocity, uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    value.target_count = 1;
    value.targets[0] = {0, RK_TARGET_VELOCITY, velocity, 0.0, 0.0};
    return value;
}

rk_robot_state snapshot(rk_robot_runtime runtime) {
    rk_robot_state value{};
    value.struct_size = sizeof(value);
    assert(rk_robot_runtime_snapshot(runtime, &value) == RK_OK);
    return value;
}

void shared_world_steps_once() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);

    const auto first_model = blueprint(11);
    const auto second_model = blueprint(12);
    rk_robot_runtime first = RK_INVALID_ROBOT_RUNTIME;
    rk_robot_runtime second = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &first_model, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &second_model, &second) == RK_OK);

    const auto first_command = target(0.4, 1);
    const auto second_command = target(-0.3, 1);
    assert(rk_robot_runtime_submit(first, &first_command) == RK_OK);
    assert(rk_robot_runtime_submit(second, &second_command) == RK_OK);

    // Participating runtimes cannot own the shared simulation clock.
    assert(rk_robot_runtime_start(first) == RK_ERROR_INVALID_STATE);
    assert(rk_simulation_step(simulation, 1000) == RK_OK);
    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 1);
    assert(std::abs(clock.simulation_time - 0.01) < 1e-12);

    const auto first_state = snapshot(first);
    const auto second_state = snapshot(second);
    assert(first_state.sequence == 1);
    assert(second_state.sequence == 1);
    assert(first_state.source_timestamp_ns == 10000000);
    assert(second_state.source_timestamp_ns == 10000000);
    assert(first_state.received_timestamp_ns > 1000);
    assert(second_state.received_timestamp_ns >= first_state.received_timestamp_ns);
    assert(first_state.sensors[1].sequence == 0); // IMU derivative needs two samples.
    // F4: joint-connected links now genuinely rotate with their commanded
    // joint position instead of staying at their identity rest orientation,
    // so each robot's small link box presents a slightly different face to
    // the other robot's LIDAR ray than the pre-F4 (always axis-aligned) box.
    assert(std::abs(first_state.sensors[2].values[0] - 0.947662419923) < 1e-9);
    assert(std::abs(second_state.sensors[2].values[4] - 0.945714778581) < 1e-9);
    assert(first_state.sensors[2].values[4] == 10.0); // Own geometry excluded.
    assert(std::abs(first_state.position[0] - 0.4) < 1e-12);
    assert(std::abs(second_state.position[0] + 0.3) < 1e-12);

    rk_robot_runtime late = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &first_model, &late) ==
           RK_ERROR_INVALID_STATE);

    assert(rk_simulation_step(simulation, 2000) == RK_OK);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 2);
    assert(snapshot(first).sequence == 2);
    assert(snapshot(second).sequence == 2);
    assert(snapshot(first).sensors[1].sequence == 1);
    assert(std::abs(snapshot(first).sensors[1].values[5] - 9.81) < 1e-9);

    rk_simulation_presentation presentation = RK_INVALID_SIMULATION_PRESENTATION;
    assert(rk_simulation_capture_presentation(simulation, &presentation) == RK_OK);
    rk_simulation_presentation_info presentation_info{};
    presentation_info.struct_size = sizeof(presentation_info);
    assert(rk_simulation_presentation_get_info(presentation, &presentation_info) == RK_OK);
    assert(presentation_info.step_index == 2 && presentation_info.pose_count == 6);
    assert(std::abs(presentation_info.simulation_time - 0.02) < 1e-12);
    bool found_first_base = false;
    for (uint32_t index = 0; index < presentation_info.pose_count; ++index) {
        rk_simulation_presentation_pose item{};
        item.struct_size = sizeof(item);
        assert(rk_simulation_presentation_get_pose(presentation, index, &item) == RK_OK);
        if (item.kind == RK_SIMULATION_PRESENTATION_ROBOT_BASE && item.robot_index == 0) {
            found_first_base = true;
            assert(std::abs(item.position[0]) < 1e-6);
        }
    }
    assert(found_first_base);
    rk_simulation_presentation_destroy(presentation);

    assert(rk_simulation_stop(simulation) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.rotation[3] = 1.0;
    pose.position[0] = 4.0;
    assert(rk_simulation_teleport_robot(simulation, 0, &pose) == RK_OK);
    rk_simulation_pose observed_pose{};
    observed_pose.struct_size = sizeof(observed_pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed_pose) == RK_OK);
    assert(std::abs(observed_pose.position[0] - 4.0) < 1e-6);
    assert(rk_simulation_get_link_pose(simulation, 0, 0, &observed_pose) == RK_OK);
    assert(std::abs(observed_pose.position[0] - 4.0) < 1e-6);
    assert(rk_simulation_reset_robot(simulation, 0) == RK_OK);
    assert(snapshot(first).sequence == 0);
    rk_simulation_object_desc object_desc{};
    object_desc.struct_size = sizeof(object_desc);
    object_desc.motion_type = 0;
    object_desc.half_extents[0] = object_desc.half_extents[1] = object_desc.half_extents[2] = 0.25;
    object_desc.rotation[3] = 1.0;
    rk_simulation_object object = RK_INVALID_SIMULATION_OBJECT;
    assert(rk_simulation_spawn_object(simulation, &object_desc, &object) == RK_OK);
    assert(object != RK_INVALID_SIMULATION_OBJECT);
    pose.position[2] = 2.0;
    assert(rk_simulation_teleport_object(simulation, object, &pose) == RK_OK);
    assert(rk_simulation_get_object_pose(simulation, object, &observed_pose) == RK_OK);
    assert(std::abs(observed_pose.position[2] - 2.0) < 1e-6);
    assert(rk_simulation_capture_presentation(simulation, &presentation) == RK_OK);
    presentation_info = {};
    presentation_info.struct_size = sizeof(presentation_info);
    assert(rk_simulation_presentation_get_info(presentation, &presentation_info) == RK_OK);
    assert(presentation_info.pose_count == 7);
    bool found_environment = false;
    for (uint32_t index = 0; index < presentation_info.pose_count; ++index) {
        rk_simulation_presentation_pose item{};
        item.struct_size = sizeof(item);
        assert(rk_simulation_presentation_get_pose(presentation, index, &item) == RK_OK);
        if (item.kind == RK_SIMULATION_PRESENTATION_ENVIRONMENT && item.object_id == object) {
            found_environment = true;
            assert(std::abs(item.position[2] - 2.0) < 1e-6);
        }
    }
    assert(found_environment);
    rk_simulation_presentation_destroy(presentation);
    assert(rk_simulation_remove_object(simulation, object) == RK_OK);
    assert(rk_simulation_reset(simulation) == RK_OK);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 0);

    rk_simulation_destroy(simulation);
    rk_robot_state destroyed{};
    destroyed.struct_size = sizeof(destroyed);
    assert(rk_robot_runtime_snapshot(first, &destroyed) == RK_ERROR_INVALID_HANDLE);
}

void failed_command_phase_does_not_advance() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = blueprint(21);
    rk_robot_runtime first = RK_INVALID_ROBOT_RUNTIME;
    rk_robot_runtime second = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &model, &second) == RK_OK);

    auto first_target = target(0.8, 1);
    first_target.targets[0].max_rate = 2.0;
    rk_robot_command emergency{};
    emergency.struct_size = sizeof(emergency);
    emergency.sequence = 1;
    emergency.kind = RK_COMMAND_EMERGENCY_STOP;
    const auto rejected_target = target(-0.8, 2);
    assert(rk_robot_runtime_submit(first, &first_target) == RK_OK);
    assert(rk_robot_runtime_submit(second, &emergency) == RK_OK);
    assert(rk_robot_runtime_submit(second, &rejected_target) == RK_OK);
    /* Emergency stop arbitrates over the later motion request in one cycle. */
    assert(rk_simulation_step(simulation, 100) == RK_OK);
    assert(std::abs(snapshot(first).position[0] - 0.02) < 1e-12);

    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 1);

    const auto rejected_after_stop = target(-0.8, 3);
    assert(rk_robot_runtime_submit(second, &rejected_after_stop) == RK_OK);
    assert(rk_simulation_step(simulation, 200) == RK_ERROR_SAFETY_STOPPED);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 1);
    assert(std::abs(snapshot(first).position[0] - 0.02) < 1e-12);

    rk_robot_command clear_stop{};
    clear_stop.struct_size = sizeof(clear_stop);
    clear_stop.sequence = 4;
    clear_stop.kind = RK_COMMAND_RESET_SAFETY;
    assert(rk_robot_runtime_submit(second, &clear_stop) == RK_OK);
    assert(rk_simulation_step(simulation, 200) == RK_OK);
    assert(std::abs(snapshot(first).position[0] - 0.04) < 1e-12);
    rk_simulation_destroy(simulation);
}

void realtime_presentation_keeps_one_revision() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = blueprint(21);
    rk_robot_runtime runtime = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, &runtime) == RK_OK);
    assert(rk_simulation_start(simulation) == RK_OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    rk_simulation_presentation presentation = RK_INVALID_SIMULATION_PRESENTATION;
    assert(rk_simulation_capture_presentation(simulation, &presentation) == RK_OK);
    rk_simulation_presentation_info captured{};
    captured.struct_size = sizeof(captured);
    assert(rk_simulation_presentation_get_info(presentation, &captured) == RK_OK);
    assert(captured.step_index > 0 && captured.pose_count == 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    rk_simulation_presentation_info reread{};
    reread.struct_size = sizeof(reread);
    assert(rk_simulation_presentation_get_info(presentation, &reread) == RK_OK);
    assert(reread.step_index == captured.step_index &&
           reread.simulation_time == captured.simulation_time &&
           reread.pose_count == captured.pose_count);
    rk_simulation_clock live{};
    live.struct_size = sizeof(live);
    assert(rk_simulation_get_clock(simulation, &live) == RK_OK);
    assert(live.step_index > captured.step_index);

    rk_simulation_presentation_destroy(presentation);
    assert(rk_simulation_stop(simulation) == RK_OK);
    rk_simulation_destroy(simulation);
}

void velocity_targets_advance_joint_coordinates() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = blueprint(31);
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, &robot) == RK_OK);
    const auto command = velocity_target(2.0, 1);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, 100) == RK_OK);
    auto state = snapshot(robot);
    assert(std::abs(state.velocity[0] - 2.0) < 1e-12);
    assert(std::abs(state.position[0] - 0.02) < 1e-12);
    assert(rk_simulation_step(simulation, 200) == RK_OK);
    state = snapshot(robot);
    assert(std::abs(state.position[0] - 0.04) < 1e-12);
    rk_simulation_destroy(simulation);
}

void sensor_geometry_and_reset() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    auto model = blueprint(1);
    // Root identity must follow topology, not link array order.
    model.joints[0].parent_link = 1;
    model.joints[0].child_link = 0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot) == RK_OK);
    rk_simulation_object_desc box{};
    box.struct_size = sizeof(box);
    box.position[0] = 2.0;
    box.rotation[2] = box.rotation[3] = std::sqrt(0.5); // quarter turn
    box.half_extents[0] = 0.5;
    box.half_extents[1] = box.half_extents[2] = 0.25;
    rk_simulation_object object = 0;
    assert(rk_simulation_spawn_object(simulation, &box, &object) == RK_OK);
    assert(rk_simulation_step(simulation, 100) == RK_OK);
    const auto before = snapshot(robot);
    assert(std::abs(before.sensors[2].values[0] - 1.75) < 1e-6);
    assert(before.sensors[2].values[2] == 10.0);
    assert(rk_simulation_step(simulation, 200) == RK_OK);
    assert(snapshot(robot).sensors[1].sequence == 1);
    assert(std::abs(snapshot(robot).sensors[1].values[5] - 9.81) < 1e-9);
    assert(before.sensors[1].sequence == 0); // Old observation remains unchanged.
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_reset(simulation) == RK_OK);
    assert(snapshot(robot).sensor_count == 0);
    assert(rk_simulation_step(simulation, 300) == RK_OK);
    assert(snapshot(robot).sensors[1].sequence == 0);
    assert(std::abs(snapshot(robot).sensors[2].values[0] - 1.75) < 1e-6);
    assert(rk_simulation_stop(simulation) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.rotation[1] = std::sqrt(0.5);
    pose.rotation[3] = std::sqrt(0.5);
    assert(rk_simulation_teleport_robot(simulation, 0, &pose) == RK_OK);
    assert(rk_simulation_step(simulation, 400) == RK_OK);
    assert(snapshot(robot).sensors[1].sequence == 0); // Teleport primes derivative.
    assert(rk_simulation_step(simulation, 500) == RK_OK);
    assert(std::abs(snapshot(robot).sensors[1].values[3] + 9.81) < 1e-9);
    assert(std::abs(snapshot(robot).sensors[1].values[5]) < 1e-9);
    assert(rk_simulation_stop(simulation) == RK_OK);
    pose.rotation[1] = 0.0;
    pose.rotation[3] = 1.0;
    assert(rk_simulation_teleport_robot(simulation, 0, &pose) == RK_OK);
    assert(rk_simulation_remove_object(simulation, object) == RK_OK);
    assert(rk_simulation_step(simulation, 600) == RK_OK);
    assert(snapshot(robot).sensors[2].values[0] == 10.0);
    rk_simulation_destroy(simulation);

    // Free fall: accelerometer measures specific force, not gravity itself.
    const double identity[4] = {0, 0, 0, 1};
    const double gravity[3] = {0, 0, -9.81};
    const double gyro[3] = {1, 2, 3};
    double imu[6];
    robotkit::sensors::imu(identity, gyro, gravity, gravity, imu);
    for (int i = 0; i < 3; ++i) {
        assert(imu[i] == gyro[i]);
        assert(imu[i + 3] == 0.0);
    }
}

} // namespace

int main() {
    shared_world_steps_once();
    realtime_presentation_keeps_one_revision();
    failed_command_phase_does_not_advance();
    velocity_targets_advance_joint_coordinates();
    sensor_geometry_and_reset();
    return 0;
}
