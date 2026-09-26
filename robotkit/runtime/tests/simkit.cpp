#include "robotkit_simkit.h"
#include "../src/sensor_math.hpp"

#include <algorithm>
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
    assert(rk_simulation_add_robot(simulation, &first_model, nullptr, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &second_model, nullptr, &second) == RK_OK);

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
    assert(rk_simulation_add_robot(simulation, &first_model, nullptr, &late) ==
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
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &first) == RK_OK);
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &second) == RK_OK);

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
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &runtime) == RK_OK);
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
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
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
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
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

void driving_base_keeps_owner_sensors_and_reset_pose() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = blueprint(1);
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.rotation[3] = 1.0;
    rk_simulation_pose observed{};
    observed.struct_size = sizeof(observed);

    assert(rk_simulation_step(simulation, 100) == RK_OK);
    assert(rk_simulation_step(simulation, 200) == RK_OK);
    assert(snapshot(robot).sensors[1].sequence == 1);
    // Driving between external steps keeps the derivative history: the IMU
    // publishes on every tick instead of re-priming after each pose update.
    // The base moves with the driven motion, 0.1 m per 0.01 s tick = 10 m/s:
    // the accelerometer sees the 0 -> 10 m/s step once, then only gravity.
    for (int tick = 1; tick <= 5; ++tick) {
        pose.position[0] = 0.1 * tick;
        assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == RK_OK);
        assert(rk_simulation_step(simulation, 200 + 100 * tick) == RK_OK);
        const auto imu = snapshot(robot).sensors[1];
        assert(imu.sequence == static_cast<uint64_t>(1 + tick));
        assert(std::abs(imu.values[3] - (tick == 1 ? 1000.0 : 0.0)) < 1e-2);
        assert(std::abs(imu.values[4]) < 1e-9 && std::abs(imu.values[5] - 9.81) < 1e-9);
        for (int axis = 0; axis < 3; ++axis) assert(std::abs(imu.values[axis]) < 1e-9);
        assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
        assert(std::abs(observed.position[0] - 0.1 * tick) < 1e-6);
    }
    // Turning the driven base 0.01 rad per tick is a 1 rad/s yaw rate.
    for (int tick = 1; tick <= 3; ++tick) {
        pose.rotation[2] = std::sin(0.005 * tick);
        pose.rotation[3] = std::cos(0.005 * tick);
        assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == RK_OK);
        assert(rk_simulation_step(simulation, 700 + 100 * tick) == RK_OK);
        const auto imu = snapshot(robot).sensors[1];
        assert(std::abs(imu.values[2] - 1.0) < 1e-3);
        assert(std::abs(imu.values[0]) < 1e-6 && std::abs(imu.values[1]) < 1e-6);
    }
    pose.rotation[2] = 0.0;
    pose.rotation[3] = 1.0;
    rk_simulation_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(rk_simulation_get_clock(simulation, &clock) == RK_OK);
    assert(clock.step_index == 10);

    // Driving is also accepted by the realtime owner without stopping it.
    assert(rk_simulation_start(simulation) == RK_OK);
    pose.position[0] = 1.5;
    assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == RK_OK);
    for (int attempt = 0; attempt < 200; ++attempt) {
        assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
        if (std::abs(observed.position[0] - 1.5) < 1e-6) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    assert(std::abs(observed.position[0] - 1.5) < 1e-6);
    assert(rk_simulation_step(simulation, 1000) == RK_ERROR_INVALID_STATE); // Still running.
    assert(rk_simulation_stop(simulation) == RK_OK);

    // Driving never replaces the reset pose, including the kinematic node.
    assert(rk_simulation_reset_robot(simulation, 0) == RK_OK);
    assert(rk_simulation_step(simulation, 2000) == RK_OK);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
    assert(std::abs(observed.position[0]) < 1e-6);

    pose.rotation[3] = 2.0;
    assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == RK_ERROR_INVALID_ARGUMENT);
    pose.rotation[3] = 1.0;
    assert(rk_simulation_drive_robot_base(simulation, 1, &pose) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_drive_robot_base(simulation, 0, nullptr) == RK_ERROR_INVALID_ARGUMENT);
    rk_simulation_destroy(simulation);
}

rk_robot_runtime_blueprint wheeled_blueprint() {
    rk_robot_runtime_blueprint value{};
    value.struct_size = sizeof(value);
    value.revision = 7;
    value.joint_count = 2;
    value.link_count = 3;
    value.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < value.link_count; ++i) {
        value.links[i].mass = 1.0;
        value.links[i].inertia_tensor[0] = value.links[i].inertia_tensor[4] =
            value.links[i].inertia_tensor[8] = 1.0;
    }
    for (uint32_t joint = 0; joint < value.joint_count; ++joint) {
        value.joints[joint] = {joint, RK_RUNTIME_JOINT_REVOLUTE, 0, joint + 1,
                               -1000.0, 1000.0, 100.0};
        value.joints[joint].parent_frame_rotation[3] = 1.0;
        value.joints[joint].child_frame_rotation[3] = 1.0;
        value.joints[joint].axis[1] = 1.0;
    }
    assert(rk_robot_runtime_blueprint_validate(&value) == RK_OK);
    return value;
}

rk_robot_command wheel_targets(double left, double right, uint64_t sequence,
                               double max_rate = 0.0) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    value.target_count = 2;
    value.targets[0] = {0, RK_TARGET_VELOCITY, left, max_rate, 0.0};
    value.targets[1] = {1, RK_TARGET_VELOCITY, right, max_rate, 0.0};
    return value;
}

rk_robot_command lifecycle(rk_command_kind kind, uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = kind;
    return value;
}

rk_simulation make_simulation(double timestep) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = timestep;
    desc.physics_substeps = 1;
    rk_simulation simulation = RK_INVALID_SIMULATION;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    return simulation;
}

void normal_stop_zeroes_wheel_velocities() {
    auto simulation = make_simulation(0.01);
    const auto model = wheeled_blueprint();
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    uint64_t sequence = 0;
    uint64_t time = 0;
    auto command = wheel_targets(2.0, 3.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    auto state = snapshot(robot);
    assert(state.velocity[0] == 2.0 && state.velocity[1] == 3.0);
    assert(std::abs(state.position[0] - 0.04) < 1e-12);

    // A normal stop halts the wheels in the physics backend at once; wheel
    // odometry (encoder positions) stops changing although nothing latches.
    command = lifecycle(RK_COMMAND_STOP, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    const auto stopped = snapshot(robot);
    assert(stopped.mode == RK_ROBOT_MODE_STOPPING && stopped.safety == RK_SAFETY_STOPPING);
    assert(stopped.velocity[0] == 0.0 && stopped.velocity[1] == 0.0);
    for (int tick = 0; tick < 5; ++tick) {
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
        state = snapshot(robot);
        assert(state.velocity[0] == 0.0 && state.velocity[1] == 0.0);
        assert(state.position[0] == stopped.position[0]);
        assert(state.position[1] == stopped.position[1]);
        assert(state.sensors[0].values[0] == stopped.position[0]); // Encoder sample.
    }

    // A new command resumes motion without a safety reset.
    command = wheel_targets(1.0, 1.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    state = snapshot(robot);
    assert(state.mode == RK_ROBOT_MODE_TRACKING && state.velocity[0] == 1.0);
    assert(state.position[0] > stopped.position[0]);
    rk_simulation_destroy(simulation);
}

rk_simulation_differential_drive_state drive_state(rk_simulation simulation) {
    rk_simulation_differential_drive_state value{};
    value.struct_size = sizeof(value);
    assert(rk_simulation_get_differential_drive_state(simulation, 0, &value) == RK_OK);
    return value;
}

void differential_drive_follows_applied_wheel_targets() {
    constexpr double dt = 0.02;
    auto simulation = make_simulation(dt);
    const auto model = wheeled_blueprint();
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    const double start_yaw = 0.5;
    rk_simulation_pose start{};
    start.struct_size = sizeof(start);
    start.position[0] = 1.0;
    start.position[1] = 2.0;
    start.position[2] = 0.3;
    start.rotation[2] = std::sin(start_yaw * 0.5);
    start.rotation[3] = std::cos(start_yaw * 0.5);
    assert(rk_simulation_teleport_robot(simulation, 0, &start) == RK_OK);

    rk_simulation_differential_drive_desc drive{};
    drive.struct_size = sizeof(drive);
    drive.left_wheel_joint = 0;
    drive.right_wheel_joint = 1;
    drive.wheel_radius = 0.1;
    drive.track_width = 0.5;
    auto invalid = drive;
    invalid.right_wheel_joint = 0;
    assert(rk_simulation_set_differential_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.right_wheel_joint = 2;
    assert(rk_simulation_set_differential_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.wheel_radius = 0.0;
    assert(rk_simulation_set_differential_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.struct_size = 0;
    assert(rk_simulation_set_differential_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_differential_drive(simulation, 1, &drive) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_differential_drive(simulation, 0, nullptr) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_differential_drive(simulation, 0, &drive) == RK_OK);
    auto plant = drive_state(simulation);
    assert(plant.enabled == 1 && plant.x == 1.0 && plant.y == 2.0 && plant.height == 0.3);
    assert(std::abs(plant.yaw - start_yaw) < 1e-12);

    // Targets submitted straight to the robot drive the base on the same tick
    // they are applied. 5 rad/s on 0.1 m wheels is 0.5 m/s: 0.01 m per tick.
    uint64_t sequence = 0;
    uint64_t time = 0;
    auto command = wheel_targets(5.0, 5.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = drive_state(simulation);
    assert(plant.left_wheel_rate == 5.0 && plant.right_wheel_rate == 5.0);
    assert(std::abs(plant.x - (1.0 + 0.01 * std::cos(start_yaw))) < 1e-12);
    assert(std::abs(plant.y - (2.0 + 0.01 * std::sin(start_yaw))) < 1e-12);
    for (int tick = 1; tick < 10; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = drive_state(simulation);
    assert(std::abs(plant.x - (1.0 + 0.1 * std::cos(start_yaw))) < 1e-12);
    assert(std::abs(plant.y - (2.0 + 0.1 * std::sin(start_yaw))) < 1e-12);
    assert(std::abs(plant.yaw - start_yaw) < 1e-12 && plant.height == 0.3);
    rk_simulation_pose observed{};
    observed.struct_size = sizeof(observed);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
    assert(std::abs(observed.position[0] - plant.x) < 1e-6);
    assert(std::abs(observed.position[1] - plant.y) < 1e-6);
    assert(std::abs(observed.position[2] - 0.3) < 1e-6);
    // Straight at constant speed: no rotation, no acceleration beyond gravity.
    auto imu = snapshot(robot).sensors[1];
    assert(imu.sequence > 0);
    for (int axis = 0; axis < 3; ++axis) assert(std::abs(imu.values[axis]) < 1e-4);
    assert(std::abs(imu.values[3]) < 1e-2 && std::abs(imu.values[4]) < 1e-2);
    assert(std::abs(imu.values[5] - 9.81) < 1e-6);

    // Arc at 0.5 m/s and 1 rad/s: wheels 2.5 and 7.5 rad/s. The gyro reads
    // the yaw rate and the accelerometer the 0.5 m/s^2 centripetal pull to
    // the left in the body frame.
    command = wheel_targets(2.5, 7.5, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const double arc_start_yaw = plant.yaw;
    for (int tick = 0; tick < 5; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = drive_state(simulation);
    assert(std::abs(plant.yaw - (arc_start_yaw + 5 * dt)) < 1e-12);
    imu = snapshot(robot).sensors[1];
    assert(std::abs(imu.values[2] - 1.0) < 1e-3);
    assert(std::abs(imu.values[0]) < 1e-4 && std::abs(imu.values[1]) < 1e-4);
    assert(std::abs(imu.values[3]) < 2e-2);
    assert(std::abs(imu.values[4] - 0.5) < 2e-2);

    // The runtime clamps each wheel at its max rate; the plant uses the
    // clamped, applied rate rather than the requested one.
    command = wheel_targets(4.25, 11.75, ++sequence, 10.0);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = drive_state(simulation);
    assert(plant.left_wheel_rate == 4.25 && plant.right_wheel_rate == 10.0);

    // A normal stop halts the base on the tick it is applied.
    command = lifecycle(RK_COMMAND_STOP, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const auto before_stop = drive_state(simulation);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = drive_state(simulation);
    assert(plant.left_wheel_rate == 0.0 && plant.right_wheel_rate == 0.0);
    assert(plant.x == before_stop.x && plant.y == before_stop.y && plant.yaw == before_stop.yaw);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    assert(drive_state(simulation).x == before_stop.x);
    imu = snapshot(robot).sensors[1];
    for (int axis = 0; axis < 3; ++axis) assert(std::abs(imu.values[axis]) < 1e-4);

    // An emergency stop halts it too, and a safety reset does not resume the
    // cleared targets.
    command = wheel_targets(5.0, 5.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    command = lifecycle(RK_COMMAND_EMERGENCY_STOP, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const auto before_estop = drive_state(simulation);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    assert(drive_state(simulation).x == before_estop.x);
    command = lifecycle(RK_COMMAND_RESET_SAFETY, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    assert(drive_state(simulation).x == before_estop.x);

    // Placing the base mid-motion is a jump, not a velocity: the plant carries
    // on from the new pose and the accelerometer sees no spike.
    command = wheel_targets(5.0, 5.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    const auto moving = drive_state(simulation);
    rk_simulation_pose placed{};
    placed.struct_size = sizeof(placed);
    placed.position[0] = moving.x;
    placed.position[1] = moving.y + 0.6;
    placed.position[2] = 0.3;
    placed.rotation[2] = std::sin(moving.yaw * 0.5);
    placed.rotation[3] = std::cos(moving.yaw * 0.5);
    const auto imu_sequence = snapshot(robot).sensors[1].sequence;
    assert(rk_simulation_place_robot_base(simulation, 0, &placed) == RK_OK);
    plant = drive_state(simulation);
    assert(plant.x == moving.x && plant.y == moving.y + 0.6 &&
           std::abs(plant.yaw - moving.yaw) < 1e-12);
    for (int tick = 0; tick < 2; ++tick) {
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
        imu = snapshot(robot).sensors[1];
        assert(imu.sequence == imu_sequence + 1 + tick); // Sensors keep running.
        assert(std::abs(imu.values[3]) < 1e-2 && std::abs(imu.values[4]) < 1e-2);
    }
    plant = drive_state(simulation);
    assert(std::abs(plant.x - (moving.x + 0.02 * std::cos(moving.yaw))) < 1e-12);
    assert(std::abs(plant.y - (moving.y + 0.6 + 0.02 * std::sin(moving.yaw))) < 1e-12);

    // Resetting the robot puts the plant back at the reset pose, at rest.
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_reset_robot(simulation, 0) == RK_OK);
    plant = drive_state(simulation);
    assert(plant.x == 1.0 && plant.y == 2.0 && std::abs(plant.yaw - start_yaw) < 1e-12);
    assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    assert(drive_state(simulation).x == 1.0);
    assert(rk_simulation_clear_differential_drive(simulation, 0) == RK_OK);
    assert(drive_state(simulation).enabled == 0);
    assert(rk_simulation_clear_differential_drive(simulation, 1) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_place_robot_base(simulation, 0, nullptr) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_place_robot_base(simulation, 1, &placed) == RK_ERROR_INVALID_ARGUMENT);
    rk_simulation_destroy(simulation);
}

// Scene nodes are single precision: 1000 m out a float step is ~6e-5 m, so a
// velocity differenced from node poses at 50 Hz is off by ~3e-3 m/s and an
// IMU differencing that reads ~0.15 m/s^2 of noise. The plant and base drives
// hand SimKit their exact double-precision twist instead.
void driven_base_far_from_origin_reads_exact_imu() {
    constexpr double dt = 0.02;
    auto simulation = make_simulation(dt);
    const auto model = wheeled_blueprint();
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    const double yaw = 0.7;
    rk_simulation_pose start{};
    start.struct_size = sizeof(start);
    start.position[0] = 1000.0;
    start.position[1] = -1000.0;
    start.position[2] = 0.3;
    start.rotation[2] = std::sin(yaw * 0.5);
    start.rotation[3] = std::cos(yaw * 0.5);
    assert(rk_simulation_teleport_robot(simulation, 0, &start) == RK_OK);
    rk_simulation_differential_drive_desc drive{};
    drive.struct_size = sizeof(drive);
    drive.left_wheel_joint = 0;
    drive.right_wheel_joint = 1;
    drive.wheel_radius = 0.1;
    drive.track_width = 0.5;
    assert(rk_simulation_set_differential_drive(simulation, 0, &drive) == RK_OK);

    // 7 rad/s on 0.1 m wheels: 0.7 m/s straight ahead.
    uint64_t sequence = 0, time = 0;
    auto command = wheel_targets(7.0, 7.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    for (int tick = 0; tick < 100; ++tick) {
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
        const auto imu = snapshot(robot).sensors[1];
        for (int axis = 0; axis < 3; ++axis) assert(std::abs(imu.values[axis]) < 1e-9);
        assert(std::abs(imu.values[3]) < 1e-6 && std::abs(imu.values[4]) < 1e-6);
        assert(std::abs(imu.values[5] - 9.81) < 1e-6);
    }
    const auto plant = drive_state(simulation);
    rk_simulation_pose observed{};
    observed.struct_size = sizeof(observed);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
    // The body holds the plant's double-precision pose, not the float node.
    assert(std::abs(observed.position[0] - plant.x) < 1e-9);
    assert(std::abs(observed.position[1] - plant.y) < 1e-9);
    assert(std::abs(plant.x - (1000.0 + 0.014 * 103 * std::cos(yaw))) < 1e-9);

    // After a stop the base rests where the plant left it, reading gravity.
    command = lifecycle(RK_COMMAND_STOP, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    auto imu = snapshot(robot).sensors[1];
    assert(std::abs(imu.values[3]) < 1e-6 && std::abs(imu.values[4]) < 1e-6);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
    assert(observed.position[0] == drive_state(simulation).x);

    // Poses driven from outside are differenced in double precision too.
    assert(rk_simulation_clear_differential_drive(simulation, 0) == RK_OK);
    rk_simulation_pose pose = observed;
    const double from_x = observed.position[0];
    for (int tick = 1; tick <= 50; ++tick) {
        pose.position[0] = from_x + 0.013 * tick;
        assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == RK_OK);
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
        if (tick < 3) continue; // Start-up step, then the IMU's first difference.
        imu = snapshot(robot).sensors[1];
        assert(std::abs(imu.values[3]) < 1e-6 && std::abs(imu.values[4]) < 1e-6);
        assert(std::abs(imu.values[5] - 9.81) < 1e-6);
    }
    rk_simulation_destroy(simulation);
}

// A ground robot rolls on the level floor: the plant keeps the base's authored
// roll and pitch (turning them with the heading) instead of snapping upright.
void differential_drive_keeps_authored_tilt() {
    constexpr double dt = 0.02;
    auto simulation = make_simulation(dt);
    const auto model = wheeled_blueprint();
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    const double roll = 0.1, pitch = -0.05, yaw = 0.3;
    const double qx[4] = {std::sin(roll * 0.5), 0.0, 0.0, std::cos(roll * 0.5)};
    const double qy[4] = {0.0, std::sin(pitch * 0.5), 0.0, std::cos(pitch * 0.5)};
    const double qz[4] = {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
    double tilt[4], start_rotation[4];
    robotkit::sensors::multiply(qy, qx, tilt); // Roll and pitch, heading removed.
    robotkit::sensors::multiply(qz, tilt, start_rotation);
    rk_simulation_pose start{};
    start.struct_size = sizeof(start);
    start.position[0] = 1.0;
    start.position[1] = 2.0;
    start.position[2] = 0.3;
    std::copy_n(start_rotation, 4, start.rotation);
    assert(rk_simulation_teleport_robot(simulation, 0, &start) == RK_OK);
    rk_simulation_differential_drive_desc drive{};
    drive.struct_size = sizeof(drive);
    drive.left_wheel_joint = 0;
    drive.right_wheel_joint = 1;
    drive.wheel_radius = 0.1;
    drive.track_width = 0.5;
    assert(rk_simulation_set_differential_drive(simulation, 0, &drive) == RK_OK);
    auto plant = drive_state(simulation);
    assert(std::abs(plant.yaw - yaw) < 1e-12 && plant.height == 0.3);

    // Arc at 0.5 m/s and 1 rad/s.
    uint64_t sequence = 0, time = 0;
    auto command = wheel_targets(2.5, 7.5, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    rk_simulation_pose observed{};
    observed.struct_size = sizeof(observed);
    for (int tick = 1; tick <= 20; ++tick) {
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
        plant = drive_state(simulation);
        assert(std::abs(plant.yaw - (yaw + tick * dt)) < 1e-12);
        assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
        // Planar motion at the seeded height; the plant reports the same pose.
        assert(std::abs(observed.position[0] - plant.x) < 1e-12);
        assert(std::abs(observed.position[1] - plant.y) < 1e-12);
        assert(observed.position[2] == 0.3 && plant.height == 0.3);
        // Rotation is the heading about world Z composed with the authored tilt.
        const double heading[4] = {0.0, 0.0, std::sin(plant.yaw * 0.5),
                                   std::cos(plant.yaw * 0.5)};
        double expected[4];
        robotkit::sensors::multiply(heading, tilt, expected);
        const double sign = expected[3] * observed.rotation[3] < 0.0 ? -1.0 : 1.0;
        for (int axis = 0; axis < 4; ++axis)
            assert(std::abs(observed.rotation[axis] - sign * expected[axis]) < 1e-12);
        if (tick < 3) continue;
        // The gyro reads the world-Z turn rate in the tilted body frame.
        const double world_rate[3] = {0.0, 0.0, 1.0};
        double body_rate[3];
        robotkit::sensors::rotate(observed.rotation, world_rate, body_rate, true);
        const auto imu = snapshot(robot).sensors[1];
        for (int axis = 0; axis < 3; ++axis)
            assert(std::abs(imu.values[axis] - body_rate[axis]) < 1e-9);
    }
    // Plant x/y still follow the planar arc: 20 ticks at 0.01 m, 0.02 rad each.
    const double radius = 0.5;
    const double end_yaw = yaw + 20 * dt;
    assert(std::abs(plant.x - (1.0 + radius * (std::sin(end_yaw) - std::sin(yaw)))) < 1e-12);
    assert(std::abs(plant.y - (2.0 - radius * (std::cos(end_yaw) - std::cos(yaw)))) < 1e-12);
    rk_simulation_destroy(simulation);
}

rk_robot_runtime_blueprint omni_blueprint() {
    auto value = wheeled_blueprint();
    value.joint_count = 3;
    value.link_count = 4;
    value.links[3] = value.links[1];
    value.joints[2] = value.joints[1];
    value.joints[2].joint = 2;
    value.joints[2].child_link = 3;
    assert(rk_robot_runtime_blueprint_validate(&value) == RK_OK);
    return value;
}

constexpr double omni_wheel_radius = 0.1;
constexpr double omni_base_radius = 0.2;

double omni_angle(int wheel) { return 1.5707963267948966 + wheel * 2.0943951023931957; }

// Joint targets that move the base at body twist (vx, vy, omega).
rk_robot_command omni_targets(double vx, double vy, double omega, uint64_t sequence) {
    rk_robot_command value{};
    value.struct_size = sizeof(value);
    value.sequence = sequence;
    value.kind = RK_COMMAND_JOINT_TARGETS;
    value.target_count = 3;
    for (uint32_t wheel = 0; wheel < 3; ++wheel) {
        const double angle = omni_angle(static_cast<int>(wheel));
        const double speed =
            -std::sin(angle) * vx + std::cos(angle) * vy + omni_base_radius * omega;
        value.targets[wheel] = {wheel, RK_TARGET_VELOCITY, speed / omni_wheel_radius, 0.0, 0.0};
    }
    return value;
}

rk_simulation_omni_drive_state omni_state(rk_simulation simulation) {
    rk_simulation_omni_drive_state value{};
    value.struct_size = sizeof(value);
    assert(rk_simulation_get_omni_drive_state(simulation, 0, &value) == RK_OK);
    return value;
}

void omni_drive_follows_applied_wheel_targets() {
    constexpr double dt = 0.02;
    auto simulation = make_simulation(dt);
    const auto model = omni_blueprint();
    rk_robot_runtime robot = RK_INVALID_ROBOT_RUNTIME;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    const double start_yaw = 0.5;
    rk_simulation_pose start{};
    start.struct_size = sizeof(start);
    start.position[0] = 1.0;
    start.position[1] = 2.0;
    start.position[2] = 0.3;
    start.rotation[2] = std::sin(start_yaw * 0.5);
    start.rotation[3] = std::cos(start_yaw * 0.5);
    assert(rk_simulation_teleport_robot(simulation, 0, &start) == RK_OK);

    rk_simulation_omni_drive_desc drive{};
    drive.struct_size = sizeof(drive);
    for (uint32_t wheel = 0; wheel < 3; ++wheel) {
        drive.wheel_joints[wheel] = wheel;
        drive.wheel_angles[wheel] = omni_angle(static_cast<int>(wheel));
    }
    drive.wheel_radius = omni_wheel_radius;
    drive.base_radius = omni_base_radius;
    auto invalid = drive;
    invalid.wheel_joints[2] = 0;
    assert(rk_simulation_set_omni_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.wheel_joints[2] = 3;
    assert(rk_simulation_set_omni_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.wheel_angles[1] = invalid.wheel_angles[2] = invalid.wheel_angles[0];
    assert(rk_simulation_set_omni_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.base_radius = 0.0;
    assert(rk_simulation_set_omni_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = drive;
    invalid.struct_size = 0;
    assert(rk_simulation_set_omni_drive(simulation, 0, &invalid) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_omni_drive(simulation, 0, nullptr) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_omni_drive(simulation, 0, &drive) == RK_OK);
    auto plant = omni_state(simulation);
    assert(plant.enabled == 1 && plant.x == 1.0 && plant.y == 2.0 && plant.height == 0.3);
    assert(std::abs(plant.yaw - start_yaw) < 1e-12);
    // One coupling per robot: the differential view reports it disabled.
    assert(drive_state(simulation).enabled == 0);

    // Strafe: 0.5 m/s along the body's +Y for 10 ticks is 0.1 m to the left
    // of the heading, with no turn.
    uint64_t sequence = 0;
    uint64_t time = 0;
    auto command = omni_targets(0.0, 0.5, 0.0, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 10; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = omni_state(simulation);
    for (uint32_t wheel = 0; wheel < 3; ++wheel)
        assert(plant.wheel_rates[wheel] == command.targets[wheel].target);
    assert(std::abs(plant.x - (1.0 - 0.1 * std::sin(start_yaw))) < 1e-9);
    assert(std::abs(plant.y - (2.0 + 0.1 * std::cos(start_yaw))) < 1e-9);
    assert(std::abs(plant.yaw - start_yaw) < 1e-12 && plant.height == 0.3);
    rk_simulation_pose observed{};
    observed.struct_size = sizeof(observed);
    assert(rk_simulation_get_robot_pose(simulation, 0, &observed) == RK_OK);
    assert(std::abs(observed.position[0] - plant.x) < 1e-6);
    assert(std::abs(observed.position[1] - plant.y) < 1e-6);

    // A constant body twist with a turn traces a circle about a fixed centre:
    // the centre sits at R(yaw) * (-vy, vx) / omega from the base.
    const double vx = 0.3, vy = 0.2, omega = 1.0;
    const double center_x = plant.x + (-std::cos(plant.yaw) * vy - std::sin(plant.yaw) * vx) / omega;
    const double center_y = plant.y + (-std::sin(plant.yaw) * vy + std::cos(plant.yaw) * vx) / omega;
    const double arc_start_yaw = plant.yaw;
    command = omni_targets(vx, vy, omega, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    for (int tick = 0; tick < 25; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = omni_state(simulation);
    const double end_yaw = arc_start_yaw + 25 * dt * omega;
    assert(std::abs(plant.yaw - end_yaw) < 1e-9);
    assert(std::abs(plant.x - (center_x + (std::cos(end_yaw) * vy + std::sin(end_yaw) * vx) / omega)) < 1e-9);
    assert(std::abs(plant.y - (center_y + (std::sin(end_yaw) * vy - std::cos(end_yaw) * vx) / omega)) < 1e-9);
    // The IMU reads the yaw rate while turning.
    const auto imu = snapshot(robot).sensors[1];
    assert(std::abs(imu.values[2] - omega) < 1e-3);

    // A normal stop halts the base on the tick it is applied.
    command = lifecycle(RK_COMMAND_STOP, ++sequence);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const auto before_stop = omni_state(simulation);
    for (int tick = 0; tick < 3; ++tick)
        assert(rk_simulation_step(simulation, time += 100) == RK_OK);
    plant = omni_state(simulation);
    for (uint32_t wheel = 0; wheel < 3; ++wheel) assert(plant.wheel_rates[wheel] == 0.0);
    assert(plant.x == before_stop.x && plant.y == before_stop.y && plant.yaw == before_stop.yaw);

    // Clearing the coupling (only the matching kind clears it) disables it.
    assert(rk_simulation_clear_differential_drive(simulation, 0) == RK_OK);
    assert(omni_state(simulation).enabled == 1);
    assert(rk_simulation_clear_omni_drive(simulation, 0) == RK_OK);
    assert(omni_state(simulation).enabled == 0);
    rk_simulation_destroy(simulation);
}

} // namespace

int main() {
    shared_world_steps_once();
    realtime_presentation_keeps_one_revision();
    failed_command_phase_does_not_advance();
    velocity_targets_advance_joint_coordinates();
    sensor_geometry_and_reset();
    driving_base_keeps_owner_sensors_and_reset_pose();
    normal_stop_zeroes_wheel_velocities();
    differential_drive_follows_applied_wheel_targets();
    driven_base_far_from_origin_reads_exact_imu();
    differential_drive_keeps_authored_tilt();
    omni_drive_follows_applied_wheel_targets();
    return 0;
}
