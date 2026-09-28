#include "robotkit_simkit.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static rk_robot_state state(rk_robot_runtime robot) {
    rk_robot_state value{};
    value.struct_size = sizeof(value);
    assert(rk_robot_runtime_snapshot(robot, &value) == RK_OK);
    return value;
}

static void convex_link_and_box_link_build() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 1;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (auto &link : model.links) {
        link.mass = 1.0;
        link.inertia_tensor[0] = link.inertia_tensor[4] = link.inertia_tensor[8] = 1.0;
    }
    model.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0};
    model.joints[0].parent_frame_rotation[3] = model.joints[0].child_frame_rotation[3] = 1.0;
    model.joints[0].axis[2] = 1.0;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.rotation[3] = 1.0;
    robot_desc.collision_hull_count[0] = 8;
    for (int index = 0; index < 8; ++index) {
        robot_desc.collision_hull_vertices[index * 3] = (index & 1) ? 0.5 : -0.5;
        robot_desc.collision_hull_vertices[index * 3 + 1] = (index & 2) ? 0.5 : -0.5;
        robot_desc.collision_hull_vertices[index * 3 + 2] = (index & 4) ? 0.5 : -0.5;
    }
    robot_desc.collision_half_extents[3] = 0.2;
    robot_desc.collision_half_extents[4] = 0.3;
    robot_desc.collision_half_extents[5] = 0.4;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    assert(rk_simulation_start(simulation) == RK_OK);
    rk_simulation_presentation presentation = 0;
    assert(rk_simulation_capture_presentation(simulation, &presentation) == RK_OK);
    rk_simulation_presentation_destroy(presentation);
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_step(simulation, 0) == RK_OK);
    rk_simulation_destroy(simulation);
}

static void tool_hulls_collide_only_on_their_pieces() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.links[0].mass = 1.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 1.0;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.rotation[3] = 1.0;
    robot_desc.tool_link_index = 0;
    robot_desc.tool_piece_count = 2;
    robot_desc.tool_piece_vertex_count[0] = 8;
    robot_desc.tool_piece_vertex_count[1] = 8;
    for (int piece = 0; piece < 2; ++piece) for (int vertex = 0; vertex < 8; ++vertex) {
        auto *point = robot_desc.tool_piece_vertices + piece * 64 * 3 + vertex * 3;
        point[0] = piece == 0 ? ((vertex & 1) ? 1.0 : 0.0) :
            ((vertex & 1) ? 1.0 : 0.9);
        point[1] = (vertex & 2) ? 0.05 : -0.05;
        point[2] = piece == 0 ? ((vertex & 4) ? 0.1 : 0.0) :
            ((vertex & 4) ? 1.0 : 0.0);
    }
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    rk_simulation_object_desc obstacle{};
    obstacle.struct_size = sizeof(obstacle);
    obstacle.motion_type = 2;
    obstacle.rotation[3] = 1.0;
    obstacle.mass = 1.0;
    obstacle.half_extents[0] = obstacle.half_extents[1] =
        obstacle.half_extents[2] = 0.05;
    obstacle.position[0] = 0.5;
    obstacle.position[2] = 1.2;
    rk_simulation_object in_gap = 0, on_tool = 0;
    assert(rk_simulation_spawn_object(simulation, &obstacle, &in_gap) == RK_OK);
    obstacle.position[0] = 0.95;
    assert(rk_simulation_spawn_object(simulation, &obstacle, &on_tool) == RK_OK);
    for (int tick = 0; tick < 20; ++tick)
        assert(rk_simulation_step(simulation, tick) == RK_OK);
    rk_simulation_pose gap_pose{}, tool_pose{};
    gap_pose.struct_size = tool_pose.struct_size = sizeof(rk_simulation_pose);
    assert(rk_simulation_get_object_pose(simulation, in_gap, &gap_pose) == RK_OK);
    assert(rk_simulation_get_object_pose(simulation, on_tool, &tool_pose) == RK_OK);
    assert(gap_pose.position[2] < 1.03);
    assert(tool_pose.position[2] > gap_pose.position[2] + 0.03);
    rk_simulation_destroy(simulation);
}

static void tool_piece_contact_is_reported(double obstacle_z, bool expected_active,
                                            bool fixed_obstacle = false) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.links[0].mass = 1.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 1.0;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.rotation[3] = 1.0;
    robot_desc.tool_link_index = 0;
    robot_desc.tool_piece_count = 1;
    robot_desc.tool_gap = 0.03;
    robot_desc.tool_piece_vertex_count[0] = 8;
    for (int vertex = 0; vertex < 8; ++vertex) {
        auto *point = robot_desc.tool_piece_vertices + vertex * 3;
        point[0] = (vertex & 1) ? 0.05 : -0.05;
        point[1] = (vertex & 2) ? 0.05 : -0.05;
        point[2] = (vertex & 4) ? 1.0 : 0.0;
    }
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    rk_simulation_object_desc obstacle{};
    obstacle.struct_size = sizeof(obstacle);
    obstacle.motion_type = fixed_obstacle ? 0 : 2;
    obstacle.rotation[3] = 1.0;
    obstacle.mass = 1.0;
    obstacle.half_extents[0] = obstacle.half_extents[1] =
        obstacle.half_extents[2] = 0.05;
    obstacle.position[2] = obstacle_z;
    rk_simulation_object object = 0;
    assert(rk_simulation_spawn_object(simulation, &obstacle, &object) == RK_OK);
    assert(rk_simulation_step(simulation, 0) == RK_OK);
    uint32_t count = 0;
    assert(rk_simulation_get_robot_contacts(simulation, robot, nullptr, 0, &count) == RK_OK);
    bool found = false;
    for (uint32_t i = 0; i < count; ++i) {
        rk_robot_contact contact{};
        contact.struct_size = sizeof(contact);
        assert(rk_simulation_get_robot_contact(simulation, robot, i, &contact) == RK_OK);
        if (contact.tool_piece_index == 0) {
            found = true;
            assert((contact.active != 0) == expected_active);
            assert(contact.link_index == 0);
            assert(contact.other_object == object);
        }
    }
    assert(found);
    rk_simulation_destroy(simulation);
}

// Drops a two-box robot onto a floor. A floating base falls and comes to rest
// on it; the same robot with a kinematic base stays where it was placed.
static void floating_base_falls_and_settles(bool floating) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.floating_base = floating ? 1 : 0;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < model.link_count; ++i) {
        model.links[i].mass = 1.0;
        model.links[i].inertia_tensor[0] = model.links[i].inertia_tensor[4] =
            model.links[i].inertia_tensor[8] = 0.02 / 3.0;
    }
    model.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0};
    model.joints[0].parent_frame_position[0] = 0.25;
    model.joints[0].parent_frame_rotation[3] = model.joints[0].child_frame_rotation[3] = 1.0;
    model.joints[0].axis[0] = 1.0;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.position[2] = 0.5;
    robot_desc.initial_pose.rotation[3] = 1.0;
    for (int axis = 0; axis < 6; ++axis) robot_desc.collision_half_extents[axis] = 0.1;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    rk_simulation_object_desc floor{};
    floor.struct_size = sizeof(floor);
    floor.rotation[3] = 1.0;
    floor.position[2] = -0.5;
    floor.half_extents[0] = floor.half_extents[1] = 5.0;
    floor.half_extents[2] = 0.5;
    rk_simulation_object floor_object = 0;
    assert(rk_simulation_spawn_object(simulation, &floor, &floor_object) == RK_OK);

    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    pose.position[2] = 0.5;
    pose.rotation[3] = 1.0;
    const rk_result drive = floating ? RK_ERROR_INVALID_STATE : RK_OK;
    assert(rk_simulation_drive_robot_base(simulation, 0, &pose) == drive);
    rk_simulation_differential_drive_desc wheels{};
    wheels.struct_size = sizeof(wheels);
    wheels.left_wheel_joint = 0;
    wheels.right_wheel_joint = 1;
    wheels.wheel_radius = wheels.track_width = 0.1;
    // Joint 1 does not exist, but a floating base is refused before wheels are checked.
    assert(rk_simulation_set_differential_drive(simulation, 0, &wheels) ==
           (floating ? RK_ERROR_INVALID_STATE : RK_ERROR_INVALID_ARGUMENT));

    for (int tick = 0; tick < 400; ++tick)
        assert(rk_simulation_step(simulation, tick) == RK_OK);
    rk_simulation_twist twist{};
    twist.struct_size = sizeof(twist);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    assert(rk_simulation_get_robot_base_velocity(simulation, 0, &twist) == RK_OK);
    double speed = 0.0;
    for (int axis = 0; axis < 3; ++axis)
        speed += twist.linear[axis] * twist.linear[axis] +
            twist.angular[axis] * twist.angular[axis];
    speed = std::sqrt(speed);
    if (floating) {
        // Both 0.2 m boxes rest flat on the floor, whose top is at z = 0.
        assert(std::abs(pose.position[2] - 0.1) < 0.01);
        assert(speed < 0.05);
    } else {
        assert(std::abs(pose.position[2] - 0.5) < 1e-9);
    }

    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_reset(simulation) == RK_OK);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    assert(rk_simulation_get_robot_base_velocity(simulation, 0, &twist) == RK_OK);
    assert(std::abs(pose.position[2] - 0.5) < 1e-9);
    for (int axis = 0; axis < 3; ++axis)
        assert(twist.linear[axis] == 0.0 && twist.angular[axis] == 0.0);
    rk_simulation_destroy(simulation);
}

int main() {
    floating_base_falls_and_settles(false);
    floating_base_falls_and_settles(true);
    convex_link_and_box_link_build();
    tool_hulls_collide_only_on_their_pieces();
    tool_piece_contact_is_reported(1.07, false);
    tool_piece_contact_is_reported(1.04, true);
    tool_piece_contact_is_reported(1.07, false, true);
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < model.link_count; ++i) {
        model.links[i].mass = 1.0;
        model.links[i].inertia_tensor[0] = model.links[i].inertia_tensor[4] = model.links[i].inertia_tensor[8] = 1.0;
    }
    model.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, 100.0};
    model.joints[0].parent_frame_rotation[3] = model.joints[0].child_frame_rotation[3] = 1.0;
    model.joints[0].axis[2] = 1.0;
    model.sensor_count = 2;
    model.sensors[0].kind = RK_SENSOR_IMU;
    model.sensors[0].link = 1; // Articulated link, not the static base.
    model.sensors[0].position[0] = 0.5; // Nonzero mount produces centripetal acceleration.
    model.sensors[0].rotation[3] = 1.0;
    model.sensors[1].kind = RK_SENSOR_LIDAR;
    model.sensors[1].rotation[3] = 1.0;
    model.sensors[1].ray_count = 16;
    model.sensors[1].max_range = 20.0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);

    // Falling box crosses and leaves the LiDAR plane. Contact response is
    // checked independently by the sim_mujoco backend's plane-drop fixture.
    rk_simulation_object_desc box{};
    box.struct_size = sizeof(box);
    box.position[0] = 2.0;
    box.position[2] = 1.0;
    box.rotation[3] = 1.0;
    box.half_extents[0] = box.half_extents[1] = box.half_extents[2] = 0.25;
    box.mass = 1.0;
    box.motion_type = 2;
    rk_simulation_object falling = 0;
    assert(rk_simulation_spawn_object(simulation, &box, &falling) == RK_OK);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.5, 0.0, 100.0};
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    bool saw_motion = false, saw_specific_force = false, saw_occlusion = false;
    for (int tick = 0; tick < 400; ++tick) {
        assert(rk_simulation_step(simulation, tick) == RK_OK);
        const auto sample = state(robot);
        if (tick == 0) {
            assert(sample.sensors[0].sequence == 0);
            assert(sample.sensors[1].values[0] == 20.0);
        }
        if (tick > 0) {
            const auto &imu = sample.sensors[0];
            assert(imu.value_count == 6);
            // MuJoCo's hinge velocity and the mounted gyroscope agree.
            assert(std::abs(imu.values[2] - sample.velocity[0]) < 1e-8);
            if (std::abs(imu.values[2]) > 0.1) saw_motion = true;
            if (imu.values[3] < -0.01) saw_specific_force = true;
            assert(std::abs(imu.values[5] - 9.81) < 1e-6);
        }
        if (sample.sensors[1].values[0] < 2.0) saw_occlusion = true;
    }
    const auto final = state(robot);
    assert(saw_motion && saw_specific_force && saw_occlusion);
    assert(std::abs(final.position[0] - 0.5) < 0.05);
    assert(final.sensors[1].values[0] == 20.0);
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_remove_object(simulation, falling) == RK_OK);
    assert(rk_simulation_step(simulation, 500) == RK_OK);
    assert(state(robot).sensors[1].values[0] == 20.0);
    assert(rk_simulation_stop(simulation) == RK_OK);
    assert(rk_simulation_reset(simulation) == RK_OK);
    assert(rk_simulation_step(simulation, 0) == RK_OK);
    assert(std::abs(state(robot).position[0]) < 1e-9);
    assert(state(robot).sensors[0].sequence == 0);
    rk_simulation_destroy(simulation);
    std::puts("MuJoCo articulated IMU and moving-occluder LiDAR tests passed");
}
