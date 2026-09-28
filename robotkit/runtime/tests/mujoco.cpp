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

// A floating one-link robot stands on a sphere and a sideways capsule placed
// under opposite ends; it rests level on them at their radius, not on the
// link's default bounds box.
static void link_primitives_collide_in_link_frame() {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 2;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.floating_base = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    model.links[0].mass = 2.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 0.05;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.position[2] = 0.3;
    robot_desc.initial_pose.rotation[3] = 1.0;
    robot_desc.link_shape_count = 2;
    auto &sphere = robot_desc.link_shapes[0];
    sphere.type = RK_LINK_SHAPE_SPHERE;
    sphere.size[0] = 0.05;
    sphere.position[0] = -0.2;
    sphere.position[2] = -0.1;
    sphere.rotation[3] = 1.0;
    auto &capsule = robot_desc.link_shapes[1];
    capsule.type = RK_LINK_SHAPE_CAPSULE;
    capsule.size[0] = 0.05;
    capsule.size[1] = 0.08;
    capsule.position[0] = 0.2;
    capsule.position[2] = -0.1;
    capsule.rotation[0] = std::sqrt(0.5); // Local Z turned onto world Y.
    capsule.rotation[3] = std::sqrt(0.5);

    rk_robot_runtime robot = 0;
    auto invalid = robot_desc;
    invalid.link_shapes[1].link = 1;
    assert(rk_simulation_add_robot(simulation, &model, &invalid, &robot) != RK_OK);
    invalid = robot_desc;
    invalid.link_shapes[0].size[0] = 0.0;
    assert(rk_simulation_add_robot(simulation, &model, &invalid, &robot) != RK_OK);
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    rk_simulation_object_desc floor{};
    floor.struct_size = sizeof(floor);
    floor.rotation[3] = 1.0;
    floor.position[2] = -0.5;
    floor.half_extents[0] = floor.half_extents[1] = 5.0;
    floor.half_extents[2] = 0.5;
    rk_simulation_object floor_object = 0;
    assert(rk_simulation_spawn_object(simulation, &floor, &floor_object) == RK_OK);
    for (int tick = 0; tick < 400; ++tick)
        assert(rk_simulation_step(simulation, tick) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    // Primitive centres sit 0.1 m below the link origin and one radius above the floor.
    assert(std::abs(pose.position[2] - 0.15) < 0.005);
    assert(std::abs(pose.rotation[3]) > 0.9999);
    rk_simulation_destroy(simulation);
}

// A 1 kg arm on a kinematic base, hinged about Y with its centre of mass
// 0.5 m out along X: at q = 0 gravity loads the hinge with 4.905 N m and
// pulls q positive.
static rk_robot_runtime_blueprint gravity_arm(double max_effort, double friction_loss = 0.0,
                                              double damping = 0.0) {
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_NONE;
    for (auto &link : model.links) {
        link.mass = 1.0;
        link.inertia_tensor[0] = link.inertia_tensor[4] = link.inertia_tensor[8] = 0.01;
    }
    model.links[1].center_of_mass[0] = 0.5;
    model.joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -3.14, 3.14, max_effort};
    model.joints[0].parent_frame_rotation[3] = model.joints[0].child_frame_rotation[3] = 1.0;
    model.joints[0].axis[1] = 1.0;
    model.joint_dynamics[0].friction_loss = friction_loss;
    model.joint_dynamics[0].damping = damping;
    return model;
}

struct ArmRun {
    double position;
    double effort;
};

// Runs the arm for `seconds` under one command and reports the final joint state.
static ArmRun run_gravity_arm(const rk_robot_runtime_blueprint &model,
                              const rk_robot_command &command, double seconds) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 5;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    if (command.target_count > 0) assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const int ticks = static_cast<int>(seconds / desc.fixed_timestep);
    for (int tick = 0; tick < ticks; ++tick)
        assert(rk_simulation_step(simulation, static_cast<uint64_t>(tick) * 10'000'000u) == RK_OK);
    const auto value = state(robot);
    rk_simulation_destroy(simulation);
    return {value.position[0], value.effort[0]};
}

static rk_robot_command arm_command(uint32_t mode, double target) {
    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, mode, target, 0.0, 0.0};
    return command;
}

// TODO.md physics regression: a position command stalls when the joint's
// torque limit is below its gravity load, and moves once the limit is above it.
static void actuator_limit_stalls_then_lifts() {
    const double load = 9.81 * 0.5;
    const auto lift = arm_command(RK_TARGET_POSITION, -0.3);
    // Joint damping settles the arm where the limited effort balances gravity:
    // load * cos(q) = 3 N m.
    const auto weak = run_gravity_arm(gravity_arm(3.0, 0.0, 2.0), lift, 3.0);
    assert(std::abs(weak.position - std::acos(3.0 / load)) < 0.02); // Sagged, not lifted.
    assert(std::abs(std::abs(weak.effort) - 3.0) < 1e-6); // Pushing at its limit.
    const auto strong = run_gravity_arm(gravity_arm(20.0, 0.0, 2.0), lift, 3.0);
    assert(std::abs(strong.position + 0.3) < 0.01);
    assert(std::abs(strong.effort) < 20.0);
}

// A servo target is a plain PD with feedforward: it sags by load / stiffness,
// feedforward cancels the load, and validation bounds its terms.
static void servo_target_runs_through_the_runtime() {
    const double load = 9.81 * 0.5;
    auto command = arm_command(RK_TARGET_SERVO, 0.0);
    command.servos[0] = {0.0, 200.0, 10.0, 0.0};
    const auto sag = run_gravity_arm(gravity_arm(50.0), command, 2.0);
    assert(std::abs(sag.position - load / 200.0) < 0.002);
    command.servos[0].feedforward = -load; // Gravity pulls q positive; push back.
    const auto held = run_gravity_arm(gravity_arm(50.0), command, 2.0);
    assert(std::abs(held.position) < 1e-3);

    auto invalid = command;
    invalid.servos[0].stiffness = -1.0;
    assert(rk_robot_command_validate(&invalid) == RK_ERROR_INVALID_ARGUMENT);
    invalid = command;
    invalid.targets[0].joint = RK_MAX_SERVO_JOINTS;
    assert(rk_robot_command_validate(&invalid) == RK_ERROR_INVALID_ARGUMENT);

    // Feedforward beyond the joint's effort limit is refused like an effort target.
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    desc.backend = 1;
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    const auto model = gravity_arm(3.0);
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    assert(rk_simulation_step(simulation, 0) == RK_ERROR_LIMIT);
    rk_simulation_destroy(simulation);
}

// Joint friction loss compiled into the blueprint holds an unpowered arm,
// apart from the slow creep of MuJoCo's soft dry friction.
static void blueprint_joint_friction_holds_an_arm() {
    rk_robot_command none{};
    none.struct_size = sizeof(none);
    assert(run_gravity_arm(gravity_arm(0.0, 10.0), none, 0.4).position < 0.02);
    assert(run_gravity_arm(gravity_arm(0.0, 0.0), none, 0.4).position > 0.5);
}

// Rest height of a 10 kg floating sphere robot on a floor, for a contact time constant.
static double sphere_rest_height(double time_constant) {
    rk_simulation_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.fixed_timestep = 0.005;
    desc.physics_substeps = 2;
    desc.backend = 1;
    desc.integrator = 2; // NKSIM_INTEGRATOR_IMPLICIT_FAST
    rk_simulation simulation = 0;
    assert(rk_simulation_create(&desc, &simulation) == RK_OK);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.floating_base = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_NONE;
    model.links[0].mass = 10.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 0.01;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.position[2] = 0.06;
    robot_desc.initial_pose.rotation[3] = 1.0;
    robot_desc.link_shape_count = 1;
    auto &sphere = robot_desc.link_shapes[0];
    sphere.type = RK_LINK_SHAPE_SPHERE;
    sphere.size[0] = 0.05;
    sphere.rotation[3] = 1.0;
    sphere.contact_time_constant = time_constant;
    sphere.contact_damping_ratio = 1.0;
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
    for (int tick = 0; tick < 400; ++tick)
        assert(rk_simulation_step(simulation, tick) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    rk_simulation_destroy(simulation);
    return pose.position[2];
}

// A softer contact (longer time constant) lets a robot's shape sink further.
static void link_shape_contact_softness_reaches_the_backend() {
    const double stiff = sphere_rest_height(0.02);
    const double soft = sphere_rest_height(0.2);
    assert(std::abs(stiff - 0.05) < 0.005);
    assert(soft < stiff - 0.005);
}

int main() {
    actuator_limit_stalls_then_lifts();
    servo_target_runs_through_the_runtime();
    blueprint_joint_friction_holds_an_arm();
    link_shape_contact_softness_reaches_the_backend();
    link_primitives_collide_in_link_frame();
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
