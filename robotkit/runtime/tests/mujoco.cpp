#include "robotkit_simkit.h"
#include "nativekit_sim_mujoco.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>

static rk_robot_state state(rk_robot_runtime robot) {
    rk_robot_state value{};
    value.struct_size = sizeof(value);
    assert(rk_robot_runtime_snapshot(robot, &value) == RK_OK);
    return value;
}

// A session an owning test drives directly, with a Simulation joined to it
// (rk_simulation_create_in_session) on the MuJoCo backend, standing in for
// the deleted self-owned rk_simulation_create(..., backend=1) mode.
struct SessionFixture {
    nkscene_scene scene = 0;
    nksim_world world = 0;
    nksim_session session = 0;
    rk_simulation simulation = RK_INVALID_SIMULATION;

    explicit SessionFixture(double fixed_timestep, uint32_t physics_substeps = 1) {
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc world_desc{};
        world_desc.struct_size = sizeof(world_desc);
        world_desc.scene = scene;
        world_desc.fixed_timestep = fixed_timestep;
        world_desc.physics_substeps = physics_substeps;
        world_desc.gravity[2] = -9.81;
        assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
        nksim_session_desc session_desc{};
        session_desc.struct_size = sizeof(session_desc);
        session_desc.scene = scene;
        session_desc.world = world;
        assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
        assert(rk_simulation_create_in_session(session, &simulation) == RK_OK);
    }
    SessionFixture(const SessionFixture &) = delete;
    SessionFixture &operator=(const SessionFixture &) = delete;
    ~SessionFixture() {
        if (simulation != RK_INVALID_SIMULATION) rk_simulation_destroy(simulation);
        if (session != 0) nksim_session_destroy(session);
        if (world != 0) nksim_world_destroy(world);
        if (scene != 0) nkscene_scene_destroy(scene);
    }
};

static rk_result from_sim(nksim_result result) {
    switch (result) {
    case NKSIM_OK: return RK_OK;
    case NKSIM_ERROR_INVALID_ARGUMENT: return RK_ERROR_INVALID_ARGUMENT;
    case NKSIM_ERROR_INVALID_HANDLE: return RK_ERROR_INVALID_HANDLE;
    case NKSIM_ERROR_INVALID_STATE: return RK_ERROR_INVALID_STATE;
    case NKSIM_ERROR_OUT_OF_MEMORY: return RK_ERROR_OUT_OF_MEMORY;
    default: return RK_ERROR_BACKEND;
    }
}

static rk_result step(nksim_session session, uint64_t timestamp_ns) {
    return from_sim(nksim_session_step(session, timestamp_ns, nullptr));
}
static rk_result start(nksim_session session) { return from_sim(nksim_session_start(session)); }
static rk_result stop(nksim_session session) { return from_sim(nksim_session_stop(session)); }
static rk_result reset(nksim_session session) { return from_sim(nksim_session_reset(session)); }

static nksim_object spawn_object(nksim_session session, uint32_t motion_type,
                                 const double position[3], const double rotation[4],
                                 const double half_extents[3], double mass = 0.0) {
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = motion_type;
    desc.shape.struct_size = sizeof(desc.shape);
    desc.shape.type = NKSIM_SHAPE_BOX;
    std::copy_n(half_extents, 3, desc.shape.parameters);
    desc.pose.struct_size = sizeof(desc.pose);
    std::copy_n(position, 3, desc.pose.position);
    std::copy_n(rotation, 4, desc.pose.rotation);
    desc.mass = mass;
    nksim_object object = 0;
    assert(nksim_session_create_object(session, &desc, &object) == NKSIM_OK);
    return object;
}

static void remove_object(nksim_session session, nksim_object object) {
    assert(nksim_session_destroy_object(session, object) == NKSIM_OK);
}

static void object_pose(nksim_session session, nksim_object object, double out_position[3],
                        double out_rotation[4]) {
    nksim_body body = 0;
    assert(nksim_session_get_object_body(session, object, &body) == NKSIM_OK);
    nksim_body_state value{};
    value.struct_size = sizeof(value);
    assert(nksim_session_get_body_state(session, body, &value) == NKSIM_OK);
    std::copy_n(value.position, 3, out_position);
    std::copy_n(value.rotation, 4, out_rotation);
}

static void convex_link_and_box_link_build() {
    SessionFixture fixture(0.005);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    assert(start(session) == RK_OK);
    rk_simulation_presentation presentation = 0;
    assert(rk_simulation_capture_presentation(simulation, &presentation) == RK_OK);
    rk_simulation_presentation_destroy(presentation);
    assert(stop(session) == RK_OK);
    assert(step(session, 0) == RK_OK);
}

static void tool_hulls_collide_only_on_their_pieces() {
    SessionFixture fixture(0.01, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    const double half_extents[3] = {0.05, 0.05, 0.05};
    const double rotation[4] = {0.0, 0.0, 0.0, 1.0};
    const double in_gap_position[3] = {0.5, 0.0, 1.2};
    const double on_tool_position[3] = {0.95, 0.0, 1.2};
    auto in_gap = spawn_object(session, NKSIM_MOTION_DYNAMIC, in_gap_position, rotation,
        half_extents, 1.0);
    auto on_tool = spawn_object(session, NKSIM_MOTION_DYNAMIC, on_tool_position, rotation,
        half_extents, 1.0);
    for (int tick = 0; tick < 20; ++tick)
        assert(step(session, tick) == RK_OK);
    double gap_position[3], gap_rotation[4], tool_position[3], tool_rotation[4];
    object_pose(session, in_gap, gap_position, gap_rotation);
    object_pose(session, on_tool, tool_position, tool_rotation);
    assert(gap_position[2] < 1.03);
    assert(tool_position[2] > gap_position[2] + 0.03);
}

static void tool_piece_contact_is_reported(double obstacle_z, bool expected_active,
                                            bool fixed_obstacle = false) {
    SessionFixture fixture(0.01, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    const double half_extents[3] = {0.05, 0.05, 0.05};
    const double rotation[4] = {0.0, 0.0, 0.0, 1.0};
    const double position[3] = {0.0, 0.0, obstacle_z};
    auto object = spawn_object(session, fixed_obstacle ? NKSIM_MOTION_STATIC : NKSIM_MOTION_DYNAMIC,
        position, rotation, half_extents, 1.0);
    assert(step(session, 0) == RK_OK);
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
}

int main() {
    convex_link_and_box_link_build();
    tool_hulls_collide_only_on_their_pieces();
    tool_piece_contact_is_reported(1.07, false);
    tool_piece_contact_is_reported(1.04, true);
    tool_piece_contact_is_reported(1.07, false, true);

    SessionFixture fixture(0.005, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    const double box_position[3] = {2.0, 0.0, 1.0};
    const double box_rotation[4] = {0.0, 0.0, 0.0, 1.0};
    const double box_half_extents[3] = {0.25, 0.25, 0.25};
    auto falling = spawn_object(session, NKSIM_MOTION_DYNAMIC, box_position, box_rotation,
        box_half_extents, 1.0);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 1;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.5, 0.0, 100.0};
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    bool saw_motion = false, saw_specific_force = false, saw_occlusion = false;
    for (int tick = 0; tick < 400; ++tick) {
        assert(step(session, tick) == RK_OK);
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
    assert(stop(session) == RK_OK);
    remove_object(session, falling);
    assert(step(session, 500) == RK_OK);
    assert(state(robot).sensors[1].values[0] == 20.0);
    assert(stop(session) == RK_OK);
    assert(reset(session) == RK_OK);
    assert(step(session, 0) == RK_OK);
    assert(std::abs(state(robot).position[0]) < 1e-9);
    assert(state(robot).sensors[0].sequence == 0);
    std::puts("MuJoCo articulated IMU and moving-occluder LiDAR tests passed");
}
