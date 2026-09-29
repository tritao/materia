#include "robotkit_simkit.h"
#include "nativekit_sim_mujoco.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <thread>

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

    explicit SessionFixture(double fixed_timestep, uint32_t physics_substeps = 1,
                            uint32_t integrator = NKSIM_INTEGRATOR_DEFAULT) {
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc world_desc{};
        world_desc.struct_size = sizeof(world_desc);
        world_desc.scene = scene;
        world_desc.fixed_timestep = fixed_timestep;
        world_desc.physics_substeps = physics_substeps;
        world_desc.gravity[2] = -9.81;
        world_desc.integrator = integrator;
        assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
        nksim_session_desc session_desc{};
        session_desc.struct_size = sizeof(session_desc);
        session_desc.scene = scene;
        session_desc.world = world;
        assert(nksim_session_create(&session_desc, &session) == NKSIM_OK);
        assert(rk_simulation_create_in_session(session, &simulation) == RK_OK);
    }
    // Steps the session; a tick the robots refused reports RobotKit's reason.
    rk_result step(uint64_t timestamp_ns) const;
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
rk_result SessionFixture::step(uint64_t timestamp_ns) const {
    const auto result = ::step(session, timestamp_ns);
    rk_result rejected = RK_OK;
    assert(rk_simulation_get_rejection(simulation, &rejected) == RK_OK);
    return result != RK_OK && rejected != RK_OK ? rejected : result;
}
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

static nksim_result spawn_plane(nksim_session session, uint32_t motion_type = NKSIM_MOTION_STATIC,
                                double mass = 0.0) {
    nksim_object_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.motion_type = motion_type;
    desc.shape.struct_size = sizeof(desc.shape);
    desc.shape.type = NKSIM_SHAPE_PLANE;
    desc.shape.parameters[2] = 1.0; // The ground: normal +Z through the origin.
    desc.pose.struct_size = sizeof(desc.pose);
    desc.pose.rotation[3] = 1.0;
    desc.mass = mass;
    nksim_object object = 0;
    return nksim_session_create_object(session, &desc, &object);
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
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
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
        assert(fixture.step(tick) == RK_OK);
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
    rk_robot_contact_list list = RK_INVALID_ROBOT_CONTACT_LIST;
    assert(rk_simulation_capture_robot_contacts(simulation, robot, &list) == RK_OK);
    uint32_t captured_count = 0;
    uint64_t captured_step = 0;
    assert(rk_robot_contact_list_count(list, &captured_count) == RK_OK);
    assert(rk_robot_contact_list_step_index(list, &captured_step) == RK_OK);
    assert(captured_step == 1);
    uint32_t count = 0;
    assert(rk_simulation_get_robot_contacts(simulation, robot, nullptr, 0, &count) == RK_OK);
    assert(captured_count == count);
    alignas(rk_robot_contact) unsigned char legacy_storage[80]{};
    auto *legacy = reinterpret_cast<rk_robot_contact *>(legacy_storage);
    legacy->struct_size = sizeof(legacy_storage);
    uint32_t legacy_count = 0;
    assert(rk_simulation_get_robot_contacts(simulation, robot, legacy, 1, &legacy_count) == RK_OK);
    assert(legacy_count == count && legacy->struct_size == sizeof(legacy_storage));
    bool found = false;
    for (uint32_t i = 0; i < count; ++i) {
        rk_robot_contact contact{};
        contact.struct_size = sizeof(contact);
        assert(rk_simulation_get_robot_contact(simulation, robot, i, &contact) == RK_OK);
        rk_robot_contact captured{};
        captured.struct_size = sizeof(captured);
        assert(rk_robot_contact_list_get(list, i, &captured) == RK_OK);
        assert(captured.link_index == contact.link_index);
        assert(captured.tool_piece_index == contact.tool_piece_index);
        if (contact.tool_piece_index == 0) {
            found = true;
            assert((contact.active != 0) == expected_active);
            assert(contact.link_index == 0);
            assert(contact.other_object == object);
            assert(contact.other_kind == RK_CONTACT_OTHER_OBJECT);
        }
    }
    assert(found);
    rk_robot_contact_list_destroy(list);
}

static void captured_contacts_survive_realtime_steps() {
    SessionFixture fixture(0.001);
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    model.links[0].mass = 1.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 1.0;
    rk_simulation_robot_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.initial_pose.struct_size = sizeof(desc.initial_pose);
    desc.initial_pose.rotation[3] = 1.0;
    desc.collision_half_extents[0] = desc.collision_half_extents[1] =
        desc.collision_half_extents[2] = 0.1;
    desc.tool_link_index = 0;
    desc.tool_piece_count = 1;
    desc.tool_gap = 0.03;
    desc.tool_piece_vertex_count[0] = 8;
    for (int vertex = 0; vertex < 8; ++vertex) {
        auto *point = desc.tool_piece_vertices + vertex * 3;
        point[0] = (vertex & 1) ? 0.1 : -0.1;
        point[1] = (vertex & 2) ? 0.1 : -0.1;
        point[2] = (vertex & 4) ? 0.1 : -0.1;
    }
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(fixture.simulation, &model, &desc, &robot) == RK_OK);
    const double position[3] = {0.0, 0.0, 0.15};
    const double rotation[4] = {0.0, 0.0, 0.0, 1.0};
    const double half[3] = {0.1, 0.1, 0.1};
    const auto object = spawn_object(fixture.session, NKSIM_MOTION_STATIC,
        position, rotation, half);
    assert(start(fixture.session) == RK_OK);
    uint64_t previous_step = 0;
    bool saw_contact = false;
    for (int attempt = 0; attempt < 200; ++attempt) {
        rk_robot_contact_list list = RK_INVALID_ROBOT_CONTACT_LIST;
        assert(rk_simulation_capture_robot_contacts(fixture.simulation, robot, &list) == RK_OK);
        uint64_t step_index = 0;
        uint32_t count = 0;
        assert(rk_robot_contact_list_step_index(list, &step_index) == RK_OK);
        assert(rk_robot_contact_list_count(list, &count) == RK_OK);
        assert(step_index >= previous_step);
        if (count != 0) saw_contact = true;
        previous_step = step_index;
        for (uint32_t index = 0; index < count; ++index) {
            rk_robot_contact contact{};
            contact.struct_size = sizeof(contact);
            assert(rk_robot_contact_list_get(list, index, &contact) == RK_OK);
            assert(contact.link_index == 0);
            assert(contact.other_kind == RK_CONTACT_OTHER_OBJECT);
            assert(contact.other_object == object);
        }
        // A capture remains readable after the worker has had time to advance.
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        uint32_t stable_count = 0;
        uint64_t stable_step = 0;
        assert(rk_robot_contact_list_count(list, &stable_count) == RK_OK);
        assert(rk_robot_contact_list_step_index(list, &stable_step) == RK_OK);
        assert(stable_count == count && stable_step == step_index);
        rk_robot_contact_list_destroy(list);
    }
    assert(stop(fixture.session) == RK_OK);
    assert(previous_step > 0);
    assert(saw_contact);
}

static void unowned_ground_contact_reports_world() {
    SessionFixture fixture(0.01);
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(fixture.scene, &transaction) == NKS_OK);
    nkscene_node_id node{};
    assert(nkscene_tx_create_node(transaction, &node) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = transform.matrix[5] = transform.matrix[10] =
        transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    if (changes != 0) nkscene_change_set_destroy(changes);
    const double normal[3] = {0.0, 0.0, 1.0};
    nksim_shape plane = 0;
    assert(nksim_shape_create_plane(fixture.world, normal, 0.0, &plane) == NKSIM_OK);
    nksim_body_desc ground_desc{};
    ground_desc.struct_size = sizeof(ground_desc);
    ground_desc.node = node;
    ground_desc.motion_type = NKSIM_MOTION_STATIC;
    ground_desc.shape = plane;
    ground_desc.collision_layer = ground_desc.collision_mask = 1;
    nksim_body ground = 0;
    assert(nksim_body_create(fixture.world, &ground_desc, &ground) == NKSIM_OK);

    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.floating_base = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    model.links[0].mass = 1.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 1.0;
    rk_simulation_robot_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.initial_pose.struct_size = sizeof(desc.initial_pose);
    desc.initial_pose.position[2] = 0.09;
    desc.initial_pose.rotation[3] = 1.0;
    desc.collision_half_extents[0] = desc.collision_half_extents[1] =
        desc.collision_half_extents[2] = 0.1;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(fixture.simulation, &model, &desc, &robot) == RK_OK);
    for (int tick = 0; tick < 5; ++tick) assert(fixture.step(tick) == RK_OK);
    rk_robot_contact_list list = RK_INVALID_ROBOT_CONTACT_LIST;
    assert(rk_simulation_capture_robot_contacts(fixture.simulation, robot, &list) == RK_OK);
    uint32_t count = 0;
    assert(rk_robot_contact_list_count(list, &count) == RK_OK);
    bool found = false;
    for (uint32_t index = 0; index < count; ++index) {
        rk_robot_contact contact{};
        contact.struct_size = sizeof(contact);
        assert(rk_robot_contact_list_get(list, index, &contact) == RK_OK);
        if (contact.other_kind == RK_CONTACT_OTHER_WORLD && contact.other_object == 0)
            found = true;
    }
    assert(found);
    rk_robot_contact_list_destroy(list);
}

// Drops a two-box robot onto a floor. A floating base falls and comes to rest
// on it; the same robot with a kinematic base stays where it was placed.
static void floating_base_falls_and_settles(bool floating) {
    SessionFixture fixture(0.005, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    const double floor_position[3] = {0.0, 0.0, -0.5}, floor_rotation[4] = {0.0, 0.0, 0.0, 1.0},
                 floor_half_extents[3] = {5.0, 5.0, 0.5};
    spawn_object(session, NKSIM_MOTION_STATIC, floor_position, floor_rotation, floor_half_extents);

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
        assert(fixture.step(tick) == RK_OK);
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

    assert(stop(session) == RK_OK);
    assert(reset(session) == RK_OK);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    assert(rk_simulation_get_robot_base_velocity(simulation, 0, &twist) == RK_OK);
    assert(std::abs(pose.position[2] - 0.5) < 1e-9);
    for (int axis = 0; axis < 3; ++axis)
        assert(twist.linear[axis] == 0.0 && twist.angular[axis] == 0.0);
}

// A push is a force on the base for a tick. Falling free (no floor), a floating
// two-link robot of 2 kg given 20 N along x for 5 ticks of 5 ms gains
// 20 * 0.025 / 2 = 0.25 m/s, and a kinematic one ignores the force.
static void robot_base_can_be_pushed(bool floating) {
    SessionFixture fixture(0.005, 2);
    const auto simulation = fixture.simulation;
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 2;
    model.joint_count = 1;
    model.floating_base = floating ? 1 : 0;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_NONE;
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
    robot_desc.initial_pose.position[2] = 10.0;
    robot_desc.initial_pose.rotation[3] = 1.0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);

    rk_simulation_wrench push{};
    push.struct_size = sizeof(push);
    push.force[0] = 20.0;
    assert(rk_simulation_apply_robot_force(simulation, 7, &push) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_apply_robot_force(simulation, 0, nullptr) == RK_ERROR_INVALID_ARGUMENT);
    rk_simulation_wrench bad = push;
    bad.force[1] = std::nan("");
    assert(rk_simulation_apply_robot_force(simulation, 0, &bad) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_apply_robot_force(999, 0, &push) == RK_ERROR_INVALID_HANDLE);
    for (int tick = 0; tick < 5; ++tick) {
        assert(rk_simulation_apply_robot_force(simulation, 0, &push) == RK_OK);
        assert(fixture.step(tick) == RK_OK);
    }
    assert(fixture.step(5) == RK_OK); // an unpushed tick adds nothing
    rk_simulation_twist twist{};
    twist.struct_size = sizeof(twist);
    assert(rk_simulation_get_robot_base_velocity(simulation, 0, &twist) == RK_OK);
    if (floating) {
        assert(std::abs(twist.linear[0] - 0.25) < 0.01);
        assert(std::abs(twist.linear[1]) < 1e-6);
    } else {
        assert(std::abs(twist.linear[0]) < 1e-9);
    }
}

// A floating one-link robot stands on a sphere and a sideways capsule placed
// under opposite ends; it rests level on them at their radius, not on the
// link's default bounds box.
static void link_primitives_collide_in_link_frame() {
    SessionFixture fixture(0.005, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    const double floor_position[3] = {0.0, 0.0, -0.5}, floor_rotation[4] = {0.0, 0.0, 0.0, 1.0},
                 floor_half_extents[3] = {5.0, 5.0, 0.5};
    spawn_object(session, NKSIM_MOTION_STATIC, floor_position, floor_rotation, floor_half_extents);
    for (int tick = 0; tick < 400; ++tick)
        assert(fixture.step(tick) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    // Primitive centres sit 0.1 m below the link origin and one radius above the floor.
    assert(std::abs(pose.position[2] - 0.15) < 0.005);
    assert(std::abs(pose.rotation[3]) > 0.9999);
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
    SessionFixture fixture(0.01, 5);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    if (command.target_count > 0) assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    const int ticks = static_cast<int>(seconds / 0.01);
    for (int tick = 0; tick < ticks; ++tick)
        assert(fixture.step(static_cast<uint64_t>(tick) * 10'000'000u) == RK_OK);
    const auto value = state(robot);
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
    SessionFixture fixture(0.01, 1);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    const auto model = gravity_arm(3.0);
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    assert(rk_robot_runtime_submit(robot, &command) == RK_OK);
    // The refused command faults its robot, not the shared tick.
    assert(fixture.step(0) == RK_OK);
    assert(state(robot).safety == RK_SAFETY_FAULT);
}

// Joint friction loss compiled into the blueprint holds an unpowered arm,
// apart from the slow creep of MuJoCo's soft dry friction.
static void blueprint_joint_friction_holds_an_arm() {
    rk_robot_command none{};
    none.struct_size = sizeof(none);
    assert(run_gravity_arm(gravity_arm(0.0, 10.0), none, 0.4).position < 0.02);
    assert(run_gravity_arm(gravity_arm(0.0, 0.0), none, 0.4).position > 0.5);
}

// Rest height of a 10 kg floating sphere robot on a floor, for a contact time
// constant and contact filter.
static double sphere_rest_height(double time_constant, uint32_t contact_filter = 0,
                                 bool ground_plane = false) {
    SessionFixture fixture(0.005, 2, NKSIM_INTEGRATOR_IMPLICIT_FAST);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
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
    sphere.contact_filter = contact_filter;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    if (ground_plane) {
        assert(spawn_plane(session, NKSIM_MOTION_DYNAMIC, 1.0) == NKSIM_ERROR_INVALID_ARGUMENT);
        assert(spawn_plane(session) == NKSIM_OK);
    } else {
        const double floor_position[3] = {0.0, 0.0, -0.5}, floor_rotation[4] = {0.0, 0.0, 0.0, 1.0},
                     floor_half_extents[3] = {5.0, 5.0, 0.5};
        spawn_object(session, NKSIM_MOTION_STATIC, floor_position, floor_rotation, floor_half_extents);
    }
    for (int tick = 0; tick < 400; ++tick)
        assert(fixture.step(tick) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    return pose.position[2];
}

// A softer contact (longer time constant) lets a robot's shape sink further.
static void link_shape_contact_softness_reaches_the_backend() {
    const double stiff = sphere_rest_height(0.02);
    const double soft = sphere_rest_height(0.2);
    assert(std::abs(stiff - 0.05) < 0.005);
    assert(soft < stiff - 0.005);
}

// An unpowered arm falls onto its 0.3 rad stop and, since MuJoCo's stops are
// compliant, rests slightly past it. With exact limits the runtime faults that
// robot (the tick itself succeeds); with an observed-limit tolerance it holds
// on the stop.
static rk_result arm_on_its_stop(double tolerance, double &position) {
    auto model = gravity_arm(0.0);
    model.joints[0].upper_limit = 0.3;
    model.observed_limit_tolerance = tolerance;
    SessionFixture fixture(0.01, 5);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    rk_result result = RK_OK;
    for (int tick = 0; tick < 200; ++tick)
        assert(fixture.step(static_cast<uint64_t>(tick) * 10'000'000u) == RK_OK);
    position = state(robot).position[0];
    if (state(robot).safety == RK_SAFETY_FAULT) result = RK_ERROR_LIMIT;
    return result;
}

static void observed_limit_tolerance_allows_compliant_stops() {
    double position = 0.0;
    assert(arm_on_its_stop(0.0, position) == RK_ERROR_LIMIT);
    assert(arm_on_its_stop(0.05, position) == RK_OK);
    assert(position > 0.3 && position < 0.35);
    auto invalid = gravity_arm(0.0);
    invalid.observed_limit_tolerance = -0.1;
    assert(rk_robot_runtime_blueprint_validate(&invalid) == RK_ERROR_INVALID_ARGUMENT);
}

// A robot can start in a joint pose: set positions move the links it carries
// before the first step, out-of-limit poses are refused, and reset returns
// the joints to zero.
static void robots_start_in_a_joint_pose() {
    SessionFixture fixture(0.01, 1);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    auto model = gravity_arm(0.0);
    model.joints[0].lower_limit = -1.0;
    model.joints[0].upper_limit = 1.0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, nullptr, &robot) == RK_OK);
    const double outside[] = {1.5};
    assert(rk_simulation_set_joint_positions(simulation, 0, outside, 1) == RK_ERROR_INVALID_ARGUMENT);
    const double pose[] = {-0.5};
    assert(rk_simulation_set_joint_positions(simulation, 0, pose, 2) == RK_ERROR_INVALID_ARGUMENT);
    assert(rk_simulation_set_joint_positions(simulation, 0, pose, 1) == RK_OK);
    // The arm link hangs from the hinge at its origin: -0.5 rad about Y
    // raises its +X axis, which the link frame's rotation shows.
    rk_simulation_pose link{};
    link.struct_size = sizeof(link);
    assert(rk_simulation_get_link_pose(simulation, 0, 1, &link) == RK_OK);
    assert(std::abs(link.rotation[1] - std::sin(-0.25)) < 1e-9);
    assert(fixture.step(0) == RK_OK);
    assert(std::abs(state(robot).position[0] + 0.5) < 0.01);
    assert(rk_simulation_set_joint_positions(simulation, 0, pose, 1) == RK_ERROR_INVALID_STATE);
    assert(stop(session) == RK_OK);
    assert(reset(session) == RK_OK);
    assert(fixture.step(0) == RK_OK);
    assert(std::abs(state(robot).position[0]) < 0.01);
}

// A shape that collides only through pairs passes through the floor; one that
// also meets the environment rests on it. Pairs must join two links' shapes.
static void link_shape_contact_filters_reach_the_backend() {
    assert(sphere_rest_height(0.02, 1) < -1.0);
    assert(std::abs(sphere_rest_height(0.02, 2) - 0.05) < 0.005);
    assert(std::abs(sphere_rest_height(0.02, 2, true) - 0.05) < 0.005); // On a ground plane.

    SessionFixture fixture(0.01, 1);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    const auto model = gravity_arm(0.0);
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.link_shape_count = 2;
    for (uint32_t index = 0; index < 2; ++index) {
        auto &shape = robot_desc.link_shapes[index];
        shape.link = index;
        shape.type = RK_LINK_SHAPE_SPHERE;
        shape.size[0] = 0.05;
        shape.rotation[3] = 1.0;
        shape.contact_filter = 1;
    }
    robot_desc.contact_pair_count = 1;
    robot_desc.contact_pairs[0].shape_a = 0;
    robot_desc.contact_pairs[0].shape_b = 0; // Both on link 0: refused.
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) != RK_OK);
    robot_desc.contact_pairs[0].shape_b = 1;
    robot_desc.contact_pairs[0].friction[0] = 0.5;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    assert(fixture.step(0) == RK_OK);
}

// A link with no shape under the "none" collision approximation touches
// nothing: a floating robot made of one passes through the floor.
static void shapeless_link_without_approximation_collides_with_nothing() {
    SessionFixture fixture(0.01, 2);
    auto simulation = fixture.simulation;
    auto session = fixture.session;
    rk_robot_runtime_blueprint model{};
    model.struct_size = sizeof(model);
    model.link_count = 1;
    model.floating_base = 1;
    model.collision_approximation = RK_COLLISION_APPROXIMATION_NONE;
    model.links[0].mass = 1.0;
    model.links[0].inertia_tensor[0] = model.links[0].inertia_tensor[4] =
        model.links[0].inertia_tensor[8] = 0.01;
    rk_simulation_robot_desc robot_desc{};
    robot_desc.struct_size = sizeof(robot_desc);
    robot_desc.initial_pose.struct_size = sizeof(robot_desc.initial_pose);
    robot_desc.initial_pose.position[2] = 0.1;
    robot_desc.initial_pose.rotation[3] = 1.0;
    rk_robot_runtime robot = 0;
    assert(rk_simulation_add_robot(simulation, &model, &robot_desc, &robot) == RK_OK);
    assert(spawn_plane(session) == NKSIM_OK);
    for (int tick = 0; tick < 100; ++tick)
        assert(fixture.step(tick) == RK_OK);
    rk_simulation_pose pose{};
    pose.struct_size = sizeof(pose);
    assert(rk_simulation_get_robot_pose(simulation, 0, &pose) == RK_OK);
    assert(pose.position[2] < -1.0);
}

int main() {
    shapeless_link_without_approximation_collides_with_nothing();
    link_shape_contact_filters_reach_the_backend();
    robots_start_in_a_joint_pose();
    observed_limit_tolerance_allows_compliant_stops();
    actuator_limit_stalls_then_lifts();
    servo_target_runs_through_the_runtime();
    blueprint_joint_friction_holds_an_arm();
    link_shape_contact_softness_reaches_the_backend();
    link_primitives_collide_in_link_frame();
    floating_base_falls_and_settles(false);
    floating_base_falls_and_settles(true);
    robot_base_can_be_pushed(false);
    robot_base_can_be_pushed(true);
    convex_link_and_box_link_build();
    tool_hulls_collide_only_on_their_pieces();
    tool_piece_contact_is_reported(1.07, false);
    tool_piece_contact_is_reported(1.04, true);
    tool_piece_contact_is_reported(1.07, false, true);
    captured_contacts_survive_realtime_steps();
    unowned_ground_contact_reports_world();

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
        assert(fixture.step(tick) == RK_OK);
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
