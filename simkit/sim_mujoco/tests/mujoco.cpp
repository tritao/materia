#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_mujoco.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <cmath>
#include <cstdio>

namespace {

nkscene_node_id make_node_xyz(nkscene_scene scene, double x, double y, double z) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_node_id node{};
    assert(nkscene_tx_create_node(transaction, &node) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[13] = static_cast<float>(y);
    transform.matrix[14] = static_cast<float>(z);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return node;
}

nkscene_node_id make_node(nkscene_scene scene, double x) {
    return make_node_xyz(scene, x, 0.0, 0.0);
}

void set_node_x(nkscene_scene scene, nkscene_node_id node, double x) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
}

nksim_shape make_box(nksim_world world) {
    const double half_extents[] = {0.1, 0.1, 0.1};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    return shape;
}

nksim_body make_body(nksim_world world, nkscene_node_id node,
                    uint32_t motion_type, double mass, nksim_shape shape = 0) {
    nksim_body_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.node = node;
    desc.motion_type = motion_type;
    desc.mass = mass;
    desc.shape = shape;
    desc.collision_layer = 1;
    desc.collision_mask = 1;
    nksim_body body = 0;
    assert(nksim_body_create(world, &desc, &body) == NKSIM_OK);
    return body;
}

void step_world(nksim_world world, int count) {
    for (int index = 0; index < count; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        nkscene_change_set_destroy(step.scene_changes);
    }
}

void revolute_joint_is_owned_by_nativekit() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto arm_node = make_node(scene, 1.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    nksim_body_desc base_desc{};
    base_desc.struct_size = sizeof(base_desc);
    base_desc.node = base_node;
    base_desc.motion_type = NKSIM_MOTION_STATIC;
    nksim_body base = 0;
    assert(nksim_body_create(world, &base_desc, &base) == NKSIM_OK);

    const double half_extents[] = {0.1, 0.1, 0.5};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    nksim_body_desc arm_desc{};
    arm_desc.struct_size = sizeof(arm_desc);
    arm_desc.node = arm_node;
    arm_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    arm_desc.mass = 1.0;
    arm_desc.shape = shape;
    arm_desc.collision_layer = 1;
    arm_desc.collision_mask = 1;
    nksim_body arm = 0;
    assert(nksim_body_create(world, &arm_desc, &arm) == NKSIM_OK);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[2] = 1.0;
    // The pivot sits at the base's own origin; the arm's rest pose is 1
    // unit away along X, so in the arm's own frame the pivot is at -1 (F1:
    // anchor_a/anchor_b must agree with the bodies' rest poses).
    joint_desc.anchor_b[0] = -1.0;
    joint_desc.lower_limit = -0.6;
    joint_desc.upper_limit = 0.6;
    joint_desc.max_force = 20.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 0.5;
    target.max_force = 20.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);

    // The 20 N m clamp saturates the controller while the arm accelerates, so
    // it overshoots the target into the 0.6 limit before settling at 0.5.
    for (int index = 0; index < 100; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        assert(step.scene_changes != 0);
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(joint_state.position > 0.1); // Limits must be radians, not degrees.
    assert(joint_state.position < 0.6);

    nksim_body_state body_state{};
    body_state.struct_size = sizeof(body_state);
    assert(nksim_body_get_state(world, arm, &body_state) == NKSIM_OK);
    // F1: the arm now genuinely swings from the base's pivot (anchor_a at
    // the base origin, anchor_b 1 unit into the arm) instead of rotating in
    // place around its own center, so its world position traces the joint
    // angle around that pivot rather than staying fixed at (1, 0, 0).
    assert(std::abs(body_state.position[0] - std::cos(joint_state.position)) < 1e-3);
    assert(std::abs(body_state.position[1] - std::sin(joint_state.position)) < 1e-3);
    assert(std::abs(body_state.angular_velocity[2] - joint_state.velocity) < 1e-9);
    auto invalid_pose = body_state;
    invalid_pose.position[0] += 10.0;
    assert(nksim_body_set_state(world, arm, &invalid_pose) == NKSIM_ERROR_UNSUPPORTED);
    nksim_body_state unchanged{};
    unchanged.struct_size = sizeof(unchanged);
    assert(nksim_body_get_state(world, arm, &unchanged) == NKSIM_OK);
    assert(unchanged.position[0] == body_state.position[0]);
    const auto extra_node = make_node(scene, 10.0);
    const auto extra_body = make_body(world, extra_node, NKSIM_MOTION_STATIC, 0.0);
    step_world(world, 1);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(nksim_body_get_state(world, arm, &body_state) == NKSIM_OK);
    assert(std::abs(body_state.rotation[2] - std::sin(joint_state.position * 0.5)) < 1e-9);
    nksim_body_destroy(world, extra_body);
    target.target = 2.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    step_world(world, 100);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(joint_state.position > 0.5 && joint_state.position < 0.65);
    assert(nksim_world_reset(world) == NKSIM_OK);
    step_world(world, 1);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(std::abs(joint_state.position) < 1e-9);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void prismatic_joint_uses_mujoco_velocity_control() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto slider_node = make_node(scene, 0.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto shape = make_box(world);
    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
    const auto slider = make_body(world, slider_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_PRISMATIC;
    joint_desc.body_a = base;
    joint_desc.body_b = slider;
    joint_desc.axis_a[0] = 1.0;
    joint_desc.max_force = 50.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_VELOCITY;
    target.target = 1.0;
    target.max_force = 50.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    step_world(world, 20);

    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
    assert(state.position > 0.0);
    assert(state.velocity > 0.0);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, slider);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void fixed_joint_rebuilds_and_can_be_removed() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto child_node = make_node(scene, 1.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto shape = make_box(world);
    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
    const auto child = make_body(world, child_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_FIXED;
    joint_desc.body_a = base;
    joint_desc.body_b = child;
    // Rest poses are 1 unit apart along X (F1: anchor_a/anchor_b must agree
    // with the bodies' actual rest poses).
    joint_desc.anchor_b[0] = -1.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    step_world(world, 2);

    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
    assert(state.position == 0.0);
    nksim_joint_destroy(world, joint);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_ERROR_INVALID_HANDLE);
    nksim_body_destroy(world, child);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void kinematic_scene_state_drives_mujoco() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto body = make_body(world, node, NKSIM_MOTION_KINEMATIC, 1.0);

    set_node_x(scene, node, 3.0);
    step_world(world, 1);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.position[0] - 3.0) < 1e-6);
    assert(state.linear_velocity[0] == 0.0); // First tick has no motion history.

    // MuJoCo pins the body at the node pose; the reported twist is the node's
    // finite difference over the tick.
    for (int tick = 1; tick <= 3; ++tick) {
        set_node_x(scene, node, 3.0 + 0.05 * tick);
        step_world(world, 1);
        assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
        assert(std::abs(state.position[0] - (3.0 + 0.05 * tick)) < 1e-6);
        assert(std::abs(state.linear_velocity[0] - 5.0) < 1e-4);
        assert(std::abs(state.linear_velocity[1]) < 1e-9);
        assert(std::abs(state.angular_velocity[2]) < 1e-9);
    }
    step_world(world, 1);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.position[0] - 3.15) < 1e-6);
    assert(std::abs(state.linear_velocity[0]) < 1e-4);

    // A state write is a teleport, not a 1000 m/s velocity.
    set_node_x(scene, node, 13.0);
    state.position[0] = 13.0;
    state.linear_velocity[0] = 0.0;
    assert(nksim_body_set_state(world, body, &state) == NKSIM_OK);
    step_world(world, 1);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.position[0] - 13.0) < 1e-6);
    assert(state.linear_velocity[0] == 0.0);

    nksim_body_destroy(world, body);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void plane_shape_stops_dynamic_body() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto floor_node = make_node_xyz(scene, 0.0, 0.0, 0.0);
    const auto cube_node = make_node_xyz(scene, 0.0, 0.0, 2.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const double normal[] = {0.0, 0.0, 1.0};
    const double half_extents[] = {0.25, 0.25, 0.25};
    nksim_shape floor_shape = 0;
    nksim_shape cube_shape = 0;
    assert(nksim_shape_create_plane(world, normal, 0.0, &floor_shape) == NKSIM_OK);
    assert(nksim_shape_create_box(world, half_extents, &cube_shape) == NKSIM_OK);
    const auto floor = make_body(world, floor_node, NKSIM_MOTION_STATIC, 0.0,
                                  floor_shape);
    const auto cube = make_body(world, cube_node, NKSIM_MOTION_DYNAMIC, 1.0,
                                cube_shape);
    step_world(world, 300);

    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, cube, &state) == NKSIM_OK);
    assert(state.position[2] > 0.20 && state.position[2] < 0.60);
    assert(std::abs(state.linear_velocity[2]) < 0.1);

    nksim_body_destroy(world, cube);
    nksim_body_destroy(world, floor);
    nksim_shape_destroy(world, cube_shape);
    nksim_shape_destroy(world, floor_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

double run_deterministic_fall() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto shape = make_box(world);
    const auto body = make_body(world, node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    step_world(world, 100);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    const auto z = state.position[2];
    nksim_body_destroy(world, body);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return z;
}

void mujoco_replay_is_deterministic() {
    const auto first = run_deterministic_fall();
    const auto second = run_deterministic_fall();
    assert(first == second);
}

void rotated_free_body_preserves_world_angular_velocity() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 0.0);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.001;
    desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const auto shape = make_box(world);
    const auto body = make_body(world, node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    state.rotation[2] = state.rotation[3] = std::sqrt(0.5);
    state.angular_velocity[0] = 1.0;
    assert(nksim_body_set_state(world, body, &state) == NKSIM_OK);
    step_world(world, 1);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    assert(std::abs(state.angular_velocity[0] - 1.0) < 1e-9);
    assert(std::abs(state.angular_velocity[1]) < 1e-9);
    assert(std::abs(state.angular_velocity[2]) < 1e-9);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void wheel_velocity_target_does_not_stall() {
    // F2: a MuJoCo position actuator's bias (-kp*q - kv*qdot) applies even at
    // ctrl=0, so the idle position actuator drags a velocity-commanded wheel
    // back toward q=0 and it stalls well short of the commanded rate.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto wheel_node = make_node(scene, 0.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
    const auto shape = make_box(world);
    const auto wheel = make_body(world, wheel_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = wheel;
    joint_desc.axis_a[2] = 1.0;
    // No lower/upper limit: an unlimited (continuous) wheel joint.
    joint_desc.max_force = 50.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_VELOCITY;
    target.target = 1.0;
    target.max_force = 50.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);

    nksim_joint_state before{};
    before.struct_size = sizeof(before);
    step_world(world, 400); // 4 s: let the velocity settle.
    assert(nksim_joint_get_state(world, joint, &before) == NKSIM_OK);
    step_world(world, 100); // one more second.
    nksim_joint_state after{};
    after.struct_size = sizeof(after);
    assert(nksim_joint_get_state(world, joint, &after) == NKSIM_OK);

    assert(std::abs(after.velocity - 1.0) < 0.05);
    assert(after.position > before.position); // still turning, not stalled.

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, wheel);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void position_target_holds_under_gravity() {
    // F2: torque is scaled by the joint's own mass-matrix diagonal plus
    // gravity/Coriolis bias, so a fixed kp/kv no longer sags under a heavier
    // link's weight.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto arm_node = make_node(scene, 0.5); // Rest pose 0.5m out along the pivot's X.

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
    const auto shape = make_box(world);
    const auto arm = make_body(world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[1] = 1.0; // Hinge about Y: gravity torques it in the X-Z plane.
    joint_desc.anchor_b[0] = -0.5; // The arm's center is 0.5m out along the pivot's local X (matches its rest pose).
    joint_desc.max_force = 200.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 0.0; // Hold horizontal against gravity.
    target.max_force = 200.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    step_world(world, 300); // 3 s to settle.

    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(std::abs(joint_state.position) < 1e-3);
    assert(std::abs(joint_state.velocity) < 1e-2);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void effort_target_respects_max_force_clamp() {
    // F2: effort mode is unchanged (torque == target, clamped to max_force).
    // An effort target far beyond max_force must behave exactly like a
    // target of max_force itself.
    auto run = [](double effort_target, double max_force) {
        nkscene_scene scene = 0;
        assert(nkscene_scene_create(&scene) == NKS_OK);
        const auto base_node = make_node(scene, 0.0);
        const auto wheel_node = make_node(scene, 0.0);
        nksim_world_desc world_desc{};
        world_desc.struct_size = sizeof(world_desc);
        world_desc.scene = scene;
        world_desc.fixed_timestep = 0.01;
        world_desc.physics_substeps = 1;
        world_desc.gravity[2] = 0.0;
        nksim_world world = 0;
        assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
        const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
        const auto shape = make_box(world);
        const auto wheel = make_body(world, wheel_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
        nksim_joint_desc joint_desc{};
        joint_desc.struct_size = sizeof(joint_desc);
        joint_desc.type = NKSIM_JOINT_REVOLUTE;
        joint_desc.body_a = base;
        joint_desc.body_b = wheel;
        joint_desc.axis_a[2] = 1.0;
        joint_desc.max_force = max_force < 0.0 ? max_force : 1000.0;
        nksim_joint joint = 0;
        assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
        nksim_joint_target target{};
        target.struct_size = sizeof(target);
        target.joint = joint;
        target.mode = NKSIM_JOINT_TARGET_EFFORT;
        target.target = effort_target;
        target.max_force = max_force;
        assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
        step_world(world, 1);
        nksim_joint_state state{};
        state.struct_size = sizeof(state);
        assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
        nksim_joint_destroy(world, joint);
        nksim_body_destroy(world, wheel);
        nksim_body_destroy(world, base);
        nksim_shape_destroy(world, shape);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return state.velocity;
    };
    const auto clamped = run(500.0, 10.0);
    const auto at_limit = run(10.0, 10.0);
    assert(std::abs(clamped - at_limit) < 1e-9);
    assert(std::abs(clamped) > 1e-6); // The clamp still lets it move.
    assert(std::abs(run(500.0, -1.0)) < 1e-12); // RobotKit's explicit zero-effort limit disables force.
}

void kinematic_root_child_velocity_matches_joint_across_substeps() {
    // A KINEMATIC root that carries other bodies stays welded to its scripted
    // pose, so the reaction torque of its child's hinge actuator cannot move
    // it within a step's substeps. The child's world-frame angular velocity
    // (what a mounted IMU reads) then matches its hinge velocity.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto arm_node = make_node(scene, 0.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 4; // >1: the bug only appears within a step's substeps.
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    // A "kinematic root" the way a robot base is: driven by the scene node.
    const auto base = make_body(world, base_node, NKSIM_MOTION_KINEMATIC, 1.0);
    const auto shape = make_box(world);
    const auto arm = make_body(world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[2] = 1.0;
    joint_desc.max_force = 100.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    // Effort mode applies torque directly (no mass-matrix lookup), so this
    // exercises only the kinematic-root leak, independent of the actuator's
    // own per-DOF mass-matrix addressing.
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_EFFORT;
    target.target = 5.0;
    target.max_force = 0.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    step_world(world, 5);

    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);

    nksim_body_state arm_state{};
    arm_state.struct_size = sizeof(arm_state);
    assert(nksim_body_get_state(world, arm, &arm_state) == NKSIM_OK);

    // The base never actually moves (its scene node pose is never changed),
    // so the arm's world angular velocity about Z should be exactly its own
    // hinge qvel; any gap is spurious velocity leaked from the base.
    assert(std::abs(arm_state.angular_velocity[2] - joint_state.velocity) < 1e-8);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void two_joint_arm_on_kinematic_base_holds_position_under_gravity() {
    // Coverage for the M9-relevant shape (a multi-joint arm mounted on a
    // KINEMATIC base): joint2 has an ancestor dof (joint1), which is exactly
    // the row shape where the old data->M[dof_Madr[dof]] diagonal lookup
    // read the wrong cell (dof_Madr[dof] addresses the START of a dof's
    // sparse row, not its diagonal — see the ARCHITECTURE.md note on full
    // computed-torque control). The single-joint reproduction above
    // (kinematic_root_child_velocity_matches_joint_across_substeps, using
    // effort mode to bypass the mass matrix entirely) is what isolated and
    // confirmed that root cause: this test exercises the fixed controller
    // end to end on the actual multi-joint/kinematic-base shape M9 needs.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto link1_node = make_node(scene, 0.0);
    const auto link2_node = make_node_xyz(scene, 0.5, 0.0, 0.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto base = make_body(world, base_node, NKSIM_MOTION_KINEMATIC, 1.0);
    const auto shape = make_box(world);
    const auto link1 = make_body(world, link1_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    const auto link2 = make_body(world, link2_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

    nksim_joint_desc joint1_desc{};
    joint1_desc.struct_size = sizeof(joint1_desc);
    joint1_desc.type = NKSIM_JOINT_REVOLUTE;
    joint1_desc.body_a = base;
    joint1_desc.body_b = link1;
    joint1_desc.axis_a[1] = 1.0; // Hinge about Y: gravity torques it in the X-Z plane.
    joint1_desc.max_force = 200.0;
    nksim_joint joint1 = 0;
    assert(nksim_joint_create(world, &joint1_desc, &joint1) == NKSIM_OK);

    nksim_joint_desc joint2_desc{};
    joint2_desc.struct_size = sizeof(joint2_desc);
    joint2_desc.type = NKSIM_JOINT_REVOLUTE;
    joint2_desc.body_a = link1;
    joint2_desc.body_b = link2;
    joint2_desc.axis_a[1] = 1.0;
    joint2_desc.anchor_b[0] = -0.5; // link2's rest pose is 0.5m out along its own pivot's local X.
    joint2_desc.max_force = 200.0;
    nksim_joint joint2 = 0;
    assert(nksim_joint_create(world, &joint2_desc, &joint2) == NKSIM_OK);

    std::array<nksim_joint_target, 2> targets{};
    for (auto &target : targets) {
        target.struct_size = sizeof(target);
        target.mode = NKSIM_JOINT_TARGET_POSITION;
        target.target = 0.0; // Hold both joints horizontal against gravity.
        target.max_force = 200.0;
    }
    targets[0].joint = joint1;
    targets[1].joint = joint2;
    assert(nksim_world_set_joint_targets(world, targets.data(),
        static_cast<uint32_t>(targets.size())) == NKSIM_OK);
    step_world(world, 300); // 3 s to settle.

    nksim_joint_state state1{}, state2{};
    state1.struct_size = state2.struct_size = sizeof(state1);
    assert(nksim_joint_get_state(world, joint1, &state1) == NKSIM_OK);
    assert(nksim_joint_get_state(world, joint2, &state2) == NKSIM_OK);
    assert(std::abs(state1.position) < 1e-3);
    assert(std::abs(state1.velocity) < 1e-2);
    assert(std::abs(state2.position) < 1e-3);
    assert(std::abs(state2.velocity) < 1e-2);

    nksim_joint_destroy(world, joint2);
    nksim_joint_destroy(world, joint1);
    nksim_body_destroy(world, link2);
    nksim_body_destroy(world, link1);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void non_adjacent_links_do_not_self_collide() {
    // F4: link2 is separated from the base at rest, then the second joint
    // folds it back through the base. Direct parent/child pairs remain
    // excluded, but this non-adjacent contact must stop the fold.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto link1_node = make_node(scene, 0.6);
    const auto link2_node = make_node(scene, 1.2); // 0.65m clear of the base at rest.

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0; // Isolate contact response from gravity.
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const double base_half[] = {0.5, 0.5, 0.5};
    const double link_half[] = {0.05, 0.05, 0.05};
    nksim_shape base_shape = 0, link_shape = 0;
    assert(nksim_shape_create_box(world, base_half, &base_shape) == NKSIM_OK);
    assert(nksim_shape_create_box(world, link_half, &link_shape) == NKSIM_OK);
    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0, base_shape);
    const auto link1 = make_body(world, link1_node, NKSIM_MOTION_DYNAMIC, 1.0, link_shape);
    const auto link2 = make_body(world, link2_node, NKSIM_MOTION_DYNAMIC, 1.0, link_shape);

    nksim_joint_desc joint1_desc{};
    joint1_desc.struct_size = sizeof(joint1_desc);
    joint1_desc.type = NKSIM_JOINT_REVOLUTE;
    joint1_desc.body_a = base;
    joint1_desc.body_b = link1;
    joint1_desc.axis_a[2] = 1.0;
    joint1_desc.anchor_a[0] = 0.6; // Pivot at link1's own rest center.
    nksim_joint joint1 = 0;
    assert(nksim_joint_create(world, &joint1_desc, &joint1) == NKSIM_OK);

    nksim_joint_desc joint2_desc{};
    joint2_desc.struct_size = sizeof(joint2_desc);
    joint2_desc.type = NKSIM_JOINT_REVOLUTE;
    joint2_desc.body_a = link1;
    joint2_desc.body_b = link2;
    joint2_desc.axis_a[1] = 1.0; // About Y, so an X-direction contact push (see below) produces torque.
    // Pivot at link1's rest center; link2's rest center is 0.6m along +X.
    // Folding around +Y therefore drives it through the base at q=pi.
    joint2_desc.anchor_b[0] = -0.6;
    joint2_desc.lower_limit = -3.2;
    joint2_desc.upper_limit = 3.2;
    joint2_desc.max_force = 100.0;
    nksim_joint joint2 = 0;
    assert(nksim_joint_create(world, &joint2_desc, &joint2) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint2;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 3.0;
    target.max_force = 100.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    step_world(world, 300);

    nksim_joint_state state1{}, state2{};
    state1.struct_size = sizeof(state1);
    state2.struct_size = sizeof(state2);
    assert(nksim_joint_get_state(world, joint1, &state1) == NKSIM_OK);
    assert(nksim_joint_get_state(world, joint2, &state2) == NKSIM_OK);
    // The target folds link2 toward the base, but contact stops it before the
    // requested angle. A whole-component exclusion would incorrectly reach
    // the target and let the link pass through the base.
    assert(std::abs(state1.position) < 1e-6);
    assert(state2.position > 0.5);
    assert(state2.position < 2.5);

    nksim_joint_destroy(world, joint2);
    nksim_joint_destroy(world, joint1);
    nksim_body_destroy(world, link2);
    nksim_body_destroy(world, link1);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, link_shape);
    nksim_shape_destroy(world, base_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// -- Cross-backend acceptance test (M8.5) -----------------------------------
//
// A small self-contained rigid-transform helper (translation + xyzw
// quaternion, matching robotkit.spatial.Transform3's a_T_b convention)
// reproducing the exact forward-kinematics composition F1 uses for rest
// poses and F4 uses for the default backend's kinematic recompute:
// world_T_child = world_T_parent . T(anchor_a, rotation_a) . M(q) . T(anchor_b, rotation_b)^-1.
// This is "RobotKit's M2 FK" (KinematicChain's own composition) run in C++
// so both native backends can be checked against one reference without a
// Haxe roundtrip.
namespace fk {

using V3 = std::array<double, 3>;
using Q4 = std::array<double, 4>;

Q4 qconj(const Q4 &q) { return {-q[0], -q[1], -q[2], q[3]}; }
Q4 qmul(const Q4 &a, const Q4 &b) {
    return {
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
    };
}
V3 qrot(const Q4 &q, const V3 &v) {
    const V3 t{2.0*(q[1]*v[2]-q[2]*v[1]), 2.0*(q[2]*v[0]-q[0]*v[2]), 2.0*(q[0]*v[1]-q[1]*v[0])};
    return {v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1], v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2], v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0]};
}
struct Xf { V3 pos{0,0,0}; Q4 rot{0,0,0,1}; };
Xf compose(const Xf &a, const Xf &b) {
    const auto r = qrot(a.rot, b.pos);
    return {{a.pos[0]+r[0], a.pos[1]+r[1], a.pos[2]+r[2]}, qmul(a.rot, b.rot)};
}
Xf inverse(const Xf &a) {
    const auto inv = qconj(a.rot);
    const auto p = qrot(inv, {-a.pos[0], -a.pos[1], -a.pos[2]});
    return {p, inv};
}
Xf revolute(const V3 &axis, double q) {
    const double half = q * 0.5, s = std::sin(half);
    return {{0,0,0}, {axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half)}};
}
/** world_T_child at joint value q, given anchor_a/rotation_a (body_a side), anchor_b/rotation_b (body_b side), and the joint-frame axis. */
Xf child_pose(const Xf &world_T_parent, const Xf &a_side, const V3 &axis, double q, const Xf &b_side) {
    return compose(compose(compose(world_T_parent, a_side), revolute(axis, q)), inverse(b_side));
}

} // namespace fk

/** A scene node with a full rest transform (translation and rotation), unlike make_node[_xyz]. */
nkscene_node_id make_node_posed(nkscene_scene scene, const fk::V3 &pos, const fk::Q4 &rot) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_node_id node{};
    assert(nkscene_tx_create_node(transaction, &node) == NKS_OK);
    nkscene_transform transform{};
    const double x = rot[0], y = rot[1], z = rot[2], w = rot[3];
    const double xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z, wx = w*x, wy = w*y, wz = w*z;
    transform.matrix[0] = static_cast<float>(1.0 - 2.0*(yy+zz));
    transform.matrix[1] = static_cast<float>(2.0*(xy+wz));
    transform.matrix[2] = static_cast<float>(2.0*(xz-wy));
    transform.matrix[4] = static_cast<float>(2.0*(xy-wz));
    transform.matrix[5] = static_cast<float>(1.0 - 2.0*(xx+zz));
    transform.matrix[6] = static_cast<float>(2.0*(yz+wx));
    transform.matrix[8] = static_cast<float>(2.0*(xz+wy));
    transform.matrix[9] = static_cast<float>(2.0*(yz-wx));
    transform.matrix[10] = static_cast<float>(1.0 - 2.0*(xx+yy));
    transform.matrix[12] = static_cast<float>(pos[0]);
    transform.matrix[13] = static_cast<float>(pos[1]);
    transform.matrix[14] = static_cast<float>(pos[2]);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return node;
}

void cross_backend_link_poses_agree_with_fk() {
    // 3-link arm (base + link1 + link2), non-zero offsets on both joints and
    // a rotated joint frame on the second (rotation_b is a 90 degree turn
    // about X, so its axis_a=(0,0,1) is NOT link2's own local Z).
    const fk::V3 axis_z{0.0, 0.0, 1.0};
    const fk::Xf joint1_a{{0.08, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}; // anchor_a, rotation_a
    const fk::Xf joint1_b{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}; // anchor_b, rotation_b
    const double half90 = 0.5 * 1.5707963267948966;
    const fk::Q4 rot_x90{std::sin(half90), 0.0, 0.0, std::cos(half90)};
    const fk::Xf joint2_a{{0.0, 0.0, 0.06}, {0.0, 0.0, 0.0, 1.0}};
    const fk::Xf joint2_b{{0.0, 0.0, -0.02}, rot_x90};

    const fk::Xf world_T_base{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    const auto rest1 = fk::child_pose(world_T_base, joint1_a, axis_z, 0.0, joint1_b);
    const auto rest2 = fk::child_pose(rest1, joint2_a, axis_z, 0.0, joint2_b);

    const double q1 = 0.3, q2 = 0.4;
    const auto expected1_q = fk::child_pose(world_T_base, joint1_a, axis_z, q1, joint1_b);
    const auto expected2_q = fk::child_pose(expected1_q, joint2_a, axis_z, q2, joint2_b);

    auto run_backend = [&](bool use_mujoco, int steps) {
        nkscene_scene scene = 0;
        assert(nkscene_scene_create(&scene) == NKS_OK);
        const auto base_node = make_node_posed(scene, world_T_base.pos, world_T_base.rot);
        const auto link1_node = make_node_posed(scene, rest1.pos, rest1.rot);
        const auto link2_node = make_node_posed(scene, rest2.pos, rest2.rot);

        nksim_world_desc world_desc{};
        world_desc.struct_size = sizeof(world_desc);
        world_desc.scene = scene;
        world_desc.fixed_timestep = 0.01;
        world_desc.physics_substeps = 2;
        world_desc.gravity[2] = 0.0;
        nksim_world world = 0;
        assert((use_mujoco ? nksim_mujoco_world_create(&world_desc, &world)
                           : nksim_world_create(&world_desc, &world)) == NKSIM_OK);

        const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
        const auto shape = make_box(world);
        const auto link1 = make_body(world, link1_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
        const auto link2 = make_body(world, link2_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);

        nksim_joint_desc joint1_desc{};
        joint1_desc.struct_size = sizeof(joint1_desc);
        joint1_desc.type = NKSIM_JOINT_REVOLUTE;
        joint1_desc.body_a = base;
        joint1_desc.body_b = link1;
        std::copy(joint1_a.pos.begin(), joint1_a.pos.end(), joint1_desc.anchor_a);
        std::copy(joint1_b.pos.begin(), joint1_b.pos.end(), joint1_desc.anchor_b);
        std::copy(axis_z.begin(), axis_z.end(), joint1_desc.axis_a);
        std::copy(joint1_a.rot.begin(), joint1_a.rot.end(), joint1_desc.rotation_a);
        std::copy(joint1_b.rot.begin(), joint1_b.rot.end(), joint1_desc.rotation_b);
        joint1_desc.max_force = 1000.0;
        nksim_joint joint1 = 0;
        assert(nksim_joint_create(world, &joint1_desc, &joint1) == NKSIM_OK);

        nksim_joint_desc joint2_desc{};
        joint2_desc.struct_size = sizeof(joint2_desc);
        joint2_desc.type = NKSIM_JOINT_REVOLUTE;
        joint2_desc.body_a = link1;
        joint2_desc.body_b = link2;
        std::copy(joint2_a.pos.begin(), joint2_a.pos.end(), joint2_desc.anchor_a);
        std::copy(joint2_b.pos.begin(), joint2_b.pos.end(), joint2_desc.anchor_b);
        std::copy(axis_z.begin(), axis_z.end(), joint2_desc.axis_a);
        std::copy(joint2_a.rot.begin(), joint2_a.rot.end(), joint2_desc.rotation_a);
        std::copy(joint2_b.rot.begin(), joint2_b.rot.end(), joint2_desc.rotation_b);
        joint2_desc.max_force = 1000.0;
        nksim_joint joint2 = 0;
        assert(nksim_joint_create(world, &joint2_desc, &joint2) == NKSIM_OK);

        nksim_joint_target targets[2]{};
        targets[0].struct_size = sizeof(targets[0]);
        targets[0].joint = joint1;
        targets[0].mode = NKSIM_JOINT_TARGET_POSITION;
        targets[0].target = q1;
        targets[0].max_force = 1000.0;
        targets[1] = targets[0];
        targets[1].joint = joint2;
        targets[1].target = q2;
        assert(nksim_world_set_joint_targets(world, targets, 2) == NKSIM_OK);

        for (int i = 0; i < steps; ++i) {
            nksim_step_result step{};
            step.struct_size = sizeof(step);
            assert(nksim_world_step(world, &step) == NKSIM_OK);
            nkscene_change_set_destroy(step.scene_changes);
        }

        nksim_body_state state1{}, state2{};
        state1.struct_size = sizeof(state1);
        state2.struct_size = sizeof(state2);
        assert(nksim_body_get_state(world, link1, &state1) == NKSIM_OK);
        assert(nksim_body_get_state(world, link2, &state2) == NKSIM_OK);

        const double tolerance = use_mujoco ? 1e-3 : 1e-9;
        for (int i = 0; i < 3; ++i) {
            assert(std::abs(state1.position[i] - expected1_q.pos[i]) < tolerance);
            assert(std::abs(state2.position[i] - expected2_q.pos[i]) < tolerance);
        }

        // Teleporting the root carries the whole arm in both backends.
        nksim_body_state base_state{};
        base_state.struct_size = sizeof(base_state);
        assert(nksim_body_get_state(world, base, &base_state) == NKSIM_OK);
        base_state.position[0] += 2.0;
        base_state.position[1] += 1.0;
        assert(nksim_body_set_state(world, base, &base_state) == NKSIM_OK);
        if (use_mujoco) {
            // MuJoCo settles the constraint over a few steps rather than
            // instantaneously; the default backend recomputes it exactly.
            for (int i = 0; i < 5; ++i) {
                nksim_step_result step{};
                step.struct_size = sizeof(step);
                assert(nksim_world_step(world, &step) == NKSIM_OK);
                nkscene_change_set_destroy(step.scene_changes);
            }
        }
        assert(nksim_body_get_state(world, link1, &state1) == NKSIM_OK);
        assert(nksim_body_get_state(world, link2, &state2) == NKSIM_OK);
        assert(std::abs(state1.position[0] - (expected1_q.pos[0] + 2.0)) < tolerance);
        assert(std::abs(state1.position[1] - (expected1_q.pos[1] + 1.0)) < tolerance);
        assert(std::abs(state2.position[0] - (expected2_q.pos[0] + 2.0)) < tolerance);
        assert(std::abs(state2.position[1] - (expected2_q.pos[1] + 1.0)) < tolerance);

        nksim_joint_destroy(world, joint2);
        nksim_joint_destroy(world, joint1);
        nksim_body_destroy(world, link2);
        nksim_body_destroy(world, link1);
        nksim_body_destroy(world, base);
        nksim_shape_destroy(world, shape);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
    };

    run_backend(false, 1); // Default backend: an instant, exact position-mode set.
    run_backend(true, 400); // MuJoCo: let the PD controller settle.
}

void set_node_pose(nkscene_scene scene, nkscene_node_id node, double x, double y, double z,
                   double yaw) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = static_cast<float>(std::cos(yaw));
    transform.matrix[1] = static_cast<float>(std::sin(yaw));
    transform.matrix[4] = static_cast<float>(-std::sin(yaw));
    transform.matrix[5] = static_cast<float>(std::cos(yaw));
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[13] = static_cast<float>(y);
    transform.matrix[14] = static_cast<float>(z);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
}

nksim_body make_box_body(nksim_world world, nkscene_node_id node, uint32_t motion_type,
                         double mass, double hx, double hy, double hz) {
    const double half_extents[] = {hx, hy, hz};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    return make_body(world, node, motion_type, mass, shape);
}

struct HingeRig {
    nkscene_scene scene = 0;
    nkscene_node_id base_node{};
    nksim_world world = 0;
    nksim_body floor = 0;
    nksim_body base = 0;
    nksim_body arm = 0;
    nksim_joint joint = 0;
};

// A 0.3 m box base with a unit-mass, unit-inertia arm hinged about z at x = 1
// and driven by a 10 N m effort target. The base overlaps a static floor: a
// KINEMATIC-vs-STATIC pair such as this one generates no contact
// (add_self_collision_excludes() excludes every pair where neither body is
// DYNAMIC), so this overlap is deliberately harmless, and exercises that.
HingeRig make_hinge_rig(uint32_t base_motion) {
    HingeRig rig;
    assert(nkscene_scene_create(&rig.scene) == NKS_OK);
    const auto floor_node = make_node_xyz(rig.scene, 0.0, 0.0, 0.0);
    rig.base_node = make_node_xyz(rig.scene, 0.0, 0.0, 0.0);
    const auto arm_node = make_node_xyz(rig.scene, 1.0, 0.0, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = rig.scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    assert(nksim_mujoco_world_create(&world_desc, &rig.world) == NKSIM_OK);
    const double normal[] = {0.0, 0.0, 1.0};
    nksim_shape floor_shape = 0;
    assert(nksim_shape_create_plane(rig.world, normal, -0.2, &floor_shape) == NKSIM_OK);
    rig.floor = make_body(rig.world, floor_node, NKSIM_MOTION_STATIC, 0.0, floor_shape);
    rig.base = make_box_body(rig.world, rig.base_node, base_motion,
                             base_motion == NKSIM_MOTION_STATIC ? 0.0 : 1.0, 0.3, 0.3, 0.3);
    rig.arm = make_box_body(rig.world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, 0.1, 0.1, 0.1);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = rig.base;
    joint_desc.body_b = rig.arm;
    joint_desc.axis_a[2] = 1.0;
    joint_desc.anchor_a[0] = 1.0; // The hinge sits at the arm's origin.
    joint_desc.max_force = 100.0;
    assert(nksim_joint_create(rig.world, &joint_desc, &rig.joint) == NKSIM_OK);
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = rig.joint;
    target.mode = NKSIM_JOINT_TARGET_EFFORT;
    target.target = 10.0;
    target.max_force = 100.0;
    assert(nksim_world_set_joint_targets(rig.world, &target, 1) == NKSIM_OK);
    return rig;
}

void destroy_hinge_rig(HingeRig &rig) {
    nksim_joint_destroy(rig.world, rig.joint);
    nksim_body_destroy(rig.world, rig.arm);
    nksim_body_destroy(rig.world, rig.base);
    nksim_body_destroy(rig.world, rig.floor);
    nksim_world_destroy(rig.world);
    nkscene_scene_destroy(rig.scene);
}

nksim_joint_state joint_state_of(nksim_world world, nksim_joint joint) {
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
    return state;
}

nksim_body_state body_state_of(nksim_world world, nksim_body body) {
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, body, &state) == NKSIM_OK);
    return state;
}

// A kinematic root is prescribed motion: the reaction torque of the motor on
// its hinged child must not spin it (a free unit-inertia base would take half
// the motor's work and roughly halve the hinge rate), so the hinge moves as it
// does on a static base, and
// the arm's world angular velocity is exactly the base's prescribed rate plus
// the hinge rate.
void kinematic_base_is_not_moved_by_child_reaction() {
    auto kinematic = make_hinge_rig(NKSIM_MOTION_KINEMATIC);
    auto reference = make_hinge_rig(NKSIM_MOTION_STATIC);
    for (int tick = 0; tick < 10; ++tick) {
        step_world(kinematic.world, 1);
        step_world(reference.world, 1);
        const auto joint = joint_state_of(kinematic.world, kinematic.joint);
        const auto expected = joint_state_of(reference.world, reference.joint);
        const auto arm = body_state_of(kinematic.world, kinematic.arm);
        assert(std::abs(joint.velocity - expected.velocity) < 1e-6);
        assert(std::abs(joint.position - expected.position) < 1e-6);
        assert(std::abs(arm.angular_velocity[2] - joint.velocity) < 1e-9);
        assert(std::abs(arm.angular_velocity[0]) < 1e-9);
        assert(std::abs(arm.angular_velocity[1]) < 1e-9);
        for (int axis = 0; axis < 3; ++axis)
            assert(std::abs(arm.linear_velocity[axis]) < 1e-9);
        assert(std::abs(arm.position[0] - 1.0) < 1e-9);
        assert(std::abs(arm.position[1]) < 1e-9 && std::abs(arm.position[2]) < 1e-9);
    }
    assert(joint_state_of(kinematic.world, kinematic.joint).velocity > 0.1);

    // Drive the base along x while it yaws about the hinge axis. The arm rides
    // on it: its world rate is the base rate plus the hinge rate, its velocity
    // is the base twist carried to the arm origin, and (the arm's centre of
    // mass being on the hinge axis) the hinge still matches the static base.
    const double dt = 0.01, speed = 2.0, yaw_rate = 0.5;
    auto base = body_state_of(kinematic.world, kinematic.base);
    for (int tick = 1; tick <= 20; ++tick) {
        const double time = tick * dt;
        auto drive = base;
        drive.position[0] = speed * time;
        drive.position[1] = drive.position[2] = 0.0;
        drive.rotation[0] = drive.rotation[1] = 0.0;
        drive.rotation[2] = std::sin(0.5 * yaw_rate * time);
        drive.rotation[3] = std::cos(0.5 * yaw_rate * time);
        drive.linear_velocity[0] = speed;
        drive.linear_velocity[1] = drive.linear_velocity[2] = 0.0;
        drive.angular_velocity[0] = drive.angular_velocity[1] = 0.0;
        drive.angular_velocity[2] = yaw_rate;
        set_node_pose(kinematic.scene, kinematic.base_node, drive.position[0], 0.0, 0.0,
                      yaw_rate * time);
        assert(nksim_body_drive(kinematic.world, kinematic.base, &drive) == NKSIM_OK);
        step_world(kinematic.world, 1);
        step_world(reference.world, 1);
        base = body_state_of(kinematic.world, kinematic.base);
        const auto arm = body_state_of(kinematic.world, kinematic.arm);
        const auto joint = joint_state_of(kinematic.world, kinematic.joint);
        const auto expected = joint_state_of(reference.world, reference.joint);
        assert(std::abs(base.position[0] - drive.position[0]) < 1e-12);
        assert(std::abs(base.rotation[2] - drive.rotation[2]) < 1e-12);
        assert(std::abs(joint.velocity - expected.velocity) < 1e-6);
        assert(std::abs(arm.angular_velocity[2] - (yaw_rate + joint.velocity)) < 1e-9);
        const double yaw = yaw_rate * time;
        const double offset[2] = {std::cos(yaw), std::sin(yaw)};
        assert(std::abs(arm.position[0] - (drive.position[0] + offset[0])) < 1e-9);
        assert(std::abs(arm.position[1] - offset[1]) < 1e-9);
        assert(std::abs(arm.linear_velocity[0] - (speed - yaw_rate * offset[1])) < 1e-9);
        assert(std::abs(arm.linear_velocity[1] - yaw_rate * offset[0]) < 1e-9);
        assert(std::abs(arm.linear_velocity[2]) < 1e-9);
    }
    destroy_hinge_rig(kinematic);
    destroy_hinge_rig(reference);
}

// Contacts see a kinematic body's twist: friction carries a box resting on a
// moving kinematic platform along with it.
//
// Previously disabled: a kinematic body used to be pinned in MuJoCo without
// degrees of freedom and moved between steps through body_pos/body_quat. A
// contact's velocity is J * qvel, and a body with no DOFs contributes nothing
// to it (engine_core_util.c's mj_objectVelocity: "dof-less body (static or
// mocap): quick return"), so the platform slid out from under the box
// (which stayed at x = 0 with zero velocity) instead of dragging it by
// friction. A KINEMATIC root that carries no other bodies now owns a real
// free joint instead (see mujoco_backend.cpp's configure_body/step()), so
// its qvel is its twist and this works.
void kinematic_platform_carries_resting_box() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto platform_node = make_node_xyz(scene, 0.0, 0.0, 0.0);
    const auto box_node = make_node_xyz(scene, 0.0, 0.0, 0.2);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto platform = make_box_body(world, platform_node, NKSIM_MOTION_KINEMATIC, 1.0,
                                        2.0, 2.0, 0.1);
    const auto box = make_box_body(world, box_node, NKSIM_MOTION_DYNAMIC, 1.0, 0.1, 0.1, 0.1);
    step_world(world, 20); // Settle.
    nksim_body_state platform_state{}, box_state{};
    platform_state.struct_size = box_state.struct_size = sizeof(nksim_body_state);
    assert(nksim_body_get_state(world, platform, &platform_state) == NKSIM_OK);
    const double speed = 0.5;
    for (int tick = 1; tick <= 100; ++tick) {
        const double x = speed * world_desc.fixed_timestep * tick;
        set_node_x(scene, platform_node, x);
        auto drive = platform_state;
        drive.position[0] = x;
        drive.linear_velocity[0] = speed;
        assert(nksim_body_drive(world, platform, &drive) == NKSIM_OK);
        step_world(world, 1);
        assert(nksim_body_get_state(world, platform, &platform_state) == NKSIM_OK);
        assert(platform_state.position[0] == x && platform_state.position[2] == 0.0);
    }
    assert(nksim_body_get_state(world, box, &box_state) == NKSIM_OK);
    assert(std::abs(box_state.linear_velocity[0] - speed) < 0.01);
    assert(box_state.position[0] > 0.45);
    // Resting on the kinematic platform as on static ground, not sinking.
    assert(box_state.position[2] > 0.199 && box_state.position[2] < 0.201);

    nksim_body_destroy(world, box);
    nksim_body_destroy(world, platform);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A childless KINEMATIC root's free joint (configure_body) makes it visible to
// MuJoCo's contact solver so a DYNAMIC body touching it gets real friction
// (kinematic_platform_carries_resting_box above) — but it must not also
// make MuJoCo generate contacts between two bodies that can never move:
// two overlapping KINEMATIC bodies, or a KINEMATIC body overlapping a
// STATIC one. add_self_collision_excludes() restores that skip explicitly,
// for any pair where neither body is DYNAMIC, via mjs_addExclude (the same
// body-pair-exclude list MuJoCo's own mj_collision already consults right
// after broadphase, before any narrowphase geom work — see that function's
// comment for the full citation). With the default (unconfigured, zero
// margin/gap) shapes used here, the informational near-contact fallback in
// read_contacts() (for a case like a kinematic tool needing its own
// proximity to a fixed obstacle) also reports nothing, since its own
// detection band is zero — so the snapshot's contact count is exactly zero
// for every pair among a kinematic base, a second overlapping kinematic
// body, and a static floor, despite deep geometric overlap between all
// three. The base's own jointed DYNAMIC child, held by a position target,
// is undisturbed throughout.
void kinematic_bodies_never_contact_static_or_each_other() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto floor_node = make_node_xyz(scene, 0.0, 0.0, 0.0);
    const auto base_node = make_node_xyz(scene, 0.0, 0.0, 0.0);
    const auto other_node = make_node_xyz(scene, 0.1, 0.1, 0.0);
    const auto arm_node = make_node_xyz(scene, 1.0, 0.0, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const double normal[] = {0.0, 0.0, 1.0};
    nksim_shape floor_shape = 0;
    // Deep overlap: the floor surface is well inside both kinematic boxes.
    assert(nksim_shape_create_plane(world, normal, -0.2, &floor_shape) == NKSIM_OK);
    const auto floor = make_body(world, floor_node, NKSIM_MOTION_STATIC, 0.0, floor_shape);
    const auto base = make_box_body(world, base_node, NKSIM_MOTION_KINEMATIC, 1.0, 0.3, 0.3, 0.3);
    const auto other = make_box_body(world, other_node, NKSIM_MOTION_KINEMATIC, 1.0, 0.3, 0.3, 0.3);
    const auto shape = make_box(world);
    const auto arm = make_body(world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[1] = 1.0;
    joint_desc.anchor_a[0] = 1.0;
    joint_desc.max_force = 100.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 0.0; // Hold the arm exactly horizontal against gravity.
    target.max_force = 100.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);

    step_world(world, 30);

    nksim_contact contacts[16]{};
    uint32_t count = 0;
    assert(nksim_world_get_contacts(world, contacts, 16, &count) == NKSIM_OK);
    assert(count == 0);

    const auto state = joint_state_of(world, joint);
    assert(std::abs(state.position) < 1e-6 && std::abs(state.velocity) < 1e-4);
    const auto base_state = body_state_of(world, base);
    assert(base_state.position[0] == 0.0 && base_state.position[1] == 0.0 &&
           base_state.position[2] == 0.0);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, other);
    nksim_body_destroy(world, base);
    nksim_body_destroy(world, floor);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void distant_kinematic_pairs_skip_distance_calls() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double half[] = {0.05, 0.05, 0.05};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half, &shape) == NKSIM_OK);
    assert(nksim_shape_set_contact(world, shape, 0.0, 0.03) == NKSIM_OK);
    nksim_shape tool_pieces[16]{};
    nksim_shape_pose tool_poses[16]{};
    for (int index = 0; index < 16; ++index) {
        tool_pieces[index] = shape;
        tool_poses[index].position[0] = 100.0 + index * 10.0;
        tool_poses[index].rotation[3] = 1.0;
    }
    nksim_shape tool = 0;
    assert(nksim_shape_create_compound(world, tool_pieces, tool_poses, 16, &tool) == NKSIM_OK);
    make_body(world, make_node_xyz(scene, 0.0, 0.0, 0.0),
        NKSIM_MOTION_KINEMATIC, 1.0, tool);
    for (int index = 0; index < 50; ++index)
        make_body(world, make_node_xyz(scene, 1000.0 + index * 10.0, 0.0, 0.0),
            NKSIM_MOTION_STATIC, 0.0, shape);
    const auto before = nksim_mujoco_distance_call_count();
    step_world(world, 1);
    assert(nksim_mujoco_distance_call_count() == before);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void kinematic_parent_child_gap_is_filtered() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double half[] = {0.1, 0.1, 0.1};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half, &shape) == NKSIM_OK);
    assert(nksim_shape_set_contact(world, shape, 0.0, 0.03) == NKSIM_OK);
    const auto parent = make_body(world, make_node_xyz(scene, 0.0, 0.0, 0.0),
        NKSIM_MOTION_KINEMATIC, 1.0, shape);
    const auto child = make_body(world, make_node_xyz(scene, 0.21, 0.0, 0.0),
        NKSIM_MOTION_KINEMATIC, 1.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_FIXED;
    joint_desc.body_a = parent;
    joint_desc.body_b = child;
    joint_desc.anchor_b[0] = -0.21;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    step_world(world, 1);
    uint32_t count = 0;
    assert(nksim_world_get_contacts(world, nullptr, 0, &count) == NKSIM_OK);
    assert(count == 0);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A kinematic root with a child is jointless in MuJoCo and shares the world's
// weld id. Its gap to an unrelated static obstacle must still be reported.
void world_welded_kinematic_root_reports_static_proximity() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double half[] = {0.1, 0.1, 0.1};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half, &shape) == NKSIM_OK);
    assert(nksim_shape_set_contact(world, shape, 0.0, 0.03) == NKSIM_OK);
    const auto root = make_body(world, make_node_xyz(scene, 0.0, 0.0, 0.0),
        NKSIM_MOTION_KINEMATIC, 1.0, shape);
    const auto child = make_body(world, make_node_xyz(scene, 1.0, 0.0, 0.0),
        NKSIM_MOTION_KINEMATIC, 1.0, shape);
    const auto obstacle = make_body(world, make_node_xyz(scene, 0.21, 0.0, 0.0),
        NKSIM_MOTION_STATIC, 0.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_FIXED;
    joint_desc.body_a = root;
    joint_desc.body_b = child;
    joint_desc.anchor_b[0] = -1.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    step_world(world, 1);
    nksim_contact contacts[16]{};
    uint32_t count = 0;
    assert(nksim_world_get_contacts(world, contacts, 16, &count) == NKSIM_OK);
    bool found = false;
    for (uint32_t i = 0; i < count; ++i)
        if ((contacts[i].body_a == root && contacts[i].body_b == obstacle) ||
            (contacts[i].body_b == root && contacts[i].body_a == obstacle)) found = true;
    assert(found);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void kinematic_chain_self_collision_mask_blocks_proximity() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double half[] = {0.1, 0.1, 0.1};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half, &shape) == NKSIM_OK);
    assert(nksim_shape_set_contact(world, shape, 0.0, 0.03) == NKSIM_OK);
    const auto link = [&](double x) {
        nksim_body_desc body_desc{};
        body_desc.struct_size = sizeof(body_desc);
        body_desc.node = make_node_xyz(scene, x, 0.0, 0.0);
        body_desc.motion_type = NKSIM_MOTION_KINEMATIC;
        body_desc.mass = 1.0;
        body_desc.shape = shape;
        body_desc.collision_layer = 2;
        body_desc.collision_mask = 1; // RobotKit's self-collision-disabled mask.
        nksim_body body = 0;
        assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);
        return body;
    };
    const auto first = link(0.0);
    const auto middle = link(0.4);
    const auto last = link(0.21);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_FIXED;
    joint_desc.body_a = first;
    joint_desc.body_b = middle;
    joint_desc.anchor_b[0] = -0.4;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    joint_desc.body_a = middle;
    joint_desc.body_b = last;
    joint_desc.anchor_b[0] = 0.19;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    step_world(world, 1);
    uint32_t count = 0;
    assert(nksim_world_get_contacts(world, nullptr, 0, &count) == NKSIM_OK);
    assert(count == 0);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A body without explicit inertial properties has its centre of mass at its
// origin. Left undefined, MuJoCo copied the body's parent-relative position
// into its inertial frame, displacing the centre of mass by that offset: an
// unactuated arm hinged at its own origin then swung under gravity.
void body_without_inertials_has_center_of_mass_at_origin() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto arm_node = make_node(scene, 1.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto base = make_body(world, base_node, NKSIM_MOTION_STATIC, 0.0);
    const auto shape = make_box(world);
    const auto arm = make_body(world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[1] = 1.0;
    joint_desc.anchor_a[0] = 1.0; // Hinged about y at the arm's own origin.
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);
    step_world(world, 50);
    const auto state = joint_state_of(world, joint);
    assert(std::abs(state.position) < 1e-9 && std::abs(state.velocity) < 1e-9);
    const auto arm_state = body_state_of(world, arm);
    assert(std::abs(arm_state.position[0] - 1.0) < 1e-9 && std::abs(arm_state.position[2]) < 1e-9);
    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, base);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

nksim_body_force body_force(nksim_body body, std::array<double, 3> force,
                            std::array<double, 3> torque) {
    nksim_body_force value{};
    value.struct_size = sizeof(value);
    value.body = body;
    for (int axis = 0; axis < 3; ++axis) {
        value.force[axis] = force[axis];
        value.torque[axis] = torque[axis];
    }
    return value;
}

void applied_force_and_torque_act_on_their_own_axes() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto pushed_node = make_node(scene, 0.0);
    const auto twisted_node = make_node(scene, 5.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto shape = make_box(world);
    const auto pushed = make_body(world, pushed_node, NKSIM_MOTION_DYNAMIC, 2.0, shape);
    // Explicit inertia, so the torque's effect is exact: I = 0.02 kg m^2 on every axis.
    nksim_body_desc twisted_desc{};
    twisted_desc.struct_size = sizeof(twisted_desc);
    twisted_desc.node = twisted_node;
    twisted_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    twisted_desc.mass = 2.0;
    twisted_desc.shape = shape;
    twisted_desc.collision_layer = 1;
    twisted_desc.collision_mask = 1;
    twisted_desc.has_inertial_properties = 1;
    twisted_desc.inertia_tensor[0] = twisted_desc.inertia_tensor[4] =
        twisted_desc.inertia_tensor[8] = 0.02;
    nksim_body twisted = 0;
    assert(nksim_body_create(world, &twisted_desc, &twisted) == NKSIM_OK);

    // Two forces on the pushed body must sum, as in the default backend.
    const std::array<nksim_body_force, 3> forces{
        body_force(pushed, {4.0, 0.0, 0.0}, {0.0, 0.0, 0.0}),
        body_force(pushed, {4.0, 0.0, 0.0}, {0.0, 0.0, 0.0}),
        body_force(twisted, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.5}),
    };
    assert(nksim_world_apply_forces(world, forces.data(),
                                    static_cast<uint32_t>(forces.size())) == NKSIM_OK);
    step_world(world, 1);

    nksim_body_state pushed_state{};
    pushed_state.struct_size = sizeof(pushed_state);
    assert(nksim_body_get_state(world, pushed, &pushed_state) == NKSIM_OK);
    // 8 N on 2 kg for 0.01 s along +X, with no rotation.
    assert(std::abs(pushed_state.linear_velocity[0] - 0.04) < 1e-9);
    assert(std::abs(pushed_state.linear_velocity[1]) < 1e-12);
    assert(std::abs(pushed_state.linear_velocity[2]) < 1e-12);
    for (int axis = 0; axis < 3; ++axis)
        assert(std::abs(pushed_state.angular_velocity[axis]) < 1e-12);

    nksim_body_state twisted_state{};
    twisted_state.struct_size = sizeof(twisted_state);
    assert(nksim_body_get_state(world, twisted, &twisted_state) == NKSIM_OK);
    // 0.5 N m about +Z on 0.02 kg m^2 for 0.01 s spins it at 0.25 rad/s and pushes it nowhere.
    assert(std::abs(twisted_state.angular_velocity[2] - 0.25) < 1e-9);
    assert(std::abs(twisted_state.angular_velocity[0]) < 1e-12);
    assert(std::abs(twisted_state.angular_velocity[1]) < 1e-12);
    for (int axis = 0; axis < 3; ++axis)
        assert(std::abs(twisted_state.linear_velocity[axis]) < 1e-12);

    // Forces are cleared after each step.
    const auto before = pushed_state.linear_velocity[0];
    step_world(world, 1);
    assert(nksim_body_get_state(world, pushed, &pushed_state) == NKSIM_OK);
    assert(std::abs(pushed_state.linear_velocity[0] - before) < 1e-12);

    nksim_body_destroy(world, twisted);
    nksim_body_destroy(world, pushed);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void coupled_prismatic_joints_use_equality_and_convex_collision() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const auto base = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_STATIC, 0.0);
    const auto leader_body = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0);
    const double vertices[] = {
        -0.1,-0.1,-0.1, 0.1,-0.1,-0.1, -0.1,0.1,-0.1, 0.1,0.1,-0.1,
        -0.1,-0.1,0.1, 0.1,-0.1,0.1, -0.1,0.1,0.1, 0.1,0.1,0.1};
    nksim_shape hull = 0;
    assert(nksim_shape_create_convex(world, vertices, 24, &hull) == NKSIM_OK);
    const auto follower_body = make_body(world, make_node(scene, 0.0),
                                         NKSIM_MOTION_DYNAMIC, 1.0, hull);
    nksim_joint_desc joint{};
    joint.struct_size = sizeof(joint);
    joint.type = NKSIM_JOINT_PRISMATIC;
    joint.body_a = base;
    joint.axis_a[0] = 1.0;
    joint.lower_limit = -1.0;
    joint.upper_limit = 1.0;
    joint.max_force = 100.0;
    nksim_joint source = 0, target = 0;
    joint.body_b = leader_body;
    assert(nksim_joint_create(world, &joint, &source) == NKSIM_OK);
    joint.body_b = follower_body;
    assert(nksim_joint_create(world, &joint, &target) == NKSIM_OK);
    nksim_joint_coupling_desc coupling{};
    coupling.struct_size = sizeof(coupling);
    coupling.leader = source;
    coupling.follower = target;
    coupling.ratio = -2.0;
    coupling.offset = 0.1;
    assert(nksim_joint_couple(world, &coupling) == NKSIM_OK);
    nksim_joint_target command{};
    command.struct_size = sizeof(command);
    command.joint = source;
    command.mode = NKSIM_JOINT_TARGET_POSITION;
    command.target = 0.2;
    command.max_force = 100.0;
    assert(nksim_world_set_joint_targets(world, &command, 1) == NKSIM_OK);
    step_world(world, 300);
    nksim_joint_state a{}, b{};
    a.struct_size = b.struct_size = sizeof(a);
    assert(nksim_joint_get_state(world, source, &a) == NKSIM_OK);
    assert(nksim_joint_get_state(world, target, &b) == NKSIM_OK);
    assert(std::abs(a.position) > 0.1);
    assert(std::abs(b.position + 2.0 * a.position - 0.1) < 0.02);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A follower with two leaders is held by a fixed tendon and a tendon equality, like a CoreXY
// motor turning with both axes.
void a_follower_with_two_leaders_uses_a_tendon_equality() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const auto base = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_STATIC, 0.0);
    nksim_joint_desc joint{};
    joint.struct_size = sizeof(joint);
    joint.type = NKSIM_JOINT_PRISMATIC;
    joint.body_a = base;
    joint.axis_a[0] = 1.0;
    joint.lower_limit = -1.0;
    joint.upper_limit = 1.0;
    joint.max_force = 100.0;
    nksim_joint joints[3] = {};
    for (auto &handle : joints) {
        joint.body_b = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0);
        assert(nksim_joint_create(world, &joint, &handle) == NKSIM_OK);
    }
    // follower = 2 a - 3 b + 0.1 + 0.05.
    nksim_joint_coupling_desc term{};
    term.struct_size = sizeof(term);
    term.follower = joints[2];
    term.leader = joints[0];
    term.ratio = 2.0;
    term.offset = 0.1;
    assert(nksim_joint_couple(world, &term) == NKSIM_OK);
    assert(nksim_joint_couple(world, &term) == NKSIM_ERROR_INVALID_ARGUMENT);
    nksim_joint_coupling_desc cycle{};
    cycle.struct_size = sizeof(cycle);
    cycle.follower = joints[0];
    cycle.leader = joints[2];
    cycle.ratio = 1.0;
    assert(nksim_joint_couple(world, &cycle) == NKSIM_ERROR_INVALID_ARGUMENT);
    term.leader = joints[1];
    term.ratio = -3.0;
    term.offset = 0.05;
    assert(nksim_joint_couple(world, &term) == NKSIM_OK);
    nksim_joint_target commands[2]{};
    for (int index = 0; index < 2; ++index) {
        commands[index].struct_size = sizeof(commands[index]);
        commands[index].joint = joints[index];
        commands[index].mode = NKSIM_JOINT_TARGET_POSITION;
        commands[index].target = index == 0 ? 0.1 : -0.05;
        commands[index].max_force = 100.0;
    }
    assert(nksim_world_set_joint_targets(world, commands, 2) == NKSIM_OK);
    step_world(world, 400);
    nksim_joint_state state[3]{};
    for (int index = 0; index < 3; ++index) {
        state[index].struct_size = sizeof(state[index]);
        assert(nksim_joint_get_state(world, joints[index], &state[index]) == NKSIM_OK);
    }
    assert(std::abs(state[0].position) > 0.05 && std::abs(state[1].position) > 0.02);
    assert(std::abs(state[2].position - (2.0 * state[0].position - 3.0 * state[1].position + 0.15)) < 0.02);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A part of one machine can start inside another part's hull: a lead screw
// runs through its nut bracket, whose convex hull fills the bore. Such pairs
// overlap at rest, so the backend must exclude them like overlapping boxes,
// or the carriage carrying the bracket drags on the screw and never arrives.
void convex_hulls_overlapping_at_rest_do_not_collide() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    // The screw: a long convex bar along X, fixed in the world.
    const double screw_vertices[] = {
        -0.5,-0.02,-0.02, 0.5,-0.02,-0.02, -0.5,0.02,-0.02, 0.5,0.02,-0.02,
        -0.5,-0.02,0.02, 0.5,-0.02,0.02, -0.5,0.02,0.02, 0.5,0.02,0.02};
    nksim_shape screw_hull = 0;
    assert(nksim_shape_create_convex(world, screw_vertices, 24, &screw_hull) == NKSIM_OK);
    const auto base = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_STATIC, 0.0, screw_hull);
    // The bracket: a block around the screw, sliding along it.
    const double bracket_vertices[] = {
        -0.05,-0.05,-0.05, 0.05,-0.05,-0.05, -0.05,0.05,-0.05, 0.05,0.05,-0.05,
        -0.05,-0.05,0.05, 0.05,-0.05,0.05, -0.05,0.05,0.05, 0.05,0.05,0.05};
    nksim_shape bracket_hull = 0;
    assert(nksim_shape_create_convex(world, bracket_vertices, 24, &bracket_hull) == NKSIM_OK);
    const auto carriage = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0);
    const auto bracket = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0, bracket_hull);
    nksim_joint_desc slide{};
    slide.struct_size = sizeof(slide);
    slide.type = NKSIM_JOINT_PRISMATIC;
    slide.body_a = base;
    slide.body_b = carriage;
    slide.axis_a[0] = 1.0;
    slide.lower_limit = -0.4;
    slide.upper_limit = 0.4;
    slide.max_force = 100.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &slide, &joint) == NKSIM_OK);
    nksim_joint_desc weld{};
    weld.struct_size = sizeof(weld);
    weld.type = NKSIM_JOINT_FIXED;
    weld.body_a = carriage;
    weld.body_b = bracket;
    nksim_joint fixed = 0;
    assert(nksim_joint_create(world, &weld, &fixed) == NKSIM_OK);
    nksim_joint_target command{};
    command.struct_size = sizeof(command);
    command.joint = joint;
    command.mode = NKSIM_JOINT_TARGET_POSITION;
    command.target = 0.3;
    command.max_force = 100.0;
    assert(nksim_world_set_joint_targets(world, &command, 1) == NKSIM_OK);
    step_world(world, 200);
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
    assert(std::abs(state.position - 0.3) < 0.01);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void assembly_closures_compile_as_equalities() {
    for (const auto closure_type : {NKSIM_JOINT_FIXED, NKSIM_JOINT_REVOLUTE,
                                    NKSIM_JOINT_PRISMATIC, NKSIM_JOINT_SPHERICAL,
                                    NKSIM_JOINT_CYLINDRICAL, NKSIM_JOINT_PLANAR}) {
        nkscene_scene scene = 0;
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.scene = scene;
        desc.fixed_timestep = 0.01;
        desc.physics_substeps = 2;
        nksim_world world = 0;
        assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
        const auto base = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_STATIC, 0.0);
        const auto first = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0);
        const auto second = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_DYNAMIC, 1.0);
        nksim_joint_desc joint{};
        joint.struct_size = sizeof(joint);
        joint.type = NKSIM_JOINT_PRISMATIC;
        joint.body_a = base;
        joint.axis_a[0] = 1.0;
        joint.lower_limit = -1.0;
        joint.upper_limit = 1.0;
        joint.max_force = 100.0;
        nksim_joint handle = 0;
        joint.body_b = first;
        assert(nksim_joint_create(world, &joint, &handle) == NKSIM_OK);
        joint.body_b = second;
        assert(nksim_joint_create(world, &joint, &handle) == NKSIM_OK);
        nksim_closure_desc closure{};
        closure.struct_size = sizeof(closure);
        closure.type = closure_type;
        closure.body_a = first;
        closure.body_b = second;
        closure.axis_a[0] = 1.0;
        const auto result = nksim_closure_create(world, &closure);
        if (result != NKSIM_OK)
            std::fprintf(stderr, "closure type %u failed with %d\n", closure_type, result);
        assert(result == NKSIM_OK);
        step_world(world, 2);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
    }
}

void compound_shape_preserves_an_l_shaped_gap() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double horizontal_half[] = {0.5, 0.05, 0.05};
    const double vertical_half[] = {0.05, 0.05, 0.5};
    nksim_shape horizontal = 0, vertical = 0, compound = 0, sphere = 0;
    assert(nksim_shape_create_box(world, horizontal_half, &horizontal) == NKSIM_OK);
    assert(nksim_shape_create_box(world, vertical_half, &vertical) == NKSIM_OK);
    const nksim_shape children[] = {horizontal, vertical};
    nksim_shape_pose poses[2]{};
    poses[0].position[0] = 0.5; poses[0].position[2] = 0.05;
    poses[1].position[0] = 0.95; poses[1].position[2] = 0.5;
    poses[0].rotation[3] = poses[1].rotation[3] = 1.0;
    assert(nksim_shape_create_compound(world, children, poses, 2, &compound) == NKSIM_OK);
    assert(nksim_shape_create_sphere(world, 0.08, &sphere) == NKSIM_OK);
    const auto obstacle = make_body(world, make_node_xyz(scene, 0.0, 0.0, 0.0),
                                    NKSIM_MOTION_STATIC, 0.0, compound);
    const auto gap = make_body(world, make_node_xyz(scene, 0.5, 0.0, 1.2),
                               NKSIM_MOTION_DYNAMIC, 1.0, sphere);
    const auto contact = make_body(world, make_node_xyz(scene, 0.95, 0.0, 1.2),
                                   NKSIM_MOTION_DYNAMIC, 1.0, sphere);
    step_world(world, 20);
    nksim_body_state gap_state{}, contact_state{};
    gap_state.struct_size = contact_state.struct_size = sizeof(nksim_body_state);
    assert(nksim_body_get_state(world, gap, &gap_state) == NKSIM_OK);
    assert(nksim_body_get_state(world, contact, &contact_state) == NKSIM_OK);
    assert(std::abs(gap_state.position[0] - 0.5) < 1e-6);
    assert(gap_state.position[2] < 1.03);
    assert(contact_state.position[2] > gap_state.position[2] + 0.04);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

struct MeshContactResult { double velocity; uint32_t contacts; bool active; double distance; };

MeshContactResult convex_mesh_contact(double margin, double gap, double height = 0.165) {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 2;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    const double vertices[] = {
        -0.1,-0.1,-0.1, 0.1,-0.1,-0.1, -0.1,0.1,-0.1, 0.1,0.1,-0.1,
        -0.1,-0.1,0.1, 0.1,-0.1,0.1, -0.1,0.1,0.1, 0.1,0.1,0.1};
    nksim_shape mesh = 0, sphere = 0;
    assert(nksim_shape_create_convex(world, vertices, 24, &mesh) == NKSIM_OK);
    assert(nksim_shape_set_contact(world, mesh, margin, gap) == NKSIM_OK);
    assert(nksim_shape_create_sphere(world, 0.05, &sphere) == NKSIM_OK);
    const auto obstacle = make_body(world, make_node_xyz(scene, 0.0, 0.0, 0.0),
                                    NKSIM_MOTION_STATIC, 0.0, mesh);
    const auto moving = make_body(world, make_node_xyz(scene, 0.0, 0.0, height),
                                  NKSIM_MOTION_DYNAMIC, 1.0, sphere);
    step_world(world, 3);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, moving, &state) == NKSIM_OK);
    const double velocity = state.linear_velocity[2];
    nksim_contact contacts[8]{};
    uint32_t count = 0;
    assert(nksim_world_get_contacts(world, contacts, 8, &count) == NKSIM_OK);
    bool found = false, active = false;
    double distance = 0.0;
    for (uint32_t i = 0; i < std::min(count, 8u); ++i) {
        if ((contacts[i].body_a == obstacle && contacts[i].body_b == moving) ||
            (contacts[i].body_b == obstacle && contacts[i].body_a == moving)) {
            found = true;
            active = contacts[i].active != 0;
            distance = contacts[i].distance;
            assert(contacts[i].part_a == 0 && contacts[i].part_b == 0);
        }
    }
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return {velocity, found ? count : 0u, active, distance};
}

void convex_mesh_margin_detects_before_gap_force() {
    const auto physical = convex_mesh_contact(0.03, 0.0);
    const auto proximity = convex_mesh_contact(0.0, 0.03);
    const auto touching = convex_mesh_contact(0.0, 0.03, 0.145);
    assert(physical.velocity > 1e-5 && physical.contacts > 0 && physical.active);
    assert(std::abs(proximity.velocity) < 1e-8 && proximity.contacts > 0 && !proximity.active);
    assert(proximity.distance > 0.0);
    assert(touching.velocity > 1e-5 && touching.contacts > 0 && touching.active);
}

} // namespace

// An upright cylinder rests on its flat end at half its height; a capsule of
// the same radius and height would stand a radius taller on its rounded cap.
void cylinder_rests_on_its_flat_end() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto floor_node = make_node_xyz(scene, 0.0, 0.0, 0.0);
    const auto cylinder_node = make_node_xyz(scene, 0.0, 0.0, 0.5);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const double normal[] = {0.0, 0.0, 1.0};
    nksim_shape floor_shape = 0, cylinder_shape = 0, invalid = 0;
    assert(nksim_shape_create_plane(world, normal, 0.0, &floor_shape) == NKSIM_OK);
    assert(nksim_shape_create_cylinder(world, 0.1, 0.4, &cylinder_shape) == NKSIM_OK);
    assert(nksim_shape_create_cylinder(world, 0.1, 0.0, &invalid) ==
           NKSIM_ERROR_INVALID_ARGUMENT);
    const auto floor = make_body(world, floor_node, NKSIM_MOTION_STATIC, 0.0, floor_shape);
    const auto cylinder = make_body(world, cylinder_node, NKSIM_MOTION_DYNAMIC, 1.0,
                                    cylinder_shape);
    step_world(world, 400);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, cylinder, &state) == NKSIM_OK);
    assert(std::abs(state.position[2] - 0.2) < 0.005);
    assert(std::abs(state.linear_velocity[2]) < 0.01);
    nksim_body_destroy(world, cylinder);
    nksim_body_destroy(world, floor);
    nksim_shape_destroy(world, cylinder_shape);
    nksim_shape_destroy(world, floor_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// A 1 kg arm, centre 0.5 m from a hinge about Y, in a MuJoCo world. Gravity
// loads the hinge with 4.905 N m at the horizontal pose.
struct ArmRig {
    nkscene_scene scene = 0;
    nksim_world world = 0;
    nksim_body base = 0, arm = 0;
    nksim_shape shape = 0;
    nksim_joint joint = 0;
};

ArmRig make_arm_rig(double armature, double damping, double friction_loss,
                   double limit_time_constant = 0.0) {
    ArmRig rig;
    assert(nkscene_scene_create(&rig.scene) == NKS_OK);
    const auto base_node = make_node(rig.scene, 0.0);
    const auto arm_node = make_node(rig.scene, 0.5);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = rig.scene;
    world_desc.fixed_timestep = 0.002;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    assert(nksim_mujoco_world_create(&world_desc, &rig.world) == NKSIM_OK);
    rig.base = make_body(rig.world, base_node, NKSIM_MOTION_STATIC, 0.0);
    rig.shape = make_box(rig.world);
    rig.arm = make_body(rig.world, arm_node, NKSIM_MOTION_DYNAMIC, 1.0, rig.shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = rig.base;
    joint_desc.body_b = rig.arm;
    joint_desc.axis_a[1] = 1.0;
    joint_desc.anchor_b[0] = -0.5;
    joint_desc.armature = armature;
    joint_desc.damping = damping;
    joint_desc.friction_loss = friction_loss;
    if (limit_time_constant > 0.0) {
        joint_desc.lower_limit = -1.0;
        joint_desc.upper_limit = 0.3;
        joint_desc.limit_time_constant = limit_time_constant;
        joint_desc.limit_damping_ratio = 1.0;
    }
    assert(nksim_joint_create(rig.world, &joint_desc, &rig.joint) == NKSIM_OK);
    return rig;
}

double arm_angle(const ArmRig &rig) {
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(rig.world, rig.joint, &state) == NKSIM_OK);
    return state.position;
}

void destroy_arm_rig(ArmRig &rig) {
    nksim_joint_destroy(rig.world, rig.joint);
    nksim_body_destroy(rig.world, rig.arm);
    nksim_body_destroy(rig.world, rig.base);
    nksim_shape_destroy(rig.world, rig.shape);
    nksim_world_destroy(rig.world);
    nkscene_scene_destroy(rig.scene);
}

double servo_arm_angle(double stiffness, double feedforward, double max_force) {
    auto rig = make_arm_rig(0.0, 0.0, 0.0);
    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = rig.joint;
    target.mode = NKSIM_JOINT_TARGET_SERVO;
    target.max_force = max_force;
    target.stiffness = stiffness;
    target.damping = 5.0;
    target.feedforward = feedforward;
    assert(nksim_world_set_joint_targets(rig.world, &target, 1) == NKSIM_OK);
    step_world(rig.world, 1000);
    const double angle = arm_angle(rig);
    destroy_arm_rig(rig);
    return angle;
}

// A servo is a plain PD: it sags by load / stiffness, feedforward cancels the
// load, and max_force caps the effort.
void servo_target_is_a_saturating_pd() {
    const double load = 9.81 * 0.5;
    const double sag = servo_arm_angle(100.0, 0.0, 0.0);
    assert(std::abs(std::abs(sag) - load / 100.0) < 0.003);
    assert(std::abs(servo_arm_angle(100.0, sag > 0.0 ? -load : load, 0.0)) < 1e-3);
    assert(std::abs(servo_arm_angle(100.0, 0.0, 2.0)) > 1.0);

    auto rig = make_arm_rig(0.0, 0.0, 0.0);
    nksim_joint_target invalid{};
    invalid.struct_size = sizeof(invalid);
    invalid.joint = rig.joint;
    invalid.mode = NKSIM_JOINT_TARGET_SERVO;
    invalid.stiffness = -1.0;
    assert(nksim_world_set_joint_targets(rig.world, &invalid, 1) == NKSIM_ERROR_INVALID_ARGUMENT);
    invalid.stiffness = 1.0;
    invalid.struct_size = offsetof(nksim_joint_target, velocity); // Too short to carry servo terms.
    assert(nksim_world_set_joint_targets(rig.world, &invalid, 1) == NKSIM_ERROR_INVALID_ARGUMENT);
    invalid.mode = NKSIM_JOINT_TARGET_EFFORT; // The legacy prefix still works for other modes.
    assert(nksim_world_set_joint_targets(rig.world, &invalid, 1) == NKSIM_OK);
    destroy_arm_rig(rig);
}

// A saturated servo has the same force and velocity derivative as a constant effort.
// A joint-only force clamp otherwise leaves its unsaturated damping in implicitfast.
void servo_interpolates_accelerating_reference_between_controller_ticks() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 40;
    desc.integrator = NKSIM_INTEGRATOR_IMPLICIT_FAST;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    auto base = make_body(world, make_node(scene, 0), NKSIM_MOTION_STATIC, 0);
    auto slider = make_body(world, make_node(scene, 0), NKSIM_MOTION_DYNAMIC, 1);
    nksim_joint_desc joint{};
    joint.struct_size = sizeof(joint);
    joint.type = NKSIM_JOINT_PRISMATIC;
    joint.body_a = base;
    joint.body_b = slider;
    joint.axis_a[0] = 1;
    joint.lower_limit = -1;
    joint.upper_limit = 1;
    nksim_joint id = 0;
    assert(nksim_joint_create(world, &joint, &id) == NKSIM_OK);
    assert(nksim_joint_set_state(world, id, 0, 0.5) == NKSIM_OK);
    for (int tick = 0; tick < 10; ++tick) {
        const double t = tick * 0.01;
        nksim_joint_target command{};
        command.struct_size = sizeof(command);
        command.joint = id;
        command.mode = NKSIM_JOINT_TARGET_SERVO;
        command.target = 0.5 * t + 0.15 * t * t;
        command.velocity = 0.5 + 0.3 * t;
        const double end_time = t + 0.01;
        command.end_position = 0.5 * end_time + 0.15 * end_time * end_time;
        command.end_velocity = 0.5 + 0.3 * end_time;
        command.reference_duration = 0.01;
        command.stiffness = 1000;
        command.damping = 63;
        command.reflected_inertia = 1.0; // One kilogram of reflected mass.
        assert(nksim_world_set_joint_targets(world, &command, 1) == NKSIM_OK);
        step_world(world, 1);
        nksim_joint_state state{};
        state.struct_size = sizeof(state);
        assert(nksim_joint_get_state(world, id, &state) == NKSIM_OK);
        const double end = (tick + 1) * 0.01;
        assert(std::abs(state.position - (0.5 * end + 0.15 * end * end)) < 0.00002);
    }
    assert(nksim_world_configure_integration(world, 1, NKSIM_INTEGRATOR_EULER) == NKSIM_ERROR_INVALID_STATE);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void saturated_servo_matches_effort_with_implicit_integration() {
    auto travel = [](bool servo) {
        nkscene_scene scene = 0;
        assert(nkscene_scene_create(&scene) == NKS_OK);
        nksim_world_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.scene = scene;
        desc.fixed_timestep = 0.001;
        desc.physics_substeps = 1;
        desc.integrator = NKSIM_INTEGRATOR_IMPLICIT_FAST;
        nksim_world world = 0;
        assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
        auto base = make_body(world, make_node(scene, 0), NKSIM_MOTION_STATIC, 0);
        auto rotor = make_body(world, make_node(scene, 0), NKSIM_MOTION_DYNAMIC, 1);
        nksim_joint_desc joint{};
        joint.struct_size = sizeof(joint);
        joint.type = NKSIM_JOINT_REVOLUTE;
        joint.body_a = base;
        joint.body_b = rotor;
        joint.axis_a[2] = 1;
        joint.lower_limit = -1000;
        joint.upper_limit = 1000;
        joint.armature = 0.01;
        nksim_joint id = 0;
        assert(nksim_joint_create(world, &joint, &id) == NKSIM_OK);
        nksim_joint_target command{};
        command.struct_size = sizeof(command);
        command.joint = id;
        command.mode = servo ? NKSIM_JOINT_TARGET_SERVO : NKSIM_JOINT_TARGET_EFFORT;
        command.target = servo ? 100 : 1;
        command.max_force = 1;
        command.stiffness = 1000;
        command.damping = 10;
        assert(nksim_world_set_joint_targets(world, &command, 1) == NKSIM_OK);
        step_world(world, 10);
        nksim_joint_state state{};
        state.struct_size = sizeof(state);
        assert(nksim_joint_get_state(world, id, &state) == NKSIM_OK);
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return state.position;
    };
    auto effort = travel(false), servo = travel(true);
    assert(effort > 1e-12);
    assert(std::abs(servo - effort) < 1e-10);
}

double falling_arm_angle(double armature, double damping, double friction_loss, int steps) {
    auto rig = make_arm_rig(armature, damping, friction_loss);
    step_world(rig.world, steps);
    const double angle = std::abs(arm_angle(rig));
    destroy_arm_rig(rig);
    return angle;
}

// Joint damping and armature slow a released arm (unit inertia about its
// centre, 1.25 kg m^2 about the hinge); friction loss above the gravity load
// holds it, apart from the creep MuJoCo's soft dry friction allows.
void joint_dynamics_reach_the_backend() {
    const double free_fall = falling_arm_angle(0.0, 0.0, 0.0, 150);
    assert(free_fall > 0.15);
    assert(falling_arm_angle(0.0, 5.0, 0.0, 150) < 0.8 * free_fall);
    assert(falling_arm_angle(0.5, 0.0, 0.0, 150) < 0.8 * free_fall);
    assert(falling_arm_angle(0.0, 0.0, 10.0, 500) < 0.01);
    assert(falling_arm_angle(0.0, 0.0, 0.0, 500) > 1.0);
}

// Distance a box slides down a 20 degree incline in one second when both the
// box and the incline have the given sliding friction.
double incline_slide(double friction) {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const double tilt = 20.0 * 3.14159265358979323846 / 180.0;
    const double normal[] = {std::sin(tilt), 0.0, std::cos(tilt)};
    const auto plane_node = make_node(scene, 0.0);
    const auto box_node = make_node_xyz(scene, normal[0] * 0.1, 0.0, normal[2] * 0.1);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.002;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    nksim_shape plane_shape = 0, box_shape = 0;
    assert(nksim_shape_create_plane(world, normal, 0.0, &plane_shape) == NKSIM_OK);
    box_shape = make_box(world);
    nksim_surface surface{};
    surface.struct_size = sizeof(surface);
    surface.friction_dimensions = 3;
    surface.friction[0] = friction;
    assert(nksim_shape_set_surface(world, plane_shape, &surface) == NKSIM_OK);
    assert(nksim_shape_set_surface(world, box_shape, &surface) == NKSIM_OK);
    const auto plane = make_body(world, plane_node, NKSIM_MOTION_STATIC, 0.0, plane_shape);
    const auto box = make_body(world, box_node, NKSIM_MOTION_DYNAMIC, 1.0, box_shape);
    nksim_surface invalid = surface;
    invalid.friction_dimensions = 2;
    assert(nksim_shape_set_surface(world, box_shape, &invalid) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_shape_set_surface(world, box_shape, &surface) == NKSIM_ERROR_INVALID_STATE);
    nksim_body_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_body_get_state(world, box, &state) == NKSIM_OK);
    state.rotation[0] = 0.0;
    state.rotation[1] = std::sin(tilt / 2.0);
    state.rotation[2] = 0.0;
    state.rotation[3] = std::cos(tilt / 2.0);
    assert(nksim_body_set_state(world, box, &state) == NKSIM_OK);
    const double start = state.position[0];
    step_world(world, 500);
    assert(nksim_body_get_state(world, box, &state) == NKSIM_OK);
    const double slide = std::abs(state.position[0] - start) / std::cos(tilt);
    nksim_body_destroy(world, box);
    nksim_body_destroy(world, plane);
    nksim_shape_destroy(world, box_shape);
    nksim_shape_destroy(world, plane_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return slide;
}

// tan(20 degrees) is 0.36: a box with friction 1 holds, with friction 0.1 slides.
void surface_friction_decides_sliding() {
    assert(incline_slide(1.0) < 0.01);
    assert(incline_slide(0.1) > 0.5);
}

void world_options_are_validated() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.scene = scene;
    desc.fixed_timestep = 0.01;
    desc.physics_substeps = 1;
    desc.integrator = NKSIM_INTEGRATOR_IMPLICIT_FAST;
    desc.friction_cone = NKSIM_FRICTION_CONE_ELLIPTIC;
    desc.solver_iterations = 5;
    desc.line_search_iterations = 8;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    nksim_world_destroy(world);
    desc.integrator = 9;
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_ERROR_INVALID_ARGUMENT);
    desc.integrator = 7; // Beyond the prefix an older caller supplies, so never read.
    desc.struct_size = offsetof(nksim_world_desc, integrator);
    assert(nksim_mujoco_world_create(&desc, &world) == NKSIM_OK);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// Contact filters: a box that collides only through pairs falls through the
// floor, one that also meets the environment rests on it, and one with an
// explicit pair to the floor rests on it too.
void contact_filters_and_pairs_decide_contacts() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.005;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const double normal[] = {0.0, 0.0, 1.0};
    nksim_shape floor_shape = 0;
    assert(nksim_shape_create_plane(world, normal, 0.0, &floor_shape) == NKSIM_OK);
    const auto floor = make_body(world, make_node(scene, 0.0), NKSIM_MOTION_STATIC, 0.0, floor_shape);
    nksim_body boxes[3]{};
    nksim_shape shapes[3]{};
    const uint32_t filters[] = {NKSIM_CONTACT_PAIRS_ONLY, NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT,
                                NKSIM_CONTACT_PAIRS_ONLY};
    assert(nksim_world_begin_topology_update(world) == NKSIM_OK);
    for (int index = 0; index < 3; ++index) {
        shapes[index] = make_box(world);
        nksim_surface surface{};
        surface.struct_size = sizeof(surface);
        surface.contact_filter = filters[index];
        assert(nksim_shape_set_surface(world, shapes[index], &surface) == NKSIM_OK);
        boxes[index] = make_body(world, make_node_xyz(scene, index * 1.0, 0.0, 0.3),
                                 NKSIM_MOTION_DYNAMIC, 1.0, shapes[index]);
    }
    nksim_contact_pair_desc pair{};
    pair.struct_size = sizeof(pair);
    pair.body_a = boxes[2];
    pair.body_b = floor;
    pair.surface.struct_size = sizeof(pair.surface);
    assert(nksim_contact_pair_create(world, &pair) == NKSIM_OK);
    auto invalid = pair;
    invalid.part_a = 1; // The box shape has one part.
    assert(nksim_contact_pair_create(world, &invalid) == NKSIM_ERROR_INVALID_ARGUMENT);
    assert(nksim_world_end_topology_update(world) == NKSIM_OK);
    step_world(world, 200);
    double heights[3]{};
    for (int index = 0; index < 3; ++index) {
        nksim_body_state state{};
        state.struct_size = sizeof(state);
        assert(nksim_body_get_state(world, boxes[index], &state) == NKSIM_OK);
        heights[index] = state.position[2];
    }
    assert(heights[0] < -1.0);
    assert(std::abs(heights[1] - 0.1) < 0.005);
    assert(std::abs(heights[2] - 0.1) < 0.005);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

// An arm falling onto its 0.3 rad stop rests further past it when the limit is
// softer (a longer time constant).
void joint_limit_softness_reaches_the_backend() {
    const auto rest = [](double time_constant) {
        auto rig = make_arm_rig(0.0, 1.0, 0.0, time_constant);
        step_world(rig.world, 1500);
        const double angle = arm_angle(rig);
        destroy_arm_rig(rig);
        return angle - 0.3;
    };
    const double stiff = rest(0.004), soft = rest(0.1);
    assert(stiff > 0.0 && stiff < 0.01);
    assert(soft > 5.0 * stiff);
}

int main() {
    joint_limit_softness_reaches_the_backend();
    contact_filters_and_pairs_decide_contacts();
    servo_target_is_a_saturating_pd();
    servo_interpolates_accelerating_reference_between_controller_ticks();
    saturated_servo_matches_effort_with_implicit_integration();
    joint_dynamics_reach_the_backend();
    surface_friction_decides_sliding();
    world_options_are_validated();
    revolute_joint_is_owned_by_nativekit();
    prismatic_joint_uses_mujoco_velocity_control();
    fixed_joint_rebuilds_and_can_be_removed();
    kinematic_scene_state_drives_mujoco();
    plane_shape_stops_dynamic_body();
    cylinder_rests_on_its_flat_end();
    mujoco_replay_is_deterministic();
    rotated_free_body_preserves_world_angular_velocity();
    wheel_velocity_target_does_not_stall();
    position_target_holds_under_gravity();
    effort_target_respects_max_force_clamp();
    non_adjacent_links_do_not_self_collide();
    cross_backend_link_poses_agree_with_fk();
    kinematic_root_child_velocity_matches_joint_across_substeps();
    two_joint_arm_on_kinematic_base_holds_position_under_gravity();
    kinematic_base_is_not_moved_by_child_reaction();
    kinematic_platform_carries_resting_box();
    kinematic_bodies_never_contact_static_or_each_other();
    distant_kinematic_pairs_skip_distance_calls();
    kinematic_parent_child_gap_is_filtered();
    world_welded_kinematic_root_reports_static_proximity();
    kinematic_chain_self_collision_mask_blocks_proximity();
    body_without_inertials_has_center_of_mass_at_origin();
    applied_force_and_torque_act_on_their_own_axes();
    coupled_prismatic_joints_use_equality_and_convex_collision();
    a_follower_with_two_leaders_uses_a_tendon_equality();
    convex_hulls_overlapping_at_rest_do_not_collide();
    assembly_closures_compile_as_equalities();
    compound_shape_preserves_an_l_shaped_gap();
    convex_mesh_margin_detects_before_gap_force();
    return 0;
}
