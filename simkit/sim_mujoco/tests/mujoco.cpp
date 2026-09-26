#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_mujoco.h"

#include <cassert>
#include <cmath>

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

    for (int index = 0; index < 40; ++index) {
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
        joint_desc.max_force = 1000.0; // Unclamped at the joint description itself.
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
}

void non_adjacent_links_do_not_self_collide() {
    // F3: only parent/child joint pairs were excluded from contact, but
    // after F1's rest-pose fix every link sits at its real offset, so two
    // NON-adjacent links of the same robot (base and link2 here) can
    // genuinely overlap and must still not push on each other.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto link1_node = make_node(scene, 0.6);
    const auto link2_node = make_node(scene, 0.1); // Overlaps the base's own box again.

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
    // Pivot at world (0.1, 0, 0.4): offset from link2's own rest center
    // (0.1, 0, 0) along Z, so a contact force separating base and link2
    // along X (their minimum-penetration axis) has a real lever arm
    // instead of passing straight through link2's center.
    joint2_desc.anchor_a[0] = -0.5;
    joint2_desc.anchor_a[2] = 0.4;
    joint2_desc.anchor_b[2] = 0.4;
    nksim_joint joint2 = 0;
    assert(nksim_joint_create(world, &joint2_desc, &joint2) == NKSIM_OK);

    step_world(world, 20);

    nksim_joint_state state1{}, state2{};
    state1.struct_size = sizeof(state1);
    state2.struct_size = sizeof(state2);
    assert(nksim_joint_get_state(world, joint1, &state1) == NKSIM_OK);
    assert(nksim_joint_get_state(world, joint2, &state2) == NKSIM_OK);
    // No gravity, no actuation, no adjacent overlap: with base and link2
    // correctly excluded from contact, nothing should move either joint.
    assert(std::abs(state1.position) < 1e-6);
    assert(std::abs(state2.position) < 1e-6);
    assert(std::abs(state1.velocity) < 1e-6);
    assert(std::abs(state2.velocity) < 1e-6);

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

} // namespace

int main() {
    revolute_joint_is_owned_by_nativekit();
    prismatic_joint_uses_mujoco_velocity_control();
    fixed_joint_rebuilds_and_can_be_removed();
    kinematic_scene_state_drives_mujoco();
    plane_shape_stops_dynamic_body();
    mujoco_replay_is_deterministic();
    rotated_free_body_preserves_world_angular_velocity();
    wheel_velocity_target_does_not_stall();
    position_target_holds_under_gravity();
    effort_target_respects_max_force_clamp();
    non_adjacent_links_do_not_self_collide();
    return 0;
}
