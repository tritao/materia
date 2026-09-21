#include "nativekit_scene.h"
#include "nativekit_sim.h"
#include "nativekit_sim_mujoco.h"

#include <cassert>
#include <cmath>

namespace {

nkscene_occurrence_id make_occurrence_xyz(nkscene_scene scene, double x, double y, double z) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_occurrence_id occurrence{};
    assert(nkscene_tx_create_occurrence(transaction, &occurrence) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[13] = static_cast<float>(y);
    transform.matrix[14] = static_cast<float>(z);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, occurrence, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return occurrence;
}

nkscene_occurrence_id make_occurrence(nkscene_scene scene, double x) {
    return make_occurrence_xyz(scene, x, 0.0, 0.0);
}

void set_occurrence_x(nkscene_scene scene, nkscene_occurrence_id occurrence, double x) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[15] = 1.0f;
    assert(nkscene_tx_set_transform(transaction, occurrence, &transform) == NKS_OK);
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

nksim_body make_body(nksim_world world, nkscene_occurrence_id occurrence,
                    uint32_t motion_type, double mass, nksim_shape shape = 0) {
    nksim_body_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.occurrence = occurrence;
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
    const auto base_occurrence = make_occurrence(scene, 0.0);
    const auto arm_occurrence = make_occurrence(scene, 1.0);

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
    base_desc.occurrence = base_occurrence;
    base_desc.motion_type = NKSIM_MOTION_STATIC;
    nksim_body base = 0;
    assert(nksim_body_create(world, &base_desc, &base) == NKSIM_OK);

    const double half_extents[] = {0.1, 0.1, 0.5};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    nksim_body_desc arm_desc{};
    arm_desc.struct_size = sizeof(arm_desc);
    arm_desc.occurrence = arm_occurrence;
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
    joint_desc.lower_limit = -0.1;
    joint_desc.upper_limit = 0.1;
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

    for (int index = 0; index < 20; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        assert(step.scene_changes != 0);
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    assert(joint_state.position > 0.0);
    assert(joint_state.position < 0.12);

    nksim_body_state body_state{};
    body_state.struct_size = sizeof(body_state);
    assert(nksim_body_get_state(world, arm, &body_state) == NKSIM_OK);
    assert(std::abs(body_state.position[0] - 1.0) < 1e-6);

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
    const auto base_occurrence = make_occurrence(scene, 0.0);
    const auto slider_occurrence = make_occurrence(scene, 0.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto shape = make_box(world);
    const auto base = make_body(world, base_occurrence, NKSIM_MOTION_STATIC, 0.0);
    const auto slider = make_body(world, slider_occurrence, NKSIM_MOTION_DYNAMIC, 1.0, shape);

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
    const auto base_occurrence = make_occurrence(scene, 0.0);
    const auto child_occurrence = make_occurrence(scene, 1.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);

    const auto shape = make_box(world);
    const auto base = make_body(world, base_occurrence, NKSIM_MOTION_STATIC, 0.0);
    const auto child = make_body(world, child_occurrence, NKSIM_MOTION_DYNAMIC, 1.0, shape);
    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_FIXED;
    joint_desc.body_a = base;
    joint_desc.body_b = child;
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
    const auto occurrence = make_occurrence(scene, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = 0.0;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto body = make_body(world, occurrence, NKSIM_MOTION_KINEMATIC, 1.0);

    set_occurrence_x(scene, occurrence, 3.0);
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
    const auto floor_occurrence = make_occurrence_xyz(scene, 0.0, 0.0, 0.0);
    const auto cube_occurrence = make_occurrence_xyz(scene, 0.0, 0.0, 2.0);

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
    const auto floor = make_body(world, floor_occurrence, NKSIM_MOTION_STATIC, 0.0,
                                  floor_shape);
    const auto cube = make_body(world, cube_occurrence, NKSIM_MOTION_DYNAMIC, 1.0,
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
    const auto occurrence = make_occurrence(scene, 0.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_mujoco_world_create(&world_desc, &world) == NKSIM_OK);
    const auto shape = make_box(world);
    const auto body = make_body(world, occurrence, NKSIM_MOTION_DYNAMIC, 1.0, shape);
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

} // namespace

int main() {
    revolute_joint_is_owned_by_nativekit();
    prismatic_joint_uses_mujoco_velocity_control();
    fixed_joint_rebuilds_and_can_be_removed();
    kinematic_scene_state_drives_mujoco();
    plane_shape_stops_dynamic_body();
    mujoco_replay_is_deterministic();
    return 0;
}
