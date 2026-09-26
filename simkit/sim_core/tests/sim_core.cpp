#include "nativekit_scene.h"
#include "nativekit_sim.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

namespace {

nkscene_transform make_transform(double x, double y, double z) {
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[15] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[13] = static_cast<float>(y);
    transform.matrix[14] = static_cast<float>(z);
    return transform;
}

nkscene_node_id make_node(nkscene_scene scene, double z) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_node_id node{};
    assert(nkscene_tx_create_node(transaction, &node) == NKS_OK);
    const auto transform = make_transform(0.0, 0.0, z);
    assert(nkscene_tx_set_transform(transaction, node, &transform) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return node;
}

struct NestedNodes {
    nkscene_node_id parent{};
    nkscene_node_id child{};
};

NestedNodes make_nested_nodes(nkscene_scene scene) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    NestedNodes result;
    assert(nkscene_tx_create_node(transaction, &result.parent) == NKS_OK);
    assert(nkscene_tx_create_node(transaction, &result.child) == NKS_OK);
    const auto parent_transform = make_transform(0.0, 0.0, 5.0);
    const auto child_transform = make_transform(1.0, 2.0, 3.0);
    assert(nkscene_tx_set_transform(transaction, result.parent, &parent_transform) == NKS_OK);
    assert(nkscene_tx_set_transform(transaction, result.child, &child_transform) == NKS_OK);
    assert(nkscene_tx_set_parent(transaction, result.child, result.parent) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return result;
}

void falling_body_updates_scene_and_snapshot() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 10.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&world_desc, &world) == NKSIM_OK);
    assert(nksim_world_begin_topology_update(world) == NKSIM_OK);
    assert(nksim_world_begin_topology_update(world) == NKSIM_ERROR_INVALID_STATE);
    nksim_step_result blocked_step{};
    blocked_step.struct_size = sizeof(blocked_step);
    assert(nksim_world_step(world, &blocked_step) == NKSIM_ERROR_INVALID_STATE);
    assert(nksim_world_end_topology_update(world) == NKSIM_OK);
    assert(nksim_world_end_topology_update(world) == NKSIM_ERROR_INVALID_STATE);

    const double half_extents[] = {0.5, 0.5, 0.5};
    nksim_shape shape = 0;
    assert(nksim_shape_create_box(world, half_extents, &shape) == NKSIM_OK);
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = node;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 2.0;
    body_desc.shape = shape;
    body_desc.collision_layer = 1;
    body_desc.collision_mask = 1;
    body_desc.has_inertial_properties = 1;
    body_desc.center_of_mass[0] = 0.1;
    body_desc.inertia_tensor[0] = 1.0;
    body_desc.inertia_tensor[4] = 2.0;
    body_desc.inertia_tensor[8] = 3.0;
    nksim_body body = 0;
    assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);
    auto invalid_inertia = body_desc;
    invalid_inertia.inertia_tensor[8] = 0.0;
    nksim_body invalid_body = 0;
    assert(nksim_body_create(world, &invalid_inertia, &invalid_body) == NKSIM_ERROR_INVALID_ARGUMENT);
    auto small_robot_inertia = body_desc;
    small_robot_inertia.inertia_tensor[0] = 1e-12;
    small_robot_inertia.inertia_tensor[4] = 2e-12;
    small_robot_inertia.inertia_tensor[8] = 3e-12;
    nksim_body small_inertia_body = 0;
    assert(nksim_body_create(world, &small_robot_inertia, &small_inertia_body) == NKSIM_OK);
    nksim_body_destroy(world, small_inertia_body);

    nksim_body_force force{};
    force.struct_size = sizeof(force);
    force.body = body;
    force.force[0] = 2.0;
    assert(nksim_world_apply_forces(world, &force, 1) == NKSIM_OK);

    for (int index = 0; index < 100; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        assert(step.step_index == static_cast<uint64_t>(index + 1));
        assert(std::abs(step.simulation_time - (index + 1) * 0.01) < 1e-12);
        assert(step.physics_substeps == 2);
        assert(step.scene_changes != 0);
        uint64_t revision = 0;
        assert(nkscene_change_set_get_revision(step.scene_changes, &revision) == NKS_OK);
        assert(revision >= 2);
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_OK);
    assert(clock.step_index == 100);
    assert(std::abs(clock.time - 1.0) < 1e-12);

    nksim_body_state body_state{};
    body_state.struct_size = sizeof(body_state);
    assert(nksim_body_get_state(world, body, &body_state) == NKSIM_OK);
    assert(body_state.position[2] < 10.0);
    assert(body_state.linear_velocity[2] < 0.0);

    nkscene_snapshot scene_snapshot = 0;
    assert(nkscene_scene_snapshot(scene, &scene_snapshot) == NKS_OK);
    nkscene_snapshot_node node_state{};
    node_state.struct_size = sizeof(node_state);
    assert(nkscene_snapshot_get_node(scene_snapshot, 0, &node_state) == NKS_OK);
    assert(std::abs(node_state.world_transform.matrix[14] - body_state.position[2]) < 1e-5);
    nkscene_snapshot_destroy(scene_snapshot);

    nksim_snapshot snapshot = 0;
    assert(nksim_world_snapshot(world, &snapshot) == NKSIM_OK);
    uint64_t body_count = 0;
    assert(nksim_snapshot_get_body_count(snapshot, &body_count) == NKSIM_OK);
    assert(body_count == 1);
    nksim_snapshot_body_page page{};
    page.struct_size = sizeof(page);
    assert(nksim_snapshot_get_body_page(snapshot, 0, &page) == NKSIM_OK);
    assert(page.count == 1);
    assert(page.bodies[0].body == body);
    nksim_snapshot_destroy(snapshot);

    nksim_body_destroy(world, body);
    nksim_shape_destroy(world, shape);
    nksim_world_destroy(world);
    assert(nkscene_scene_snapshot(scene, &scene_snapshot) == NKS_OK);
    nkscene_snapshot_destroy(scene_snapshot);
    nkscene_scene_destroy(scene);
}

void nested_dynamic_body_updates_local_transform() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto nodes = make_nested_nodes(scene);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.1;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -10.0;
    nksim_world world = 0;
    assert(nksim_world_create(&world_desc, &world) == NKSIM_OK);

    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = nodes.child;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 1.0;
    nksim_body body = 0;
    assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);

    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(world, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);

    nksim_body_state body_state{};
    body_state.struct_size = sizeof(body_state);
    assert(nksim_body_get_state(world, body, &body_state) == NKSIM_OK);
    assert(std::abs(body_state.position[2] - 7.9) < 1e-12);

    nkscene_snapshot snapshot = 0;
    assert(nkscene_scene_snapshot(scene, &snapshot) == NKS_OK);
    uint64_t count = 0;
    assert(nkscene_snapshot_get_node_count(snapshot, &count) == NKS_OK);
    nkscene_snapshot_node parent_state{};
    nkscene_snapshot_node child_state{};
    parent_state.struct_size = sizeof(parent_state);
    child_state.struct_size = sizeof(child_state);
    for (uint64_t index = 0; index < count; ++index) {
        nkscene_snapshot_node state{};
        state.struct_size = sizeof(state);
        assert(nkscene_snapshot_get_node(snapshot, index, &state) == NKS_OK);
        if (state.node.value == nodes.parent.value)
            parent_state = state;
        if (state.node.value == nodes.child.value)
            child_state = state;
    }
    assert(std::abs(parent_state.world_transform.matrix[14] - 5.0f) < 1e-5f);
    assert(std::abs(child_state.world_transform.matrix[14] - 7.9f) < 1e-5f);
    assert(std::abs(child_state.local_transform.matrix[12] - 1.0f) < 1e-5f);
    assert(std::abs(child_state.local_transform.matrix[13] - 2.0f) < 1e-5f);
    assert(std::abs(child_state.local_transform.matrix[14] - 2.9f) < 1e-5f);
    nkscene_snapshot_destroy(snapshot);

    nksim_body_destroy(world, body);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

struct ReplayResult {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    std::array<float, 16> scene_transform{};
    std::uint64_t step_index = 0;
    double time = 0.0;
};

ReplayResult run_replay() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto node = make_node(scene, 10.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 2;
    world_desc.gravity[0] = 1.5;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&world_desc, &world) == NKSIM_OK);

    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = node;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 2.0;
    nksim_body body = 0;
    assert(nksim_body_create(world, &body_desc, &body) == NKSIM_OK);
    for (int index = 0; index < 200; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        nkscene_change_set_destroy(step.scene_changes);
    }

    ReplayResult result;
    nksim_body_state body_state{};
    body_state.struct_size = sizeof(body_state);
    assert(nksim_body_get_state(world, body, &body_state) == NKSIM_OK);
    std::copy(std::begin(body_state.position), std::end(body_state.position), result.position.begin());
    std::copy(std::begin(body_state.linear_velocity), std::end(body_state.linear_velocity),
              result.velocity.begin());
    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    assert(nksim_world_get_clock(world, &clock) == NKSIM_OK);
    result.step_index = clock.step_index;
    result.time = clock.time;

    nkscene_snapshot snapshot = 0;
    assert(nkscene_scene_snapshot(scene, &snapshot) == NKS_OK);
    nkscene_snapshot_node state{};
    state.struct_size = sizeof(state);
    assert(nkscene_snapshot_get_node(snapshot, 0, &state) == NKS_OK);
    std::copy(std::begin(state.world_transform.matrix), std::end(state.world_transform.matrix),
              result.scene_transform.begin());
    nkscene_snapshot_destroy(snapshot);

    nksim_body_destroy(world, body);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return result;
}

void repeated_replays_are_identical() {
    const auto first = run_replay();
    const auto second = run_replay();
    assert(first.position == second.position);
    assert(first.velocity == second.velocity);
    assert(first.scene_transform == second.scene_transform);
    assert(first.step_index == second.step_index);
    assert(first.time == second.time);
}

void batched_joint_targets_are_accepted() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto first = make_node(scene, 0.0);
    const auto second = make_node(scene, 1.0);
    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&world_desc, &world) == NKSIM_OK);

    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.motion_type = NKSIM_MOTION_KINEMATIC;
    body_desc.mass = 1.0;
    body_desc.node = first;
    nksim_body body_a = 0;
    assert(nksim_body_create(world, &body_desc, &body_a) == NKSIM_OK);
    body_desc.node = second;
    nksim_body body_b = 0;
    assert(nksim_body_create(world, &body_desc, &body_b) == NKSIM_OK);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = body_a;
    joint_desc.body_b = body_b;
    joint_desc.axis_a[2] = 1.0;
    joint_desc.max_force = 10.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 0.5;
    target.max_force = 10.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);
    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(world, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);
    nksim_joint_state state{};
    state.struct_size = sizeof(state);
    assert(nksim_joint_get_state(world, joint, &state) == NKSIM_OK);
    assert(std::abs(state.position - 0.5) < 1e-12);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, body_a);
    nksim_body_destroy(world, body_b);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

void joint_child_bodies_follow_their_joint_in_default_backend() {
    // F4: the default (test) backend used to track joint positions as
    // numbers only; a joint-connected DYNAMIC body never moved with its
    // joint and instead free-fell under gravity like an unconnected body.
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);
    const auto base_node = make_node(scene, 0.0);
    const auto arm_node = make_node(scene, 1.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    assert(nksim_world_create(&world_desc, &world) == NKSIM_OK);

    nksim_body_desc base_desc{};
    base_desc.struct_size = sizeof(base_desc);
    base_desc.node = base_node;
    base_desc.motion_type = NKSIM_MOTION_STATIC;
    nksim_body base = 0;
    assert(nksim_body_create(world, &base_desc, &base) == NKSIM_OK);

    nksim_body_desc arm_desc{};
    arm_desc.struct_size = sizeof(arm_desc);
    arm_desc.node = arm_node;
    arm_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    arm_desc.mass = 1.0;
    nksim_body arm = 0;
    assert(nksim_body_create(world, &arm_desc, &arm) == NKSIM_OK);

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = base;
    joint_desc.body_b = arm;
    joint_desc.axis_a[2] = 1.0;
    joint_desc.anchor_b[0] = -1.0; // Pivot at the base's origin; the arm's rest pose is 1m out.
    joint_desc.max_force = 1000.0;
    nksim_joint joint = 0;
    assert(nksim_joint_create(world, &joint_desc, &joint) == NKSIM_OK);

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 1.5707963267948966; // pi/2
    target.max_force = 1000.0;
    assert(nksim_world_set_joint_targets(world, &target, 1) == NKSIM_OK);

    for (int index = 0; index < 5; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        assert(nksim_world_step(world, &step) == NKSIM_OK);
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    assert(nksim_joint_get_state(world, joint, &joint_state) == NKSIM_OK);
    nksim_body_state arm_state{};
    arm_state.struct_size = sizeof(arm_state);
    assert(nksim_body_get_state(world, arm, &arm_state) == NKSIM_OK);
    // The arm swings with the joint, from the base's pivot, and does not
    // free-fall independently under gravity.
    assert(std::abs(arm_state.position[0] - std::cos(joint_state.position)) < 1e-6);
    assert(std::abs(arm_state.position[1] - std::sin(joint_state.position)) < 1e-6);
    assert(std::abs(arm_state.position[2]) < 1e-9);

    // Teleporting the (static) root carries the arm along with it.
    nksim_body_state base_state{};
    base_state.struct_size = sizeof(base_state);
    assert(nksim_body_get_state(world, base, &base_state) == NKSIM_OK);
    base_state.position[0] = 5.0;
    base_state.position[1] = 2.0;
    assert(nksim_body_set_state(world, base, &base_state) == NKSIM_OK);
    assert(nksim_body_get_state(world, arm, &arm_state) == NKSIM_OK);
    assert(std::abs(arm_state.position[0] - (5.0 + std::cos(joint_state.position))) < 1e-6);
    assert(std::abs(arm_state.position[1] - (2.0 + std::sin(joint_state.position))) < 1e-6);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, arm);
    nksim_body_destroy(world, base);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
}

} // namespace

int main() {
    falling_body_updates_scene_and_snapshot();
    nested_dynamic_body_updates_local_transform();
    repeated_replays_are_identical();
    batched_joint_targets_are_accepted();
    joint_child_bodies_follow_their_joint_in_default_backend();
    return 0;
}
