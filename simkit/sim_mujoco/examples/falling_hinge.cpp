#include "nativekit_scene.h"
#include "nativekit_sim_mujoco.h"

#include <cmath>
#include <cstdio>

namespace {

nkscene_node_id make_node(nkscene_scene scene, double x, double y, double z) {
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK)
        return {};
    nkscene_node_id node{};
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[12] = static_cast<float>(x);
    transform.matrix[13] = static_cast<float>(y);
    transform.matrix[14] = static_cast<float>(z);
    transform.matrix[15] = 1.0f;
    if (nkscene_tx_create_node(transaction, &node) != NKS_OK ||
        nkscene_tx_set_transform(transaction, node, &transform) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return {};
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return {};
    nkscene_change_set_destroy(changes);
    return node;
}

nksim_body make_body(nksim_world world, nkscene_node_id node,
                    uint32_t motion, double mass, nksim_shape shape) {
    nksim_body_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.node = node;
    desc.motion_type = motion;
    desc.mass = mass;
    desc.shape = shape;
    desc.collision_layer = 1;
    desc.collision_mask = 1;
    nksim_body body = 0;
    if (nksim_body_create(world, &desc, &body) != NKSIM_OK)
        return 0;
    return body;
}

} // namespace

int main() {
    nkscene_scene scene = 0;
    if (nkscene_scene_create(&scene) != NKS_OK)
        return 1;

    const auto floor_node = make_node(scene, 0.0, 0.0, 0.0);
    const auto cube_node = make_node(scene, 0.0, 0.0, 3.0);
    const auto hinge_base_node = make_node(scene, 0.0, 0.0, 1.0);
    const auto hinge_arm_node = make_node(scene, 1.0, 0.0, 1.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 1.0 / 120.0;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    if (nksim_mujoco_world_create(&world_desc, &world) != NKSIM_OK)
        return 1;

    const double cube_extents[] = {0.25, 0.25, 0.25};
    const double arm_extents[] = {0.1, 0.1, 0.75};
    nksim_shape floor_shape = 0;
    nksim_shape cube_shape = 0;
    nksim_shape arm_shape = 0;
    const double floor_normal[] = {0.0, 0.0, 1.0};
    if (nksim_shape_create_plane(world, floor_normal, 0.0, &floor_shape) != NKSIM_OK ||
        nksim_shape_create_box(world, cube_extents, &cube_shape) != NKSIM_OK ||
        nksim_shape_create_box(world, arm_extents, &arm_shape) != NKSIM_OK) {
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return 1;
    }

    const auto floor = make_body(world, floor_node, NKSIM_MOTION_STATIC, 0.0,
                                 floor_shape);
    const auto cube = make_body(world, cube_node, NKSIM_MOTION_DYNAMIC, 1.0,
                                cube_shape);
    const auto hinge_base = make_body(world, hinge_base_node, NKSIM_MOTION_STATIC,
                                      0.0, 0);
    const auto hinge_arm = make_body(world, hinge_arm_node, NKSIM_MOTION_DYNAMIC,
                                     1.0, arm_shape);
    if (!floor || !cube || !hinge_base || !hinge_arm) {
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return 1;
    }

    nksim_joint_desc joint_desc{};
    joint_desc.struct_size = sizeof(joint_desc);
    joint_desc.type = NKSIM_JOINT_REVOLUTE;
    joint_desc.body_a = hinge_base;
    joint_desc.body_b = hinge_arm;
    joint_desc.axis_a[2] = 1.0;
    joint_desc.lower_limit = -1.0;
    joint_desc.upper_limit = 1.0;
    joint_desc.max_force = 10.0;
    nksim_joint joint = 0;
    if (nksim_joint_create(world, &joint_desc, &joint) != NKSIM_OK) {
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return 1;
    }

    nksim_joint_target target{};
    target.struct_size = sizeof(target);
    target.joint = joint;
    target.mode = NKSIM_JOINT_TARGET_POSITION;
    target.target = 0.35;
    target.max_force = 10.0;
    if (nksim_world_set_joint_targets(world, &target, 1) != NKSIM_OK)
        return 1;

    for (int index = 0; index < 240; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        if (nksim_world_step(world, &step) != NKSIM_OK)
            return 1;
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_body_state cube_state{};
    cube_state.struct_size = sizeof(cube_state);
    nksim_joint_state joint_state{};
    joint_state.struct_size = sizeof(joint_state);
    const auto body_result = nksim_body_get_state(world, cube, &cube_state);
    const auto joint_result = nksim_joint_get_state(world, joint, &joint_state);
    if (body_result != NKSIM_OK || joint_result != NKSIM_OK)
        return 1;
    std::printf("cube z = %.3f, hinge position = %.3f\n",
                cube_state.position[2], joint_state.position);

    nksim_joint_destroy(world, joint);
    nksim_body_destroy(world, hinge_arm);
    nksim_body_destroy(world, hinge_base);
    nksim_body_destroy(world, cube);
    nksim_body_destroy(world, floor);
    nksim_shape_destroy(world, arm_shape);
    nksim_shape_destroy(world, cube_shape);
    nksim_shape_destroy(world, floor_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return std::isfinite(cube_state.position[2]) && std::isfinite(joint_state.position) ? 0 : 1;
}
