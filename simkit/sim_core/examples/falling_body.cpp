#include "nativekit_scene.h"
#include "nativekit_sim.h"

#include <cstdio>

int main() {
    nkscene_scene scene = 0;
    if (nkscene_scene_create(&scene) != NKS_OK)
        return 1;

    nkscene_transaction transaction = 0;
    nkscene_node_id node{};
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK ||
        nkscene_tx_create_node(transaction, &node) != NKS_OK) {
        nkscene_scene_destroy(scene);
        return 1;
    }
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[14] = 2.0f;
    transform.matrix[15] = 1.0f;
    if (nkscene_tx_set_transform(transaction, node, &transform) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        nkscene_scene_destroy(scene);
        return 1;
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        nkscene_scene_destroy(scene);
        return 1;
    }
    nkscene_change_set_destroy(changes);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 1.0 / 60.0;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    if (nksim_world_create(&world_desc, &world) != NKSIM_OK) {
        nkscene_scene_destroy(scene);
        return 1;
    }

    const double half_extents[] = {0.5, 0.5, 0.5};
    nksim_shape shape = 0;
    nksim_body body = 0;
    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.node = node;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 1.0;
    body_desc.collision_layer = 1;
    body_desc.collision_mask = 1;
    if (nksim_shape_create_box(world, half_extents, &shape) != NKSIM_OK ||
        (body_desc.shape = shape, nksim_body_create(world, &body_desc, &body)) != NKSIM_OK) {
        nksim_world_destroy(world);
        nkscene_scene_destroy(scene);
        return 1;
    }

    for (int index = 0; index < 120; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        if (nksim_world_step(world, &step) != NKSIM_OK) {
            nksim_world_destroy(world);
            nkscene_scene_destroy(scene);
            return 1;
        }
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_body_state state{};
    state.struct_size = sizeof(state);
    const auto result = nksim_body_get_state(world, body, &state);
    if (result == NKSIM_OK)
        std::printf("sim time: %.3f, body z: %.6f\n", 120.0 / 60.0, state.position[2]);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return result == NKSIM_OK ? 0 : 1;
}
