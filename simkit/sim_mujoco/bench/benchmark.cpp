#include "nativekit_scene.h"
#include "nativekit_sim_mujoco.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

std::vector<nkscene_node_id> make_box_nodes(nkscene_scene scene,
                                                        int count) {
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK)
        return {};
    std::vector<nkscene_node_id> result;
    result.reserve(count);
    for (int index = 0; index < count; ++index) {
        nkscene_node_id node{};
        if (nkscene_tx_create_node(transaction, &node) != NKS_OK)
            return {};
        const int row = index / 8;
        const int column = index % 8;
        nkscene_transform transform{};
        transform.matrix[0] = 1.0f;
        transform.matrix[5] = 1.0f;
        transform.matrix[10] = 1.0f;
        transform.matrix[12] = static_cast<float>((column - 3.5) * 0.55);
        transform.matrix[13] = static_cast<float>((row % 2) * 0.275);
        transform.matrix[14] = static_cast<float>(0.3 + row * 0.55);
        transform.matrix[15] = 1.0f;
        if (nkscene_tx_set_transform(transaction, node, &transform) != NKS_OK)
            return {};
        result.push_back(node);
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return {};
    nkscene_change_set_destroy(changes);
    return result;
}

nksim_body create_body(nksim_world world, nkscene_node_id node,
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
    return nksim_body_create(world, &desc, &body) == NKSIM_OK ? body : 0;
}

} // namespace

int main() {
    constexpr int box_count = 64;
    constexpr int step_count = 1000;
    nkscene_scene scene = 0;
    if (nkscene_scene_create(&scene) != NKS_OK)
        return 1;
    const auto nodes = make_box_nodes(scene, box_count);
    if (nodes.size() != box_count)
        return 1;

    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK)
        return 1;
    nkscene_node_id floor_node{};
    if (nkscene_tx_create_node(transaction, &floor_node) != NKS_OK)
        return 1;
    nkscene_transform floor_transform{};
    floor_transform.matrix[0] = 1.0f;
    floor_transform.matrix[5] = 1.0f;
    floor_transform.matrix[10] = 1.0f;
    floor_transform.matrix[15] = 1.0f;
    if (nkscene_tx_set_transform(transaction, floor_node, &floor_transform) != NKS_OK)
        return 1;
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return 1;
    nkscene_change_set_destroy(changes);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = scene;
    world_desc.fixed_timestep = 1.0 / 120.0;
    world_desc.physics_substeps = 2;
    world_desc.gravity[2] = -9.81;
    nksim_world world = 0;
    if (nksim_mujoco_world_create(&world_desc, &world) != NKSIM_OK)
        return 1;

    const double plane_normal[] = {0.0, 0.0, 1.0};
    const double half_extents[] = {0.25, 0.25, 0.25};
    nksim_shape floor_shape = 0;
    nksim_shape box_shape = 0;
    if (nksim_shape_create_plane(world, plane_normal, 0.0, &floor_shape) != NKSIM_OK ||
        nksim_shape_create_box(world, half_extents, &box_shape) != NKSIM_OK)
        return 1;
    const auto floor = create_body(world, floor_node, NKSIM_MOTION_STATIC,
                                   0.0, floor_shape);
    std::vector<nksim_body> boxes;
    boxes.reserve(box_count);
    for (const auto node : nodes)
        boxes.push_back(create_body(world, node, NKSIM_MOTION_DYNAMIC, 1.0,
                                    box_shape));
    if (!floor || boxes.size() != box_count)
        return 1;

    const auto start = std::chrono::steady_clock::now();
    for (int index = 0; index < step_count; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        if (nksim_world_step(world, &step) != NKSIM_OK)
            return 1;
        nkscene_change_set_destroy(step.scene_changes);
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

    nksim_body_state state{};
    state.struct_size = sizeof(state);
    if (nksim_body_get_state(world, boxes.back(), &state) != NKSIM_OK)
        return 1;
    std::printf("MuJoCo: %d bodies x %d steps: %lld us (%.2f us/step), top z = %.3f\n",
                box_count, step_count, static_cast<long long>(micros),
                static_cast<double>(micros) / step_count, state.position[2]);

    for (auto body : boxes)
        nksim_body_destroy(world, body);
    nksim_body_destroy(world, floor);
    nksim_shape_destroy(world, box_shape);
    nksim_shape_destroy(world, floor_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return std::isfinite(state.position[2]) ? 0 : 1;
}
