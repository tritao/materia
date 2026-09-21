#include "nativekit_scene.h"
#include "nativekit_sim_mujoco.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

std::vector<nkscene_occurrence_id> make_occurrences(nkscene_scene scene,
                                                    int count) {
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(scene, &transaction) != NKS_OK)
        return {};
    std::vector<nkscene_occurrence_id> result;
    result.reserve(count);
    for (int index = 0; index < count; ++index) {
        nkscene_occurrence_id occurrence{};
        if (nkscene_tx_create_occurrence(transaction, &occurrence) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            return {};
        }
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
        if (nkscene_tx_set_transform(transaction, occurrence, &transform) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            return {};
        }
        result.push_back(occurrence);
    }
    nkscene_change_set changes = 0;
    if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
        return {};
    nkscene_change_set_destroy(changes);
    return result;
}

nksim_body create_body(nksim_world world, nkscene_occurrence_id occurrence,
                      uint32_t motion, double mass, nksim_shape shape) {
    nksim_body_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.occurrence = occurrence;
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
    constexpr int box_count = 32;
    nkscene_scene scene = 0;
    if (nkscene_scene_create(&scene) != NKS_OK)
        return 1;

    const auto floor_occurrence = make_occurrences(scene, 1);
    const auto box_occurrences = make_occurrences(scene, box_count);
    if (floor_occurrence.size() != 1 || box_occurrences.size() != box_count)
        return 1;

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
    if (nksim_shape_create_plane(world, plane_normal, -0.3, &floor_shape) != NKSIM_OK ||
        nksim_shape_create_box(world, half_extents, &box_shape) != NKSIM_OK)
        return 1;

    const auto floor = create_body(world, floor_occurrence[0], NKSIM_MOTION_STATIC,
                                   0.0, floor_shape);
    std::vector<nksim_body> boxes;
    boxes.reserve(box_count);
    for (const auto occurrence : box_occurrences)
        boxes.push_back(create_body(world, occurrence, NKSIM_MOTION_DYNAMIC, 1.0,
                                    box_shape));
    if (!floor || boxes.size() != box_count)
        return 1;

    for (int index = 0; index < 600; ++index) {
        nksim_step_result step{};
        step.struct_size = sizeof(step);
        if (nksim_world_step(world, &step) != NKSIM_OK)
            return 1;
        nkscene_change_set_destroy(step.scene_changes);
    }

    nksim_body_state top_state{};
    top_state.struct_size = sizeof(top_state);
    if (nksim_body_get_state(world, boxes.back(), &top_state) != NKSIM_OK)
        return 1;
    std::printf("boxes = %d, top z = %.3f\n", box_count, top_state.position[2]);

    for (auto body : boxes)
        nksim_body_destroy(world, body);
    nksim_body_destroy(world, floor);
    nksim_shape_destroy(world, box_shape);
    nksim_shape_destroy(world, floor_shape);
    nksim_world_destroy(world);
    nkscene_scene_destroy(scene);
    return std::isfinite(top_state.position[2]) ? 0 : 1;
}
