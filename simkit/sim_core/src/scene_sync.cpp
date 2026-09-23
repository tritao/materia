#include "internal.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace nksim {
namespace {

nkscene_transform transform_from_state(const nksim_body_state &state) noexcept {
    const double x = state.rotation[0];
    const double y = state.rotation[1];
    const double z = state.rotation[2];
    const double w = state.rotation[3];

    nkscene_transform transform{};
    transform.matrix[0] = static_cast<float>(1.0 - 2.0 * (y * y + z * z));
    transform.matrix[1] = static_cast<float>(2.0 * (x * y + z * w));
    transform.matrix[2] = static_cast<float>(2.0 * (x * z - y * w));
    transform.matrix[3] = 0.0f;
    transform.matrix[4] = static_cast<float>(2.0 * (x * y - z * w));
    transform.matrix[5] = static_cast<float>(1.0 - 2.0 * (x * x + z * z));
    transform.matrix[6] = static_cast<float>(2.0 * (y * z + x * w));
    transform.matrix[7] = 0.0f;
    transform.matrix[8] = static_cast<float>(2.0 * (x * z + y * w));
    transform.matrix[9] = static_cast<float>(2.0 * (y * z - x * w));
    transform.matrix[10] = static_cast<float>(1.0 - 2.0 * (x * x + y * y));
    transform.matrix[11] = 0.0f;
    transform.matrix[12] = static_cast<float>(state.position[0]);
    transform.matrix[13] = static_cast<float>(state.position[1]);
    transform.matrix[14] = static_cast<float>(state.position[2]);
    transform.matrix[15] = 1.0f;
    return transform;
}

bool world_to_local(const nkscene_transform &parent, const nkscene_transform &world,
                   nkscene_transform &local) noexcept {
    const double a00 = parent.matrix[0];
    const double a01 = parent.matrix[4];
    const double a02 = parent.matrix[8];
    const double a10 = parent.matrix[1];
    const double a11 = parent.matrix[5];
    const double a12 = parent.matrix[9];
    const double a20 = parent.matrix[2];
    const double a21 = parent.matrix[6];
    const double a22 = parent.matrix[10];
    const double determinant = a00 * (a11 * a22 - a12 * a21) -
        a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-12)
        return false;

    const double inverse = 1.0 / determinant;
    const double i00 = (a11 * a22 - a12 * a21) * inverse;
    const double i01 = (a02 * a21 - a01 * a22) * inverse;
    const double i02 = (a01 * a12 - a02 * a11) * inverse;
    const double i10 = (a12 * a20 - a10 * a22) * inverse;
    const double i11 = (a00 * a22 - a02 * a20) * inverse;
    const double i12 = (a02 * a10 - a00 * a12) * inverse;
    const double i20 = (a10 * a21 - a11 * a20) * inverse;
    const double i21 = (a01 * a20 - a00 * a21) * inverse;
    const double i22 = (a00 * a11 - a01 * a10) * inverse;

    const double parent_x = parent.matrix[12];
    const double parent_y = parent.matrix[13];
    const double parent_z = parent.matrix[14];
    const double inverse_tx = -(i00 * parent_x + i01 * parent_y + i02 * parent_z);
    const double inverse_ty = -(i10 * parent_x + i11 * parent_y + i12 * parent_z);
    const double inverse_tz = -(i20 * parent_x + i21 * parent_y + i22 * parent_z);

    const double world_x[3] = {world.matrix[0], world.matrix[1], world.matrix[2]};
    const double world_y[3] = {world.matrix[4], world.matrix[5], world.matrix[6]};
    const double world_z[3] = {world.matrix[8], world.matrix[9], world.matrix[10]};
    const double world_translation[3] = {world.matrix[12], world.matrix[13], world.matrix[14]};
    const double inverse_rows[3][3] = {
        {i00, i01, i02},
        {i10, i11, i12},
        {i20, i21, i22},
    };
    const double world_columns[3][3] = {
        {world_x[0], world_x[1], world_x[2]},
        {world_y[0], world_y[1], world_y[2]},
        {world_z[0], world_z[1], world_z[2]},
    };

    local = {};
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            double value = 0.0;
            for (int index = 0; index < 3; ++index)
                value += inverse_rows[row][index] * world_columns[column][index];
            local.matrix[column * 4 + row] = static_cast<float>(value);
        }
    }
    local.matrix[12] = static_cast<float>(
        i00 * world_translation[0] + i01 * world_translation[1] + i02 * world_translation[2] +
        inverse_tx);
    local.matrix[13] = static_cast<float>(
        i10 * world_translation[0] + i11 * world_translation[1] + i12 * world_translation[2] +
        inverse_ty);
    local.matrix[14] = static_cast<float>(
        i20 * world_translation[0] + i21 * world_translation[1] + i22 * world_translation[2] +
        inverse_tz);
    local.matrix[3] = 0.0f;
    local.matrix[7] = 0.0f;
    local.matrix[11] = 0.0f;
    local.matrix[15] = 1.0f;
    return true;
}

} // namespace

nksim_result World::synchronize_scene(nkscene_change_set *out_changes) {
    if (!out_changes)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_changes = 0;
    nkscene_transaction transaction = 0;
    if (nkscene_transaction_begin(world_desc.scene, &transaction) != NKS_OK)
        return NKSIM_ERROR_SCENE;

    nkscene_snapshot scene_snapshot = 0;
    if (nkscene_scene_snapshot(world_desc.scene, &scene_snapshot) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        return NKSIM_ERROR_SCENE;
    }
    std::uint64_t node_count = 0;
    if (nkscene_snapshot_get_node_count(scene_snapshot, &node_count) != NKS_OK) {
        nkscene_snapshot_destroy(scene_snapshot);
        nkscene_transaction_cancel(transaction);
        return NKSIM_ERROR_SCENE;
    }
    std::unordered_map<std::uint64_t, nkscene_snapshot_node> nodes;
    nodes.reserve(static_cast<std::size_t>(node_count));
    for (std::uint64_t index = 0; index < node_count; ++index) {
        nkscene_snapshot_node node{};
        node.struct_size = sizeof(node);
        if (nkscene_snapshot_get_node(scene_snapshot, index, &node) != NKS_OK) {
            nkscene_snapshot_destroy(scene_snapshot);
            nkscene_transaction_cancel(transaction);
            return NKSIM_ERROR_SCENE;
        }
        nodes.emplace(node.node.value, node);
    }

    std::unordered_map<std::uint64_t, nkscene_transform> dynamic_world_transforms;
    bodies.for_each([&](nksim_body, const Body &body) {
        if (body.desc.motion_type == NKSIM_MOTION_DYNAMIC)
            dynamic_world_transforms.emplace(body.desc.node.value,
                                             transform_from_state(body.state));
    });

    nksim_result result = NKSIM_OK;
    bodies.for_each([&](nksim_body, const Body &body) {
        if (result != NKSIM_OK || body.desc.motion_type != NKSIM_MOTION_DYNAMIC)
            return;
        const auto node = nodes.find(body.desc.node.value);
        if (node == nodes.end()) {
            result = NKSIM_ERROR_SCENE;
            return;
        }

        nkscene_transform transform = transform_from_state(body.state);
        if (node->second.parent.value) {
            nkscene_transform parent_world{};
            const auto dynamic_parent = dynamic_world_transforms.find(
                node->second.parent.value);
            if (dynamic_parent != dynamic_world_transforms.end()) {
                parent_world = dynamic_parent->second;
            } else {
                const auto parent = nodes.find(node->second.parent.value);
                if (parent == nodes.end()) {
                    result = NKSIM_ERROR_SCENE;
                    return;
                }
                parent_world = parent->second.world_transform;
            }
            if (!world_to_local(parent_world, transform, transform)) {
                result = NKSIM_ERROR_SCENE;
                return;
            }
        }
        if (nkscene_tx_set_transform(transaction, body.desc.node, &transform) != NKS_OK)
            result = NKSIM_ERROR_SCENE;
    });
    nkscene_snapshot_destroy(scene_snapshot);
    if (result != NKSIM_OK) {
        nkscene_transaction_cancel(transaction);
        return result;
    }
    if (nkscene_transaction_commit_with_changes(transaction, out_changes) != NKS_OK) {
        nkscene_transaction_cancel(transaction);
        *out_changes = 0;
        return NKSIM_ERROR_SCENE;
    }
    return NKSIM_OK;
}

} // namespace nksim
