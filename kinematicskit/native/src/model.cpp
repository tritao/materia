#include "model.hpp"

#include <cmath>
#include <mutex>
#include <new>

namespace kk {

namespace {

bool finite(const double *values, size_t count) {
    for (size_t i = 0; i < count; ++i)
        if (!std::isfinite(values[i])) return false;
    return true;
}

// Handle layout: a 12-bit generation above a 20-bit slot. Slot 0 is never
// used, so a zero handle is always invalid.
constexpr uint32_t kSlotBits = 20;
constexpr uint32_t kSlotMask = (1u << kSlotBits) - 1;
constexpr uint32_t kGenerationMask = 0xFFF;

struct Entry {
    std::unique_ptr<Model> model;
    uint32_t generation = 1;
};

std::mutex table_mutex;
std::vector<Entry> table(1);
std::vector<uint32_t> free_slots;

} // namespace

bool Model::build(const int32_t *ints, uint32_t int_count, const double *reals, uint32_t real_count) {
    if (!ints || !reals || int_count < 5 || ints[0] != 1) return false;
    const int64_t bodies = ints[1], joints = ints[2], dofs = ints[3], frames = ints[4];
    if (bodies < 0 || joints < 0 || dofs < 0 || frames < 0 || bodies > 1000000 || joints > 1000000 ||
        frames > 1000000 || dofs > joints)
        return false;
    const uint64_t expected_ints = 5 + 2 * uint64_t(bodies) + 7 * uint64_t(joints) + uint64_t(frames);
    const uint64_t expected_reals = 7 * uint64_t(bodies) + 20 * uint64_t(joints) + 7 * uint64_t(frames);
    if (int_count != expected_ints || real_count != expected_reals || !finite(reals, real_count)) return false;
    body_count = uint32_t(bodies);
    joint_count = uint32_t(joints);
    dof_count = uint32_t(dofs);
    frame_count = uint32_t(frames);

    size_t i = 5;
    auto body_ok = [&](int32_t b) { return b >= 0 && b < bodies; };
    auto joint_ok = [&](int32_t j) { return j >= 0 && j < joints; };
    body_parent_joint.assign(ints + i, ints + i + bodies); i += bodies;
    body_order.assign(ints + i, ints + i + bodies); i += bodies;
    joint_kind.resize(joints); joint_parent.resize(joints); joint_child.resize(joints);
    joint_dof.resize(joints); joint_source.resize(joints);
    for (int64_t j = 0; j < joints; ++j) {
        joint_kind[j] = ints[i++]; joint_parent[j] = ints[i++]; joint_child[j] = ints[i++];
        joint_dof[j] = ints[i++]; joint_source[j] = ints[i++];
    }
    joint_order.assign(ints + i, ints + i + joints); i += joints;
    joint_value_order.assign(ints + i, ints + i + joints); i += joints;
    frame_body.assign(ints + i, ints + i + frames); i += frames;

    for (int64_t b = 0; b < bodies; ++b) {
        if (body_parent_joint[b] != -1 && !joint_ok(body_parent_joint[b])) return false;
        if (!body_ok(body_order[b])) return false;
    }
    for (int64_t j = 0; j < joints; ++j) {
        if (joint_kind[j] < 0 || joint_kind[j] > 2 || !body_ok(joint_parent[j]) || !body_ok(joint_child[j]) ||
            joint_dof[j] < -1 || joint_dof[j] >= dofs || (joint_kind[j] != 0 && joint_dof[j] < 0) ||
            (joint_source[j] != -1 && !joint_ok(joint_source[j])) || !joint_ok(joint_order[j]) ||
            !joint_ok(joint_value_order[j]))
            return false;
    }
    for (int64_t f = 0; f < frames; ++f) if (!body_ok(frame_body[f])) return false;

    size_t r = 0;
    root_pose.assign(reals + r, reals + r + 7 * bodies); r += 7 * bodies;
    parent_t_joint.resize(7 * joints); joint_t_child.resize(7 * joints); axis.resize(3 * joints);
    ratio.resize(joints); offset.resize(joints); scale.resize(joints);
    for (int64_t j = 0; j < joints; ++j) {
        for (int k = 0; k < 7; ++k) parent_t_joint[7 * j + k] = reals[r++];
        for (int k = 0; k < 7; ++k) joint_t_child[7 * j + k] = reals[r++];
        for (int k = 0; k < 3; ++k) axis[3 * j + k] = reals[r++];
        ratio[j] = reals[r++]; offset[j] = reals[r++]; scale[j] = reals[r++];
    }
    frame_offset.assign(reals + r, reals + r + 7 * frames);

    // Movable joints from each body's root to the body, root first (bodies are in parent-first order).
    body_chain.assign(bodies, {});
    for (int32_t body : body_order) {
        const int32_t joint = body_parent_joint[body];
        if (joint < 0) continue;
        body_chain[body] = body_chain[joint_parent[joint]];
        if (joint_kind[joint] != 0) body_chain[body].push_back(joint);
    }
    values.assign(joints, 0.0);
    poses.assign(7 * bodies, 0.0);
    origins.assign(3 * joints, 0.0);
    axes.assign(3 * joints, 0.0);
    return true;
}

void Model::evaluate(const double *q, const double *roots) {
    for (int32_t joint : joint_value_order) {
        const int32_t source = joint_source[joint];
        if (source >= 0) values[joint] = values[source] * ratio[joint] + offset[joint];
        else values[joint] = joint_dof[joint] < 0 ? 0.0 : q[joint_dof[joint]];
    }
    for (int32_t body : body_order) if (body_parent_joint[body] < 0) {
        const double *root = roots ? roots + 7 * body : root_pose.data() + 7 * body;
        for (int k = 0; k < 7; ++k) poses[7 * body + k] = root[k];
    }
    double scratch[21];
    for (int32_t joint : joint_order) {
        const size_t parent = 7 * size_t(joint_parent[joint]), child = 7 * size_t(joint_child[joint]);
        compose(poses.data() + parent, parent_t_joint.data() + 7 * joint, scratch);
        if (joint_kind[joint] == 0) {
            compose(scratch, joint_t_child.data() + 7 * joint, poses.data() + child);
            continue;
        }
        const double ax = axis[3 * joint], ay = axis[3 * joint + 1], az = axis[3 * joint + 2];
        origins[3 * joint] = scratch[0]; origins[3 * joint + 1] = scratch[1]; origins[3 * joint + 2] = scratch[2];
        rotate(scratch, ax, ay, az, axes.data() + 3 * joint);
        const double value = values[joint];
        if (joint_kind[joint] == 1) {
            const double s = std::sin(value * 0.5);
            scratch[7] = 0.0; scratch[8] = 0.0; scratch[9] = 0.0;
            scratch[10] = ax * s; scratch[11] = ay * s; scratch[12] = az * s; scratch[13] = std::cos(value * 0.5);
        } else {
            scratch[7] = ax * value; scratch[8] = ay * value; scratch[9] = az * value;
            scratch[10] = 0.0; scratch[11] = 0.0; scratch[12] = 0.0; scratch[13] = 1.0;
        }
        compose(scratch, scratch + 7, scratch + 14);
        compose(scratch + 14, joint_t_child.data() + 7 * joint, poses.data() + child);
    }
}

void Model::point_jacobian(uint32_t body, double px, double py, double pz, const int32_t *column_of_dof,
                           uint32_t width, double *out) const {
    for (uint32_t i = 0; i < 6 * width; ++i) out[i] = 0.0;
    for (int32_t joint : body_chain[body]) {
        const int32_t column = column_of_dof[joint_dof[joint]];
        if (column < 0) continue;
        const double s = scale[joint];
        const double ax = axes[3 * joint], ay = axes[3 * joint + 1], az = axes[3 * joint + 2];
        if (joint_kind[joint] == 2) {
            out[column] += s * ax;
            out[width + column] += s * ay;
            out[2 * width + column] += s * az;
        } else {
            const double rx = px - origins[3 * joint], ry = py - origins[3 * joint + 1], rz = pz - origins[3 * joint + 2];
            out[column] += s * (ay * rz - az * ry);
            out[width + column] += s * (az * rx - ax * rz);
            out[2 * width + column] += s * (ax * ry - ay * rx);
            out[3 * width + column] += s * ax;
            out[4 * width + column] += s * ay;
            out[5 * width + column] += s * az;
        }
    }
}

void compose(const double *a, const double *b, double *out) {
    const double x = a[0], y = a[1], z = a[2], qx = a[3], qy = a[4], qz = a[5], qw = a[6];
    const double bx = b[0], by = b[1], bz = b[2], bqx = b[3], bqy = b[4], bqz = b[5], bqw = b[6];
    const double tx = 2.0 * (qy * bz - qz * by);
    const double ty = 2.0 * (qz * bx - qx * bz);
    const double tz = 2.0 * (qx * by - qy * bx);
    out[0] = bx + qw * tx + qy * tz - qz * ty + x;
    out[1] = by + qw * ty + qz * tx - qx * tz + y;
    out[2] = bz + qw * tz + qx * ty - qy * tx + z;
    out[3] = qw * bqx + qx * bqw + qy * bqz - qz * bqy;
    out[4] = qw * bqy - qx * bqz + qy * bqw + qz * bqx;
    out[5] = qw * bqz + qx * bqy - qy * bqx + qz * bqw;
    out[6] = qw * bqw - qx * bqx - qy * bqy - qz * bqz;
}

void rotate(const double *a, double vx, double vy, double vz, double *out) {
    const double qx = a[3], qy = a[4], qz = a[5], qw = a[6];
    const double tx = 2.0 * (qy * vz - qz * vy);
    const double ty = 2.0 * (qz * vx - qx * vz);
    const double tz = 2.0 * (qx * vy - qy * vx);
    out[0] = vx + qw * tx + qy * tz - qz * ty;
    out[1] = vy + qw * ty + qz * tx - qx * tz;
    out[2] = vz + qw * tz + qx * ty - qy * tx;
}

uint32_t store(std::unique_ptr<Model> model) {
    std::lock_guard<std::mutex> lock(table_mutex);
    uint32_t slot;
    if (!free_slots.empty()) {
        slot = free_slots.back();
        free_slots.pop_back();
    } else {
        if (table.size() > kSlotMask) return 0;
        slot = uint32_t(table.size());
        table.emplace_back();
    }
    table[slot].model = std::move(model);
    return (table[slot].generation << kSlotBits) | slot;
}

Model *find(uint32_t handle) {
    std::lock_guard<std::mutex> lock(table_mutex);
    const uint32_t slot = handle & kSlotMask;
    if (slot == 0 || slot >= table.size()) return nullptr;
    Entry &entry = table[slot];
    if (entry.generation != (handle >> kSlotBits) || !entry.model) return nullptr;
    return entry.model.get();
}

void release(uint32_t handle) {
    std::lock_guard<std::mutex> lock(table_mutex);
    const uint32_t slot = handle & kSlotMask;
    if (slot == 0 || slot >= table.size()) return;
    Entry &entry = table[slot];
    if (entry.generation != (handle >> kSlotBits) || !entry.model) return;
    entry.model.reset();
    // A slot retires when its generation is exhausted instead of wrapping around.
    if (entry.generation < kGenerationMask) {
        ++entry.generation;
        free_slots.push_back(slot);
    }
}

} // namespace kk
