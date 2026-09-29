#include "nativekit_sim_mujoco.h"

#include "PhysicsBackend.hpp"
#include "internal.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nksim_mujoco {
namespace {

std::atomic<std::uint64_t> distance_call_count{0};

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

struct BodyRecord {
    nksim::BackendBodyDesc desc{};
    nksim::BackendBodyState state{};
    std::string name;
    double free_mass = 0.0;
    std::array<double, 3> free_inertia{};
    bool free_inertia_saved = false;
};

struct RestBox {
    Vec3 center{};
    Vec3 half{};
    Vec3 axis[3]{};
};

struct JointRecord {
    nksim::BackendJointDesc desc{};
    nksim::BackendJointState state{};
    std::string name;
    std::uint32_t target_mode = 0;
    double target = 0.0;
    double target_max_force = 0.0;
    // NKSIM_JOINT_TARGET_SERVO terms.
    double target_velocity = 0.0;
    double target_stiffness = 0.0;
    double target_damping = 0.0;
    double target_feedforward = 0.0;
};

double dot(const Vec3 &a, const Vec3 &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 add(const Vec3 &a, const Vec3 &b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 normalize(Vec3 value, const Vec3 &fallback = {1.0, 0.0, 0.0}) {
    const auto length = std::sqrt(dot(value, value));
    if (!std::isfinite(length) || length < 1e-12)
        return fallback;
    for (auto &component : value)
        component /= length;
    return value;
}

Quat normalize(Quat value) {
    const auto length = std::sqrt(value[0] * value[0] + value[1] * value[1] +
                                   value[2] * value[2] + value[3] * value[3]);
    if (!std::isfinite(length) || length < 1e-12)
        return {0.0, 0.0, 0.0, 1.0};
    for (auto &component : value)
        component /= length;
    return value;
}

Quat rotation_from_z(Vec3 normal) {
    normal = normalize(normal, {0.0, 0.0, 1.0});
    const auto scalar = 1.0 + normal[2];
    if (scalar < 1e-12)
        return {1.0, 0.0, 0.0, 0.0};
    return normalize({-normal[1], normal[0], 0.0, scalar});
}

Quat conjugate(const Quat &value) {
    return {-value[0], -value[1], -value[2], value[3]};
}

Quat multiply(const Quat &a, const Quat &b) {
    return {
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    };
}

Vec3 rotate(const Quat &rotation, const Vec3 &value) {
    const Quat vector{value[0], value[1], value[2], 0.0};
    const auto rotated = multiply(multiply(rotation, vector), conjugate(rotation));
    return {rotated[0], rotated[1], rotated[2]};
}

// Nominal mass/inertia given to a KINEMATIC root's free joint (see
// configure_body). Chosen well above any plausible dynamic payload so a
// contact impulse against it approximates what an immovable obstacle
// moving at the same velocity would give; the body's own qpos/qvel are
// re-pinned to the prescribed trajectory every substep regardless (see
// step()), so these values never affect this body's own motion.
constexpr double kKinematicBodyMass = 1000.0;

// The incremental rotation a body with constant LOCAL-frame angular
// velocity `local_angular_velocity` accumulates over `dt`, matching
// mju_quatIntegrate's own convention exactly (vendored MuJoCo,
// engine_util_spatial.c: "quat = quat * qrot(vel*dt)", i.e. a right
// multiply by an axis-angle quaternion built from the LOCAL angular
// velocity) — mj_integratePosInd (engine_support.c) applies a free
// joint's qpos update the same way, so placing this body's qpos with the
// identical formula keeps step()'s per-substep placement consistent with
// what MuJoCo's own integrator would produce.
Quat axis_angle(const Vec3 &local_angular_velocity, double dt) {
    const auto length = std::sqrt(dot(local_angular_velocity, local_angular_velocity));
    if (!std::isfinite(length) || length < 1e-12)
        return {0.0, 0.0, 0.0, 1.0};
    const auto angle = length * dt;
    const auto axis = normalize(local_angular_velocity);
    const auto half = angle * 0.5;
    return {axis[0] * std::sin(half), axis[1] * std::sin(half), axis[2] * std::sin(half),
            std::cos(half)};
}

RestBox rest_box(const BodyRecord &body) {
    RestBox result;
    result.center = {body.desc.position[0], body.desc.position[1], body.desc.position[2]};
    result.half = {body.desc.shape_parameters[0], body.desc.shape_parameters[1],
                   body.desc.shape_parameters[2]};
    const auto rotation = normalize(Quat{body.desc.rotation[0], body.desc.rotation[1],
                                         body.desc.rotation[2], body.desc.rotation[3]});
    for (int axis = 0; axis < 3; ++axis) {
        Vec3 local{0.0, 0.0, 0.0};
        local[axis] = 1.0;
        result.axis[axis] = rotate(rotation, local);
    }
    return result;
}

bool rest_boxes_overlap(const RestBox &a, const RestBox &b) {
    double rotation[3][3]{};
    double absolute[3][3]{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        rotation[i][j] = dot(a.axis[i], b.axis[j]);
        absolute[i][j] = std::abs(rotation[i][j]) + 1e-10;
    }
    const auto delta = subtract(b.center, a.center);
    double translated_a[3]{};
    double translated_b[3]{};
    for (int i = 0; i < 3; ++i) {
        translated_a[i] = dot(delta, a.axis[i]);
        translated_b[i] = dot(delta, b.axis[i]);
    }
    for (int i = 0; i < 3; ++i) {
        double extent_b = 0.0;
        for (int j = 0; j < 3; ++j) extent_b += b.half[j] * absolute[i][j];
        if (std::abs(translated_a[i]) > a.half[i] + extent_b)
            return false;
    }
    for (int j = 0; j < 3; ++j) {
        double extent_a = 0.0;
        for (int i = 0; i < 3; ++i) extent_a += a.half[i] * absolute[i][j];
        if (std::abs(translated_b[j]) > extent_a + b.half[j])
            return false;
    }
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        const auto separating_axis = cross(a.axis[i], b.axis[j]);
        const auto axis_length = std::sqrt(dot(separating_axis, separating_axis));
        if (axis_length < 1e-12) continue;
        const auto axis = normalize(separating_axis);
        double extent_a = 0.0, extent_b = 0.0;
        for (int k = 0; k < 3; ++k) {
            extent_a += a.half[k] * std::abs(dot(a.axis[k], axis));
            extent_b += b.half[k] * std::abs(dot(b.axis[k], axis));
        }
        if (std::abs(dot(delta, axis)) > extent_a + extent_b)
            return false;
    }
    return true;
}

std::vector<RestBox> rest_piece_boxes(const BodyRecord &body) {
    std::vector<RestBox> result;
    const auto body_rotation = normalize(Quat{body.desc.rotation[0], body.desc.rotation[1],
                                               body.desc.rotation[2], body.desc.rotation[3]});
    for (const auto &part : body.desc.shape_parts) {
        if (part.type == NKSIM_SHAPE_PLANE) continue;
        RestBox box{};
        Vec3 local_center{part.position[0], part.position[1], part.position[2]};
        if (part.type == NKSIM_SHAPE_CONVEX && !part.vertices.empty()) {
            Vec3 minimum{INFINITY, INFINITY, INFINITY}, maximum{-INFINITY, -INFINITY, -INFINITY};
            for (std::size_t index = 0; index < part.vertices.size(); ++index) {
                minimum[index % 3] = std::min(minimum[index % 3],
                    static_cast<double>(part.vertices[index]));
                maximum[index % 3] = std::max(maximum[index % 3],
                    static_cast<double>(part.vertices[index]));
            }
            for (int axis = 0; axis < 3; ++axis) {
                local_center[axis] += (minimum[axis] + maximum[axis]) * 0.5;
                box.half[axis] = (maximum[axis] - minimum[axis]) * 0.5;
            }
        } else if (part.type == NKSIM_SHAPE_BOX) {
            for (int axis = 0; axis < 3; ++axis) box.half[axis] = part.parameters[axis];
        } else if (part.type == NKSIM_SHAPE_SPHERE) {
            box.half = {part.parameters[0], part.parameters[0], part.parameters[0]};
        } else if (part.type == NKSIM_SHAPE_CAPSULE) {
            box.half = {part.parameters[0], part.parameters[0],
                        part.parameters[0] + part.parameters[1] * 0.5};
        } else if (part.type == NKSIM_SHAPE_CYLINDER) {
            box.half = {part.parameters[0], part.parameters[0], part.parameters[1] * 0.5};
        }
        const auto rotation = normalize(multiply(body_rotation, normalize(part.rotation)));
        const auto offset = rotate(body_rotation, local_center);
        box.center = {body.desc.position[0] + offset[0],
                      body.desc.position[1] + offset[1], body.desc.position[2] + offset[2]};
        for (int axis = 0; axis < 3; ++axis) {
            Vec3 basis{0.0, 0.0, 0.0};
            basis[axis] = 1.0;
            box.axis[axis] = rotate(rotation, basis);
        }
        result.push_back(box);
    }
    return result;
}

double rest_shape_radius(const BodyRecord &body) {
    switch (body.desc.shape_type) {
    case NKSIM_SHAPE_BOX:
        return std::sqrt(body.desc.shape_parameters[0] * body.desc.shape_parameters[0] +
                         body.desc.shape_parameters[1] * body.desc.shape_parameters[1] +
                         body.desc.shape_parameters[2] * body.desc.shape_parameters[2]);
    case NKSIM_SHAPE_SPHERE:
        return body.desc.shape_parameters[0];
    case NKSIM_SHAPE_CAPSULE:
    case NKSIM_SHAPE_CYLINDER:
        return std::sqrt(body.desc.shape_parameters[0] * body.desc.shape_parameters[0] +
                         0.25 * body.desc.shape_parameters[1] * body.desc.shape_parameters[1]);
    default:
        return 0.0;
    }
}

bool geometries_overlap_at_rest(const BodyRecord &a, const BodyRecord &b) {
    if (a.desc.shape_parts.size() > 1 || b.desc.shape_parts.size() > 1) {
        for (const auto &first : rest_piece_boxes(a))
            for (const auto &second : rest_piece_boxes(b))
                if (rest_boxes_overlap(first, second)) return true;
        return false;
    }
    if (a.desc.shape_type == NKSIM_SHAPE_BOX && b.desc.shape_type == NKSIM_SHAPE_BOX)
        return rest_boxes_overlap(rest_box(a), rest_box(b));
    const auto radius_a = rest_shape_radius(a), radius_b = rest_shape_radius(b);
    if (radius_a <= 0.0 || radius_b <= 0.0) return false;
    const Vec3 center_a{a.desc.position[0], a.desc.position[1], a.desc.position[2]};
    const Vec3 center_b{b.desc.position[0], b.desc.position[1], b.desc.position[2]};
    const auto delta = subtract(center_b, center_a);
    const auto distance = std::sqrt(dot(delta, delta));
    return distance <= radius_a + radius_b;
}

// Both sides of a joint describe the same physical pivot: anchor_a/rotation_a
// in body_a's own rest frame, anchor_b/rotation_b in body_b's. With rest poses
// now correct (F1), this must agree; a mismatch means the joint description
// itself is inconsistent (e.g. hand-built rather than derived from one set
// of link transforms) and must be diagnosed, not silently resolved by
// trusting anchor_b alone as the previous implementation did.
bool joint_frame_matches_rest_pose(const nksim::BackendJointDesc &joint,
                                   const nksim::BackendBodyDesc &parent,
                                   const nksim::BackendBodyDesc &child) {
    const Vec3 parent_position{parent.position[0], parent.position[1], parent.position[2]};
    const Vec3 child_position{child.position[0], child.position[1], child.position[2]};
    const auto from_parent = add(parent_position,
        rotate(normalize(Quat{parent.rotation[0], parent.rotation[1], parent.rotation[2], parent.rotation[3]}),
               joint.anchor_a));
    const auto from_child = add(child_position,
        rotate(normalize(Quat{child.rotation[0], child.rotation[1], child.rotation[2], child.rotation[3]}),
               joint.anchor_b));
    const auto delta = subtract(from_parent, from_child);
    return std::sqrt(dot(delta, delta)) <= 1e-6;
}

void write_pose(const Vec3 &position, const Quat &rotation, mjsBody &body) {
    std::copy(position.begin(), position.end(), body.pos);
    const auto normalized = normalize(rotation);
    body.quat[0] = normalized[3];
    body.quat[1] = normalized[0];
    body.quat[2] = normalized[1];
    body.quat[3] = normalized[2];
}

class MujocoBackend final : public nksim::PhysicsBackend {
public:
    ~MujocoBackend() override {
        if (data)
            mj_deleteData(data);
        if (model)
            mj_deleteModel(model);
        if (spec)
            mj_deleteSpec(spec);
    }

    nksim_result initialize(const nksim_world_desc &desc) override {
        spec = mj_makeSpec();
        if (!spec)
            return NKSIM_ERROR_OUT_OF_MEMORY;
        spec->compiler.degree = 0; // SimKit joint limits are SI radians.
        spec->option.gravity[0] = desc.gravity[0];
        spec->option.gravity[1] = desc.gravity[1];
        spec->option.gravity[2] = desc.gravity[2];
        switch (desc.integrator) {
        case NKSIM_INTEGRATOR_EULER: spec->option.integrator = mjINT_EULER; break;
        case NKSIM_INTEGRATOR_IMPLICIT_FAST: spec->option.integrator = mjINT_IMPLICITFAST; break;
        case NKSIM_INTEGRATOR_RK4: spec->option.integrator = mjINT_RK4; break;
        default: break;
        }
        if (desc.friction_cone == NKSIM_FRICTION_CONE_PYRAMIDAL)
            spec->option.cone = mjCONE_PYRAMIDAL;
        else if (desc.friction_cone == NKSIM_FRICTION_CONE_ELLIPTIC)
            spec->option.cone = mjCONE_ELLIPTIC;
        if (desc.solver_iterations != 0)
            spec->option.iterations = static_cast<int>(desc.solver_iterations);
        if (desc.line_search_iterations != 0)
            spec->option.ls_iterations = static_cast<int>(desc.line_search_iterations);
        return rebuild();
    }

    nksim_result begin_topology_update() override {
        if (topology_update)
            return NKSIM_ERROR_INVALID_STATE;
        try {
            saved_bodies = bodies;
            saved_body_order = body_order;
            saved_joints = joints;
            saved_joint_order = joint_order;
            saved_couplings = couplings;
            saved_closures = closures;
            saved_contact_pairs = contact_pairs;
            saved_next_body = next_body;
            saved_next_joint = next_joint;
        } catch (const std::bad_alloc &) {
            clear_topology_backup();
            return NKSIM_ERROR_OUT_OF_MEMORY;
        }
        topology_update = true;
        return NKSIM_OK;
    }

    nksim_result end_topology_update() override {
        if (!topology_update)
            return NKSIM_ERROR_INVALID_STATE;
        topology_update = false;
        const auto result = rebuild();
        if (result != NKSIM_OK) {
            bodies.swap(saved_bodies);
            body_order.swap(saved_body_order);
            joints.swap(saved_joints);
            joint_order.swap(saved_joint_order);
            couplings.swap(saved_couplings);
            closures.swap(saved_closures);
            contact_pairs.swap(saved_contact_pairs);
            next_body = saved_next_body;
            next_joint = saved_next_joint;
            (void)rebuild();
        }
        clear_topology_backup();
        return result;
    }

    nksim_result body_create(const nksim::BackendBodyDesc &desc,
                             std::uint64_t *out_body) override {
        if (!out_body)
            return NKSIM_ERROR_INVALID_ARGUMENT;

        const auto id = next_body++;
        BodyRecord record;
        record.desc = desc;
        record.name = "nk_body_" + std::to_string(id);
        record.state.backend_body = id;
        record.state.position = desc.position;
        record.state.rotation = normalize(desc.rotation);
        bodies.emplace(id, std::move(record));
        body_order.push_back(id);

        if (topology_update) {
            *out_body = id;
            return NKSIM_OK;
        }

        const auto result = rebuild();
        if (result != NKSIM_OK) {
            bodies.erase(id);
            body_order.pop_back();
            rebuild();
            return result;
        }

        *out_body = id;
        return NKSIM_OK;
    }

    nksim_result body_destroy(std::uint64_t id) override {
        const auto found = bodies.find(id);
        if (found == bodies.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            if (joint.desc.body_a == id || joint.desc.body_b == id)
                return NKSIM_ERROR_INVALID_STATE;
        }
        for (const auto &closure : closures)
            if (closure.body_a == id || closure.body_b == id)
                return NKSIM_ERROR_INVALID_STATE;
        contact_pairs.erase(std::remove_if(contact_pairs.begin(), contact_pairs.end(),
            [id](const nksim::BackendContactPair &pair) { return pair.body_a == id || pair.body_b == id; }),
            contact_pairs.end());
        bodies.erase(found);
        body_order.erase(std::remove(body_order.begin(), body_order.end(), id), body_order.end());
        return topology_update ? NKSIM_OK : rebuild();
    }

    nksim_result body_set_state(std::uint64_t id,
                                const nksim::BackendBodyState &state) override {
        const auto found = bodies.find(id);
        if (found == bodies.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        const auto body_id = model_body_id(id);
        if (body_id < 0)
            return NKSIM_ERROR_INVALID_HANDLE;
        const auto joint_id = model->body_jntadr[body_id];
        if (joint_id >= 0 && model->jnt_type[joint_id] == mjJNT_FREE) {
            // Both a DYNAMIC root and a childless KINEMATIC root
            // (configure_body gives both a free joint) land here. For a KINEMATIC root this is
            // just bookkeeping for the tick: it records the driven pose and
            // twist in found->second.state below, which step()'s
            // kinematic_drives() reads to place the body correctly across
            // every substep this tick, not only right now.
            set_free_body_state(body_id, state);
        } else if (const auto incoming = parent_joint_id(id); incoming != 0) {
            // A constrained link can be reset to its declared rest pose, but
            // cannot be independently teleported away from its articulation.
            const auto &desc = found->second.desc;
            for (int i = 0; i < 3; ++i)
                if (std::abs(state.position[i] - desc.position[i]) > 1e-9)
                    return NKSIM_ERROR_UNSUPPORTED;
            for (int i = 0; i < 4; ++i)
                if (std::abs(state.rotation[i] - desc.rotation[i]) > 1e-9)
                    return NKSIM_ERROR_UNSUPPORTED;
            auto &joint = joints.at(incoming);
            joint.state.position = joint.state.velocity = joint.state.effort = 0.0;
            joint.target_mode = 0;
            if (joint_id >= 0) {
                data->qpos[model->jnt_qposadr[joint_id]] = 0.0;
                data->qvel[model->jnt_dofadr[joint_id]] = 0.0;
            }
            apply_joint_targets();
        } else {
            // A STATIC root's pose, or a welded KINEMATIC root's (one that
            // carries other bodies), is model state, not a free-joint qpos.
            std::copy(state.position.begin(), state.position.end(), model->body_pos + 3*body_id);
            const auto q = normalize(state.rotation);
            model->body_quat[4*body_id] = q[3];
            for (int i = 0; i < 3; ++i) model->body_quat[4*body_id+i+1] = q[i];
        }
        found->second.state = state;
        found->second.state.backend_body = id;
        mj_forward(model, data);
        return NKSIM_OK;
    }

    nksim_result body_set_motion_type(std::uint64_t id, std::uint32_t motion_type,
                                      double) override {
        auto found = bodies.find(id);
        if (found == bodies.end()) return NKSIM_ERROR_INVALID_HANDLE;
        if (motion_type != NKSIM_MOTION_DYNAMIC && motion_type != NKSIM_MOTION_KINEMATIC)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        if (found->second.desc.motion_type == motion_type) return NKSIM_OK;
        // Only a childless root has the free joint used by both motion types.
        // A jointed body would need a different compiled topology.
        if (parent_joint_id(id) != 0 || has_children(id)) return NKSIM_ERROR_UNSUPPORTED;
        const int model_id = model_body_id(id);
        if (model_id < 0) return NKSIM_ERROR_INVALID_HANDLE;
        const int joint_id = model->body_jntadr[model_id];
        if (joint_id < 0 || model->jnt_type[joint_id] != mjJNT_FREE)
            return NKSIM_ERROR_UNSUPPORTED;
        auto &record = found->second;
        if (!record.free_inertia_saved) {
            record.free_mass = model->body_mass[model_id];
            std::copy_n(model->body_inertia + 3 * model_id, 3, record.free_inertia.begin());
            record.free_inertia_saved = true;
        }
        const bool held = motion_type == NKSIM_MOTION_KINEMATIC;
        model->body_mass[model_id] = held ? kKinematicBodyMass : record.free_mass;
        for (int axis = 0; axis < 3; ++axis)
            model->body_inertia[3 * model_id + axis] =
                held ? kKinematicBodyMass : record.free_inertia[axis];
        model->body_gravcomp[model_id] = held ? 1.0 : 0.0;
        // mj_setConst recomputes body_invweight0 and dof_invweight0 from the
        // edited inertial properties (engine_setconst.c). The held object
        // keeps its original collision pairs, including STATIC/KINEMATIC
        // pairs; only contacts against DYNAMIC bodies can move another body.
        const std::vector<mjtNum> qpos(data->qpos, data->qpos + model->nq);
        const std::vector<mjtNum> qvel(data->qvel, data->qvel + model->nv);
        mj_setConst(model, data);
        std::copy(qpos.begin(), qpos.end(), data->qpos);
        std::copy(qvel.begin(), qvel.end(), data->qvel);
        mj_forward(model, data);
        record.desc.motion_type = motion_type;
        return NKSIM_OK;
    }

    nksim_result apply_forces(const nksim::BackendBodyForce *forces,
                              std::uint32_t count) override {
        if (count != 0 && !forces)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto body_id = model_body_id(forces[index].backend_body);
            if (body_id < 0)
                return NKSIM_ERROR_INVALID_HANDLE;
            // xfrc_applied is force then torque, applied at the body's centre of mass.
            // Accumulate, like the default backend, so several forces on one body sum.
            auto *external = data->xfrc_applied + body_id * 6;
            for (int axis = 0; axis < 3; ++axis) {
                external[axis] += forces[index].force[axis];
                external[axis + 3] += forces[index].torque[axis];
            }
        }
        return NKSIM_OK;
    }

    nksim_result joint_create(const nksim::BackendJointDesc &desc,
                              std::uint64_t *out_joint) override {
        if (!out_joint)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        if (bodies.find(desc.body_a) == bodies.end() ||
            bodies.find(desc.body_b) == bodies.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        if (desc.body_a == desc.body_b || parent_joint_id(desc.body_b) != 0)
            return NKSIM_ERROR_INVALID_STATE;
        if (desc.type != NKSIM_JOINT_FIXED && desc.type != NKSIM_JOINT_REVOLUTE &&
            desc.type != NKSIM_JOINT_PRISMATIC)
            return NKSIM_ERROR_UNSUPPORTED;
        if (would_create_cycle(desc.body_a, desc.body_b))
            return NKSIM_ERROR_INVALID_STATE;

        const auto id = next_joint++;
        JointRecord record;
        record.desc = desc;
        record.name = "nk_joint_" + std::to_string(id);
        record.state.backend_joint = id;
        joints.emplace(id, std::move(record));
        joint_order.push_back(id);

        if (topology_update) {
            *out_joint = id;
            return NKSIM_OK;
        }

        const auto result = rebuild();
        if (result != NKSIM_OK) {
            joints.erase(id);
            joint_order.pop_back();
            rebuild();
            return result;
        }

        *out_joint = id;
        return NKSIM_OK;
    }

    nksim_result joint_destroy(std::uint64_t id) override {
        const auto found = joints.find(id);
        if (found == joints.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        joints.erase(found);
        joint_order.erase(std::remove(joint_order.begin(), joint_order.end(), id), joint_order.end());
        couplings.erase(std::remove_if(couplings.begin(), couplings.end(),
            [id](const auto &value) { return value.leader == id || value.follower == id; }),
            couplings.end());
        return topology_update ? NKSIM_OK : rebuild();
    }

    nksim_result joint_couple(const nksim::BackendJointCoupling &coupling) override {
        if (joints.find(coupling.leader) == joints.end() ||
            joints.find(coupling.follower) == joints.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        for (const auto &existing : couplings)
            if (existing.follower == coupling.follower)
                return NKSIM_ERROR_INVALID_ARGUMENT;
        couplings.push_back(coupling);
        return topology_update ? NKSIM_OK : rebuild();
    }

    nksim_result closure_create(const nksim::BackendClosure &closure) override {
        if (bodies.find(closure.body_a) == bodies.end() ||
            bodies.find(closure.body_b) == bodies.end())
            return NKSIM_ERROR_INVALID_HANDLE;
        if (closure.type != NKSIM_JOINT_FIXED && closure.type != NKSIM_JOINT_REVOLUTE &&
            closure.type != NKSIM_JOINT_PRISMATIC)
            return NKSIM_ERROR_UNSUPPORTED;
        closures.push_back(closure);
        return topology_update ? NKSIM_OK : rebuild();
    }

    nksim_result contact_pair_create(const nksim::BackendContactPair &pair) override {
        const auto a = bodies.find(pair.body_a), b = bodies.find(pair.body_b);
        if (a == bodies.end() || b == bodies.end()) return NKSIM_ERROR_INVALID_HANDLE;
        if (pair.part_a >= a->second.desc.shape_parts.size() ||
            pair.part_b >= b->second.desc.shape_parts.size())
            return NKSIM_ERROR_INVALID_ARGUMENT;
        contact_pairs.push_back(pair);
        return topology_update ? NKSIM_OK : rebuild();
    }

    nksim_result set_joint_targets(const nksim::BackendJointTarget *targets,
                                   std::uint32_t count) override {
        if (count != 0 && !targets)
            return NKSIM_ERROR_INVALID_ARGUMENT;

        for (std::uint32_t index = 0; index < count; ++index) {
            const auto found = joints.find(targets[index].backend_joint);
            if (found == joints.end())
                return NKSIM_ERROR_INVALID_HANDLE;
            if (targets[index].mode < NKSIM_JOINT_TARGET_POSITION ||
                targets[index].mode > NKSIM_JOINT_TARGET_SERVO ||
                !std::isfinite(targets[index].target) ||
                !std::isfinite(targets[index].max_force))
                return NKSIM_ERROR_INVALID_ARGUMENT;
            if (found->second.desc.type == NKSIM_JOINT_FIXED)
                return NKSIM_ERROR_UNSUPPORTED;
        }

        for (std::uint32_t index = 0; index < count; ++index) {
            auto &record = joints.at(targets[index].backend_joint);
            record.target_mode = targets[index].mode;
            record.target = targets[index].target;
            record.target_max_force = targets[index].max_force;
            record.target_velocity = targets[index].velocity;
            record.target_stiffness = targets[index].stiffness;
            record.target_damping = targets[index].damping;
            record.target_feedforward = targets[index].feedforward;
        }
        return NKSIM_OK;
    }

    // A KINEMATIC root's target for this whole outer step was already
    // written into its free joint's qpos/qvel by body_set_state (called by
    // World::refresh_kinematic_bodies before World's own call into
    // backend->step()), via the same set_free_body_state() path a DYNAMIC
    // free body uses. Left there for the whole step, the body would sit
    // still through every substep — the exact "teleport" bug this backend
    // used to have. kinematic_drives() below instead re-derives, for a
    // given substep, the pose that lies `remaining` seconds before that
    // known end pose along the constant twist the caller supplied
    // (position extrapolates linearly; rotation undoes axis_angle(), the
    // exact inverse of the forward integration MuJoCo's own free joint
    // uses — see axis_angle()'s comment), so placing it before every
    // mj_step() call makes the body move smoothly across the step and lets
    // the contact solver see its true instantaneous velocity.
    struct KinematicDrive {
        int qpos_adr = 0;
        int qvel_adr = 0;
        Vec3 position{};
        Quat rotation{0.0, 0.0, 0.0, 1.0};
        Vec3 linear_velocity{};
        Vec3 local_angular_velocity{};
    };

    std::vector<KinematicDrive> kinematic_drives() const {
        std::vector<KinematicDrive> drives;
        for (const auto body_id : body_order) {
            const auto &record = bodies.at(body_id);
            if (!free_kinematic_root(body_id))
                continue;
            const auto model_id = model_body_id(body_id);
            if (model_id < 0)
                continue;
            const auto joint_id = model->body_jntadr[model_id];
            if (joint_id < 0 || model->jnt_type[joint_id] != mjJNT_FREE)
                continue;
            KinematicDrive drive;
            drive.qpos_adr = model->jnt_qposadr[joint_id];
            drive.qvel_adr = model->jnt_dofadr[joint_id];
            drive.position = {record.state.position[0], record.state.position[1],
                              record.state.position[2]};
            drive.rotation = normalize(Quat{record.state.rotation[0], record.state.rotation[1],
                                            record.state.rotation[2], record.state.rotation[3]});
            drive.linear_velocity = {record.state.linear_velocity[0],
                                     record.state.linear_velocity[1],
                                     record.state.linear_velocity[2]};
            const Vec3 world_angular{record.state.angular_velocity[0],
                                     record.state.angular_velocity[1],
                                     record.state.angular_velocity[2]};
            // set_free_body_state() stores the same world-to-local
            // conversion for a DYNAMIC free body's qvel; matching it here
            // keeps both paths consistent.
            drive.local_angular_velocity = rotate(conjugate(drive.rotation), world_angular);
            drives.push_back(drive);
        }
        return drives;
    }

    void place_kinematic_drives(const std::vector<KinematicDrive> &drives, double remaining) {
        for (const auto &drive : drives) {
            Vec3 position;
            for (int axis = 0; axis < 3; ++axis)
                position[axis] = drive.position[axis] - drive.linear_velocity[axis] * remaining;
            const auto undo = axis_angle(drive.local_angular_velocity, remaining);
            const auto rotation = normalize(multiply(drive.rotation, conjugate(undo)));
            std::copy(position.begin(), position.end(), data->qpos + drive.qpos_adr);
            data->qpos[drive.qpos_adr + 3] = rotation[3];
            data->qpos[drive.qpos_adr + 4] = rotation[0];
            data->qpos[drive.qpos_adr + 5] = rotation[1];
            data->qpos[drive.qpos_adr + 6] = rotation[2];
            std::copy(drive.linear_velocity.begin(), drive.linear_velocity.end(),
                      data->qvel + drive.qvel_adr);
            std::copy(drive.local_angular_velocity.begin(), drive.local_angular_velocity.end(),
                      data->qvel + drive.qvel_adr + 3);
        }
    }

    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!model || !data || !std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        model->opt.timestep = dt / static_cast<double>(substeps);
        const auto drives = kinematic_drives();
        for (std::uint32_t index = 0; index < substeps; ++index) {
            if (!drives.empty())
                place_kinematic_drives(drives,
                    static_cast<double>(substeps - index) * model->opt.timestep);
            apply_joint_targets();
            mj_step(model, data);
        }
        // Any reaction mj_step computed back onto a kinematic root's free
        // joint (it has real mass so the contact solve treats it like any
        // other body) is discarded here: it lands back exactly on its
        // authored trajectory regardless of what physics did to it.
        if (!drives.empty())
            place_kinematic_drives(drives, 0.0);
        // mj_step integrates qpos/qvel after computing derived body quantities.
        // Refresh them so snapshots contain pose and velocity at the same time,
        // and so read_contacts() sees contacts recomputed from the final,
        // exact kinematic placement above rather than the last substep's.
        mj_forward(model, data);
        std::fill(data->xfrc_applied, data->xfrc_applied + model->nbody * 6, 0.0);
        return NKSIM_OK;
    }

    nksim_result read_body_states(nksim::BackendBodyState *states,
                                  std::uint32_t count) override {
        if (count != 0 && !states)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            auto found = bodies.find(states[index].backend_body);
            if (found == bodies.end())
                return NKSIM_ERROR_INVALID_HANDLE;
            const auto body_id = model_body_id(states[index].backend_body);
            if (body_id < 0)
                return NKSIM_ERROR_INVALID_HANDLE;
            auto &state = states[index];
            std::copy(data->xpos + body_id * 3, data->xpos + body_id * 3 + 3,
                      state.position.begin());
            state.rotation = {data->xquat[body_id * 4 + 1], data->xquat[body_id * 4 + 2],
                              data->xquat[body_id * 4 + 3], data->xquat[body_id * 4 + 0]};
            mjtNum velocity[6];
            // World-oriented velocity at the body origin, including all
            // ancestor joints. MuJoCo returns angular then linear components.
            mj_objectVelocity(model, data, mjOBJ_XBODY, body_id, velocity, 0);
            std::copy_n(velocity, 3, state.angular_velocity.begin());
            std::copy_n(velocity + 3, 3, state.linear_velocity.begin());
            state.sleeping = 0;
            found->second.state = state;
        }
        return NKSIM_OK;
    }

    nksim_result joint_set_state(std::uint64_t id, double position, double velocity) override {
        const auto found = joints.find(id);
        if (found == joints.end()) return NKSIM_ERROR_INVALID_HANDLE;
        const auto joint_id = model_joint_id(found->second);
        if (joint_id < 0) return NKSIM_ERROR_INVALID_STATE;
        const auto type = model->jnt_type[joint_id];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE) return NKSIM_ERROR_UNSUPPORTED;
        data->qpos[model->jnt_qposadr[joint_id]] = position;
        data->qvel[model->jnt_dofadr[joint_id]] = velocity;
        found->second.state.position = position;
        found->second.state.velocity = velocity;
        mj_forward(model, data);
        return NKSIM_OK;
    }

    nksim_result read_joint_states(nksim::BackendJointState *states,
                                   std::uint32_t count) override {
        if (count != 0 && !states)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            auto found = joints.find(states[index].backend_joint);
            if (found == joints.end())
                return NKSIM_ERROR_INVALID_HANDLE;
            auto &state = states[index];
            state.position = 0.0;
            state.velocity = 0.0;
            state.effort = 0.0;
            const auto joint_id = model_joint_id(found->second);
            if (joint_id >= 0) {
                state.position = data->qpos[model->jnt_qposadr[joint_id]];
                state.velocity = data->qvel[model->jnt_dofadr[joint_id]];
                state.effort = data->qfrc_actuator[model->jnt_dofadr[joint_id]];
            }
            found->second.state = state;
        }
        return NKSIM_OK;
    }

    nksim_result read_contacts(std::vector<nksim::BackendContact> &out) override {
        out.clear();
        if (!model || !data) return NKSIM_OK;
        std::unordered_set<std::uint64_t> reported_pairs;
        const auto pair_key = [](int first, int second) {
            const auto low = static_cast<std::uint32_t>(std::min(first, second));
            const auto high = static_cast<std::uint32_t>(std::max(first, second));
            return (static_cast<std::uint64_t>(low) << 32) | high;
        };
        for (int i = 0; i < data->ncon; ++i) {
            const auto &source = data->contact[i];
            reported_pairs.insert(pair_key(source.geom[0], source.geom[1]));
            nksim::BackendContact contact{};
            auto first = geom_owner.find(source.geom[0]);
            auto second = geom_owner.find(source.geom[1]);
            if (first != geom_owner.end()) {
                contact.body_a = first->second.first;
                contact.part_a = first->second.second;
            }
            if (second != geom_owner.end()) {
                contact.body_b = second->second.first;
                contact.part_b = second->second.second;
            }
            std::copy_n(source.pos, 3, contact.position.begin());
            std::copy_n(source.frame, 3, contact.normal.begin());
            contact.distance = source.dist;
            contact.active = source.efc_address >= 0;
            out.push_back(contact);
        }
        // MuJoCo does not create contacts between two bodies with no dynamic
        // degrees of freedom. A tool on a kinematic flange still needs to
        // report its proximity to a fixed cell obstacle.
        for (const auto &candidate : proximity_candidates) {
            const int first = candidate.first, second = candidate.second;
            if (reported_pairs.count(pair_key(first, second)) != 0) continue;
            const double detection = candidate.detection;
            const double dx = data->geom_xpos[3 * first] - data->geom_xpos[3 * second];
            const double dy = data->geom_xpos[3 * first + 1] - data->geom_xpos[3 * second + 1];
            const double dz = data->geom_xpos[3 * first + 2] - data->geom_xpos[3 * second + 2];
            const double reach = model->geom_rbound[first] + model->geom_rbound[second] + detection;
            if (model->geom_type[first] != mjGEOM_PLANE &&
                model->geom_type[second] != mjGEOM_PLANE &&
                dx * dx + dy * dy + dz * dz > reach * reach) continue;
            mjtNum fromto[6]{};
            distance_call_count.fetch_add(1, std::memory_order_relaxed);
            const double distance = mj_geomDistance(model, data, first, second, detection, fromto);
            if (!std::isfinite(distance) || distance >= detection) continue;
            nksim::BackendContact contact{};
            contact.body_a = geom_owner.at(first).first;
            contact.part_a = geom_owner.at(first).second;
            contact.body_b = geom_owner.at(second).first;
            contact.part_b = geom_owner.at(second).second;
            contact.distance = distance;
            for (int axis = 0; axis < 3; ++axis)
                contact.position[axis] = (fromto[axis] + fromto[axis + 3]) * 0.5;
            const Vec3 delta{fromto[3] - fromto[0], fromto[4] - fromto[1],
                             fromto[5] - fromto[2]};
            contact.normal = normalize(delta);
            // Neither body can receive a MuJoCo contact force.
            contact.active = false;
            out.push_back(contact);
        }
        return NKSIM_OK;
    }

private:
    void clear_topology_backup() noexcept {
        saved_bodies.clear();
        saved_body_order.clear();
        saved_joints.clear();
        saved_joint_order.clear();
        saved_couplings.clear();
        saved_closures.clear();
        saved_contact_pairs.clear();
    }

    const JointRecord *parent_joint(std::uint64_t body_id) const {
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            if (joint.desc.body_b == body_id)
                return &joint;
        }
        return nullptr;
    }

    std::uint64_t parent_joint_id(std::uint64_t body_id) const {
        for (const auto joint_id : joint_order) {
            if (joints.at(joint_id).desc.body_b == body_id)
                return joint_id;
        }
        return 0;
    }

    bool has_children(std::uint64_t body_id) const {
        for (const auto joint_id : joint_order)
            if (joints.at(joint_id).desc.body_a == body_id)
                return true;
        return false;
    }

    // A KINEMATIC root that carries no other bodies moves on a free joint
    // (see configure_body). One with bodies beneath it, such as a robot's
    // base, stays welded to its scripted pose: a free joint's own dynamics
    // leak into the joints beneath it, however stiff it is made.
    bool free_kinematic_root(std::uint64_t body_id) const {
        const auto &record = bodies.at(body_id);
        return record.desc.motion_type == NKSIM_MOTION_KINEMATIC &&
               parent_joint_id(body_id) == 0 && !has_children(body_id);
    }

    bool would_create_cycle(std::uint64_t parent, std::uint64_t child) const {
        auto current = parent;
        while (current != 0) {
            if (current == child)
                return true;
            const auto *joint = parent_joint(current);
            current = joint ? joint->desc.body_a : 0;
        }
        return false;
    }

    nksim_result validate_topology() const {
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            if (bodies.find(joint.desc.body_a) == bodies.end() ||
                bodies.find(joint.desc.body_b) == bodies.end())
                return NKSIM_ERROR_INVALID_STATE;
            if (joint.desc.body_a == joint.desc.body_b ||
                would_create_cycle(joint.desc.body_a, joint.desc.body_b))
                return NKSIM_ERROR_INVALID_STATE;
        }
        for (const auto body_id : body_order) {
            bool found = false;
            for (const auto joint_id : joint_order) {
                if (joints.at(joint_id).desc.body_b != body_id)
                    continue;
                if (found)
                    return NKSIM_ERROR_INVALID_STATE;
                found = true;
            }
        }
        return NKSIM_OK;
    }

    nksim_result rebuild() {
        if (!spec)
            return NKSIM_ERROR_INVALID_STATE;
        const auto topology_result = validate_topology();
        if (topology_result != NKSIM_OK)
            return topology_result;

        auto *world = mjs_findBody(spec, "world");
        if (!world)
            return NKSIM_ERROR_BACKEND;
        // Adjacent articulated bodies must not fight their own joint through
        // contact, including a child connected to a world-welded static root.
        std::vector<mjsElement *> exclusions;
        for (auto *element = mjs_firstElement(spec, mjOBJ_EXCLUDE); element;
             element = mjs_nextElement(spec, element)) exclusions.push_back(element);
        for (auto *element = mjs_firstElement(spec, mjOBJ_PAIR); element;
             element = mjs_nextElement(spec, element)) exclusions.push_back(element);
        for (auto *element : exclusions)
            if (mjs_delete(spec, element) != 0) return NKSIM_ERROR_BACKEND;
        std::vector<mjsElement *> elements;
        for (auto *element = mjs_firstElement(spec, mjOBJ_BODY); element;
             element = mjs_nextElement(spec, element)) {
            // Deleting a body releases its complete subtree. Only remove the
            // direct world children; deleting every body element separately
            // leaves MuJoCo's internal detached-element list inconsistent.
            if (mjs_getParent(element) == world)
                elements.push_back(element);
        }
        for (auto element = elements.rbegin(); element != elements.rend(); ++element) {
            if (mjs_delete(spec, *element) != 0)
                return NKSIM_ERROR_BACKEND;
        }

        std::vector<mjsElement *> actuators;
        for (auto *element = mjs_firstElement(spec, mjOBJ_ACTUATOR); element;
             element = mjs_nextElement(spec, element)) {
            actuators.push_back(element);
        }
        for (auto element = actuators.rbegin(); element != actuators.rend(); ++element) {
            if (mjs_delete(spec, *element) != 0)
                return NKSIM_ERROR_BACKEND;
        }
        std::vector<mjsElement *> equalities;
        for (auto *element = mjs_firstElement(spec, mjOBJ_EQUALITY); element;
             element = mjs_nextElement(spec, element)) equalities.push_back(element);
        for (auto element = equalities.rbegin(); element != equalities.rend(); ++element)
            if (mjs_delete(spec, *element) != 0) return NKSIM_ERROR_BACKEND;
        std::vector<mjsElement *> meshes;
        for (auto *element = mjs_firstElement(spec, mjOBJ_MESH); element;
             element = mjs_nextElement(spec, element)) meshes.push_back(element);
        for (auto element = meshes.rbegin(); element != meshes.rend(); ++element)
            if (mjs_delete(spec, *element) != 0) return NKSIM_ERROR_BACKEND;

        rebuild_bodies.clear();
        std::unordered_set<std::uint64_t> added;
        for (const auto body_id : body_order) {
            if (parent_joint_id(body_id) == 0) {
                const auto result = add_body(body_id, world, added);
                if (result != NKSIM_OK)
                    return result;
            }
        }
        if (added.size() != bodies.size())
            return NKSIM_ERROR_INVALID_STATE;
        const auto exclude_result = add_self_collision_excludes();
        if (exclude_result != NKSIM_OK)
            return exclude_result;
        const auto pair_result = add_contact_pairs();
        if (pair_result != NKSIM_OK)
            return pair_result;
        const auto actuator_result = add_joint_actuators();
        if (actuator_result != NKSIM_OK)
            return actuator_result;
        for (const auto &coupling : couplings) {
            const auto leader = joints.find(coupling.leader);
            const auto follower = joints.find(coupling.follower);
            if (leader == joints.end() || follower == joints.end())
                return NKSIM_ERROR_INVALID_STATE;
            auto *equality = mjs_addEquality(spec, nullptr);
            if (!equality) return NKSIM_ERROR_OUT_OF_MEMORY;
            equality->type = mjEQ_JOINT;
            equality->objtype = mjOBJ_JOINT;
            mjs_setString(equality->name1, follower->second.name.c_str());
            mjs_setString(equality->name2, leader->second.name.c_str());
            equality->data[0] = coupling.offset;
            equality->data[1] = coupling.ratio;
        }
        for (std::size_t closure_index = 0; closure_index < closures.size(); ++closure_index) {
            const auto &closure = closures[closure_index];
            const auto &first = bodies.at(closure.body_a).name;
            const auto &second = bodies.at(closure.body_b).name;
            if (closure.type == NKSIM_JOINT_PRISMATIC) {
                auto *parent = rebuild_bodies.at(closure.body_a);
                auto *aux = mjs_addBody(parent, nullptr);
                if (!aux) return NKSIM_ERROR_OUT_OF_MEMORY;
                const auto aux_name = first + "_slide_closure_" + std::to_string(closure_index);
                if (mjs_setName(aux->element, aux_name.c_str()) != 0)
                    return NKSIM_ERROR_BACKEND;
                const auto &parent_pose = bodies.at(closure.body_a).desc;
                const auto &child_pose = bodies.at(closure.body_b).desc;
                const auto parent_q = normalize(parent_pose.rotation);
                const auto child_q = normalize(child_pose.rotation);
                const auto relative_q = normalize(multiply(conjugate(parent_q), child_q));
                const auto relative_p = rotate(conjugate(parent_q),
                    subtract(child_pose.position, parent_pose.position));
                write_pose(relative_p, relative_q, *aux);
                aux->explicitinertial = 1;
                aux->mass = 1e-6;
                aux->inertia[0] = aux->inertia[1] = aux->inertia[2] = 1e-8;
                auto *slide = mjs_addJoint(aux, nullptr);
                if (!slide) return NKSIM_ERROR_OUT_OF_MEMORY;
                slide->type = mjJNT_SLIDE;
                const auto axis = normalize(rotate(conjugate(relative_q), closure.axis_a));
                std::copy(axis.begin(), axis.end(), slide->axis);
                auto *equality = mjs_addEquality(spec, nullptr);
                if (!equality) return NKSIM_ERROR_OUT_OF_MEMORY;
                equality->type = mjEQ_WELD;
                equality->objtype = mjOBJ_BODY;
                mjs_setString(equality->name1, aux_name.c_str());
                mjs_setString(equality->name2, second.c_str());
                continue;
            }
            const int count = closure.type == NKSIM_JOINT_FIXED ? 1 : 2;
            for (int point = 0; point < count; ++point) {
                auto *equality = mjs_addEquality(spec, nullptr);
                if (!equality) return NKSIM_ERROR_OUT_OF_MEMORY;
                equality->type = closure.type == NKSIM_JOINT_FIXED ? mjEQ_WELD : mjEQ_CONNECT;
                equality->objtype = mjOBJ_BODY;
                mjs_setString(equality->name1, first.c_str());
                mjs_setString(equality->name2, second.c_str());
                for (int axis = 0; axis < 3; ++axis)
                    equality->data[axis] = closure.anchor_a[axis] +
                        point * 0.1 * closure.axis_a[axis];
            }
        }

        if (model && data) {
            if (mj_recompile(spec, nullptr, model, data) != 0) {
                model = nullptr;
                data = nullptr;
                return NKSIM_ERROR_BACKEND;
            }
        } else {
            model = mj_compile(spec, nullptr);
            if (!model)
                return NKSIM_ERROR_BACKEND;
            data = mj_makeData(model);
            if (!data) {
                mj_deleteModel(model);
                model = nullptr;
                return NKSIM_ERROR_OUT_OF_MEMORY;
            }
        }

        restore_model_state();
        geom_owner.clear();
        for (const auto body_id : body_order) {
            const auto &record = bodies.at(body_id);
            for (std::size_t part = 0; part < record.desc.shape_parts.size(); ++part) {
                const auto name = record.name + "_part_" + std::to_string(part);
                const auto geom_id = mj_name2id(model, mjOBJ_GEOM, name.c_str());
                if (geom_id >= 0) geom_owner.emplace(geom_id,
                    std::make_pair(body_id, static_cast<std::int32_t>(part)));
            }
        }
        proximity_candidates.clear();
        for (int first = 0; first < model->ngeom; ++first) {
            const auto first_owner = geom_owner.find(first);
            if (first_owner == geom_owner.end() ||
                bodies.at(first_owner->second.first).desc.motion_type == NKSIM_MOTION_DYNAMIC)
                continue;
            for (int second = first + 1; second < model->ngeom; ++second) {
                const auto second_owner = geom_owner.find(second);
                if (second_owner == geom_owner.end() ||
                    first_owner->second.first == second_owner->second.first ||
                    bodies.at(second_owner->second.first).desc.motion_type == NKSIM_MOTION_DYNAMIC)
                    continue;
                if ((model->geom_contype[first] & model->geom_conaffinity[second]) == 0 &&
                    (model->geom_contype[second] & model->geom_conaffinity[first]) == 0)
                    continue;
                const auto low = std::min(first_owner->second.first, second_owner->second.first);
                const auto high = std::max(first_owner->second.first, second_owner->second.first);
                if (real_excludes.count({low, high}) != 0) continue;
                const double detection = model->geom_margin[first] + model->geom_margin[second] +
                    model->geom_gap[first] + model->geom_gap[second];
                if (detection > 0.0) proximity_candidates.push_back({first, second, detection});
            }
        }
        apply_joint_targets();
        mj_forward(model, data);
        return NKSIM_OK;
    }

    // F2: a single motor actuator per non-fixed joint. apply_joint_targets
    // computes the actual torque every substep (position/velocity gains
    // scaled by the joint's own mass-matrix diagonal, plus gravity/Coriolis
    // compensation), rather than relying on MuJoCo's built-in position and
    // velocity actuators, whose bias (-kp*q - kv*qdot for a position
    // actuator) applies even at ctrl=0 — an idle position actuator dragged a
    // velocity- or effort-commanded joint back toward q=0.
    // F4: within one articulation, keep direct parent/child pairs out of contact,
    // and keep pairs that already overlap in the authored rest pose out of contact.
    // Unrelated environment bodies must never inherit a self-collision exclusion.
    // Other links
    // in one articulation remain collision-enabled, so a folded arm can
    // contact its own base instead of passing through it. Rest-pose geometry
    // is tested with an oriented-box SAT for the shapes RobotKit supplies;
    // the generic sphere/capsule fallback is conservative for those shapes.
    //
    // A KINEMATIC root now carries a real free joint (configure_body), so it
    // no longer gets MuJoCo's own free pass on collision between two
    // dof-less bodies (engine_collision_driver.c's mj_collision: the
    // exclude_signature check — see below — sits at the exact point in the
    // pipeline that used to be preceded by filterBodyPair's "both dof-less:
    // no forces can act, skip"; that skip is now gone for a KINEMATIC body,
    // since it has dof). Neither a KINEMATIC nor a STATIC body can ever be
    // moved by a contact (both are pinned to a prescribed pose every
    // substep or forever), so a contact between two non-DYNAMIC bodies —
    // two overlapping kinematic actor capsules, or a kinematic robot base
    // resting on the static floor — could still never do anything physical;
    // it would only waste solver work and couple bodies that can't respond
    // into the same constraint island as real contacts. This loop restores
    // that old skip explicitly, for any pair where neither body is DYNAMIC,
    // regardless of articulation.
    //
    // Excludes, not contype/conaffinity, restore it: mjs_addExclude adds a
    // body pair (by name) to model->exclude_signature, which mj_collision
    // consults right after broadphase and before any narrowphase geom work
    // (the same early, cheap point where the self-collision excludes below
    // already rely on it) — an excluded pair costs nothing further and
    // never appears in d->contact[]. contype/conaffinity is a per-GEOM
    // bitmask copied directly, bit for bit, from SimKit's own
    // collision_layer/collision_mask (configure_body); tagging "is this
    // body's owner DYNAMIC" into it would mean reserving a bit out of that
    // caller-controlled mask, which is not available to steal without
    // narrowing what collision_layer/collision_mask can express for every
    // body, DYNAMIC or not. Excludes leave every pair that involves a
    // DYNAMIC body — including a KINEMATIC body's contact and friction
    // against one — governed by exactly the same contype/conaffinity logic,
    // and exactly the same collision_layer/collision_mask semantics, as
    // before this whole fix.
    nksim_result add_self_collision_excludes() {
        real_excludes.clear();
        const auto is_same_articulation = [&](std::uint64_t first, std::uint64_t second) {
            return same_articulation(first, second);
        };
        auto is_parent_child = [&](std::uint64_t first, std::uint64_t second) {
            for (const auto joint_id : joint_order) {
                const auto &joint = joints.at(joint_id);
                if ((joint.desc.body_a == first && joint.desc.body_b == second) ||
                    (joint.desc.body_a == second && joint.desc.body_b == first))
                    return true;
            }
            return false;
        };
        for (std::size_t i = 0; i < body_order.size(); ++i) {
            for (std::size_t j = i + 1; j < body_order.size(); ++j) {
                const auto first = body_order[i], second = body_order[j];
                const auto &body_a = bodies.at(first), &body_b = bodies.at(second);
                // A held free body may become dynamic again without another
                // rebuild. Keep its static contacts in the compiled model.
                const auto can_be_dynamic = [](const BodyRecord &body) {
                    return body.desc.motion_type == NKSIM_MOTION_DYNAMIC ||
                        (body.desc.motion_type == NKSIM_MOTION_KINEMATIC && body.free_inertia_saved);
                };
                const bool neither_dynamic = !can_be_dynamic(body_a) && !can_be_dynamic(body_b);
                const bool real_exclude = is_same_articulation(first, second) &&
                    (is_parent_child(first, second) || geometries_overlap_at_rest(body_a, body_b));
                if (real_exclude) real_excludes.emplace(std::min(first, second), std::max(first, second));
                const bool needs_exclude = neither_dynamic || real_exclude;
                if (!needs_exclude) continue;
                auto *exclude = mjs_addExclude(spec);
                if (!exclude) return NKSIM_ERROR_OUT_OF_MEMORY;
                mjs_setString(exclude->bodyname1, body_a.name.c_str());
                mjs_setString(exclude->bodyname2, body_b.name.c_str());
            }
        }
        return NKSIM_OK;
    }

    // Whether joints connect two bodies, directly or through others.
    bool same_articulation(std::uint64_t first, std::uint64_t second) const {
        std::vector<std::uint64_t> visited{first};
        for (std::size_t index = 0; index < visited.size(); ++index) {
            const auto current = visited[index];
            for (const auto joint_id : joint_order) {
                const auto &joint = joints.at(joint_id);
                const auto next = joint.desc.body_a == current ? joint.desc.body_b :
                    joint.desc.body_b == current ? joint.desc.body_a : 0;
                if (next == second) return true;
                if (next != 0 && std::find(visited.begin(), visited.end(), next) == visited.end())
                    visited.push_back(next);
            }
        }
        return false;
    }

    // Explicit pairs, and for each part that also meets the environment, one
    // pair with every layered part of every body outside its articulation.
    nksim_result add_contact_pairs() {
        const auto geom_name = [&](std::uint64_t body, std::size_t part) {
            return bodies.at(body).name + "_part_" + std::to_string(part);
        };
        const auto add_pair = [&](std::uint64_t body_a, std::size_t part_a, std::uint64_t body_b,
                                  std::size_t part_b, const nksim::BackendShapePart &surface) {
            auto *pair = mjs_addPair(spec, nullptr);
            if (!pair) return NKSIM_ERROR_OUT_OF_MEMORY;
            mjs_setString(pair->geomname1, geom_name(body_a, part_a).c_str());
            mjs_setString(pair->geomname2, geom_name(body_b, part_b).c_str());
            // Zero surface fields keep MuJoCo's pair defaults.
            if (surface.friction_dimensions != 0)
                pair->condim = static_cast<int>(surface.friction_dimensions);
            if (surface.friction[0] > 0.0) pair->friction[0] = pair->friction[1] = surface.friction[0];
            if (surface.friction[1] > 0.0) pair->friction[2] = surface.friction[1];
            if (surface.friction[2] > 0.0) pair->friction[3] = pair->friction[4] = surface.friction[2];
            if (surface.contact_time_constant > 0.0) pair->solref[0] = surface.contact_time_constant;
            if (surface.contact_damping_ratio > 0.0) pair->solref[1] = surface.contact_damping_ratio;
            const auto &a = bodies.at(body_a).desc.shape_parts[part_a];
            const auto &b = bodies.at(body_b).desc.shape_parts[part_b];
            pair->margin = std::max(a.margin, b.margin);
            pair->gap = std::max(a.gap, b.gap);
            return NKSIM_OK;
        };
        for (const auto &pair : contact_pairs) {
            const auto result = add_pair(pair.body_a, pair.part_a, pair.body_b, pair.part_b, pair.surface);
            if (result != NKSIM_OK) return result;
        }
        // A part that meets the environment collides, as by layers, with every
        // layered part outside its own articulation: environment objects and
        // other robots' links alike, so a humanoid's foot meets a machine bed
        // as it meets the floor. A body on no layer (a link with no shape
        // under the "none" approximation) collides with nothing, and a part
        // that collides only through pairs is not part of the environment.
        std::unordered_map<std::uint64_t, std::uint64_t> group;
        const auto find = [&](std::uint64_t body) {
            while (group.count(body) != 0 && group[body] != body) body = group[body];
            return body;
        };
        for (const auto body_id : body_order) group[body_id] = body_id;
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            group[find(joint.desc.body_a)] = find(joint.desc.body_b);
        }
        for (const auto body_id : body_order) {
            const auto &body = bodies.at(body_id).desc;
            const auto &parts = body.shape_parts;
            for (std::size_t part = 0; part < parts.size(); ++part) {
                if (parts[part].contact_filter != NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT) continue;
                for (const auto other : body_order) {
                    if (find(other) == find(body_id)) continue;
                    const auto &other_body = bodies.at(other).desc;
                    if ((body.collision_layer & other_body.collision_mask) == 0 &&
                        (other_body.collision_layer & body.collision_mask) == 0)
                        continue;
                    for (std::size_t other_part = 0;
                         other_part < other_body.shape_parts.size(); ++other_part) {
                        if (other_body.shape_parts[other_part].contact_filter != NKSIM_CONTACT_LAYERS)
                            continue;
                        const auto result = add_pair(body_id, part, other, other_part, parts[part]);
                        if (result != NKSIM_OK) return result;
                    }
                }
            }
        }
        return NKSIM_OK;
    }

    nksim_result add_joint_actuators() {
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            if (joint.desc.type == NKSIM_JOINT_FIXED)
                continue;
            auto *actuator = mjs_addActuator(spec, nullptr);
            if (!actuator)
                return NKSIM_ERROR_OUT_OF_MEMORY;
            const auto name = joint.name + "_motor";
            if (mjs_setName(actuator->element, name.c_str()) != 0)
                return NKSIM_ERROR_BACKEND;
            actuator->trntype = mjTRN_JOINT;
            mjs_setString(actuator->target, joint.name.c_str());
            const char *error = mjs_setToMotor(actuator);
            if (error && error[0] != '\0')
                return NKSIM_ERROR_BACKEND;
            // NKSIM_JOINT_TARGET_SERVO runs as MuJoCo's own affine servo, so
            // integrators that treat velocity-dependent actuator force
            // implicitly (implicitfast) treat its damping implicitly too, as
            // for an MJCF position actuator. apply_joint_targets sets its
            // gains each step; they stay zero, and it exerts nothing, while
            // the joint is in another mode.
            auto *servo = mjs_addActuator(spec, nullptr);
            if (!servo)
                return NKSIM_ERROR_OUT_OF_MEMORY;
            const auto servo_name = joint.name + "_servo";
            if (mjs_setName(servo->element, servo_name.c_str()) != 0)
                return NKSIM_ERROR_BACKEND;
            servo->trntype = mjTRN_JOINT;
            mjs_setString(servo->target, joint.name.c_str());
            servo->gaintype = mjGAIN_FIXED;
            servo->biastype = mjBIAS_AFFINE;
            servo->ctrllimited = mjLIMITED_FALSE;
            // Its limit is the joint's actuator force range, as for an MJCF
            // actuatorfrcrange: MuJoCo keeps a joint-clamped actuator in the
            // implicit velocity derivative, but drops one clamped by its own
            // forcerange.
            servo->forcelimited = mjLIMITED_FALSE;
        }
        return NKSIM_OK;
    }

    nksim_result add_body(std::uint64_t id, mjsBody *parent,
                          std::unordered_set<std::uint64_t> &added) {
        const auto found = bodies.find(id);
        if (found == bodies.end() || !parent)
            return NKSIM_ERROR_INVALID_STATE;
        if (!added.insert(id).second)
            return NKSIM_ERROR_INVALID_STATE;

        auto *body = mjs_addBody(parent, nullptr);
        if (!body)
            return NKSIM_ERROR_OUT_OF_MEMORY;
        if (mjs_setName(body->element, found->second.name.c_str()) != 0)
            return NKSIM_ERROR_BACKEND;
        rebuild_bodies[id] = body;

        const auto *incoming = parent_joint(id);
        const auto *parent_record = incoming ? &bodies.at(incoming->desc.body_a) : nullptr;
        if (incoming && parent_record &&
            !joint_frame_matches_rest_pose(incoming->desc, parent_record->desc, found->second.desc))
            return NKSIM_ERROR_INVALID_STATE;
        // Rebuild articulations from rest transforms. Using the live pose here
        // and then restoring qpos applies joint displacement twice.
        const auto world_position = incoming ? found->second.desc.position : found->second.state.position;
        const auto world_rotation = normalize(incoming ? found->second.desc.rotation : found->second.state.rotation);
        Vec3 local_position = world_position;
        Quat local_rotation = world_rotation;
        if (parent_record) {
            const auto parent_rotation = normalize(parent_record->desc.rotation);
            const auto inverse_parent = conjugate(parent_rotation);
            local_position = rotate(inverse_parent,
                                    subtract(world_position, parent_record->desc.position));
            local_rotation = normalize(multiply(inverse_parent, world_rotation));
        }
        write_pose(local_position, local_rotation, *body);

        const auto body_result = configure_body(*body, found->second.desc, incoming,
                                                free_kinematic_root(id), found->second.name);
        if (body_result != NKSIM_OK)
            return body_result;

        for (const auto child_id : body_order) {
            const auto *joint = parent_joint(child_id);
            if (!joint || joint->desc.body_a != id)
                continue;
            const auto result = add_body(child_id, body, added);
            if (result != NKSIM_OK)
                return result;
        }
        return NKSIM_OK;
    }

    nksim_result configure_body(mjsBody &body,
                                       const nksim::BackendBodyDesc &desc,
                                       const JointRecord *incoming,
                                       bool kinematic_root,
                                       const std::string &body_name) {
        // A KINEMATIC root (e.g. a walking actor's capsule, or a robot's own
        // base link) is externally scripted from its owning World's scene
        // node or an nksim_body_drive() every outer step
        // (World::refresh_kinematic_bodies) — it must never be pushed by
        // contact forces, and it must still make its motion visible to
        // MuJoCo's contact solver so friction/velocity transfer with a
        // DYNAMIC body works.
        //
        // A plain MuJoCo "mocap" body (mjsBody.mocap = true) does not do
        // this: a mocap body has zero degrees of freedom, and MuJoCo
        // explicitly special-cases dof-less bodies twice in the vendored
        // source. mj_objectVelocity/mj_objectAcceleration
        // (engine_core_util.c: "dof-less body (static or mocap): quick
        // return") always report zero velocity for one, and
        // filterBodyPair() in engine_collision_driver.c skips collision
        // detection outright "if (dofnum1 == 0 && dofnum2 == 0)" ("both
        // dof-less: no forces can act, skip") — i.e. even the contacts that
        // *are* generated against a mocap body (it can carry colliding
        // geoms; only the KINEMATIC-vs-STATIC/KINEMATIC-vs-KINEMATIC case is
        // skipped, since a DYNAMIC body always has dof) are resolved with
        // zero relative velocity on the mocap side — exactly the bug this
        // fix targets, not a solution to it.
        //
        // So a KINEMATIC root that carries no other bodies instead gets a
        // genuine free joint, exactly like a DYNAMIC body, giving it real degrees of freedom that
        // participate normally in MuJoCo's contact Jacobian and cvel.
        // step() below re-places its qpos/qvel onto the exact prescribed
        // trajectory before every physics substep, not just once per outer
        // step (re-pinning only once per OUTER step would leave the body
        // sitting still through every substep in between — the same
        // teleport problem, one level down — and would let a substep's
        // contact force give it spurious velocity).
        // Re-placing before every substep means the solver
        // always sees the correct instantaneous relative velocity for
        // friction, while any reaction it computes back onto this body is
        // discarded before it can move the body or accumulate.
        //
        // The mass/inertia below are nominal: World zeroes desc.mass for
        // anything but a DYNAMIC body, and this body's own dynamics never
        // actually run (step() pins it every substep), so the values only
        // set how much of a contact impulse a DYNAMIC body picks up on
        // impact. kKinematicBodyMass is chosen well above any plausible
        // dynamic payload so that impulse approximates what an immovable
        // obstacle of the same velocity would give.
        // Gravity compensation keeps its own weight from pulling it off its
        // path within a substep, under whatever rests on it.
        if (kinematic_root)
            body.gravcomp = 1.0;
        if (desc.motion_type == NKSIM_MOTION_DYNAMIC || kinematic_root) {
            body.mass = desc.motion_type == NKSIM_MOTION_DYNAMIC ? desc.mass : kKinematicBodyMass;
            if (body.mass > 0.0) {
                body.explicitinertial = 1;
                if (desc.motion_type == NKSIM_MOTION_DYNAMIC && desc.has_inertial_properties) {
                    std::copy(desc.center_of_mass.begin(), desc.center_of_mass.end(), body.ipos);
                    const auto &m = desc.inertia_tensor;
                    body.fullinertia[0] = m[0];
                    body.fullinertia[1] = m[4];
                    body.fullinertia[2] = m[8];
                    body.fullinertia[3] = m[1];
                    body.fullinertia[4] = m[2];
                    body.fullinertia[5] = m[5];
                } else {
                    // Centre of mass at the body origin. Left undefined, MuJoCo
                    // copies the body's parent-relative position into ipos and
                    // displaces the centre of mass by that offset.
                    std::fill(body.ipos, body.ipos + 3, 0.0);
                    body.inertia[0] = body.inertia[1] = body.inertia[2] = 1.0;
                }
            }
        }

        if (!incoming) {
            if (desc.motion_type == NKSIM_MOTION_DYNAMIC || kinematic_root) {
                auto *free_joint = mjs_addFreeJoint(&body);
                if (!free_joint)
                    return NKSIM_ERROR_OUT_OF_MEMORY;
            }
        } else if (incoming->desc.type != NKSIM_JOINT_FIXED) {
            auto *joint = mjs_addJoint(&body, nullptr);
            if (!joint)
                return NKSIM_ERROR_OUT_OF_MEMORY;
            if (mjs_setName(joint->element, incoming->name.c_str()) != 0)
                return NKSIM_ERROR_BACKEND;
            joint->type = incoming->desc.type == NKSIM_JOINT_REVOLUTE
                ? mjJNT_HINGE : mjJNT_SLIDE;
            // Place the joint at the joint frame in body-local (child)
            // coordinates: anchor_b/rotation_b, per F1. axis_a is the
            // joint-frame axis rotated into body_a's frame by rotation_a;
            // undoing rotation_a recovers the raw joint-frame axis, then
            // rotation_b re-expresses it in body_b's (this body's) frame —
            // this needs only per-joint local data, not the bodies' world
            // rest rotations.
            std::copy(incoming->desc.anchor_b.begin(), incoming->desc.anchor_b.end(), joint->pos);
            const auto axis_joint_frame = rotate(conjugate(normalize(incoming->desc.rotation_a)),
                                                 normalize(incoming->desc.axis_a));
            const auto axis_local = normalize(rotate(normalize(incoming->desc.rotation_b), axis_joint_frame));
            std::copy(axis_local.begin(), axis_local.end(), joint->axis);
            if (incoming->desc.lower_limit < incoming->desc.upper_limit) {
                joint->limited = mjLIMITED_TRUE;
                joint->range[0] = incoming->desc.lower_limit;
                joint->range[1] = incoming->desc.upper_limit;
            } else {
                joint->limited = mjLIMITED_FALSE;
            }
            joint->armature = incoming->desc.armature;
            joint->damping[0] = incoming->desc.damping;
            joint->frictionloss = incoming->desc.friction_loss;
            if (incoming->desc.limit_time_constant > 0.0)
                joint->solref_limit[0] = incoming->desc.limit_time_constant;
            if (incoming->desc.limit_damping_ratio > 0.0)
                joint->solref_limit[1] = incoming->desc.limit_damping_ratio;
            const auto &impedance = incoming->desc.limit_impedance;
            if (std::any_of(impedance.begin(), impedance.end(), [](double v) { return v != 0.0; }))
                std::copy(impedance.begin(), impedance.end(), joint->solimp_limit);
        }

        if (desc.shape_parts.empty())
            return NKSIM_OK;
        for (std::size_t index = 0; index < desc.shape_parts.size(); ++index) {
            const auto &part = desc.shape_parts[index];
            auto *geom = mjs_addGeom(&body, nullptr);
            if (!geom) return NKSIM_ERROR_OUT_OF_MEMORY;
            const auto geom_name = body_name + "_part_" + std::to_string(index);
            if (mjs_setName(geom->element, geom_name.c_str()) != 0)
                return NKSIM_ERROR_BACKEND;
            switch (part.type) {
            case NKSIM_SHAPE_BOX:
                geom->type = mjGEOM_BOX;
                break;
            case NKSIM_SHAPE_SPHERE:
                geom->type = mjGEOM_SPHERE;
                break;
            case NKSIM_SHAPE_CAPSULE:
                geom->type = mjGEOM_CAPSULE;
                break;
            case NKSIM_SHAPE_CYLINDER:
                geom->type = mjGEOM_CYLINDER;
                break;
            case NKSIM_SHAPE_PLANE:
                geom->type = mjGEOM_PLANE;
                break;
            case NKSIM_SHAPE_CONVEX: {
                if (part.vertices.size() < 12 || part.vertices.size() > 192 ||
                    part.vertices.size() % 3 != 0)
                    return NKSIM_ERROR_INVALID_ARGUMENT;
                auto *mesh = mjs_addMesh(spec, nullptr);
                if (!mesh) return NKSIM_ERROR_OUT_OF_MEMORY;
                const auto name = *mjs_getName(body.element) + "_collision_" + std::to_string(index);
                if (mjs_setName(mesh->element, name.c_str()) != 0)
                    return NKSIM_ERROR_BACKEND;
                mjs_setFloat(mesh->uservert, part.vertices.data(),
                             static_cast<int>(part.vertices.size()));
                mesh->maxhullvert = 64;
                geom->type = mjGEOM_MESH;
                mjs_setString(geom->meshname, name.c_str());
                break;
            }
            default:
                return NKSIM_ERROR_UNSUPPORTED;
            }
            if (part.type == NKSIM_SHAPE_PLANE) {
                const Vec3 normal = normalize(Vec3{part.parameters[0],
                                                   part.parameters[1],
                                                   part.parameters[2]});
                geom->pos[0] = normal[0] * part.parameters[3];
                geom->pos[1] = normal[1] * part.parameters[3];
                geom->pos[2] = normal[2] * part.parameters[3];
                const auto rotation = rotation_from_z(normal);
                geom->quat[0] = rotation[3];
                geom->quat[1] = rotation[0];
                geom->quat[2] = rotation[1];
                geom->quat[3] = rotation[2];
                // A zero x/y size is MuJoCo's infinite-plane convention.
                geom->size[0] = 0.0;
                geom->size[1] = 0.0;
                geom->size[2] = 1.0;
            } else {
                std::copy(part.position.begin(), part.position.end(), geom->pos);
                geom->quat[0] = part.rotation[3];
                geom->quat[1] = part.rotation[0];
                geom->quat[2] = part.rotation[1];
                geom->quat[3] = part.rotation[2];
                if (part.type != NKSIM_SHAPE_CONVEX) {
                    geom->size[0] = part.parameters[0];
                    // MuJoCo sizes capsules and cylinders by half-length.
                    geom->size[1] = part.type == NKSIM_SHAPE_CAPSULE ||
                            part.type == NKSIM_SHAPE_CYLINDER
                        ? part.parameters[1] * 0.5 : part.parameters[1];
                    geom->size[2] = part.parameters[2];
                }
            }
            // This pinned MuJoCo adds geom margin and gap for detection, then
            // creates a force constraint only below geom margin.
            geom->margin = part.margin;
            geom->gap = part.gap;
            // Zero surface fields keep MuJoCo's defaults.
            if (part.friction_dimensions != 0)
                geom->condim = static_cast<int>(part.friction_dimensions);
            for (int axis = 0; axis < 3; ++axis)
                if (part.friction[axis] > 0.0) geom->friction[axis] = part.friction[axis];
            if (part.contact_time_constant > 0.0) geom->solref[0] = part.contact_time_constant;
            if (part.contact_damping_ratio > 0.0) geom->solref[1] = part.contact_damping_ratio;
            // A part that collides through pairs stays out of layer collision.
            const bool layered = part.contact_filter == NKSIM_CONTACT_LAYERS;
            geom->contype = layered ? static_cast<int>(desc.collision_layer) : 0;
            geom->conaffinity = layered ? static_cast<int>(desc.collision_mask) : 0;
        }
        return NKSIM_OK;
    }

    void restore_model_state() {
        for (const auto body_id : body_order) {
            const auto found = bodies.find(body_id);
            const auto model_id = model_body_id(body_id);
            if (found == bodies.end() || model_id < 0)
                continue;
            const auto joint_id = model->body_jntadr[model_id];
            if (joint_id >= 0 && model->jnt_type[joint_id] == mjJNT_FREE)
                set_free_body_state(model_id, found->second.state);
        }
        for (const auto joint_id : joint_order) {
            const auto found = joints.find(joint_id);
            if (found == joints.end())
                continue;
            const auto model_id = model_joint_id(found->second);
            if (model_id < 0 || (model->jnt_type[model_id] != mjJNT_HINGE &&
                                 model->jnt_type[model_id] != mjJNT_SLIDE))
                continue;
            data->qpos[model->jnt_qposadr[model_id]] = found->second.state.position;
            data->qvel[model->jnt_dofadr[model_id]] = found->second.state.velocity;
        }
    }

    void set_free_body_state(int body_id, const nksim::BackendBodyState &state) {
        const auto joint_id = model->body_jntadr[body_id];
        const auto qpos = model->jnt_qposadr[joint_id];
        const auto qvel = model->jnt_dofadr[joint_id];
        data->qpos[qpos + 0] = state.position[0];
        data->qpos[qpos + 1] = state.position[1];
        data->qpos[qpos + 2] = state.position[2];
        const auto rotation = normalize(state.rotation);
        data->qpos[qpos + 3] = rotation[3];
        data->qpos[qpos + 4] = rotation[0];
        data->qpos[qpos + 5] = rotation[1];
        data->qpos[qpos + 6] = rotation[2];
        std::copy(state.linear_velocity.begin(), state.linear_velocity.end(), data->qvel + qvel);
        const auto local_angular = rotate(conjugate(rotation), state.angular_velocity);
        std::copy(local_angular.begin(), local_angular.end(), data->qvel + qvel + 3);
    }

    // Full computed-torque control: tau = M * qacc_desired + qfrc_bias,
    // via mj_mulM (M applied as an operator over the WHOLE system, never
    // materialized densely) rather than a single diagonal entry. This
    // replaces two bugs at once: (1) data->M is a sparse, per-dof-ROW
    // format where dof_Madr[dof] addresses the *start* of that dof's row
    // (ancestor dofs first, own diagonal last — see mj_mulM's own row-length
    // computation via dof_Madr[j+1]-dof_Madr[j] in engine_derivative.c), not
    // the diagonal itself, so a naive M[dof_Madr[dof]] read is only
    // coincidentally correct for a dof with no ancestor dofs; (2) even with
    // the right diagonal, a single-DOF PD gain has no cross-joint
    // compensation, which showed up as steady-state coupling error on the
    // cross-backend acceptance test (M8.5). Building one desired-acceleration
    // vector over every actuated dof and multiplying by the full mass matrix
    // captures cross-joint coupling for free — mj_mulM(qacc_des) naturally
    // distributes each dof's desired acceleration through the whole
    // articulated system's inertia, not just its own row.
    void apply_joint_targets() {
        if (model->nu > 0)
            std::fill(data->ctrl, data->ctrl + model->nu, 0.0);
        const auto nv = static_cast<std::size_t>(model->nv);
        if (nv == 0)
            return;
        std::vector<mjtNum> qacc_desired(nv, 0.0);
        std::vector<mjtNum> m_qacc(nv, 0.0);

        // Effort-mode joints need no mass matrix at all; apply directly.
        // Position/velocity-mode joints contribute a desired acceleration
        // that mj_mulM below turns into torque through the full inertia.
        for (const auto joint_id : joint_order) {
            auto &joint = joints.at(joint_id);
            if (joint.target_mode != NKSIM_JOINT_TARGET_POSITION &&
                joint.target_mode != NKSIM_JOINT_TARGET_VELOCITY)
                continue;
            const auto model_id = model_joint_id(joint);
            if (model_id < 0)
                continue;
            const auto type = model->jnt_type[model_id];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
                continue;

            const auto dof = static_cast<std::size_t>(model->jnt_dofadr[model_id]);
            const auto q = data->qpos[model->jnt_qposadr[model_id]];
            const auto qdot = data->qvel[dof];

            switch (joint.target_mode) {
            case NKSIM_JOINT_TARGET_POSITION: {
                constexpr double omega_n = 2.0 * 3.14159265358979323846 * 10.0;
                constexpr double zeta = 1.0;
                qacc_desired[dof] = omega_n * omega_n * (joint.target - q) - 2.0 * zeta * omega_n * qdot;
                break;
            }
            case NKSIM_JOINT_TARGET_VELOCITY: {
                constexpr double kv = 50.0;
                qacc_desired[dof] = kv * (joint.target - qdot);
                break;
            }
            }
        }
        mj_mulM(model, data, m_qacc.data(), qacc_desired.data());

        for (const auto joint_id : joint_order) {
            auto &joint = joints.at(joint_id);
            // A plain joint-space PD with feedforward, the law a motor driver
            // runs: force = kp (target - q) + kd (velocity - qdot) + ff, as
            // gain kp on ctrl = target with an affine bias [kd v + ff, -kp, -kd].
            // Outside servo mode every term is zero, so it exerts nothing.
            const auto servo = model_actuator_id(joint, "servo");
            if (servo >= 0) {
                const bool servoing = joint.target_mode == NKSIM_JOINT_TARGET_SERVO;
                auto *servo_gain = model->actuator_gainprm + mjNGAIN * servo;
                auto *servo_bias = model->actuator_biasprm + mjNBIAS * servo;
                servo_gain[0] = servoing ? joint.target_stiffness : 0.0;
                servo_bias[0] = servoing
                    ? joint.target_damping * joint.target_velocity + joint.target_feedforward : 0.0;
                servo_bias[1] = servoing ? -joint.target_stiffness : 0.0;
                servo_bias[2] = servoing ? -joint.target_damping : 0.0;
                data->ctrl[servo] = servoing ? joint.target : 0.0;
                // Every mode clamps to this same bound, so a joint-level clamp
                // changes nothing for the motor's already-clamped torque.
                const auto joint_model = model_joint_id(joint);
                const auto limit = joint.target_max_force > 0.0
                    ? joint.target_max_force : joint.desc.max_force;
                if (joint_model >= 0) {
                    model->jnt_actfrclimited[joint_model] = limit > 0.0 ? 1 : 0;
                    model->jnt_actfrcrange[2 * joint_model] = limit > 0.0 ? -limit : 0.0;
                    model->jnt_actfrcrange[2 * joint_model + 1] = limit > 0.0 ? limit : 0.0;
                }
            }
            if (joint.target_mode == 0)
                continue;
            const auto model_id = model_joint_id(joint);
            if (model_id < 0)
                continue;
            const auto type = model->jnt_type[model_id];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
                continue;
            const auto actuator = model_actuator_id(joint, "motor");
            if (actuator < 0)
                continue;

            const auto dof = static_cast<std::size_t>(model->jnt_dofadr[model_id]);
            const auto bias = data->qfrc_bias[dof];

            double torque = 0.0;
            switch (joint.target_mode) {
            case NKSIM_JOINT_TARGET_POSITION:
            case NKSIM_JOINT_TARGET_VELOCITY:
                torque = m_qacc[dof] + bias;
                break;
            case NKSIM_JOINT_TARGET_SERVO:
                break; // The servo actuator below applies it.
            case NKSIM_JOINT_TARGET_EFFORT:
            default:
                torque = joint.target;
                break;
            }

            const auto max_force = joint.target_max_force > 0.0
                ? joint.target_max_force : joint.desc.max_force;
            if (max_force > 0.0) {
                if (torque > max_force) torque = max_force;
                if (torque < -max_force) torque = -max_force;
            }
            data->ctrl[actuator] = torque;
        }
    }

    int model_body_id(std::uint64_t id) const noexcept {
        const auto found = bodies.find(id);
        if (found == bodies.end() || !model)
            return -1;
        return mj_name2id(model, mjOBJ_BODY, found->second.name.c_str());
    }

    int model_joint_id(const JointRecord &joint) const noexcept {
        if (!model)
            return -1;
        return mj_name2id(model, mjOBJ_JOINT, joint.name.c_str());
    }

    int model_actuator_id(const JointRecord &joint, const char *mode) const noexcept {
        if (!model)
            return -1;
        const auto name = joint.name + "_" + mode;
        return mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    }

    mjSpec *spec = nullptr;
    mjModel *model = nullptr;
    mjData *data = nullptr;
    std::unordered_map<std::uint64_t, BodyRecord> bodies;
    std::unordered_map<int, std::pair<std::uint64_t, std::int32_t>> geom_owner;
    struct ProximityCandidate { int first; int second; double detection; };
    std::vector<ProximityCandidate> proximity_candidates;
    std::set<std::pair<std::uint64_t, std::uint64_t>> real_excludes;
    std::vector<std::uint64_t> body_order;
    std::unordered_map<std::uint64_t, JointRecord> joints;
    std::vector<std::uint64_t> joint_order;
    std::vector<nksim::BackendJointCoupling> couplings;
    std::vector<nksim::BackendClosure> closures;
    std::unordered_map<std::uint64_t, mjsBody *> rebuild_bodies;
    std::unordered_map<std::uint64_t, BodyRecord> saved_bodies;
    std::vector<std::uint64_t> saved_body_order;
    std::unordered_map<std::uint64_t, JointRecord> saved_joints;
    std::vector<std::uint64_t> saved_joint_order;
    std::vector<nksim::BackendJointCoupling> saved_couplings;
    std::vector<nksim::BackendClosure> saved_closures;
    std::vector<nksim::BackendContactPair> contact_pairs;
    std::vector<nksim::BackendContactPair> saved_contact_pairs;
    std::uint64_t next_body = 1;
    std::uint64_t next_joint = 1;
    std::uint64_t saved_next_body = 1;
    std::uint64_t saved_next_joint = 1;
    bool topology_update = false;
};

std::unique_ptr<nksim::PhysicsBackend> make_backend() {
    return std::make_unique<MujocoBackend>();
}

} // namespace
} // namespace nksim_mujoco

extern "C" {

uint64_t NKSIMMUJOCO_CALL nksim_mujoco_distance_call_count(void) {
    return nksim_mujoco::distance_call_count.load(std::memory_order_relaxed);
}

nksim_result NKSIMMUJOCO_CALL nksim_mujoco_world_create(
    const nksim_world_desc *desc, nksim_world *out_world) {
    if (!desc || !out_world)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    return nksim::create_world_with_backend(
        *desc, nksim_mujoco::make_backend(), out_world);
}

} // extern "C"
