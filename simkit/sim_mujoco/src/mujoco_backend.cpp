#include "nativekit_sim_mujoco.h"

#include "PhysicsBackend.hpp"
#include "internal.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nksim_mujoco {
namespace {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

struct BodyRecord {
    nksim::BackendBodyDesc desc{};
    nksim::BackendBodyState state{};
    std::string name;
};

struct JointRecord {
    nksim::BackendJointDesc desc{};
    nksim::BackendJointState state{};
    std::string name;
    std::uint32_t target_mode = 0;
    double target = 0.0;
    double target_max_force = 0.0;
};

double dot(const Vec3 &a, const Vec3 &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
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
            // Static root pose is model state, not a free-joint qpos.
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

    nksim_result apply_forces(const nksim::BackendBodyForce *forces,
                              std::uint32_t count) override {
        if (count != 0 && !forces)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto body_id = model_body_id(forces[index].backend_body);
            if (body_id < 0)
                return NKSIM_ERROR_INVALID_HANDLE;
            auto *external = data->xfrc_applied + body_id * 6;
            std::copy(forces[index].torque.begin(), forces[index].torque.end(), external);
            std::copy(forces[index].force.begin(), forces[index].force.end(), external + 3);
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
                targets[index].mode > NKSIM_JOINT_TARGET_EFFORT ||
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
        }
        return NKSIM_OK;
    }

    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!model || !data || !std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        model->opt.timestep = dt / static_cast<double>(substeps);
        for (std::uint32_t index = 0; index < substeps; ++index) {
            apply_joint_targets();
            mj_step(model, data);
        }
        // mj_step integrates qpos/qvel after computing derived body quantities.
        // Refresh them so snapshots contain pose and velocity at the same time.
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

private:
    void clear_topology_backup() noexcept {
        saved_bodies.clear();
        saved_body_order.clear();
        saved_joints.clear();
        saved_joint_order.clear();
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
        const auto actuator_result = add_joint_actuators();
        if (actuator_result != NKSIM_OK)
            return actuator_result;

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
    // F3: after F1's rest-pose fix every link sits at its real offset, so
    // non-adjacent links of the same robot (not just direct joint pairs) can
    // genuinely overlap. Exclude every pair of bodies reachable from each
    // other through the joint graph (a robot's own weakly-connected
    // articulation), not just parent/child pairs; self-collision within one
    // robot is not modelled (see ARCHITECTURE.md). A body with no joints at
    // all (e.g. an unconnected environment object) is its own singleton
    // component and gets no excludes.
    nksim_result add_self_collision_excludes() {
        std::unordered_map<std::uint64_t, std::uint64_t> parent_of;
        for (const auto body_id : body_order)
            parent_of[body_id] = body_id;
        auto find_root = [&](std::uint64_t id) {
            while (parent_of[id] != id) {
                parent_of[id] = parent_of[parent_of[id]];
                id = parent_of[id];
            }
            return id;
        };
        for (const auto joint_id : joint_order) {
            const auto &joint = joints.at(joint_id);
            const auto root_a = find_root(joint.desc.body_a);
            const auto root_b = find_root(joint.desc.body_b);
            if (root_a != root_b)
                parent_of[root_a] = root_b;
        }
        std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> components;
        for (const auto body_id : body_order)
            components[find_root(body_id)].push_back(body_id);
        for (const auto &entry : components) {
            const auto &members = entry.second;
            for (std::size_t i = 0; i < members.size(); ++i) {
                for (std::size_t j = i + 1; j < members.size(); ++j) {
                    auto *exclude = mjs_addExclude(spec);
                    if (!exclude) return NKSIM_ERROR_OUT_OF_MEMORY;
                    mjs_setString(exclude->bodyname1, bodies.at(members[i]).name.c_str());
                    mjs_setString(exclude->bodyname2, bodies.at(members[j]).name.c_str());
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

        const auto body_result = configure_body(*body, found->second.desc, incoming);
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

    static nksim_result configure_body(mjsBody &body,
                                       const nksim::BackendBodyDesc &desc,
                                       const JointRecord *incoming) {
        body.mass = desc.mass;
        if (desc.motion_type != NKSIM_MOTION_STATIC && desc.mass > 0.0) {
            body.explicitinertial = 1;
            if (desc.has_inertial_properties) {
                std::copy(desc.center_of_mass.begin(), desc.center_of_mass.end(), body.ipos);
                const auto &m = desc.inertia_tensor;
                body.fullinertia[0] = m[0];
                body.fullinertia[1] = m[4];
                body.fullinertia[2] = m[8];
                body.fullinertia[3] = m[1];
                body.fullinertia[4] = m[2];
                body.fullinertia[5] = m[5];
            } else {
                body.inertia[0] = body.inertia[1] = body.inertia[2] = 1.0;
            }
        }

        if (!incoming) {
            if (desc.motion_type != NKSIM_MOTION_STATIC && !mjs_addFreeJoint(&body))
                return NKSIM_ERROR_OUT_OF_MEMORY;
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
        }

        if (desc.shape_type == 0)
            return NKSIM_OK;
        auto *geom = mjs_addGeom(&body, nullptr);
        if (!geom)
            return NKSIM_ERROR_OUT_OF_MEMORY;
        switch (desc.shape_type) {
        case NKSIM_SHAPE_BOX:
            geom->type = mjGEOM_BOX;
            break;
        case NKSIM_SHAPE_SPHERE:
            geom->type = mjGEOM_SPHERE;
            break;
        case NKSIM_SHAPE_CAPSULE:
            geom->type = mjGEOM_CAPSULE;
            break;
        case NKSIM_SHAPE_PLANE:
            geom->type = mjGEOM_PLANE;
            break;
        default:
            return NKSIM_ERROR_UNSUPPORTED;
        }
        if (desc.shape_type == NKSIM_SHAPE_PLANE) {
            const Vec3 normal = normalize(Vec3{desc.shape_parameters[0],
                                               desc.shape_parameters[1],
                                               desc.shape_parameters[2]});
            geom->pos[0] = normal[0] * desc.shape_parameters[3];
            geom->pos[1] = normal[1] * desc.shape_parameters[3];
            geom->pos[2] = normal[2] * desc.shape_parameters[3];
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
            geom->size[0] = desc.shape_parameters[0];
            geom->size[1] = desc.shape_type == NKSIM_SHAPE_CAPSULE
                ? desc.shape_parameters[1] * 0.5
                : desc.shape_parameters[1];
            geom->size[2] = desc.shape_parameters[2];
        }
        geom->contype = static_cast<int>(desc.collision_layer);
        geom->conaffinity = static_cast<int>(desc.collision_mask);
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

    void apply_joint_targets() {
        if (model->nu > 0)
            std::fill(data->ctrl, data->ctrl + model->nu, 0.0);
        for (const auto joint_id : joint_order) {
            auto &joint = joints.at(joint_id);
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

            const auto dof = model->jnt_dofadr[model_id];
            // The mass matrix is stored sparse; the diagonal entry for dof i
            // lives at M[dof_Madr[i]] (the roadmap's "m_ii ... via
            // dof_Madr", avoiding a dense mj_fullM expansion every substep).
            const auto m_ii = data->M[model->dof_Madr[dof]];
            const auto bias = data->qfrc_bias[dof];
            const auto q = data->qpos[model->jnt_qposadr[model_id]];
            const auto qdot = data->qvel[dof];

            double torque = 0.0;
            switch (joint.target_mode) {
            case NKSIM_JOINT_TARGET_POSITION: {
                constexpr double omega_n = 2.0 * 3.14159265358979323846 * 10.0;
                constexpr double zeta = 1.0;
                torque = m_ii * (omega_n * omega_n * (joint.target - q) - 2.0 * zeta * omega_n * qdot) + bias;
                break;
            }
            case NKSIM_JOINT_TARGET_VELOCITY: {
                constexpr double kv = 50.0;
                torque = m_ii * kv * (joint.target - qdot) + bias;
                break;
            }
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
    std::vector<std::uint64_t> body_order;
    std::unordered_map<std::uint64_t, JointRecord> joints;
    std::vector<std::uint64_t> joint_order;
    std::unordered_map<std::uint64_t, BodyRecord> saved_bodies;
    std::vector<std::uint64_t> saved_body_order;
    std::unordered_map<std::uint64_t, JointRecord> saved_joints;
    std::vector<std::uint64_t> saved_joint_order;
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

nksim_result NKSIMMUJOCO_CALL nksim_mujoco_world_create(
    const nksim_world_desc *desc, nksim_world *out_world) {
    if (!desc || !out_world)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    return nksim::create_world_with_backend(
        *desc, nksim_mujoco::make_backend(), out_world);
}

} // extern "C"
