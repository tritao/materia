#include "PhysicsBackend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace nksim {
namespace {

// Minimal rigid-transform helpers (translation + xyzw quaternion), mirroring
// robotkit.spatial.Transform3's a_T_b convention exactly, for F4's
// kinematic child-body placement.
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

Quat normalize(Quat q) {
    double length = 0.0;
    for (double c : q) length += c * c;
    length = std::sqrt(length);
    if (!std::isfinite(length) || length < 1e-12) return {0.0, 0.0, 0.0, 1.0};
    for (double &c : q) c /= length;
    return q;
}

Quat conjugate(const Quat &q) { return {-q[0], -q[1], -q[2], q[3]}; }

Quat multiply(const Quat &a, const Quat &b) {
    return {
        a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
        a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
        a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
        a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2],
    };
}

Vec3 rotate(const Quat &q, const Vec3 &v) {
    const Vec3 t{
        2.0 * (q[1]*v[2] - q[2]*v[1]),
        2.0 * (q[2]*v[0] - q[0]*v[2]),
        2.0 * (q[0]*v[1] - q[1]*v[0]),
    };
    return {
        v[0] + q[3]*t[0] + q[1]*t[2] - q[2]*t[1],
        v[1] + q[3]*t[1] + q[2]*t[0] - q[0]*t[2],
        v[2] + q[3]*t[2] + q[0]*t[1] - q[1]*t[0],
    };
}

Vec3 add(const Vec3 &a, const Vec3 &b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
Vec3 sub(const Vec3 &a, const Vec3 &b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec3 scale(const Vec3 &a, double s) { return {a[0]*s, a[1]*s, a[2]*s}; }

struct Xform {
    Vec3 pos{0.0, 0.0, 0.0};
    Quat rot{0.0, 0.0, 0.0, 1.0};
};

Xform compose(const Xform &a, const Xform &b) {
    return {add(a.pos, rotate(a.rot, b.pos)), multiply(a.rot, b.rot)};
}

Xform inverse(const Xform &a) {
    const auto inv_rot = conjugate(a.rot);
    return {rotate(inv_rot, scale(a.pos, -1.0)), inv_rot};
}

Xform motion(std::uint32_t type, const Vec3 &axis, double q) {
    if (type == NKSIM_JOINT_PRISMATIC)
        return {scale(axis, q), {0.0, 0.0, 0.0, 1.0}};
    if (type == NKSIM_JOINT_REVOLUTE) {
        const double half = q * 0.5;
        const double s = std::sin(half);
        return {{0.0, 0.0, 0.0}, {axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half)}};
    }
    return Xform{};
}

/** Angular velocity (world frame) via first-order finite difference. */
Vec3 angular_velocity_from_delta(const Quat &previous, const Quat &current, double dt) {
    const auto delta = multiply(current, conjugate(previous));
    const double sin_half = std::sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
    if (sin_half < 1e-12) return {0.0, 0.0, 0.0};
    double angle = 2.0 * std::atan2(sin_half, delta[3]);
    // Take the shorter rotation for a stable rate.
    constexpr double pi = 3.14159265358979323846;
    if (angle > pi) angle -= 2.0 * pi;
    const double scale_factor = angle / (sin_half * dt);
    return {delta[0]*scale_factor, delta[1]*scale_factor, delta[2]*scale_factor};
}

struct TestBody {
    std::uint64_t id = 0;
    BackendBodyDesc desc{};
    BackendBodyState state{};
    std::array<double, 3> force{};
    std::array<double, 3> torque{};
};

struct TestJoint {
    std::uint64_t id = 0;
    BackendJointDesc desc{};
    BackendJointState state{};
};

class TestPhysicsBackend final : public PhysicsBackend {
public:
    nksim_result initialize(const nksim_world_desc &desc) override {
        gravity = {desc.gravity[0], desc.gravity[1], desc.gravity[2]};
        return NKSIM_OK;
    }

    nksim_result body_create(const BackendBodyDesc &desc,
                             std::uint64_t *out_body) override {
        if (!out_body)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        TestBody body;
        body.id = next_body++;
        body.desc = desc;
        body.state.backend_body = body.id;
        body.state.position = desc.position;
        body.state.rotation = desc.rotation;
        bodies.push_back(body);
        *out_body = body.id;
        return NKSIM_OK;
    }

    nksim_result body_destroy(std::uint64_t id) override {
        const auto before = bodies.size();
        bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
                                    [id](const TestBody &body) { return body.id == id; }),
                     bodies.end());
        return bodies.size() == before ? NKSIM_ERROR_INVALID_HANDLE : NKSIM_OK;
    }

    nksim_result body_set_state(std::uint64_t id,
                                const BackendBodyState &state) override {
        auto *body = find_body(id);
        if (!body)
            return NKSIM_ERROR_INVALID_HANDLE;
        if (auto *incoming = body->desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(id) : nullptr) {
            // F4: a joint-connected DYNAMIC body's pose is derived, not
            // independent; it can be reset to its declared rest pose (which
            // zeroes the joint), but not teleported away from its
            // articulation — mirrors the MuJoCo backend's own
            // constrained-link convention. A kinematic/static body with an
            // incoming joint (e.g. an externally-scripted second link) is
            // unaffected, exactly as before this fix.
            for (int i = 0; i < 3; ++i)
                if (std::abs(state.position[i] - body->desc.position[i]) > 1e-9)
                    return NKSIM_ERROR_UNSUPPORTED;
            for (int i = 0; i < 4; ++i)
                if (std::abs(state.rotation[i] - body->desc.rotation[i]) > 1e-9)
                    return NKSIM_ERROR_UNSUPPORTED;
            incoming->state.position = incoming->state.velocity = incoming->state.effort = 0.0;
        }
        body->state = state;
        body->state.backend_body = id;
        // F4: a root's new pose (or a reset child's zeroed joint) must
        // propagate to every descendant immediately, not just at the next step.
        return recompute_articulated_poses(0.0);
    }

    nksim_result body_set_motion_type(std::uint64_t id, std::uint32_t motion_type,
                                      double) override {
        auto *body = find_body(id);
        if (!body) return NKSIM_ERROR_INVALID_HANDLE;
        if (motion_type != NKSIM_MOTION_DYNAMIC && motion_type != NKSIM_MOTION_KINEMATIC)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        body->desc.motion_type = motion_type;
        return NKSIM_OK;
    }

    nksim_result apply_forces(const BackendBodyForce *forces,
                              std::uint32_t count) override {
        if (count != 0 && !forces)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            auto *body = find_body(forces[index].backend_body);
            if (!body)
                return NKSIM_ERROR_INVALID_HANDLE;
            for (int axis = 0; axis < 3; ++axis) {
                body->force[axis] += forces[index].force[axis];
                body->torque[axis] += forces[index].torque[axis];
            }
        }
        return NKSIM_OK;
    }

    nksim_result joint_create(const BackendJointDesc &desc,
                              std::uint64_t *out_joint) override {
        if (!out_joint)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        if (!find_body(desc.body_a) || !find_body(desc.body_b))
            return NKSIM_ERROR_INVALID_HANDLE;
        TestJoint joint;
        joint.id = next_joint++;
        joint.desc = desc;
        joint.state.backend_joint = joint.id;
        joints.push_back(joint);
        *out_joint = joint.id;
        return NKSIM_OK;
    }

    nksim_result joint_destroy(std::uint64_t id) override {
        const auto before = joints.size();
        joints.erase(std::remove_if(joints.begin(), joints.end(),
                                    [id](const TestJoint &joint) { return joint.id == id; }),
                     joints.end());
        couplings.erase(std::remove_if(couplings.begin(), couplings.end(),
            [id](const auto &value) { return value.leader == id || value.follower == id; }),
            couplings.end());
        return joints.size() == before ? NKSIM_ERROR_INVALID_HANDLE : NKSIM_OK;
    }

    nksim_result joint_couple(const BackendJointCoupling &coupling) override {
        if (!find_joint(coupling.leader) || !find_joint(coupling.follower))
            return NKSIM_ERROR_INVALID_HANDLE;
        for (const auto &existing : couplings)
            if (existing.follower == coupling.follower)
                return NKSIM_ERROR_INVALID_ARGUMENT;
        couplings.push_back(coupling);
        enforce_couplings();
        return recompute_articulated_poses(0.0);
    }

    nksim_result closure_create(const BackendClosure &) override {
        return NKSIM_ERROR_UNSUPPORTED;
    }

    nksim_result set_joint_targets(const BackendJointTarget *targets,
                                   std::uint32_t count) override {
        if (count != 0 && !targets)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            auto *joint = find_joint(targets[index].backend_joint);
            if (!joint)
                return NKSIM_ERROR_INVALID_HANDLE;
            switch (targets[index].mode) {
            case NKSIM_JOINT_TARGET_POSITION:
            case NKSIM_JOINT_TARGET_SERVO:
                // Joints here follow their targets exactly, so a servo's
                // position target is reached at once, as a position target is.
                joint->state.position = targets[index].target;
                joint->state.velocity = 0.0;
                break;
            case NKSIM_JOINT_TARGET_VELOCITY:
                joint->state.velocity = targets[index].target;
                break;
            case NKSIM_JOINT_TARGET_EFFORT:
                joint->state.effort = targets[index].target;
                break;
            default:
                return NKSIM_ERROR_INVALID_ARGUMENT;
            }
            clamp_joint_position(*joint);
        }
        // F4: an instant position-mode target must move the body right away
        // (this backend applies it exactly, with no gradual interpolation),
        // not only at the next step.
        enforce_couplings();
        return recompute_articulated_poses(0.0);
    }

    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const double substep = dt / static_cast<double>(substeps);
        for (std::uint32_t iteration = 0; iteration < substeps; ++iteration) {
            for (auto &body : bodies) {
                if (body.desc.motion_type != NKSIM_MOTION_DYNAMIC)
                    continue;
                // F4: a joint-connected body's pose is derived kinematically
                // below, not force-integrated; it must not also free-fall.
                if (parent_joint(body.id) != nullptr)
                    continue;
                const double inverse_mass = body.desc.mass > 0.0 ? 1.0 / body.desc.mass : 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    body.state.linear_velocity[axis] +=
                        (gravity[axis] + body.force[axis] * inverse_mass) * substep;
                    body.state.position[axis] += body.state.linear_velocity[axis] * substep;
                }
            }
            // This backend is intentionally kinematic, but velocity targets must
            // still advance joint coordinates for runtime and odometry tests.
            for (auto &joint : joints) {
                joint.state.position += joint.state.velocity * substep;
                clamp_joint_position(joint);
            }
            enforce_couplings();
            const auto recompute_result = recompute_articulated_poses(substep);
            if (recompute_result != NKSIM_OK)
                return recompute_result;
            // Deterministic backend's box contacts: each free dynamic body and each obstacle is
            // an oriented box (a convex shape's local bounds, turned with its body). The
            // separating-axis test over both boxes' face normals and their edge crossings finds
            // whether they overlap and the least push that parts them; an axis-aligned bound of a
            // turned box would reach well beyond it, so an arm link at an angle shoved parts it
            // never touched.
            struct Box { Vec3 center; std::array<Vec3, 3> axes; Vec3 half; };
            auto box_of = [](const TestBody &body) {
                Box box{};
                box.center = body.state.position;
                for (int axis = 0; axis < 3; ++axis) {
                    Vec3 local{};
                    local[axis] = 1.0;
                    box.axes[axis] = rotate(body.state.rotation, local);
                    box.half[axis] = body.desc.shape_parameters[axis];
                }
                if (body.desc.shape_type == NKSIM_SHAPE_CONVEX && !body.desc.shape_vertices.empty()) {
                    Vec3 minimum{INFINITY, INFINITY, INFINITY};
                    Vec3 maximum{-INFINITY, -INFINITY, -INFINITY};
                    for (std::size_t i = 0; i < body.desc.shape_vertices.size(); ++i) {
                        const auto axis = i % 3;
                        minimum[axis] = std::min(minimum[axis],
                                                 static_cast<double>(body.desc.shape_vertices[i]));
                        maximum[axis] = std::max(maximum[axis],
                                                 static_cast<double>(body.desc.shape_vertices[i]));
                    }
                    box.center = add(box.center, rotate(body.state.rotation,
                        {(minimum[0] + maximum[0]) * 0.5,
                         (minimum[1] + maximum[1]) * 0.5,
                         (minimum[2] + maximum[2]) * 0.5}));
                }
                return box;
            };
            auto dot3 = [](const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
            auto cross3 = [](const Vec3 &a, const Vec3 &b) {
                return Vec3{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
            };
            auto radius = [&](const Box &box, const Vec3 &axis) {
                double r = 0.0;
                for (int i = 0; i < 3; ++i) r += std::abs(dot3(box.axes[i], axis)) * box.half[i];
                return r;
            };
            for (auto &moving : bodies) {
                if (moving.desc.motion_type != NKSIM_MOTION_DYNAMIC ||
                    parent_joint(moving.id) != nullptr ||
                    (moving.desc.shape_type != NKSIM_SHAPE_BOX &&
                     moving.desc.shape_type != NKSIM_SHAPE_CONVEX))
                    continue;
                for (const auto &obstacle : bodies) {
                    if (obstacle.id == moving.id ||
                        (obstacle.desc.motion_type == NKSIM_MOTION_DYNAMIC &&
                         parent_joint(obstacle.id) == nullptr) ||
                        (obstacle.desc.shape_type != NKSIM_SHAPE_BOX &&
                         obstacle.desc.shape_type != NKSIM_SHAPE_CONVEX) ||
                        (moving.desc.collision_mask & obstacle.desc.collision_layer) == 0 ||
                        (obstacle.desc.collision_mask & moving.desc.collision_layer) == 0)
                        continue;
                    const auto a = box_of(moving), b = box_of(obstacle);
                    const Vec3 between{a.center[0] - b.center[0], a.center[1] - b.center[1],
                                       a.center[2] - b.center[2]};
                    // Face normals first, so a tie between a face and an edge crossing pushes off
                    // the face; an edge crossing must part the boxes clearly better to be chosen.
                    std::array<Vec3, 15> candidates{};
                    int count = 0;
                    for (int i = 0; i < 3; ++i) candidates[count++] = a.axes[i];
                    for (int i = 0; i < 3; ++i) candidates[count++] = b.axes[i];
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j) candidates[count++] = cross3(a.axes[i], b.axes[j]);
                    bool separated = false;
                    double least = std::numeric_limits<double>::infinity();
                    Vec3 normal{};
                    for (int k = 0; k < count && !separated; ++k) {
                        auto axis = candidates[k];
                        const double length = std::sqrt(dot3(axis, axis));
                        if (length < 1e-9) continue;  // parallel edges: a face axis covers it
                        for (auto &c : axis) c /= length;
                        const double distance = dot3(between, axis);
                        const double penetration = radius(a, axis) + radius(b, axis) - std::abs(distance);
                        if (penetration <= 0.0) { separated = true; break; }
                        const double weighted = k < 6 ? penetration : penetration * 1.05 + 1e-9;
                        if (weighted < least) {
                            least = weighted;
                            const double sign = distance >= 0.0 ? 1.0 : -1.0;
                            normal = {axis[0] * sign, axis[1] * sign, axis[2] * sign};
                        }
                    }
                    if (separated) continue;
                    const double push = radius(a, normal) + radius(b, normal) - std::abs(dot3(between, normal));
                    for (int axis = 0; axis < 3; ++axis) moving.state.position[axis] += normal[axis] * push;
                    const double approach = dot3(moving.state.linear_velocity, normal);
                    if (approach < 0.0)
                        for (int axis = 0; axis < 3; ++axis)
                            moving.state.linear_velocity[axis] -= approach * normal[axis];
                }
            }
        }
        for (auto &body : bodies) {
            body.force.fill(0.0);
            body.torque.fill(0.0);
        }
        return NKSIM_OK;
    }

    nksim_result read_body_states(BackendBodyState *states,
                                  std::uint32_t count) override {
        if (count != 0 && !states)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto *body = find_body(states[index].backend_body);
            if (!body)
                return NKSIM_ERROR_INVALID_HANDLE;
            states[index] = body->state;
        }
        return NKSIM_OK;
    }

    nksim_result read_joint_states(BackendJointState *states,
                                   std::uint32_t count) override {
        if (count != 0 && !states)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto *joint = find_joint(states[index].backend_joint);
            if (!joint)
                return NKSIM_ERROR_INVALID_HANDLE;
            states[index] = joint->state;
        }
        return NKSIM_OK;
    }

private:
    void enforce_couplings() {
        for (const auto &coupling : couplings) {
            const auto *leader = find_joint(coupling.leader);
            auto *follower = find_joint(coupling.follower);
            if (!leader || !follower) continue;
            follower->state.position = coupling.ratio * leader->state.position + coupling.offset;
            follower->state.velocity = coupling.ratio * leader->state.velocity;
        }
    }
    TestJoint *parent_joint(std::uint64_t body_id) noexcept {
        for (auto &joint : joints)
            if (joint.desc.body_b == body_id) return &joint;
        return nullptr;
    }

    static void clamp_joint_position(TestJoint &joint) noexcept {
        if (joint.desc.lower_limit < joint.desc.upper_limit) {
            if (joint.state.position < joint.desc.lower_limit) joint.state.position = joint.desc.lower_limit;
            if (joint.state.position > joint.desc.upper_limit) joint.state.position = joint.desc.upper_limit;
        }
    }

    int body_index(std::uint64_t id) const noexcept {
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].id == id) return static_cast<int>(i);
        return -1;
    }

    /**
     * Recomputes every joint-connected body's world pose from its parent
     * and current joint position, in topological order:
     * world_T_child = world_T_parent . T(anchor_a, rotation_a) . M(q) . T(anchor_b, rotation_b)^-1,
     * M(q) built from the joint-frame axis recovered from axis_a/rotation_a
     * (undoing rotation_a), matching KinematicChain's own FK composition.
     * A joint-connected body's velocity is set by finite difference here
     * (dt <= 0 skips this and zeroes it) so IMU-style consumers on arm
     * links read something physical; step() itself never force-integrates
     * these bodies. Returns NKSIM_ERROR_INVALID_ARGUMENT if the joint graph
     * contains a cycle.
     */
    nksim_result recompute_articulated_poses(double dt) {
        std::vector<bool> resolved(bodies.size(), false);
        std::size_t remaining = 0;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            // Only a DYNAMIC child's pose is derived from its joint; a
            // kinematic or static body with an "incoming joint" (e.g. a
            // second externally-scripted link) keeps whatever pose it was
            // given, exactly as before this fix.
            resolved[i] = bodies[i].desc.motion_type != NKSIM_MOTION_DYNAMIC ||
                parent_joint(bodies[i].id) == nullptr;
            if (!resolved[i]) ++remaining;
        }
        for (std::size_t pass = 0; pass <= bodies.size() && remaining > 0; ++pass) {
            for (auto &joint : joints) {
                const auto child_index = body_index(joint.desc.body_b);
                const auto parent_index = body_index(joint.desc.body_a);
                if (child_index < 0 || parent_index < 0 || resolved[static_cast<std::size_t>(child_index)] ||
                    !resolved[static_cast<std::size_t>(parent_index)])
                    continue;
                clamp_joint_position(joint);
                auto &parent = bodies[static_cast<std::size_t>(parent_index)];
                auto &child = bodies[static_cast<std::size_t>(child_index)];
                const Xform parent_pose{parent.state.position, parent.state.rotation};
                const Xform anchor_a{joint.desc.anchor_a, normalize(joint.desc.rotation_a)};
                const Xform anchor_b{joint.desc.anchor_b, normalize(joint.desc.rotation_b)};
                const auto axis_joint_frame = rotate(conjugate(normalize(joint.desc.rotation_a)), joint.desc.axis_a);
                const Xform new_pose = compose(
                    compose(compose(parent_pose, anchor_a), motion(joint.desc.type, axis_joint_frame, joint.state.position)),
                    inverse(anchor_b));
                const auto previous_position = child.state.position;
                const auto previous_rotation = child.state.rotation;
                child.state.position = new_pose.pos;
                child.state.rotation = normalize(new_pose.rot);
                if (dt > 0.0) {
                    child.state.linear_velocity = scale(sub(child.state.position, previous_position), 1.0 / dt);
                    child.state.angular_velocity = angular_velocity_from_delta(previous_rotation, child.state.rotation, dt);
                } else {
                    child.state.linear_velocity = {0.0, 0.0, 0.0};
                    child.state.angular_velocity = {0.0, 0.0, 0.0};
                }
                resolved[static_cast<std::size_t>(child_index)] = true;
                --remaining;
            }
        }
        return remaining > 0 ? NKSIM_ERROR_INVALID_ARGUMENT : NKSIM_OK;
    }

    TestBody *find_body(std::uint64_t id) noexcept {
        for (auto &body : bodies) {
            if (body.id == id)
                return &body;
        }
        return nullptr;
    }

    const TestBody *find_body(std::uint64_t id) const noexcept {
        for (const auto &body : bodies) {
            if (body.id == id)
                return &body;
        }
        return nullptr;
    }

    nksim_result joint_set_state(std::uint64_t id, double position, double velocity) override {
        auto *joint = find_joint(id);
        if (!joint) return NKSIM_ERROR_INVALID_HANDLE;
        joint->state.position = position;
        joint->state.velocity = velocity;
        clamp_joint_position(*joint);
        enforce_couplings();
        return recompute_articulated_poses(0.0);
    }

    TestJoint *find_joint(std::uint64_t id) noexcept {
        for (auto &joint : joints) {
            if (joint.id == id)
                return &joint;
        }
        return nullptr;
    }

    const TestJoint *find_joint(std::uint64_t id) const noexcept {
        for (const auto &joint : joints) {
            if (joint.id == id)
                return &joint;
        }
        return nullptr;
    }

    std::array<double, 3> gravity{};
    std::vector<TestBody> bodies;
    std::vector<TestJoint> joints;
    std::vector<BackendJointCoupling> couplings;
    std::uint64_t next_body = 1;
    std::uint64_t next_joint = 1;
};

} // namespace

std::unique_ptr<PhysicsBackend> make_test_physics_backend() {
    return std::make_unique<TestPhysicsBackend>();
}

} // namespace nksim
