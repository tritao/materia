#include "PhysicsBackend.hpp"
#include "convex_overlap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <map>
#include <memory>
#include <vector>
#include <functional>

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

Xform motion(std::uint32_t type, const Vec3 &axis, double q, double lead) {
    if (type == NKSIM_JOINT_PRISMATIC)
        return {scale(axis, q), {0.0, 0.0, 0.0, 1.0}};
    if (type == NKSIM_JOINT_HELICAL) {
        constexpr double two_pi = 6.28318530717958647692;
        const double half = q * 0.5;
        const double s = std::sin(half);
        return {scale(axis, lead * q / two_pi), {axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half)}};
    }
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

/** Exact overlap for convex hulls; the deterministic backend's OBB remains
 * useful for its kinematic response, but must not invent sensor contacts in
 * the empty corners of a CAD hull's bounds. */
geometry::Overlap convex_overlap(const TestBody &a, int a_piece,
                                 const TestBody &b, int b_piece, Vec3 &witness) {
    const auto support = [](const TestBody &body, int piece, const Vec3 &world_direction) {
        const auto *part = piece < 0 ? nullptr : &body.desc.shape_parts[piece];
        const auto type = part ? part->type : body.desc.shape_type;
        const auto &parameters = part ? part->parameters : body.desc.shape_parameters;
        const auto &vertices = part ? part->vertices : body.desc.shape_vertices;
        auto position = body.state.position;
        auto rotation = body.state.rotation;
        if (part) {
            position = add(position, rotate(rotation, part->position));
            rotation = multiply(rotation, part->rotation);
        }
        const auto local_direction = rotate(conjugate(rotation), world_direction);
        Vec3 local_point{};
        if (type == NKSIM_SHAPE_BOX) {
            for (int axis = 0; axis < 3; ++axis)
                local_point[axis] = local_direction[axis] >= 0.0 ? parameters[axis] : -parameters[axis];
        } else if (type == NKSIM_SHAPE_CONVEX && vertices.size() >= 12 && vertices.size() % 3 == 0) {
            double best = -std::numeric_limits<double>::infinity();
            for (std::size_t index = 0; index < vertices.size(); index += 3) {
                const Vec3 point{vertices[index], vertices[index + 1], vertices[index + 2]};
                const double projection = local_direction[0] * point[0] +
                    local_direction[1] * point[1] + local_direction[2] * point[2];
                if (projection > best) { best = projection; local_point = point; }
            }
        }
        return add(position, rotate(rotation, local_point));
    };
    return geometry::overlap_with_witness([&](const geometry::Point &direction) {
        const auto point_a = support(a, a_piece, direction);
        const Vec3 opposite{-direction[0], -direction[1], -direction[2]};
        const auto point_b = support(b, b_piece, opposite);
        return geometry::Simplex::SupportPoint{sub(point_a, point_b), point_a, point_b};
    }, witness);
}

struct TestJoint {
    std::uint32_t target_mode = NKSIM_JOINT_TARGET_POSITION;
    double effort_limit = 0;
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
        if(!desc.has_inertial_properties&&desc.mass>0&&(desc.shape_type==NKSIM_SHAPE_CONVEX||!desc.shape_parts.empty()))derive_hull_inertia(body.desc);
        body.state.backend_body = body.id;
        body.state.position = desc.position;
        body.state.rotation = desc.rotation;
        bodies.push_back(body);
        *out_body = body.id;
        return NKSIM_OK;
    }

    nksim_result body_destroy(std::uint64_t id) override {
        previous_root_velocity.erase(id);
        for(auto it=contact_geometry.begin();it!=contact_geometry.end();)if(it->first[0]==id||it->first[2]==id)it=contact_geometry.erase(it);else ++it;
        for(auto it=local_hulls.begin();it!=local_hulls.end();)if(it->first.first==id)it=local_hulls.erase(it);else ++it;
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
            previous_joint_velocity[incoming->id]=0;
        }
        body->state = state;
        body->state.backend_body = id;
        // F4: a root's new pose (or a reset child's zeroed joint) must
        // propagate to every descendant immediately, not just at the next step.
        return recompute_articulated_poses(0.0);
    }

    nksim_result body_set_mass(std::uint64_t id, double mass) override {
        auto *body = find_body(id);
        if (!body) return NKSIM_ERROR_INVALID_HANDLE;
        if (!std::isfinite(mass) || mass <= 0.0) return NKSIM_ERROR_INVALID_ARGUMENT;
        if (body->desc.motion_type == NKSIM_MOTION_STATIC) return NKSIM_ERROR_INVALID_STATE;
        if(body->desc.has_inertial_properties&&body->desc.mass>0)
            for(auto &value:body->desc.inertia_tensor)value*=mass/body->desc.mass;
        body->desc.mass = mass;
        return NKSIM_OK;
    }

    nksim_result body_exclude_pair(std::uint64_t first, std::uint64_t second) override {
        if (!find_body(first) || !find_body(second)) return NKSIM_ERROR_INVALID_HANDLE;
        if (first == second) return NKSIM_ERROR_INVALID_ARGUMENT;
        excluded_pairs.emplace(std::min(first, second), std::max(first, second));
        return NKSIM_OK;
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
        previous_joint_velocity.erase(id);
        const auto before = joints.size();
        joints.erase(std::remove_if(joints.begin(), joints.end(),
                                    [id](const TestJoint &joint) { return joint.id == id; }),
                     joints.end());
        couplings.erase(std::remove_if(couplings.begin(), couplings.end(),
            [id](const auto &value) { return value.leader == id || value.follower == id; }),
            couplings.end());
        order_followers();
        return joints.size() == before ? NKSIM_ERROR_INVALID_HANDLE : NKSIM_OK;
    }

    // This kinematic test backend cannot represent passive friction or brake holding.
    nksim_result joint_configure_brake(std::uint64_t, BrakeParameters) override {
        return NKSIM_ERROR_UNSUPPORTED;
    }
    nksim_result joint_set_brake_release(std::uint64_t, bool) override {
        return NKSIM_ERROR_UNSUPPORTED;
    }
    nksim_result joint_couple(const BackendJointCoupling &coupling) override {
        if (!find_joint(coupling.leader) || !find_joint(coupling.follower))
            return NKSIM_ERROR_INVALID_HANDLE;
        // A follower with several couplings is the sum of their terms: each leader once, no cycles.
        for (const auto &existing : couplings)
            if (existing.follower == coupling.follower && existing.leader == coupling.leader)
                return NKSIM_ERROR_INVALID_ARGUMENT;
        couplings.push_back(coupling);
        if (!order_followers()) {
            couplings.pop_back();
            order_followers();
            return NKSIM_ERROR_INVALID_ARGUMENT;
        }
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
            joint->target_mode = targets[index].mode;
            joint->effort_limit = targets[index].max_force;
            switch (targets[index].mode) {
            case NKSIM_JOINT_TARGET_POSITION:
            case NKSIM_JOINT_TARGET_SERVO:
                // Joints here follow their targets exactly, so a servo's
                // position target is reached at once, as a position target is.
                joint->state.position = targets[index].target;
                joint->state.velocity = targets[index].mode == NKSIM_JOINT_TARGET_SERVO ? targets[index].velocity : 0.0;
                break;
            case NKSIM_JOINT_TARGET_VELOCITY:
            case NKSIM_JOINT_TARGET_FLOW:
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

    nksim_result contact_pair_create(const BackendContactPair &pair) override {
        const auto *a=find_body(pair.body_a),*b=find_body(pair.body_b);
        if(!a||!b)return NKSIM_ERROR_INVALID_HANDLE;
        if(pair.part_a>=std::max(std::size_t(1),a->desc.shape_parts.size())||pair.part_b>=std::max(std::size_t(1),b->desc.shape_parts.size())||!std::isfinite(pair.surface.friction[0])||pair.surface.friction[0]<0)return NKSIM_ERROR_INVALID_ARGUMENT;
        force_pairs.push_back(pair);return NKSIM_OK;
    }
    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const double substep = dt / static_cast<double>(substeps);
        for (std::uint32_t iteration = 0; iteration < substeps; ++iteration) {
            contacts.clear();
            const bool force_mode = has_effort_axes();
            if(force_mode) {const auto result = force_step(substep); if(result != NKSIM_OK) return result;continue;}
            for (auto &body : bodies) {
                if (body.desc.motion_type != NKSIM_MOTION_DYNAMIC)
                    continue;
                // F4: a joint-connected body's pose is derived kinematically
                // below, not force-integrated; it must not also free-fall.
                if (parent_joint(body.id) != nullptr)
                    continue;
                const double inverse_mass = body.desc.mass > 0.0 ? 1.0 / body.desc.mass : 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    if(!force_mode) body.state.linear_velocity[axis] +=
                        (gravity[axis] + body.force[axis] * inverse_mass) * substep;
                    body.state.position[axis] += body.state.linear_velocity[axis] * substep;
                }
                if(force_mode) {
                    const double speed=std::sqrt(dot3(body.state.angular_velocity,body.state.angular_velocity));
                    if(speed>0) {const double angle=speed*substep, factor=std::sin(angle/2)/speed;
                        const auto w=body.state.angular_velocity;
                        body.state.rotation=normalize(multiply({w[0]*factor,w[1]*factor,w[2]*factor,std::cos(angle/2)},body.state.rotation));}
                }
            }
            // This backend is intentionally kinematic, but velocity targets must
            // still advance joint coordinates for runtime and odometry tests.
            for (auto &joint : joints) {
                if(joint.target_mode != NKSIM_JOINT_TARGET_POSITION && joint.target_mode != NKSIM_JOINT_TARGET_SERVO)
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
            auto box_of = [](const TestBody &body, int piece) {
                Box box{};
                box.center = body.state.position;
                auto rotation = body.state.rotation;
                const auto *part = piece < 0 ? nullptr : &body.desc.shape_parts[piece];
                const auto type = part ? part->type : body.desc.shape_type;
                const auto &parameters = part ? part->parameters : body.desc.shape_parameters;
                const auto &vertices = part ? part->vertices : body.desc.shape_vertices;
                if (part) {
                    box.center = add(box.center,rotate(rotation,part->position));
                    rotation = multiply(rotation,part->rotation);
                }
                for (int axis = 0; axis < 3; ++axis) {
                    Vec3 local{};
                    local[axis] = 1.0;
                    box.axes[axis] = rotate(rotation, local);
                    box.half[axis] = parameters[axis];
                }
                if (type == NKSIM_SHAPE_CONVEX && !vertices.empty()) {
                    Vec3 minimum{INFINITY, INFINITY, INFINITY};
                    Vec3 maximum{-INFINITY, -INFINITY, -INFINITY};
                    for (std::size_t i = 0; i < vertices.size(); ++i) {
                        const auto axis = i % 3;
                        minimum[axis] = std::min(minimum[axis],
                                                 static_cast<double>(vertices[i]));
                        maximum[axis] = std::max(maximum[axis],
                                                 static_cast<double>(vertices[i]));
                    }
                    for (int axis = 0; axis < 3; ++axis) box.half[axis] = (maximum[axis]-minimum[axis])*0.5;
                    box.center = add(box.center, rotate(rotation,
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
                        excluded_pairs.count({std::min(moving.id, obstacle.id),
                                              std::max(moving.id, obstacle.id)}) != 0 ||
                        (obstacle.desc.motion_type == NKSIM_MOTION_DYNAMIC &&
                         parent_joint(obstacle.id) == nullptr) ||
                        (obstacle.desc.shape_type != NKSIM_SHAPE_BOX &&
                         obstacle.desc.shape_type != NKSIM_SHAPE_CONVEX) ||
                        (moving.desc.collision_mask & obstacle.desc.collision_layer) == 0 ||
                        (obstacle.desc.collision_mask & moving.desc.collision_layer) == 0)
                        continue;
                    // Keep compound gaps: its enclosing box is a broad bound,
                    // never a solid contact surface. Recompute the moving pose
                    // after each piece resolves a contact.
                    const int moving_count = std::max(1,static_cast<int>(moving.desc.shape_parts.size()));
                    const int obstacle_count = std::max(1,static_cast<int>(obstacle.desc.shape_parts.size()));
                    for (int mi = 0; mi < moving_count; ++mi) for (int oi = 0; oi < obstacle_count; ++oi) {
                        const int m = moving.desc.shape_parts.empty() ? -1 : mi;
                        const int o = obstacle.desc.shape_parts.empty() ? -1 : oi;
                        auto supported = [](uint32_t type) { return type == NKSIM_SHAPE_BOX || type == NKSIM_SHAPE_CONVEX; };
                        if ((m >= 0 && !supported(moving.desc.shape_parts[m].type)) ||
                            (o >= 0 && !supported(obstacle.desc.shape_parts[o].type))) continue;
                        const auto a = box_of(moving,m), b = box_of(obstacle,o);
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
            // Contacts are observed once after the complete physics step. The
            // next substep clears this snapshot, so skip the all-pairs sensor
            // narrow phase on intermediate substeps.
            if (iteration + 1 < substeps) continue;
            // The kinematic test backend does not resolve robot contact forces, but
            // contact sensors still need the same geometric hit observations as
            // MuJoCo. Publish overlap points for articulated and kinematic links;
            // keep these observations separate from the free-body response above.
            auto contact_point = [&](const Box &a, const Box &b, const Vec3 &normal) {
                Vec3 reference = std::abs(normal[2]) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
                auto tangent_u = cross3(normal, reference);
                const double tangent_length = std::sqrt(dot3(tangent_u, tangent_u));
                for (double &axis : tangent_u) axis /= tangent_length;
                auto tangent_v = cross3(normal, tangent_u);
                const double normal_position = (dot3(a.center, normal) + radius(a, normal) +
                    dot3(b.center, normal) - radius(b, normal)) * 0.5;
                auto overlap_midpoint = [&](const Vec3 &axis) {
                    const double a_center = dot3(a.center, axis), b_center = dot3(b.center, axis);
                    const double low = std::max(a_center - radius(a, axis), b_center - radius(b, axis));
                    const double high = std::min(a_center + radius(a, axis), b_center + radius(b, axis));
                    return low <= high ? (low + high) * 0.5 : (a_center + b_center) * 0.5;
                };
                return add(add(scale(normal, normal_position), scale(tangent_u, overlap_midpoint(tangent_u))),
                           scale(tangent_v, overlap_midpoint(tangent_v)));
            };
            for (auto &moving : bodies) {
                if ((moving.desc.motion_type != NKSIM_MOTION_KINEMATIC && parent_joint(moving.id) == nullptr) ||
                    (moving.desc.shape_type != NKSIM_SHAPE_BOX && moving.desc.shape_type != NKSIM_SHAPE_CONVEX))
                    continue;
                for (const auto &obstacle : bodies) {
                    if (obstacle.id == moving.id ||
                        excluded_pairs.count({std::min(moving.id, obstacle.id), std::max(moving.id, obstacle.id)}) != 0 ||
                        (obstacle.desc.shape_type != NKSIM_SHAPE_BOX && obstacle.desc.shape_type != NKSIM_SHAPE_CONVEX) ||
                        (moving.desc.collision_mask & obstacle.desc.collision_layer) == 0 ||
                        (obstacle.desc.collision_mask & moving.desc.collision_layer) == 0)
                        continue;
                    const int moving_count = std::max(1, static_cast<int>(moving.desc.shape_parts.size()));
                    const int obstacle_count = std::max(1, static_cast<int>(obstacle.desc.shape_parts.size()));
                    for (int mi = 0; mi < moving_count; ++mi) for (int oi = 0; oi < obstacle_count; ++oi) {
                        const int m = moving.desc.shape_parts.empty() ? -1 : mi;
                        const int o = obstacle.desc.shape_parts.empty() ? -1 : oi;
                        auto supported = [](uint32_t type) { return type == NKSIM_SHAPE_BOX || type == NKSIM_SHAPE_CONVEX; };
                        if ((m >= 0 && !supported(moving.desc.shape_parts[m].type)) ||
                            (o >= 0 && !supported(obstacle.desc.shape_parts[o].type))) continue;
                        const auto a = box_of(moving, m), b = box_of(obstacle, o);
                        const Vec3 between = sub(a.center, b.center);
                        std::array<Vec3, 15> candidates{};
                        int count = 0;
                        for (int i = 0; i < 3; ++i) candidates[count++] = a.axes[i];
                        for (int i = 0; i < 3; ++i) candidates[count++] = b.axes[i];
                        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                            candidates[count++] = cross3(a.axes[i], b.axes[j]);
                        bool separated = false;
                        double least = std::numeric_limits<double>::infinity();
                        Vec3 normal{};
                        for (int k = 0; k < count && !separated; ++k) {
                            auto axis = candidates[k];
                            const double length = std::sqrt(dot3(axis, axis));
                            if (length < 1e-9) continue;
                            for (double &component : axis) component /= length;
                            const double distance = dot3(between, axis);
                            const double penetration = radius(a, axis) + radius(b, axis) - std::abs(distance);
                            if (penetration <= 0.0) { separated = true; break; }
                            const double weighted = k < 6 ? penetration : penetration * 1.05 + 1e-9;
                            if (weighted < least) {
                                least = weighted;
                                // Contact normals point from body_a (moving) toward body_b (obstacle).
                                const double sign = distance >= 0.0 ? -1.0 : 1.0;
                                normal = scale(axis, sign);
                            }
                        }
                        if (separated) continue;
                        const auto moving_type = m < 0 ? moving.desc.shape_type : moving.desc.shape_parts[m].type;
                        const auto obstacle_type = o < 0 ? obstacle.desc.shape_type : obstacle.desc.shape_parts[o].type;
                        Vec3 position = contact_point(a, b, normal);
                        if (moving_type == NKSIM_SHAPE_CONVEX || obstacle_type == NKSIM_SHAPE_CONVEX) {
                            const auto overlap = convex_overlap(moving, m, obstacle, o, position);
                            if (overlap == geometry::Overlap::separated) continue;
                            if (overlap == geometry::Overlap::unresolved) return NKSIM_ERROR_BACKEND;
                        }
                        BackendContact contact{};
                        contact.body_a = moving.id;
                        contact.body_b = obstacle.id;
                        contact.part_a = m;
                        contact.part_b = o;
                        contact.position = position;
                        contact.normal = normal;
                        contact.distance = -least;
                        contact.active = true;
                        contacts.push_back(contact);
                    }
                }
            }
            record_velocity_history();
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

    nksim_result read_contacts(std::vector<BackendContact> &out) override {
        out = contacts;
        return NKSIM_OK;
    }

private:
    /**
     * Fills `follower_order` with each coupled joint after the coupled joints among its leaders.
     * Returns false, leaving it partial, when the couplings form a cycle.
     */
    bool order_followers() {
        follower_order.clear();
        std::vector<std::uint64_t> pending;
        for (const auto &coupling : couplings)
            if (std::find(pending.begin(), pending.end(), coupling.follower) == pending.end())
                pending.push_back(coupling.follower);
        while (!pending.empty()) {
            bool progress = false;
            for (auto at = pending.begin(); at != pending.end();) {
                bool ready = true;
                for (const auto &coupling : couplings)
                    if (coupling.follower == *at &&
                        std::find(pending.begin(), pending.end(), coupling.leader) != pending.end())
                        ready = false;
                if (!ready) { ++at; continue; }
                follower_order.push_back(*at);
                at = pending.erase(at);
                progress = true;
            }
            if (!progress) return false;
        }
        return true;
    }

    void enforce_couplings() {
        if (follower_order.empty() && !couplings.empty()) order_followers();
        for (const auto id : follower_order) {
            auto *follower = find_joint(id);
            if (!follower) continue;
            double position = 0.0, velocity = 0.0;
            for (const auto &coupling : couplings) {
                if (coupling.follower != id) continue;
                const auto *leader = find_joint(coupling.leader);
                if (!leader) continue;
                position += coupling.ratio * leader->state.position + coupling.offset;
                velocity += coupling.ratio * leader->state.velocity;
            }
            follower->state.position = position;
            follower->state.velocity = velocity;
        }
    }
    const TestJoint *parent_joint(std::uint64_t body_id) const noexcept {
        for(const auto &joint:joints)if(joint.desc.body_b==body_id)return &joint;
        return nullptr;
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
                    compose(compose(parent_pose, anchor_a), motion(joint.desc.type, axis_joint_frame, joint.state.position, joint.desc.lead)),
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

    // Generalized inertia for mixed prescribed and effort-controlled motion.
    static double dot3(const Vec3 &a,const Vec3 &b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
    static Vec3 cross3(const Vec3 &a,const Vec3 &b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
    struct SpatialColumn { Vec3 linear{}, angular{}; };
    struct FreeCoordinate {
        std::uint64_t body = 0, joint = 0;
        int axis = 0;
    };
    struct KinematicBias {
        Vec3 v{}, w{}, a{}, alpha{};
        bool ready = false;
    };
    std::map<std::uint64_t, double> previous_joint_velocity;
    std::map<std::uint64_t, std::pair<Vec3, Vec3>> previous_root_velocity;
    void record_velocity_history() {
        for (const auto &j : joints)
            previous_joint_velocity[j.id] = j.state.velocity;
        for (const auto &b : bodies)
            if (b.desc.motion_type != NKSIM_MOTION_DYNAMIC)
                previous_root_velocity[b.id] = {b.state.linear_velocity, b.state.angular_velocity};
    }

    bool has_effort_axes() const {
        for (const auto &j : joints)
            if (j.target_mode == NKSIM_JOINT_TARGET_EFFORT)
                return true;
        return false;
    }
    nksim_result advance_efforts(double h) {
        std::vector<FreeCoordinate> coordinates;
        for (const auto &b : bodies)
            if (b.desc.motion_type == NKSIM_MOTION_DYNAMIC && parent_joint(b.id) == nullptr)
                for (int axis = 0; axis < 6; ++axis)
                    coordinates.push_back({b.id, 0, axis});
        for (const auto &j : joints) {
            bool follower = false;
            for (const auto &c : couplings)
                if (c.follower == j.id)
                    follower = true;
            if (j.target_mode == NKSIM_JOINT_TARGET_EFFORT && !follower && j.desc.type != NKSIM_JOINT_FIXED)
                coordinates.push_back({0, j.id, 0});
        }
        force_coordinates = coordinates;
        const auto n = coordinates.size();
        std::vector<double> mass(n * n, 0), force(n, 0), velocity(n, 0);
        for (std::size_t k = 0; k < n; ++k) {
            const auto &d = coordinates[k];
            if (d.joint)
                velocity[k] = find_joint(d.joint)->state.velocity;
            else {
                const auto &s = find_body(d.body)->state;
                velocity[k] = d.axis < 3 ? s.linear_velocity[d.axis] : s.angular_velocity[d.axis - 3];
            }
        }
        for (const auto &j : joints)
            previous_joint_velocity.emplace(j.id, j.state.velocity);
        std::vector<KinematicBias> bias(bodies.size());
        // Recursion in world coordinates, with unknown generalized accelerations set to zero.
        std::function<KinematicBias(std::uint64_t)> kinematics = [&](std::uint64_t id) -> KinematicBias {
            const int bi = body_index(id);
            if (bias[bi].ready)
                return bias[bi];
            const auto &b = bodies[bi];
            auto result = KinematicBias{};
            const auto *j = b.desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(id) : nullptr;
            if (!j) {
                result.v = b.state.linear_velocity;
                result.w = b.state.angular_velocity;
                if (b.desc.motion_type != NKSIM_MOTION_DYNAMIC) {
                    const auto old = previous_root_velocity.find(b.id);
                    if (old != previous_root_velocity.end()) {
                        result.a = scale(sub(result.v, old->second.first), 1 / h);
                        result.alpha = scale(sub(result.w, old->second.second), 1 / h);
                    }
                }
            } else {
                const auto &p = *find_body(j->desc.body_a);
                const auto pk = kinematics(p.id);
                const auto axis = rotate(p.state.rotation, j->desc.axis_a);
                const auto origin = add(p.state.position, rotate(p.state.rotation, j->desc.anchor_a));
                const auto r = sub(b.state.position, p.state.position), rj = sub(b.state.position, origin);
                Vec3 linear{}, angular{};
                if (j->desc.type == NKSIM_JOINT_PRISMATIC)
                    linear = axis;
                else if (j->desc.type == NKSIM_JOINT_REVOLUTE || j->desc.type == NKSIM_JOINT_HELICAL) {
                    angular = axis;
                    linear = cross3(axis, rj);
                    if (j->desc.type == NKSIM_JOINT_HELICAL)
                        linear = add(linear, scale(axis, j->desc.lead / (2 * 3.14159265358979323846)));
                }
                const double qd = j->state.velocity;
                double qdd = 0;
                for (const auto &[source, ratio] : coordinate_sources(j->id)) {
                    const auto *leader = find_joint(source);
                    bool force = false;
                    for (const auto &d : coordinates)
                        if (d.joint == source)
                            force = true;
                    if (!force)
                        qdd += ratio * (leader->state.velocity - previous_joint_velocity.at(leader->id)) / h;
                }
                const auto relative_v = scale(linear, qd), relative_w = scale(angular, qd);
                result.v = add(add(pk.v, cross3(pk.w, r)), relative_v);
                result.w = add(pk.w, relative_w);
                result.alpha = add(add(pk.alpha, cross3(pk.w, relative_w)), scale(angular, qdd));
                result.a = add(add(add(pk.a, cross3(pk.alpha, r)), cross3(pk.w, cross3(pk.w, r))),
                               add(add(scale(cross3(pk.w, relative_v), 2), scale(linear, qdd)),
                                   cross3(relative_w, cross3(relative_w, rj))));
            }
            result.ready = true;
            bias[bi] = result;
            return result;
        };
        for (const auto &b : bodies) {
            if (b.desc.motion_type != NKSIM_MOTION_DYNAMIC || b.desc.mass <= 0)
                continue;
            const auto kin = kinematics(b.id);
            const auto offset = rotate(b.state.rotation, b.desc.center_of_mass),
                       center = add(b.state.position, offset);
            std::array<double, 9> inertia = b.desc.inertia_tensor;
            if (!b.desc.has_inertial_properties) {
                // Polyhedral fallback inertias are resolved at creation; boxes have this direct formula.
                if (b.desc.shape_type != NKSIM_SHAPE_BOX)
                    return NKSIM_ERROR_UNSUPPORTED;
                const auto &r = b.desc.shape_parameters;
                inertia.fill(0);
                inertia[0] = b.desc.mass * (r[1] * r[1] + r[2] * r[2]) / 3;
                inertia[4] = b.desc.mass * (r[0] * r[0] + r[2] * r[2]) / 3;
                inertia[8] = b.desc.mass * (r[0] * r[0] + r[1] * r[1]) / 3;
            }
            auto inertia_times = [&](Vec3 v) {
                v = rotate(conjugate(b.state.rotation), v);
                Vec3 answer{};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        answer[r] += inertia[r * 3 + c] * v[c];
                return rotate(b.state.rotation, answer);
            };
            std::vector<SpatialColumn> jac(n);
            auto current = &b;
            while (true) {
                const auto *j =
                    current->desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(current->id) : nullptr;
                if (!j) {
                    for (std::size_t k = 0; k < n; ++k)
                        if (coordinates[k].body == current->id) {
                            Vec3 axis{};
                            axis[coordinates[k].axis % 3] = 1;
                            if (coordinates[k].axis < 3)
                                jac[k].linear = axis;
                            else {
                                jac[k].angular = axis;
                                jac[k].linear = cross3(axis, sub(center, current->state.position));
                            }
                        }
                    break;
                }
                const auto &p = *find_body(j->desc.body_a);
                for (const auto &[col, ratio] : force_mapping(j->id)) {
                    const auto axis = scale(rotate(p.state.rotation, j->desc.axis_a), ratio);
                    if (j->desc.type == NKSIM_JOINT_PRISMATIC)
                        jac[col].linear = add(jac[col].linear, axis);
                    else if (j->desc.type == NKSIM_JOINT_REVOLUTE || j->desc.type == NKSIM_JOINT_HELICAL) {
                        jac[col].angular = add(jac[col].angular, axis);
                        jac[col].linear =
                            add(jac[col].linear,
                                cross3(axis, sub(center, add(p.state.position,
                                                             rotate(p.state.rotation, j->desc.anchor_a)))));
                        if (j->desc.type == NKSIM_JOINT_HELICAL)
                            jac[col].linear = add(jac[col].linear,
                                                  scale(axis, j->desc.lead / (2 * 3.14159265358979323846)));
                    }
                }
                current = &p;
            }
            const auto acceleration =
                add(add(kin.a, cross3(kin.alpha, offset)), cross3(kin.w, cross3(kin.w, offset)));
            const auto net_force =
                sub(add(b.force, scale(gravity, b.desc.mass)), scale(acceleration, b.desc.mass));
            const auto net_torque =
                sub(b.torque, add(inertia_times(kin.alpha), cross3(kin.w, inertia_times(kin.w))));
            std::vector<std::size_t> active;
            std::vector<Vec3> inertia_jac(n);
            for (std::size_t i = 0; i < n; ++i)
                if (dot3(jac[i].linear, jac[i].linear) + dot3(jac[i].angular, jac[i].angular) > 0) {
                    active.push_back(i);
                    inertia_jac[i] = inertia_times(jac[i].angular);
                }
            for (auto i : active) {
                force[i] += dot3(jac[i].linear, net_force) + dot3(jac[i].angular, net_torque);
                for (auto j : active)
                    mass[i * n + j] += b.desc.mass * dot3(jac[i].linear, jac[j].linear) +
                                       dot3(jac[i].angular, inertia_jac[j]);
            }
        }
        for (const auto &j : joints) {
            const auto mapping = force_mapping(j.id);
            if (mapping.empty())
                continue;
            double effort = j.target_mode == NKSIM_JOINT_TARGET_EFFORT ? j.state.effort : 0;
            const double cap = j.effort_limit > 0 ? j.effort_limit : j.desc.max_force;
            if (cap > 0)
                effort = std::clamp(effort, -cap, cap);
            if (j.effort_limit < 0)
                effort = 0;
            for (const auto &[col, ratio] : mapping) {
                force[col] += ratio * (effort - j.desc.damping * j.state.velocity);
                for (const auto &[other, weight] : mapping)
                    // Implicit viscous damping uses the same effective inertia for external
                    // forces and contact impulses, preserving blocked-drive reaction forces.
                    mass[col * n + other] += ratio * weight * (j.desc.armature + h * j.desc.damping);
            }
        }
        // Factor independent inertia components separately, preserving all nonzero couplings.
        force_blocks.clear();
        std::vector<bool> assigned(n, false);
        for (std::size_t start = 0; start < n; ++start)
            if (!assigned[start]) {
                std::vector<std::size_t> block{start};
                assigned[start] = true;
                for (std::size_t at = 0; at < block.size(); ++at)
                    for (std::size_t other = 0; other < n; ++other)
                        if (!assigned[other] &&
                            (mass[block[at] * n + other] != 0 || mass[other * n + block[at]] != 0)) {
                            assigned[other] = true;
                            block.push_back(other);
                        }
                std::sort(block.begin(), block.end());
                force_blocks.push_back(std::move(block));
            }
        std::vector<double> lower(n * n, 0), acceleration(n, 0);
        for (const auto &block : force_blocks) {
            for (std::size_t ii = 0; ii < block.size(); ++ii)
                for (std::size_t jj = 0; jj <= ii; ++jj) {
                    const auto i = block[ii], j = block[jj];
                    double value = mass[i * n + j];
                    for (std::size_t kk = 0; kk < jj; ++kk) {
                        const auto k = block[kk];
                        value -= lower[i * n + k] * lower[j * n + k];
                    }
                    if (i == j) {
                        if (!(value > 0) || !std::isfinite(value))
                            return NKSIM_ERROR_BACKEND;
                        lower[i * n + i] = std::sqrt(value);
                    } else
                        lower[i * n + j] = value / lower[j * n + j];
                }
            for (std::size_t ii = 0; ii < block.size(); ++ii) {
                const auto i = block[ii];
                double value = force[i];
                for (std::size_t jj = 0; jj < ii; ++jj) {
                    const auto j = block[jj];
                    value -= lower[i * n + j] * acceleration[j];
                }
                acceleration[i] = value / lower[i * n + i];
            }
            for (std::size_t ii = block.size(); ii > 0; --ii) {
                const auto i = block[ii - 1];
                double value = acceleration[i];
                for (std::size_t jj = ii; jj < block.size(); ++jj) {
                    const auto j = block[jj];
                    value -= lower[j * n + i] * acceleration[j];
                }
                acceleration[i] = value / lower[i * n + i];
            }
        }
        force_coordinates = coordinates;
        force_lower = lower;
        force_velocity.resize(n);
        for (std::size_t k = 0; k < n; ++k) {
            const auto &d = coordinates[k];
            const double v = velocity[k] + h * acceleration[k];
            force_velocity[k] = v;
            if (d.joint)
                find_joint(d.joint)->state.velocity = v;
            else if (d.axis < 3)
                find_body(d.body)->state.linear_velocity[d.axis] = v;
            else
                find_body(d.body)->state.angular_velocity[d.axis - 3] = v;
        }
        for (const auto &j : joints)
            previous_joint_velocity[j.id] = j.state.velocity;
        enforce_couplings();
        return NKSIM_OK;
    }

    struct PointVelocity {
        Vec3 v{}, w{};
    };
    std::vector<BackendContactPair> force_pairs;
    std::vector<FreeCoordinate> force_coordinates;
    std::vector<double> force_lower, force_velocity;
    std::vector<std::vector<std::size_t>> force_blocks;
    struct ImpulseRow {
        std::vector<double> j, response;
        std::vector<std::size_t> active_j, active_response;
        double known = 0, inverse_effective = 0;
    };
    struct ImpulseContact {
        BackendContact contact;
        ImpulseRow normal, tangent_u, tangent_v;
        std::array<ImpulseRow, 3> angular;
        std::array<double, 3> angular_impulse{};
        double torsion = 0, rolling = 0, time_constant = 0, damping_ratio = 1, compliance = 0,
               bias_factor = 0;
        Vec3 u{}, v{};
        double normal_impulse = 0, impulse_u = 0, impulse_v = 0, friction = .5;
    };
    struct LimitImpulse {
        ImpulseRow row;
        double target = 0, impulse = 0;
    };
    struct ForceBox {
        Vec3 center{}, half{};
        std::array<Vec3, 3> axes{};
    };

    std::vector<std::pair<std::uint64_t, double>> coordinate_sources(std::uint64_t id) const {
        std::vector<double> weights(joints.size(), 0);
        std::size_t index = 0;
        for (; index < joints.size(); ++index)
            if (joints[index].id == id)
                break;
        if (index == joints.size())
            return {};
        weights[index] = 1;
        for (auto it = follower_order.rbegin(); it != follower_order.rend(); ++it) {
            std::size_t follower = 0;
            for (; follower < joints.size(); ++follower)
                if (joints[follower].id == *it)
                    break;
            if (follower == joints.size() || weights[follower] == 0)
                continue;
            const double weight = weights[follower];
            weights[follower] = 0;
            for (const auto &c : couplings)
                if (c.follower == *it)
                    for (std::size_t leader = 0; leader < joints.size(); ++leader)
                        if (joints[leader].id == c.leader) {
                            weights[leader] += weight * c.ratio;
                            break;
                        }
        }
        std::vector<std::pair<std::uint64_t, double>> result;
        for (std::size_t i = 0; i < joints.size(); ++i)
            if (weights[i] != 0)
                result.push_back({joints[i].id, weights[i]});
        return result;
    }
    std::vector<std::pair<std::size_t, double>> force_mapping(std::uint64_t id) const {
        std::vector<std::pair<std::size_t, double>> result;
        for (const auto &[source, ratio] : coordinate_sources(id))
            for (std::size_t col = 0; col < force_coordinates.size(); ++col)
                if (force_coordinates[col].joint == source) {
                    result.push_back({col, ratio});
                    break;
                }
        return result;
    }
    std::vector<Vec3> raw_point_jacobian(std::uint64_t id, const Vec3 &point) const {
        std::vector<Vec3> result(force_coordinates.size());
        const auto *current = find_body(id);
        while (current) {
            const auto *joint =
                current->desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(current->id) : nullptr;
            if (!joint) {
                for (std::size_t i = 0; i < force_coordinates.size(); ++i)
                    if (force_coordinates[i].body == current->id) {
                        Vec3 axis{};
                        axis[force_coordinates[i].axis % 3] = 1;
                        result[i] = force_coordinates[i].axis < 3
                                        ? axis
                                        : cross3(axis, sub(point, current->state.position));
                    }
                break;
            }
            const auto *parent = find_body(joint->desc.body_a);
            for (const auto &[col, ratio] : force_mapping(joint->id)) {
                const auto axis = scale(rotate(parent->state.rotation, joint->desc.axis_a), ratio);
                if (joint->desc.type == NKSIM_JOINT_PRISMATIC)
                    result[col] = add(result[col], axis);
                else if (joint->desc.type == NKSIM_JOINT_REVOLUTE ||
                         joint->desc.type == NKSIM_JOINT_HELICAL) {
                    result[col] = add(
                        result[col],
                        cross3(axis, sub(point, add(parent->state.position,
                                                    rotate(parent->state.rotation, joint->desc.anchor_a)))));
                    if (joint->desc.type == NKSIM_JOINT_HELICAL)
                        result[col] =
                            add(result[col], scale(axis, joint->desc.lead / (2 * 3.14159265358979323846)));
                }
            }
            current = parent;
        }
        return result;
    }
    PointVelocity body_velocity(std::uint64_t id) const {
        const auto *b = find_body(id);
        const auto *j = b->desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(id) : nullptr;
        if (!j)
            return {b->state.linear_velocity, b->state.angular_velocity};
        const auto *p = find_body(j->desc.body_a);
        auto pv = body_velocity(p->id);
        const auto axis = rotate(p->state.rotation, j->desc.axis_a);
        const auto origin = add(p->state.position, rotate(p->state.rotation, j->desc.anchor_a));
        Vec3 relative_v{}, relative_w{};
        if (j->desc.type == NKSIM_JOINT_PRISMATIC)
            relative_v = scale(axis, j->state.velocity);
        else if (j->desc.type == NKSIM_JOINT_REVOLUTE || j->desc.type == NKSIM_JOINT_HELICAL) {
            relative_w = scale(axis, j->state.velocity);
            relative_v = cross3(relative_w, sub(b->state.position, origin));
            if (j->desc.type == NKSIM_JOINT_HELICAL)
                relative_v = add(
                    relative_v, scale(axis, j->state.velocity * j->desc.lead / (2 * 3.14159265358979323846)));
        }
        return {add(add(pv.v, cross3(pv.w, sub(b->state.position, p->state.position))), relative_v),
                add(pv.w, relative_w)};
    }
    Vec3 point_velocity(std::uint64_t id, const Vec3 &point) const {
        auto b = find_body(id);
        auto v = body_velocity(id);
        return add(v.v, cross3(v.w, sub(point, b->state.position)));
    }
    double row_velocity(const ImpulseRow &row) const {
        double v = row.known;
        for (auto i : row.active_j)
            v += row.j[i] * force_velocity[i];
        return v;
    }
    ImpulseRow make_impulse_row(std::vector<double> jac, double known) const {
        ImpulseRow row;
        row.j = std::move(jac);
        row.known = known;
        row.response = row.j;
        const auto n = force_coordinates.size();
        for (std::size_t i = 0; i < n; ++i)
            if (row.j[i] != 0)
                row.active_j.push_back(i);
        if(row.active_j.empty())return row;
        for (const auto &block : force_blocks) {
            bool active = false;
            for (auto i : block)
                if (row.j[i] != 0) {
                    active = true;
                    break;
                }
            if (!active)
                continue;
            for (std::size_t ii = 0; ii < block.size(); ++ii) {
                const auto i = block[ii];
                double v = row.response[i];
                for (std::size_t jj = 0; jj < ii; ++jj) {
                    auto j = block[jj];
                    v -= force_lower[i * n + j] * row.response[j];
                }
                row.response[i] = v / force_lower[i * n + i];
            }
            for (std::size_t ii = block.size(); ii > 0; --ii) {
                const auto i = block[ii - 1];
                double v = row.response[i];
                for (std::size_t jj = ii; jj < block.size(); ++jj) {
                    auto j = block[jj];
                    v -= force_lower[j * n + i] * row.response[j];
                }
                row.response[i] = v / force_lower[i * n + i];
            }
            for (auto i : block)
                if (row.response[i] != 0)
                    row.active_response.push_back(i);
        }
        double effective = 0;
        for (auto i : row.active_j)
            effective += row.j[i] * row.response[i];
        row.inverse_effective = effective > 1e-20 ? 1 / effective : 0;
        return row;
    }
    ImpulseRow contact_row(std::uint64_t a, std::uint64_t b, const Vec3 &point, const Vec3 &direction) const {
        auto ja = point_jacobian(a, point), jb = point_jacobian(b, point);
        std::vector<double> row(ja.size());
        double known = dot3(sub(point_velocity(b, point), point_velocity(a, point)), direction);
        for (std::size_t i = 0; i < row.size(); ++i) {
            row[i] = dot3(sub(jb[i], ja[i]), direction);
            known -= row[i] * force_velocity[i];
        }
        return make_impulse_row(std::move(row), known);
    }
    std::vector<Vec3> raw_angular_jacobian(std::uint64_t id) const {
        std::vector<Vec3> result(force_coordinates.size());
        const auto *current = find_body(id);
        while (current) {
            const auto *j =
                current->desc.motion_type == NKSIM_MOTION_DYNAMIC ? parent_joint(current->id) : nullptr;
            if (!j) {
                for (std::size_t i = 0; i < force_coordinates.size(); ++i)
                    if (force_coordinates[i].body == current->id && force_coordinates[i].axis >= 3)
                        result[i][force_coordinates[i].axis - 3] = 1;
                break;
            }
            const auto *parent = find_body(j->desc.body_a);
            if (j->desc.type == NKSIM_JOINT_REVOLUTE || j->desc.type == NKSIM_JOINT_HELICAL)
                for (auto [col, ratio] : force_mapping(j->id))
                    result[col] =
                        add(result[col], scale(rotate(parent->state.rotation, j->desc.axis_a), ratio));
            current = parent;
        }
        return result;
    }
    struct BodyJacobian {
        std::vector<Vec3> linear, angular;
    };
    mutable std::map<std::uint64_t, BodyJacobian> step_jacobians;
    const BodyJacobian &body_jacobian(std::uint64_t id) const {
        auto found = step_jacobians.find(id);
        if (found == step_jacobians.end())
            found = step_jacobians
                        .emplace(id, BodyJacobian{raw_point_jacobian(id, find_body(id)->state.position),
                                                  raw_angular_jacobian(id)})
                        .first;
        return found->second;
    }
    std::vector<Vec3> point_jacobian(std::uint64_t id, const Vec3 &point) const {
        const auto &jac = body_jacobian(id);
        auto result = jac.linear;
        const auto offset = sub(point, find_body(id)->state.position);
        for (std::size_t i = 0; i < result.size(); ++i)
            result[i] = add(result[i], cross3(jac.angular[i], offset));
        return result;
    }
    ImpulseRow angular_row(std::uint64_t a, std::uint64_t b, const Vec3 &direction) const {
        const auto &ja = body_jacobian(a).angular, &jb = body_jacobian(b).angular;
        std::vector<double> row(ja.size());
        double known = dot3(sub(body_velocity(b).w, body_velocity(a).w), direction);
        for (std::size_t i = 0; i < row.size(); ++i) {
            row[i] = dot3(sub(jb[i], ja[i]), direction);
            known -= row[i] * force_velocity[i];
        }
        return make_impulse_row(std::move(row), known);
    }
    void apply_impulse(const ImpulseRow &row, double impulse) {
        for (auto i : row.active_response)
            force_velocity[i] += row.response[i] * impulse;
    }
    void solve_friction_disk(const ImpulseRow &u, const ImpulseRow &v, double &iu, double &iv, double cap) {
        if (cap <= 0) {
            apply_impulse(u, -iu);
            apply_impulse(v, -iv);
            iu = iv = 0;
            return;
        }
        const double uu = u.inverse_effective > 0 ? 1 / u.inverse_effective : 0,
                     vv = v.inverse_effective > 0 ? 1 / v.inverse_effective : 0;
        double uv = 0;
        for (auto i : u.active_j)
            uv += u.j[i] * v.response[i];
        double du = 0, dv = 0;
        const double determinant = uu * vv - uv * uv;
        if (determinant > 1e-20) {
            const double speed_u = row_velocity(u), speed_v = row_velocity(v);
            du = (-vv * speed_u + uv * speed_v) / determinant;
            dv = (uv * speed_u - uu * speed_v) / determinant;
        } else {
            du = -row_velocity(u) * u.inverse_effective;
            dv = -row_velocity(v) * v.inverse_effective;
        }
        double nu = iu + du, nv = iv + dv;
        const double magnitude = std::sqrt(nu * nu + nv * nv);
        if (magnitude > cap && magnitude > 0) {
            nu *= cap / magnitude;
            nv *= cap / magnitude;
        }
        apply_impulse(u, nu - iu);
        apply_impulse(v, nv - iv);
        iu = nu;
        iv = nv;
    }
    static ForceBox force_box(const TestBody &b) {
        return {b.state.position,
                {b.desc.shape_parameters[0], b.desc.shape_parameters[1], b.desc.shape_parameters[2]},
                {rotate(b.state.rotation, {1, 0, 0}), rotate(b.state.rotation, {0, 1, 0}),
                 rotate(b.state.rotation, {0, 0, 1})}};
    }
    static double box_radius(const ForceBox &b, const Vec3 &axis) {
        return b.half[0] * std::abs(dot3(b.axes[0], axis)) + b.half[1] * std::abs(dot3(b.axes[1], axis)) +
               b.half[2] * std::abs(dot3(b.axes[2], axis));
    }
    static std::vector<Vec3> clip_polygon(const std::vector<Vec3> &polygon, const Vec3 &normal,
                                          double offset) {
        std::vector<Vec3> result;
        if (polygon.empty())
            return result;
        Vec3 previous = polygon.back();
        double pd = dot3(normal, previous) - offset;
        for (const auto &current : polygon) {
            double cd = dot3(normal, current) - offset;
            if ((cd <= 0) != (pd <= 0))
                result.push_back(add(previous, scale(sub(current, previous), pd / (pd - cd))));
            if (cd <= 0)
                result.push_back(current);
            previous = current;
            pd = cd;
        }
        return result;
    }
    static Vec3 closest_edge_midpoint(const Vec3 &a0, const Vec3 &a1, const Vec3 &b0, const Vec3 &b1) {
        const auto u = sub(a1, a0), v = sub(b1, b0), w = sub(a0, b0);
        const double aa = dot3(u, u), bb = dot3(u, v), cc = dot3(v, v), dd = dot3(u, w), ee = dot3(v, w);
        double along_a = 0, along_b = 0;
        if (aa <= 1e-24)
            along_b = cc > 1e-24 ? std::clamp(ee / cc, 0.0, 1.0) : 0;
        else if (cc <= 1e-24)
            along_a = std::clamp(-dd / aa, 0.0, 1.0);
        else {
            const double determinant = aa * cc - bb * bb;
            if (determinant > 1e-14 * aa * cc)
                along_a = std::clamp((bb * ee - cc * dd) / determinant, 0.0, 1.0);
            along_b = (bb * along_a + ee) / cc;
            if (along_b < 0) {
                along_b = 0;
                along_a = std::clamp(-dd / aa, 0.0, 1.0);
            } else if (along_b > 1) {
                along_b = 1;
                along_a = std::clamp((bb - dd) / aa, 0.0, 1.0);
            }
        }
        return scale(add(add(a0, scale(u, along_a)), add(b0, scale(v, along_b))), .5);
    }
    static std::pair<Vec3, Vec3> supporting_edge(const ForceBox &box, int edge_axis, const Vec3 &direction) {
        auto center = box.center;
        for (int axis = 0; axis < 3; ++axis)
            if (axis != edge_axis)
                center = add(center,
                             scale(box.axes[axis],
                                   dot3(box.axes[axis], direction) >= 0 ? box.half[axis] : -box.half[axis]));
        const auto extent = scale(box.axes[edge_axis], box.half[edge_axis]);
        return {sub(center, extent), add(center, extent)};
    }
    static std::vector<std::pair<Vec3, double>> box_manifold(const ForceBox &a, const ForceBox &b,
                                                             Vec3 &normal) {
        std::vector<Vec3> axes;
        for (auto n : a.axes)
            axes.push_back(n);
        for (auto n : b.axes)
            axes.push_back(n);
        for (auto x : a.axes)
            for (auto y : b.axes)
                axes.push_back(cross3(x, y));
        const auto between = sub(b.center, a.center);
        double best = std::numeric_limits<double>::infinity();
        int feature = -1;
        for (std::size_t i = 0; i < axes.size(); ++i) {
            auto axis = axes[i];
            double length = std::sqrt(dot3(axis, axis));
            if (length < 1e-12)
                continue;
            axis = scale(axis, 1 / length);
            double depth = box_radius(a, axis) + box_radius(b, axis) - std::abs(dot3(between, axis));
            if (depth <= 0)
                return {};
            double weighted = i < 6 ? depth : depth * 1.01 + 1e-12;
            if (weighted < best) {
                best = weighted;
                normal = dot3(between, axis) >= 0 ? axis : scale(axis, -1);
                feature = static_cast<int>(i);
            }
        }
        std::vector<std::pair<Vec3, double>> result;
        if (feature >= 6) {
            const auto edge_a = supporting_edge(a, (feature - 6) / 3, normal);
            const auto edge_b = supporting_edge(b, (feature - 6) % 3, scale(normal, -1));
            const double depth =
                box_radius(a, normal) + box_radius(b, normal) - std::abs(dot3(between, normal));
            result.push_back(
                {closest_edge_midpoint(edge_a.first, edge_a.second, edge_b.first, edge_b.second), depth});
            return result;
        }
        const auto &reference = feature < 3 ? a : b;
        const auto &incident = feature < 3 ? b : a;
        const auto ref_normal = feature < 3 ? normal : scale(normal, -1);
        const int ref_axis = feature % 3;
        int inc_axis = 0;
        for (int i = 1; i < 3; ++i)
            if (std::abs(dot3(incident.axes[i], ref_normal)) >
                std::abs(dot3(incident.axes[inc_axis], ref_normal)))
                inc_axis = i;
        const auto face_center =
            add(incident.center, scale(incident.axes[inc_axis], dot3(incident.axes[inc_axis], ref_normal) > 0
                                                                    ? -incident.half[inc_axis]
                                                                    : incident.half[inc_axis]));
        const int u = (inc_axis + 1) % 3, v = (inc_axis + 2) % 3;
        std::vector<Vec3> polygon;
        for (const auto &sign : std::array<std::array<int, 2>, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}})
            polygon.push_back(add(face_center, add(scale(incident.axes[u], sign[0] * incident.half[u]),
                                                   scale(incident.axes[v], sign[1] * incident.half[v]))));
        for (int side = 0; side < 3; ++side)
            if (side != ref_axis)
                for (int sign : {-1, 1}) {
                    const auto n = scale(reference.axes[side], sign);
                    polygon = clip_polygon(polygon, n, dot3(n, reference.center) + reference.half[side]);
                }
        const auto ref_center = add(reference.center, scale(ref_normal, reference.half[ref_axis]));
        for (const auto &p : polygon) {
            double distance = dot3(ref_normal, sub(p, ref_center));
            if (distance <= 0)
                result.push_back({sub(p, scale(ref_normal, distance * .5)), -distance});
        }
        return result;
    }
    struct ForceHull {
        std::vector<Vec3> vertices, normals;
        std::vector<std::vector<std::size_t>> faces;
        std::vector<std::pair<std::size_t, std::size_t>> edges;
    };
    static ForceHull force_hull(const std::vector<double> &coordinates) {
        ForceHull hull;
        for (std::size_t i = 0; i + 2 < coordinates.size(); i += 3)
            hull.vertices.push_back({coordinates[i], coordinates[i + 1], coordinates[i + 2]});
        for (const auto &plane : geometry::hull_planes(coordinates)) {
            Vec3 normal{plane[0], plane[1], plane[2]}, center{};
            std::vector<std::size_t> face;
            for (std::size_t i = 0; i < hull.vertices.size(); ++i)
                if (std::abs(dot3(normal, hull.vertices[i]) - plane[3]) < 1e-9) {
                    face.push_back(i);
                    center = add(center, hull.vertices[i]);
                }
            if (face.size() < 3)
                continue;
            center = scale(center, 1.0 / face.size());
            auto u = cross3(normal, std::abs(normal[2]) < .9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
            u = scale(u, 1 / std::sqrt(dot3(u, u)));
            const auto v = cross3(normal, u);
            std::sort(face.begin(), face.end(), [&](auto a, auto b) {
                const auto x = sub(hull.vertices[a], center), y = sub(hull.vertices[b], center);
                return std::atan2(dot3(x, v), dot3(x, u)) < std::atan2(dot3(y, v), dot3(y, u));
            });
            for (std::size_t i = 0; i < face.size(); ++i) {
                auto a = face[i], b = face[(i + 1) % face.size()];
                if (a > b)
                    std::swap(a, b);
                if (std::find(hull.edges.begin(), hull.edges.end(), std::make_pair(a, b)) == hull.edges.end())
                    hull.edges.push_back({a, b});
            }
            hull.normals.push_back(normal);
            hull.faces.push_back(std::move(face));
        }
        return hull;
    }
    static std::vector<std::pair<Vec3, double>> hull_manifold(const ForceHull &a, const ForceHull &b,
                                                              Vec3 &normal) {
        if (a.faces.empty() || b.faces.empty())
            return {};
        double best = std::numeric_limits<double>::infinity(), penetration = 0;
        int kind = -1;
        std::size_t edge_a = 0, edge_b = 0;
        std::set<std::array<long long, 3>> tested_axes;
        auto axis_test = [&](Vec3 axis, int candidate, std::size_t ea = 0, std::size_t eb = 0) {
            double length = std::sqrt(dot3(axis, axis));
            if (length < 1e-12)
                return true;
            axis = scale(axis, 1 / length);
            Vec3 canonical = axis;
            int major = 0;
            for (int i = 1; i < 3; ++i)
                if (std::abs(canonical[i]) > std::abs(canonical[major]))
                    major = i;
            if (canonical[major] < 0)
                canonical = scale(canonical, -1);
            std::array<long long, 3> key{};
            for (int i = 0; i < 3; ++i)
                key[i] = std::llround(canonical[i] * 1e12);
            if (!tested_axes.insert(key).second)
                return true;
            double amin = std::numeric_limits<double>::infinity(), amax = -amin, bmin = amin, bmax = -amin;
            for (const auto &p : a.vertices) {
                double x = dot3(p, axis);
                amin = std::min(amin, x);
                amax = std::max(amax, x);
            }
            for (const auto &p : b.vertices) {
                double x = dot3(p, axis);
                bmin = std::min(bmin, x);
                bmax = std::max(bmax, x);
            }
            if (amax <= bmin || bmax <= amin)
                return false;
            double depth = amax - bmin;
            if (bmax - amin < depth) {
                depth = bmax - amin;
                axis = scale(axis, -1);
            }
            double score = candidate == 2 ? depth * 1.01 + 1e-12 : depth;
            if (score < best) {
                best = score;
                penetration = depth;
                normal = axis;
                kind = candidate;
                edge_a = ea;
                edge_b = eb;
            }
            return true;
        };
        for (const auto &n : a.normals)
            if (!axis_test(n, 0))
                return {};
        for (const auto &n : b.normals)
            if (!axis_test(n, 1))
                return {};
        auto distinct_edges = [](const ForceHull &hull) {
            std::vector<std::pair<std::size_t, Vec3>> result;
            for (std::size_t i = 0; i < hull.edges.size(); ++i) {
                auto edge = hull.edges[i];
                auto direction = sub(hull.vertices[edge.second], hull.vertices[edge.first]);
                const double norm = std::sqrt(dot3(direction, direction));
                if (norm < 1e-14)
                    continue;
                direction = scale(direction, 1 / norm);
                bool duplicate = false;
                for (const auto &item : result) {
                    const auto cross = cross3(direction, item.second);
                    if (dot3(cross, cross) < 1e-24) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate)
                    result.push_back({i, direction});
            }
            return result;
        };
        const auto edges_a = distinct_edges(a), edges_b = distinct_edges(b);
        for (const auto &[i, x] : edges_a)
            for (const auto &[j, y] : edges_b)
                if (!axis_test(cross3(x, y), 2, i, j))
                    return {};
        if (kind == 2) {
            auto x = a.edges[edge_a], y = b.edges[edge_b];
            // Select support edges parallel to the winning edge directions.
            auto support = [&](const ForceHull &h, Vec3 direction, Vec3 edge) {
                std::pair<std::size_t, std::size_t> selected{};
                double score = -std::numeric_limits<double>::infinity();
                double ee = dot3(edge, edge);
                for (auto e : h.edges) {
                    auto delta = sub(h.vertices[e.second], h.vertices[e.first]);
                    if (dot3(cross3(delta, edge), cross3(delta, edge)) > 1e-16 * dot3(delta, delta) * ee)
                        continue;
                    double value = dot3(add(h.vertices[e.first], h.vertices[e.second]), direction);
                    if (value > score) {
                        score = value;
                        selected = e;
                    }
                }
                return selected;
            };
            x = support(a, normal, sub(a.vertices[x.second], a.vertices[x.first]));
            y = support(b, scale(normal, -1), sub(b.vertices[y.second], b.vertices[y.first]));
            return {{closest_edge_midpoint(a.vertices[x.first], a.vertices[x.second], b.vertices[y.first],
                                           b.vertices[y.second]),
                     penetration}};
        }
        const auto &reference = kind == 0 ? a : b;
        const auto &incident = kind == 0 ? b : a;
        const auto rn = kind == 0 ? normal : scale(normal, -1);
        std::size_t rf = 0, inf = 0;
        for (std::size_t i = 1; i < reference.normals.size(); ++i)
            if (dot3(reference.normals[i], rn) > dot3(reference.normals[rf], rn))
                rf = i;
        for (std::size_t i = 1; i < incident.normals.size(); ++i)
            if (dot3(incident.normals[i], rn) < dot3(incident.normals[inf], rn))
                inf = i;
        std::vector<Vec3> polygon;
        for (auto i : incident.faces[inf])
            polygon.push_back(incident.vertices[i]);
        const auto &face = reference.faces[rf];
        for (std::size_t i = 0; i < face.size(); ++i) {
            const auto start = reference.vertices[face[i]],
                       end = reference.vertices[face[(i + 1) % face.size()]];
            const auto outward = cross3(sub(end, start), rn);
            polygon = clip_polygon(polygon, outward, dot3(outward, start));
        }
        std::vector<std::pair<Vec3, double>> result;
        const auto origin = reference.vertices[face[0]];
        for (const auto &p : polygon) {
            double distance = dot3(rn, sub(p, origin));
            if (distance <= 0)
                result.push_back({sub(p, scale(rn, distance * .5)), -distance});
        }
        return result;
    }
    static ForceHull body_hull(const TestBody &body) {
        std::vector<double> coordinates;
        auto append = [&](Vec3 local) {
            auto world = add(body.state.position, rotate(body.state.rotation, local));
            coordinates.insert(coordinates.end(), world.begin(), world.end());
        };
        if (body.desc.shape_type == NKSIM_SHAPE_BOX) {
            for (int x : {-1, 1})
                for (int y : {-1, 1})
                    for (int z : {-1, 1})
                        append({x * body.desc.shape_parameters[0], y * body.desc.shape_parameters[1],
                                z * body.desc.shape_parameters[2]});
        } else if (body.desc.shape_type == NKSIM_SHAPE_CONVEX) {
            for (std::size_t i = 0; i + 2 < body.desc.shape_vertices.size(); i += 3)
                append({body.desc.shape_vertices[i], body.desc.shape_vertices[i + 1],
                        body.desc.shape_vertices[i + 2]});
        }
        return force_hull(coordinates);
    }
    static TestBody part_body(const TestBody &body, int index) {
        if (index < 0)
            return body;
        const auto &part = body.desc.shape_parts[index];
        TestBody piece;
        piece.id = body.id;
        piece.state.position = add(body.state.position, rotate(body.state.rotation, part.position));
        piece.state.rotation = multiply(body.state.rotation, part.rotation);
        piece.desc.shape_type = part.type;
        piece.desc.shape_parameters = part.parameters;
        piece.desc.shape_vertices = part.vertices;
        piece.desc.shape_parts.clear();
        return piece;
    }
    std::map<std::pair<std::uint64_t, int>, ForceHull> local_hulls;
    ForceHull cached_world_hull(const TestBody &body, int part) {
        const auto key = std::make_pair(body.id, part);
        auto found = local_hulls.find(key);
        if (found == local_hulls.end()) {
            TestBody local = body;
            local.state.position = {};
            local.state.rotation = {0, 0, 0, 1};
            found = local_hulls.emplace(key, body_hull(part_body(local, part))).first;
        }
        ForceHull world = found->second;
        for (auto &point : world.vertices)
            point = add(body.state.position, rotate(body.state.rotation, point));
        for (auto &normal : world.normals)
            normal = rotate(body.state.rotation, normal);
        return world;
    }
    static bool derive_hull_inertia(BackendBodyDesc &desc) {
        double volume = 0;
        Vec3 first{};
        std::array<double, 9> second{};
        TestBody local;
        local.desc = desc;
        local.state.rotation = {0, 0, 0, 1};
        const int count = std::max(1, static_cast<int>(desc.shape_parts.size()));
        for (int part = 0; part < count; ++part) {
            const auto piece = part_body(local, desc.shape_parts.empty() ? -1 : part);
            if (piece.desc.shape_type != NKSIM_SHAPE_BOX && piece.desc.shape_type != NKSIM_SHAPE_CONVEX)
                return false;
            const auto hull = body_hull(piece);
            if (hull.faces.empty())
                return false;
            for (const auto &face : hull.faces)
                for (std::size_t i = 1; i + 1 < face.size(); ++i) {
                    const auto a = hull.vertices[face[0]], b = hull.vertices[face[i]],
                               c = hull.vertices[face[i + 1]];
                    const double tetra = dot3(a, cross3(b, c)) / 6;
                    volume += tetra;
                    const auto sum = add(add(a, b), c);
                    first = add(first, scale(sum, tetra / 4));
                    for (int r = 0; r < 3; ++r)
                        for (int col = 0; col < 3; ++col)
                            second[r * 3 + col] +=
                                tetra * (sum[r] * sum[col] + a[r] * a[col] + b[r] * b[col] + c[r] * c[col]) /
                                20;
                }
        }
        if (!(volume > 1e-18))
            return false;
        const auto center = scale(first, 1 / volume);
        const double density = desc.mass / volume;
        std::array<double, 9> centered{};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                centered[r * 3 + c] = density * second[r * 3 + c] - desc.mass * center[r] * center[c];
        const double trace = centered[0] + centered[4] + centered[8];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                desc.inertia_tensor[r * 3 + c] = (r == c ? trace : 0) - centered[r * 3 + c];
        desc.center_of_mass = center;
        desc.has_inertial_properties = true;
        return true;
    }
    bool same_articulation(std::uint64_t a, std::uint64_t b) const {
        std::vector<std::uint64_t> pending{a};
        for (std::size_t i = 0; i < pending.size(); ++i) {
            if (pending[i] == b)
                return true;
            for (const auto &j : joints) {
                std::uint64_t next = 0;
                if (j.desc.body_a == pending[i])
                    next = j.desc.body_b;
                else if (j.desc.body_b == pending[i])
                    next = j.desc.body_a;
                if (next && std::find(pending.begin(), pending.end(), next) == pending.end())
                    pending.push_back(next);
            }
        }
        return false;
    }
    struct CachedManifold {
        Vec3 relative_position{}, normal{};
        Quat relative_rotation{};
        std::vector<std::pair<Vec3, double>> points;
        bool valid = false;
    };
    std::map<std::array<std::uint64_t, 4>, CachedManifold> contact_geometry;
    std::vector<std::pair<Vec3, double>> geometry_manifold(const TestBody &a, int pa, const TestBody &b,
                                                           int pb, const TestBody &piece_a,
                                                           const TestBody &piece_b, Vec3 &normal) {
        auto &cached = contact_geometry[{a.id, static_cast<std::uint64_t>(pa + 1), b.id,
                                         static_cast<std::uint64_t>(pb + 1)}];
        const auto inverse = conjugate(a.state.rotation);
        const auto position = rotate(inverse, sub(b.state.position, a.state.position));
        const auto rotation = multiply(inverse, b.state.rotation);
        bool unchanged = cached.valid;
        for (int i = 0; i < 3; ++i)
            unchanged = unchanged && std::abs(position[i] - cached.relative_position[i]) < 1e-12;
        for (int i = 0; i < 4; ++i)
            unchanged = unchanged && std::abs(rotation[i] - cached.relative_rotation[i]) < 1e-12;
        if (!unchanged) {
            auto points =
                piece_a.desc.shape_type == NKSIM_SHAPE_BOX && piece_b.desc.shape_type == NKSIM_SHAPE_BOX
                    ? box_manifold(force_box(piece_a), force_box(piece_b), normal)
                    : hull_manifold(cached_world_hull(a, pa), cached_world_hull(b, pb), normal);
            cached.relative_position = position;
            cached.relative_rotation = rotation;
            cached.normal = rotate(inverse, normal);
            cached.points = std::move(points);
            cached.valid = true;
            for (auto &point : cached.points)
                point.first = rotate(inverse, sub(point.first, a.state.position));
        }
        normal = rotate(a.state.rotation, cached.normal);
        auto points = cached.points;
        for (auto &point : points)
            point.first = add(a.state.position, rotate(a.state.rotation, point.first));
        return points;
    }
    struct ForceBounds {
        Vec3 minimum{INFINITY, INFINITY, INFINITY}, maximum{-INFINITY, -INFINITY, -INFINITY};
        void include(const Vec3 &p) {
            for (int i = 0; i < 3; ++i) {
                minimum[i] = std::min(minimum[i], p[i]);
                maximum[i] = std::max(maximum[i], p[i]);
            }
        }
        bool overlaps(const ForceBounds &b) const {
            for (int i = 0; i < 3; ++i)
                if (maximum[i] < b.minimum[i] || b.maximum[i] < minimum[i])
                    return false;
            return true;
        }
    };
    static ForceBounds piece_bounds(const TestBody &piece) {
        ForceBounds bounds;
        if (piece.desc.shape_type == NKSIM_SHAPE_BOX) {
            const auto box = force_box(piece);
            Vec3 radius{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    radius[i] += std::abs(box.axes[j][i]) * box.half[j];
            bounds.include(sub(box.center, radius));
            bounds.include(add(box.center, radius));
        } else
            for (std::size_t i = 0; i + 2 < piece.desc.shape_vertices.size(); i += 3)
                bounds.include(add(piece.state.position,
                                   rotate(piece.state.rotation,
                                          {piece.desc.shape_vertices[i], piece.desc.shape_vertices[i + 1],
                                           piece.desc.shape_vertices[i + 2]})));
        return bounds;
    }
    nksim_result force_step(double h) {
        step_jacobians.clear();
        auto outcome = advance_efforts(h);
        if (outcome != NKSIM_OK)
            return outcome;
        if (contact_geometry.size() > 16384)
            contact_geometry.clear();
        std::vector<std::size_t> welded(bodies.size());
        for (std::size_t i = 0; i < welded.size(); ++i)
            welded[i] = i;
        auto weld_root = [&](std::size_t i) {
            while (welded[i] != i)
                i = welded[i];
            return i;
        };
        for (const auto &joint : joints)
            if (joint.desc.type == NKSIM_JOINT_FIXED) {
                const auto a = body_index(joint.desc.body_a), b = body_index(joint.desc.body_b);
                if (a >= 0 && b >= 0)
                    welded[weld_root(b)] = weld_root(a);
            }
        for (auto &group : welded)
            group = weld_root(group);
        std::set<std::pair<std::size_t, std::size_t>> adjacent;
        for (const auto &joint : joints)
            if (joint.desc.type != NKSIM_JOINT_FIXED) {
                const auto ai = body_index(joint.desc.body_a), bi = body_index(joint.desc.body_b);
                if (ai >= 0 && bi >= 0) {
                    auto a = welded[ai], b = welded[bi];
                    if (a > b)
                        std::swap(a, b);
                    adjacent.insert({a, b});
                }
            }
        std::vector<ImpulseContact> rows;
        std::vector<std::vector<TestBody>> pieces(bodies.size());
        std::vector<std::vector<ForceBounds>> bounds(bodies.size());
        std::vector<ForceBounds> body_bounds(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const auto &body = bodies[i];
            const int count = std::max(1, static_cast<int>(body.desc.shape_parts.size()));
            pieces[i].reserve(count);
            bounds[i].reserve(count);
            for (int part = 0; part < count; ++part) {
                pieces[i].push_back(part_body(body, body.desc.shape_parts.empty() ? -1 : part));
                bounds[i].push_back(piece_bounds(pieces[i].back()));
                body_bounds[i].include(bounds[i].back().minimum);
                body_bounds[i].include(bounds[i].back().maximum);
            }
        }
        for (std::size_t ai = 0; ai < bodies.size(); ++ai)
            for (std::size_t bi = ai + 1; bi < bodies.size(); ++bi) {
                const auto &a = bodies[ai], &b = bodies[bi];
                if (!body_bounds[ai].overlaps(body_bounds[bi]))
                    continue;
                if (excluded_pairs.count({std::min(a.id, b.id), std::max(a.id, b.id)}))
                    continue;
                const bool layered = (a.desc.collision_mask & b.desc.collision_layer) ||
                                     (b.desc.collision_mask & a.desc.collision_layer);
                if (a.desc.motion_type == NKSIM_MOTION_STATIC && b.desc.motion_type == NKSIM_MOTION_STATIC)
                    continue;
                const int count_a = std::max(1, static_cast<int>(a.desc.shape_parts.size())),
                          count_b = std::max(1, static_cast<int>(b.desc.shape_parts.size()));
                for (int ia = 0; ia < count_a; ++ia)
                    for (int ib = 0; ib < count_b; ++ib) {
                        if (!bounds[ai][ia].overlaps(bounds[bi][ib]))
                            continue;
                        const int pa = a.desc.shape_parts.empty() ? -1 : ia,
                                  pb = b.desc.shape_parts.empty() ? -1 : ib;
                        const BackendShapePart default_surface;
                        const auto &part_a = pa < 0 ? default_surface : a.desc.shape_parts[pa],
                                   &part_b = pb < 0 ? default_surface : b.desc.shape_parts[pb];
                        const BackendContactPair *explicit_pair = nullptr;
                        for (const auto &pair : force_pairs)
                            if ((pair.body_a == a.id && pair.body_b == b.id &&
                                 pair.part_a == static_cast<unsigned>(ia) &&
                                 pair.part_b == static_cast<unsigned>(ib)) ||
                                (pair.body_a == b.id && pair.body_b == a.id &&
                                 pair.part_a == static_cast<unsigned>(ib) &&
                                 pair.part_b == static_cast<unsigned>(ia)))
                                explicit_pair = &pair;
                        auto group_a = welded[ai], group_b = welded[bi];
                        if (group_a > group_b)
                            std::swap(group_a, group_b);
                        const bool adjacent_moving =
                            adjacent.count({group_a, group_b}) &&
                            bodies[group_a].desc.motion_type != NKSIM_MOTION_STATIC &&
                            bodies[group_b].desc.motion_type != NKSIM_MOTION_STATIC;
                        if (!explicit_pair && (group_a == group_b || adjacent_moving))
                            continue;
                        const bool environment =
                            layered &&
                            ((part_a.contact_filter == NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT &&
                              part_b.contact_filter == NKSIM_CONTACT_LAYERS) ||
                             (part_b.contact_filter == NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT &&
                              part_a.contact_filter == NKSIM_CONTACT_LAYERS)) &&
                            !same_articulation(a.id, b.id);
                        if (!explicit_pair && !environment &&
                            !(layered && part_a.contact_filter == NKSIM_CONTACT_LAYERS &&
                              part_b.contact_filter == NKSIM_CONTACT_LAYERS))
                            continue;
                        const auto &piece_a = pieces[ai][ia], &piece_b = pieces[bi][ib];
                        const auto supported = [](auto type) {
                            return type == NKSIM_SHAPE_BOX || type == NKSIM_SHAPE_CONVEX;
                        };
                        if (!supported(piece_a.desc.shape_type) || !supported(piece_b.desc.shape_type))
                            return NKSIM_ERROR_UNSUPPORTED;
                        Vec3 normal{};
                        auto manifold = geometry_manifold(a, pa, b, pb, piece_a, piece_b, normal);
                        for (const auto &[point, depth] : manifold) {
                            ImpulseContact c;
                            auto surface = [&](const BackendShapePart &value) {
                                const auto dimensions =
                                    value.friction_dimensions ? value.friction_dimensions : 3;
                                c.friction =
                                    dimensions >= 3 ? (value.friction[0] > 0 ? value.friction[0] : .5) : 0;
                                c.torsion = dimensions >= 4 ? value.friction[1] : 0;
                                c.rolling = dimensions >= 6 ? value.friction[2] : 0;
                                c.time_constant = value.contact_time_constant;
                                c.damping_ratio =
                                    value.contact_damping_ratio > 0 ? value.contact_damping_ratio : 1;
                            };
                            // Symmetric piece mixing; an explicit pair supplies its own surface.
                            BackendShapePart mixed;
                            const BackendShapePart defaults;
                            const auto &sa = pa >= 0 ? a.desc.shape_parts[pa] : defaults,
                                       &sb = pb >= 0 ? b.desc.shape_parts[pb] : defaults;
                            mixed.friction_dimensions =
                                std::max(sa.friction_dimensions ? sa.friction_dimensions : 3,
                                         sb.friction_dimensions ? sb.friction_dimensions : 3);
                            for (int axis = 0; axis < 3; ++axis)
                                mixed.friction[axis] = std::max(sa.friction[axis], sb.friction[axis]);
                            mixed.contact_time_constant =
                                std::max(sa.contact_time_constant, sb.contact_time_constant);
                            mixed.contact_damping_ratio =
                                std::max(sa.contact_damping_ratio, sb.contact_damping_ratio);
                            surface(mixed);
                            if (environment)
                                surface(part_a.contact_filter == NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT
                                            ? part_a
                                            : part_b);
                            if (explicit_pair)
                                surface(explicit_pair->surface);
                            c.contact.part_a = pa;
                            c.contact.part_b = pb;
                            c.contact.body_a = a.id;
                            c.contact.body_b = b.id;
                            c.contact.position = point;
                            c.contact.normal = normal;
                            c.contact.distance = -depth;
                            c.contact.active = true;
                            c.u = cross3(normal, std::abs(normal[2]) < .9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
                            c.u = scale(c.u, 1 / std::sqrt(dot3(c.u, c.u)));
                            c.v = cross3(normal, c.u);
                            c.normal = contact_row(a.id, b.id, point, normal);
                            if (c.normal.inverse_effective == 0) {
                                contacts.push_back(c.contact);
                                continue;
                            }
                            c.tangent_u = contact_row(a.id, b.id, point, c.u);
                            c.tangent_v = contact_row(a.id, b.id, point, c.v);
                            if (c.time_constant > 0 && c.normal.inverse_effective > 0) {
                                const double rate = 2 * c.damping_ratio / c.time_constant +
                                                    h / (c.time_constant * c.time_constant);
                                c.compliance = 1 / (c.normal.inverse_effective * h * rate);
                                c.bias_factor = 1 / (2 * c.damping_ratio * c.time_constant + h);
                            }
                            c.angular = {angular_row(a.id, b.id, normal), angular_row(a.id, b.id, c.u),
                                         angular_row(a.id, b.id, c.v)};
                            rows.push_back(std::move(c));
                        }
                    }
            }
        std::vector<LimitImpulse> limits;
        for (const auto &j : joints) {
            if (j.desc.type == NKSIM_JOINT_FIXED)
                continue;
            const auto mapping = force_mapping(j.id);
            if (mapping.empty())
                continue;
            double speed = 0;
            for (const auto &[col, ratio] : mapping)
                speed += ratio * force_velocity[col];
            double known = j.state.velocity;
            for (const auto &[col, ratio] : mapping)
                known -= ratio * find_joint(force_coordinates[col].joint)->state.velocity;
            speed += known;
            const double predicted = j.state.position + h * speed;
            for (int side : {-1, 1}) {
                const double bound = side < 0 ? j.desc.lower_limit : j.desc.upper_limit;
                if ((side < 0 && predicted >= bound) || (side > 0 && predicted <= bound))
                    continue;
                std::vector<double> jac(force_coordinates.size());
                for (const auto &[col, ratio] : mapping)
                    jac[col] = -side * ratio;
                LimitImpulse row;
                row.row = make_impulse_row(std::move(jac), -side * known);
                row.target = -side * (bound - j.state.position) / h;
                limits.push_back(std::move(row));
            }
        }
        for (int iteration = 0; iteration < 48; ++iteration) {
            for (auto &limit : limits) {
                const double old = limit.impulse;
                limit.impulse = std::max(0.0, old + (limit.target - row_velocity(limit.row)) *
                                                        limit.row.inverse_effective);
                apply_impulse(limit.row, limit.impulse - old);
            }
            for (auto &c : rows) {
                const double bias = c.time_constant > 0 ? c.bias_factor * (-c.contact.distance)
                                                        : .2 * std::max(0.0, -c.contact.distance - 1e-6) / h;
                const double response =
                    c.normal.inverse_effective / (1 + c.compliance * c.normal.inverse_effective);
                const double old = c.normal_impulse;
                c.normal_impulse =
                    std::max(0.0, old + (bias - row_velocity(c.normal) - c.compliance * old) * response);
                apply_impulse(c.normal, c.normal_impulse - old);
                solve_friction_disk(c.tangent_u, c.tangent_v, c.impulse_u, c.impulse_v,
                                    c.friction * c.normal_impulse);
                const double cap = c.torsion * c.normal_impulse, old_torsion = c.angular_impulse[0];
                c.angular_impulse[0] = std::clamp(
                    old_torsion - row_velocity(c.angular[0]) * c.angular[0].inverse_effective, -cap, cap);
                apply_impulse(c.angular[0], c.angular_impulse[0] - old_torsion);
                solve_friction_disk(c.angular[1], c.angular[2], c.angular_impulse[1], c.angular_impulse[2],
                                    c.rolling * c.normal_impulse);
            }
        }
        // Write the solved velocities, then integrate once, after contact resolution.
        for (std::size_t k = 0; k < force_coordinates.size(); ++k) {
            const auto &d = force_coordinates[k];
            if (d.joint)
                find_joint(d.joint)->state.velocity = force_velocity[k];
            else if (d.axis < 3)
                find_body(d.body)->state.linear_velocity[d.axis] = force_velocity[k];
            else
                find_body(d.body)->state.angular_velocity[d.axis - 3] = force_velocity[k];
        }
        enforce_couplings();
        for (auto &b : bodies)
            if (b.desc.motion_type == NKSIM_MOTION_DYNAMIC && parent_joint(b.id) == nullptr) {
                b.state.position = add(b.state.position, scale(b.state.linear_velocity, h));
                double speed = std::sqrt(dot3(b.state.angular_velocity, b.state.angular_velocity));
                if (speed > 0) {
                    const double f = std::sin(speed * h / 2) / speed;
                    const auto &w = b.state.angular_velocity;
                    b.state.rotation = normalize(
                        multiply({w[0] * f, w[1] * f, w[2] * f, std::cos(speed * h / 2)}, b.state.rotation));
                }
            }
        for (auto &j : joints) {
            if (j.target_mode != NKSIM_JOINT_TARGET_POSITION && j.target_mode != NKSIM_JOINT_TARGET_SERVO)
                j.state.position += h * j.state.velocity;
            clamp_joint_position(j);
        }
        enforce_couplings();
        outcome = recompute_articulated_poses(0);
        if (outcome != NKSIM_OK)
            return outcome;
        std::vector<PointVelocity> published;
        for (const auto &b : bodies)
            published.push_back(body_velocity(b.id));
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].desc.motion_type == NKSIM_MOTION_DYNAMIC && parent_joint(bodies[i].id) != nullptr) {
                bodies[i].state.linear_velocity = published[i].v;
                bodies[i].state.angular_velocity = published[i].w;
            }
        record_velocity_history();
        for (const auto &c : rows) {
            auto observation = c.contact;
            observation.force_on_body_a = scale(add(scale(c.contact.normal, c.normal_impulse),
                                                    add(scale(c.u, c.impulse_u), scale(c.v, c.impulse_v))),
                                                -1 / h);
            observation.torque_on_body_a =
                scale(add(scale(c.contact.normal, c.angular_impulse[0]),
                          add(scale(c.u, c.angular_impulse[1]), scale(c.v, c.angular_impulse[2]))),
                      -1 / h);
            observation.wrench_valid = c.normal.inverse_effective > 0;
            contacts.push_back(observation);
        }
        return NKSIM_OK;
    }

    std::array<double, 3> gravity{};
    std::vector<TestBody> bodies;
    std::vector<BackendContact> contacts;
    std::set<std::pair<std::uint64_t, std::uint64_t>> excluded_pairs;
    std::vector<TestJoint> joints;
    std::vector<BackendJointCoupling> couplings;
    std::vector<std::uint64_t> follower_order;
    std::uint64_t next_body = 1;
    std::uint64_t next_joint = 1;
};

} // namespace

std::unique_ptr<PhysicsBackend> make_test_physics_backend() {
    return std::make_unique<TestPhysicsBackend>();
}

} // namespace nksim
