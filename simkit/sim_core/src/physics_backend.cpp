#include "PhysicsBackend.hpp"
#include "convex_overlap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
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

    nksim_result body_set_mass(std::uint64_t id, double mass) override {
        auto *body = find_body(id);
        if (!body) return NKSIM_ERROR_INVALID_HANDLE;
        if (!std::isfinite(mass) || mass <= 0.0) return NKSIM_ERROR_INVALID_ARGUMENT;
        if (body->desc.motion_type == NKSIM_MOTION_STATIC) return NKSIM_ERROR_INVALID_STATE;
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
            switch (targets[index].mode) {
            case NKSIM_JOINT_TARGET_POSITION:
            case NKSIM_JOINT_TARGET_SERVO:
                // Joints here follow their targets exactly, so a servo's
                // position target is reached at once, as a position target is.
                joint->state.position = targets[index].target;
                joint->state.velocity = 0.0;
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

    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const double substep = dt / static_cast<double>(substeps);
        for (std::uint32_t iteration = 0; iteration < substeps; ++iteration) {
            contacts.clear();
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
