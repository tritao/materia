#include "nativekit_sim_session.h"
#include "internal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace nksim {
namespace {

using Vec3 = std::array<double, 3>;

void rotate(const double q[4], const double v[3], double out[3], bool inverse = false) {
    const double x = inverse ? -q[0] : q[0], y = inverse ? -q[1] : q[1],
                 z = inverse ? -q[2] : q[2], w = q[3];
    const double t[3] = {2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]),
                         2.0 * (x * v[1] - y * v[0])};
    out[0] = v[0] + w * t[0] + y * t[2] - z * t[1];
    out[1] = v[1] + w * t[1] + z * t[0] - x * t[2];
    out[2] = v[2] + w * t[2] + x * t[1] - y * t[0];
}

void multiply(const double a[4], const double b[4], double out[4]) {
    out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

bool valid_pose(const nksim_pose &pose) {
    double norm = 0.0;
    for (const double value : pose.position)
        if (!std::isfinite(value)) return false;
    for (const double value : pose.rotation) {
        if (!std::isfinite(value)) return false;
        norm += value * value;
    }
    return std::abs(norm - 1.0) < 1e-6;
}

nksim_pose make_pose(const double position[3], const double rotation[4]) {
    nksim_pose pose{};
    pose.struct_size = sizeof(pose);
    std::copy_n(position, 3, pose.position);
    std::copy_n(rotation, 4, pose.rotation);
    return pose;
}

// World-frame angular velocity that turns `from` onto `to` in `dt`, the
// shorter way around.
void angular_velocity_between(const double from[4], const double to[4], double dt,
                              double out[3]) {
    const double inverse[4] = {-from[0], -from[1], -from[2], from[3]};
    double relative[4];
    multiply(to, inverse, relative);
    if (relative[3] < 0.0)
        for (double &component : relative) component = -component;
    const double sine = std::sqrt(relative[0] * relative[0] + relative[1] * relative[1] +
                                  relative[2] * relative[2]);
    const double scale = sine > 1e-12 ? 2.0 * std::atan2(sine, relative[3]) / sine : 2.0;
    for (int axis = 0; axis < 3; ++axis) out[axis] = relative[axis] * scale / dt;
}

nksim_pose interpolate(const nksim_pose &a, const nksim_pose &b, double t) {
    nksim_pose result = a;
    for (int axis = 0; axis < 3; ++axis)
        result.position[axis] = a.position[axis] + (b.position[axis] - a.position[axis]) * t;
    double dot = 0.0;
    for (int index = 0; index < 4; ++index) dot += a.rotation[index] * b.rotation[index];
    const double sign = dot < 0.0 ? -1.0 : 1.0;
    dot *= sign;
    double wa = 1.0 - t, wb = t * sign;
    if (dot < 0.9995) {
        const double angle = std::acos(std::min(dot, 1.0));
        const double sine = std::sin(angle);
        wa = std::sin((1.0 - t) * angle) / sine;
        wb = std::sin(t * angle) / sine * sign;
    }
    double norm = 0.0;
    for (int index = 0; index < 4; ++index) {
        result.rotation[index] = a.rotation[index] * wa + b.rotation[index] * wb;
        norm += result.rotation[index] * result.rotation[index];
    }
    norm = std::sqrt(norm);
    for (double &component : result.rotation) component /= norm;
    return result;
}

nkscene_transform scene_transform(const nksim_pose &pose) {
    nkscene_transform transform{};
    for (int column = 0; column < 3; ++column) {
        double axis[3]{};
        axis[column] = 1.0;
        double rotated[3];
        rotate(pose.rotation, axis, rotated);
        for (int row = 0; row < 3; ++row)
            transform.matrix[column * 4 + row] = static_cast<float>(rotated[row]);
    }
    for (int axis = 0; axis < 3; ++axis)
        transform.matrix[12 + axis] = static_cast<float>(pose.position[axis]);
    transform.matrix[15] = 1.0f;
    return transform;
}

bool valid_shape(const nksim_shape_desc &shape) {
    switch (shape.type) {
    case NKSIM_SHAPE_BOX:
        return shape.parameters[0] > 0.0 && shape.parameters[1] > 0.0 &&
            shape.parameters[2] > 0.0 && std::isfinite(shape.parameters[0]) &&
            std::isfinite(shape.parameters[1]) && std::isfinite(shape.parameters[2]);
    case NKSIM_SHAPE_SPHERE:
        return std::isfinite(shape.parameters[0]) && shape.parameters[0] > 0.0;
    case NKSIM_SHAPE_CAPSULE:
    case NKSIM_SHAPE_CYLINDER:
        return std::isfinite(shape.parameters[0]) && std::isfinite(shape.parameters[1]) &&
            shape.parameters[0] > 0.0 && shape.parameters[1] > 0.0;
    case NKSIM_SHAPE_PLANE: {
        const double length = std::sqrt(shape.parameters[0] * shape.parameters[0] +
            shape.parameters[1] * shape.parameters[1] + shape.parameters[2] * shape.parameters[2]);
        return std::isfinite(length) && length > 0.0 && std::isfinite(shape.parameters[3]);
    }
    default:
        return false;
    }
}

// Nearest non-negative root of a t^2 + b t + c = 0 below `limit`.
double nearest_root(double a, double b, double c, double limit) {
    if (std::abs(a) < 1e-15) return limit;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) return limit;
    const double root = std::sqrt(discriminant);
    const double t0 = (-b - root) / (2.0 * a), t1 = (-b + root) / (2.0 * a);
    if (t0 >= 0.0 && t0 < limit) return t0;
    if (t1 >= 0.0 && t1 < limit) return t1;
    return limit;
}

double ray_sphere(const double o[3], const double d[3], const double centre[3], double radius,
                  double limit) {
    const double m[3] = {o[0] - centre[0], o[1] - centre[1], o[2] - centre[2]};
    const double c = m[0] * m[0] + m[1] * m[1] + m[2] * m[2] - radius * radius;
    if (c <= 0.0) return 0.0;
    const double b = 2.0 * (m[0] * d[0] + m[1] * d[1] + m[2] * d[2]);
    const double a = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    return nearest_root(a, b, c, limit);
}

// Ray against a shape in its local frame (capsules and cylinders lie along
// local Z; a plane bounds the solid half-space behind its normal).
double ray_shape(const nksim_shape_desc &shape, const double o[3], const double d[3],
                 double limit) {
    if (shape.type == NKSIM_SHAPE_PLANE) {
        const double *n = shape.parameters;
        const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double height = (n[0] * o[0] + n[1] * o[1] + n[2] * o[2]) / length - n[3];
        if (height <= 0.0) return 0.0;
        const double approach = (n[0] * d[0] + n[1] * d[1] + n[2] * d[2]) / length;
        if (approach >= 0.0) return limit;
        const double t = -height / approach;
        return t < limit ? t : limit;
    }
    if (shape.type == NKSIM_SHAPE_CYLINDER) {
        const double radius = shape.parameters[0], half = shape.parameters[1] * 0.5;
        if (o[0] * o[0] + o[1] * o[1] <= radius * radius && std::abs(o[2]) <= half)
            return 0.0;
        double nearest = limit;
        const double a = d[0] * d[0] + d[1] * d[1];
        const double t = nearest_root(a, 2.0 * (o[0] * d[0] + o[1] * d[1]),
                                      o[0] * o[0] + o[1] * o[1] - radius * radius, limit);
        if (t < limit && std::abs(o[2] + t * d[2]) <= half) nearest = t;
        for (const double end : {-half, half}) {
            if (std::abs(d[2]) < 1e-15) continue;
            const double cap = (end - o[2]) / d[2];
            const double x = o[0] + cap * d[0], y = o[1] + cap * d[1];
            if (cap >= 0.0 && cap < nearest && x * x + y * y <= radius * radius) nearest = cap;
        }
        return nearest;
    }
    if (shape.type == NKSIM_SHAPE_SPHERE) {
        const double centre[3]{};
        return ray_sphere(o, d, centre, shape.parameters[0], limit);
    }
    if (shape.type == NKSIM_SHAPE_CAPSULE) {
        const double radius = shape.parameters[0], half = shape.parameters[1] * 0.5;
        if (o[0] * o[0] + o[1] * o[1] <= radius * radius && std::abs(o[2]) <= half)
            return 0.0;
        double nearest = limit;
        const double a = d[0] * d[0] + d[1] * d[1];
        const double t = nearest_root(a, 2.0 * (o[0] * d[0] + o[1] * d[1]),
                                      o[0] * o[0] + o[1] * o[1] - radius * radius, limit);
        if (t < limit && std::abs(o[2] + t * d[2]) <= half) nearest = t;
        for (const double end : {-half, half}) {
            const double centre[3] = {0.0, 0.0, end};
            nearest = std::min(nearest, ray_sphere(o, d, centre, radius, nearest));
        }
        return nearest;
    }
    double near = 0.0, far = limit;
    for (int axis = 0; axis < 3; ++axis) {
        const double extent = shape.parameters[axis];
        if (std::abs(d[axis]) < 1e-12) {
            if (std::abs(o[axis]) > extent) return limit;
            continue;
        }
        double a = (-extent - o[axis]) / d[axis], b = (extent - o[axis]) / d[axis];
        if (a > b) std::swap(a, b);
        near = std::max(near, a);
        far = std::min(far, b);
        if (near > far) return limit;
    }
    return near;
}

std::uint64_t monotonic_now_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

struct Part {
    nkscene_node_id node{};
    nksim_shape shape = 0;
    nksim_body body = 0;
    nksim_shape_desc shape_desc{};
    nksim_pose initial{};
    /** The pose most recently written to the node. */
    nksim_pose pose{};
    /** The pose the previous tick left the body at; drives are measured from it. */
    nksim_pose tick_pose{};
};

struct Object {
    uint32_t motion_type = 0;
    Part part;
};

struct Keyframe {
    double time = 0.0;
    std::vector<nksim_pose> poses;
};

struct Actor {
    std::vector<Part> parts;
    std::deque<Keyframe> keyframes;
};

struct Frame {
    nksim_snapshot snapshot = 0;
    nksim_clock clock{};
    std::unordered_map<nksim_body, nksim_body_state> bodies;
    std::unordered_map<nksim_object, nksim_body> objects;
    std::unordered_map<nksim_actor, std::vector<nksim_body>> actors;
    ~Frame() {
        if (snapshot != 0) nksim_snapshot_destroy(snapshot);
    }
};

class Session {
public:
    Session(nkscene_scene scene, nksim_world world, const nksim_world_desc &desc)
        : scene_(scene), world_(world), fixed_timestep_(desc.fixed_timestep),
          period_(static_cast<std::int64_t>(desc.fixed_timestep * 1'000'000'000.0)) {
        std::copy_n(desc.gravity, 3, gravity_);
    }
    ~Session() { destroy(); }

    std::recursive_mutex mutex;

    nksim_result status(nksim_session_status &out) {
        std::lock_guard lock(mutex);
        out.running = running_ ? 1u : 0u;
        out.hosted = host_ != 0 ? 1u : 0u;
        out.sealed = sealed_ ? 1u : 0u;
        out.step_index = step_index_;
        out.simulation_time = simulation_time_;
        out.fixed_timestep = fixed_timestep_;
        std::copy_n(gravity_, 3, out.gravity);
        return NKSIM_OK;
    }

    nksim_result step(std::uint64_t owner_time_ns, nksim_clock *out_clock) {
        std::lock_guard lock(mutex);
        if (running_) return NKSIM_ERROR_INVALID_STATE;
        const auto result = tick(owner_time_ns, false);
        if (result == NKSIM_OK && out_clock) {
            out_clock->step_index = step_index_;
            out_clock->time = simulation_time_;
            out_clock->fixed_timestep = fixed_timestep_;
        }
        return result;
    }

    nksim_result start() {
        std::lock_guard lock(mutex);
        if (running_) return NKSIM_ERROR_INVALID_STATE;
        const auto hosted = ensure_host();
        if (hosted != NKSIM_OK) return hosted;
        sealed_ = true;
        {
            std::lock_guard state(state_mutex_);
            running_ = true;
            stopping_ = false;
        }
        worker_ = std::thread([this] { run(); });
        return NKSIM_OK;
    }

    nksim_result stop() {
        {
            std::lock_guard state(state_mutex_);
            if (running_) stopping_ = true;
        }
        if (worker_.joinable()) worker_.join();
        std::lock_guard lock(mutex);
        {
            std::lock_guard state(state_mutex_);
            running_ = false;
            stopping_ = false;
        }
        return release_host();
    }

    nksim_result reset() {
        std::lock_guard lock(mutex);
        if (running_) return NKSIM_ERROR_INVALID_STATE;
        auto result = release_host();
        if (result != NKSIM_OK) return result;
        if ((result = nksim_world_reset(world_)) != NKSIM_OK) return result;
        step_index_ = 0;
        simulation_time_ = 0.0;
        for (auto &[id, object] : objects_)
            if ((result = place(object.part, object.part.initial, object.motion_type)) != NKSIM_OK)
                return result;
        for (auto &[id, actor] : actors_) {
            actor.keyframes.clear();
            for (auto &part : actor.parts)
                if ((result = place(part, part.initial, NKSIM_MOTION_KINEMATIC)) != NKSIM_OK)
                    return result;
        }
        for (auto &[id, participant] : participants_)
            if (participant.reset && (result = participant.reset(participant.user)) != NKSIM_OK)
                return result;
        return NKSIM_OK;
    }

    nksim_result create_object(const nksim_object_desc &desc, nksim_object &out) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        if (desc.motion_type > NKSIM_MOTION_DYNAMIC || !valid_shape(desc.shape) ||
            (desc.shape.type == NKSIM_SHAPE_PLANE && desc.motion_type != NKSIM_MOTION_STATIC) ||
            desc.pose.struct_size < sizeof(desc.pose) || !valid_pose(desc.pose) ||
            (desc.motion_type == NKSIM_MOTION_DYNAMIC &&
             !(std::isfinite(desc.mass) && desc.mass > 0.0)))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        Object object;
        object.motion_type = desc.motion_type;
        const bool default_layers = desc.collision_layer == 0 && desc.collision_mask == 0;
        const auto result = create_part(object.part, desc.shape, desc.pose, desc.motion_type,
            desc.motion_type == NKSIM_MOTION_DYNAMIC ? desc.mass : 0.0,
            default_layers ? 1u : desc.collision_layer, default_layers ? 1u : desc.collision_mask);
        if (result != NKSIM_OK) return result;
        out = next_object_++;
        objects_.emplace(out, std::move(object));
        return NKSIM_OK;
    }

    nksim_result destroy_object(nksim_object id) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        const auto found = objects_.find(id);
        if (found == objects_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        destroy_part(found->second.part);
        objects_.erase(found);
        return NKSIM_OK;
    }

    nksim_result teleport_object(nksim_object id, const nksim_pose &pose) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        const auto found = objects_.find(id);
        if (found == objects_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        if (pose.struct_size < sizeof(pose) || !valid_pose(pose))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const auto result = place(found->second.part, pose, found->second.motion_type);
        if (result == NKSIM_OK) found->second.part.initial = pose;
        return result;
    }

    nksim_result drive_object(nksim_object id, const nksim_pose &pose) {
        std::lock_guard lock(mutex);
        const auto found = objects_.find(id);
        if (found == objects_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        if (found->second.motion_type != NKSIM_MOTION_KINEMATIC)
            return NKSIM_ERROR_INVALID_STATE;
        if (pose.struct_size < sizeof(pose) || !valid_pose(pose))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        Part *parts[] = {&found->second.part};
        return drive_parts(parts, &pose, 1);
    }

    nksim_result object_body(nksim_object id, nksim_body &out) {
        std::lock_guard lock(mutex);
        const auto found = objects_.find(id);
        if (found == objects_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        out = found->second.part.body;
        return NKSIM_OK;
    }

    nksim_result create_actor(const nksim_actor_part *parts, uint32_t count, nksim_actor &out) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        if (!parts || count == 0 || count > 256) return NKSIM_ERROR_INVALID_ARGUMENT;
        for (uint32_t index = 0; index < count; ++index)
            if (parts[index].struct_size < sizeof(nksim_actor_part) ||
                !valid_shape(parts[index].shape) || !valid_pose(parts[index].pose))
                return NKSIM_ERROR_INVALID_ARGUMENT;
        Actor actor;
        actor.parts.resize(count);
        auto result = nksim_world_begin_topology_update(world_);
        if (result != NKSIM_OK) return result;
        for (uint32_t index = 0; index < count && result == NKSIM_OK; ++index)
            result = create_part(actor.parts[index], parts[index].shape, parts[index].pose,
                                 NKSIM_MOTION_KINEMATIC, 0.0, 1u, 1u);
        const auto committed = nksim_world_end_topology_update(world_);
        if (result == NKSIM_OK) result = committed;
        if (result != NKSIM_OK) {
            for (auto &part : actor.parts) destroy_part(part);
            return result;
        }
        out = next_actor_++;
        actors_.emplace(out, std::move(actor));
        return NKSIM_OK;
    }

    nksim_result destroy_actor(nksim_actor id) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        const auto found = actors_.find(id);
        if (found == actors_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        for (auto &part : found->second.parts) destroy_part(part);
        actors_.erase(found);
        return NKSIM_OK;
    }

    nksim_result push_keyframe(nksim_actor id, double time, const nksim_pose *poses,
                               uint32_t count) {
        std::lock_guard lock(mutex);
        const auto found = actors_.find(id);
        if (found == actors_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        auto &actor = found->second;
        if (!std::isfinite(time) || !poses || count != actor.parts.size())
            return NKSIM_ERROR_INVALID_ARGUMENT;
        for (uint32_t index = 0; index < count; ++index)
            if (poses[index].struct_size < sizeof(nksim_pose) || !valid_pose(poses[index]))
                return NKSIM_ERROR_INVALID_ARGUMENT;
        while (!actor.keyframes.empty() && actor.keyframes.back().time >= time)
            actor.keyframes.pop_back();
        actor.keyframes.push_back({time, std::vector<nksim_pose>(poses, poses + count)});
        return NKSIM_OK;
    }

    nksim_result actor_body(nksim_actor id, uint32_t part, nksim_body &out) {
        std::lock_guard lock(mutex);
        const auto found = actors_.find(id);
        if (found == actors_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        if (part >= found->second.parts.size()) return NKSIM_ERROR_INVALID_ARGUMENT;
        out = found->second.parts[part].body;
        return NKSIM_OK;
    }

    nksim_result raycast(const double origin[3], const double direction[3], double limit,
                         double &out) {
        std::lock_guard lock(mutex);
        double nearest = limit;
        auto test = [&](const Part &part) -> nksim_result {
            nksim_body_state state{};
            state.struct_size = sizeof(state);
            const auto result = body_state(part.body, state);
            if (result != NKSIM_OK) return result;
            double delta[3], local_origin[3], local_direction[3];
            for (int axis = 0; axis < 3; ++axis) delta[axis] = origin[axis] - state.position[axis];
            rotate(state.rotation, delta, local_origin, true);
            rotate(state.rotation, direction, local_direction, true);
            nearest = std::min(nearest, ray_shape(part.shape_desc, local_origin,
                                                  local_direction, nearest));
            return NKSIM_OK;
        };
        for (const auto &[id, object] : objects_)
            if (const auto result = test(object.part); result != NKSIM_OK) return result;
        for (const auto &[id, actor] : actors_)
            for (const auto &part : actor.parts)
                if (const auto result = test(part); result != NKSIM_OK) return result;
        out = nearest;
        return NKSIM_OK;
    }

    nksim_result capture(std::shared_ptr<Frame> &out) {
        std::lock_guard lock(mutex);
        auto frame = std::make_shared<Frame>();
        const auto result = host_ != 0 ? nksim_host_get_snapshot(host_, &frame->snapshot)
                                       : nksim_world_snapshot(world_, &frame->snapshot);
        if (result != NKSIM_OK) return result;
        frame->clock.struct_size = sizeof(frame->clock);
        frame->clock.step_index = step_index_;
        frame->clock.time = simulation_time_;
        frame->clock.fixed_timestep = fixed_timestep_;
        const auto read = read_snapshot(frame->snapshot, frame->bodies);
        if (read != NKSIM_OK) return read;
        for (const auto &[id, object] : objects_) frame->objects.emplace(id, object.part.body);
        for (const auto &[id, actor] : actors_) {
            auto &bodies = frame->actors[id];
            for (const auto &part : actor.parts) bodies.push_back(part.body);
        }
        out = std::move(frame);
        return NKSIM_OK;
    }

    nksim_result add_participant(const nksim_participant_desc &desc, nksim_participant &out) {
        std::lock_guard lock(mutex);
        out = next_participant_++;
        participants_.emplace(out, desc);
        return NKSIM_OK;
    }

    nksim_result remove_participant(nksim_participant id) {
        std::lock_guard lock(mutex);
        return participants_.erase(id) != 0 ? NKSIM_OK : NKSIM_ERROR_INVALID_HANDLE;
    }

    nksim_result world(nksim_world &out) {
        std::lock_guard lock(mutex);
        if (host_ != 0) return NKSIM_ERROR_INVALID_STATE;
        out = world_;
        return NKSIM_OK;
    }

    nkscene_scene scene() const { return scene_; }

    nksim_snapshot latest_snapshot() {
        std::lock_guard lock(mutex);
        return snapshot_;
    }

    nksim_result body_state(nksim_body body, nksim_body_state &out) {
        std::lock_guard lock(mutex);
        if (host_ == 0) return nksim_body_get_state(world_, body, &out);
        const auto found = latest_.find(body);
        if (found == latest_.end()) return NKSIM_ERROR_INVALID_HANDLE;
        out = found->second;
        return NKSIM_OK;
    }

    nksim_result submit_joint_targets(const nksim_joint_target *targets, uint32_t count) {
        std::lock_guard lock(mutex);
        if (count == 0) return NKSIM_OK;
        return host_ != 0 ? nksim_host_submit_joint_targets(host_, targets, count)
                          : nksim_world_set_joint_targets(world_, targets, count);
    }

    nksim_result drive_bodies(const nksim_body_state *states, uint32_t count) {
        std::lock_guard lock(mutex);
        if (count == 0) return NKSIM_OK;
        if (host_ != 0) return nksim_host_submit_body_drives(host_, states, count);
        for (uint32_t index = 0; index < count; ++index)
            if (const auto result = nksim_body_drive(world_, states[index].body, &states[index]);
                result != NKSIM_OK)
                return result;
        return NKSIM_OK;
    }

    nksim_result set_body_states(const nksim_body_state *states, uint32_t count) {
        std::lock_guard lock(mutex);
        if (count == 0) return NKSIM_OK;
        if (host_ != 0) return nksim_host_submit_body_states(host_, states, count);
        for (uint32_t index = 0; index < count; ++index)
            if (const auto result = nksim_body_set_state(world_, states[index].body, &states[index]);
                result != NKSIM_OK)
                return result;
        return NKSIM_OK;
    }

private:
    nksim_result tick(std::uint64_t owner_time_ns, bool realtime) {
        const auto hosted = ensure_host();
        if (hosted != NKSIM_OK) return hosted;
        sealed_ = true;
        nksim_tick info{};
        info.struct_size = sizeof(info);
        info.step_index = step_index_ + 1;
        info.simulation_time = static_cast<double>(step_index_ + 1) * fixed_timestep_;
        info.fixed_timestep = fixed_timestep_;
        info.owner_time_ns = owner_time_ns;
        info.realtime = realtime ? 1u : 0u;
        // Iterate a copy so a callback may add or remove participants.
        const auto participants = participants_;
        for (const auto &[id, participant] : participants) {
            if (!participant.prepare) continue;
            const auto prepared = participant.prepare(participant.user, &info);
            if (prepared != NKSIM_OK) {
                for (const auto &[other, discarded] : participants)
                    if (discarded.discard) discarded.discard(discarded.user);
                return realtime ? NKSIM_OK : prepared;
            }
        }
        for (const auto &[id, participant] : participants)
            if (participant.submit)
                if (const auto result = participant.submit(participant.user, &info);
                    result != NKSIM_OK)
                    return result;
        auto result = advance_actors(info.simulation_time);
        if (result != NKSIM_OK) return result;
        nksim_step_result stepped{};
        stepped.struct_size = sizeof(stepped);
        if ((result = nksim_host_step(host_, &stepped)) != NKSIM_OK) return result;
        if (stepped.scene_changes != 0) nkscene_change_set_destroy(stepped.scene_changes);
        if ((result = refresh_snapshot()) != NKSIM_OK) return result;
        step_index_ = stepped.step_index;
        simulation_time_ = stepped.simulation_time;
        for (auto &[id, object] : objects_) object.part.tick_pose = object.part.pose;
        for (auto &[id, actor] : actors_)
            for (auto &part : actor.parts) part.tick_pose = part.pose;
        for (const auto &[id, participant] : participants)
            if (participant.publish)
                if ((result = participant.publish(participant.user, &info)) != NKSIM_OK)
                    return result;
        return NKSIM_OK;
    }

    void run() {
        auto next_tick = std::chrono::steady_clock::now();
        for (;;) {
            {
                std::lock_guard state(state_mutex_);
                if (stopping_) break;
            }
            next_tick += period_;
            {
                std::lock_guard lock(mutex);
                // A failed tick leaves the loop; participants record their
                // own faults and the owner stops the session.
                if (tick(monotonic_now_ns(), true) != NKSIM_OK) break;
            }
            std::this_thread::sleep_until(next_tick);
        }
    }

    // Moves every actor part to its trajectory pose at `time`.
    nksim_result advance_actors(double time) {
        for (auto &[id, actor] : actors_) {
            auto &keys = actor.keyframes;
            if (keys.empty()) continue;
            while (keys.size() >= 2 && keys[1].time <= time) keys.pop_front();
            std::vector<nksim_pose> poses = keys.front().poses;
            if (keys.size() >= 2 && time > keys[0].time) {
                const double t = (time - keys[0].time) / (keys[1].time - keys[0].time);
                for (std::size_t part = 0; part < poses.size(); ++part)
                    poses[part] = interpolate(keys[0].poses[part], keys[1].poses[part], t);
            }
            std::vector<Part *> parts;
            for (auto &part : actor.parts) parts.push_back(&part);
            const auto result = drive_parts(parts.data(), poses.data(),
                                            static_cast<uint32_t>(parts.size()));
            if (result != NKSIM_OK) return result;
        }
        return NKSIM_OK;
    }

    // Writes each part's node and drives its kinematic body continuously from
    // where the previous tick left it, in one scene transaction.
    nksim_result drive_parts(Part *const *parts, const nksim_pose *poses, uint32_t count) {
        nkscene_transaction transaction = 0;
        if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK) return NKSIM_ERROR_SCENE;
        std::vector<nksim_body_state> states(count);
        for (uint32_t index = 0; index < count; ++index) {
            const auto &pose = poses[index];
            const auto transform = scene_transform(pose);
            if (nkscene_tx_set_transform(transaction, parts[index]->node, &transform) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                return NKSIM_ERROR_SCENE;
            }
            auto &state = states[index];
            state.struct_size = sizeof(state);
            state.body = parts[index]->body;
            std::copy_n(pose.position, 3, state.position);
            std::copy_n(pose.rotation, 4, state.rotation);
            const auto &from = parts[index]->tick_pose;
            for (int axis = 0; axis < 3; ++axis)
                state.linear_velocity[axis] =
                    (pose.position[axis] - from.position[axis]) / fixed_timestep_;
            angular_velocity_between(from.rotation, pose.rotation, fixed_timestep_,
                                     state.angular_velocity);
        }
        nkscene_change_set changes = 0;
        if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
            return NKSIM_ERROR_SCENE;
        if (changes != 0) nkscene_change_set_destroy(changes);
        const auto result = drive_bodies(states.data(), count);
        if (result == NKSIM_OK)
            for (uint32_t index = 0; index < count; ++index) parts[index]->pose = poses[index];
        return result;
    }

    nksim_result create_part(Part &part, const nksim_shape_desc &shape, const nksim_pose &pose,
                             uint32_t motion_type, double mass, uint32_t layer, uint32_t mask) {
        part.shape_desc = shape;
        part.initial = part.pose = part.tick_pose = pose;
        nkscene_transaction transaction = 0;
        if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK) return NKSIM_ERROR_SCENE;
        const auto transform = scene_transform(pose);
        if (nkscene_tx_create_node(transaction, &part.node) != NKS_OK ||
            nkscene_tx_set_transform(transaction, part.node, &transform) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            return NKSIM_ERROR_SCENE;
        }
        nkscene_change_set changes = 0;
        if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
            return NKSIM_ERROR_SCENE;
        if (changes != 0) nkscene_change_set_destroy(changes);
        auto shape_desc = shape;
        shape_desc.struct_size = sizeof(shape_desc);
        auto result = nksim_shape_create(world_, &shape_desc, &part.shape);
        if (result != NKSIM_OK) {
            destroy_part(part);
            return result;
        }
        nksim_body_desc body{};
        body.struct_size = sizeof(body);
        body.node = part.node;
        body.motion_type = motion_type;
        body.mass = mass;
        body.shape = part.shape;
        body.collision_layer = layer;
        body.collision_mask = mask;
        if ((result = nksim_body_create(world_, &body, &part.body)) != NKSIM_OK) {
            destroy_part(part);
            return result;
        }
        return NKSIM_OK;
    }

    void destroy_part(Part &part) {
        if (part.body != 0) nksim_body_destroy(world_, part.body);
        if (part.shape != 0) nksim_shape_destroy(world_, part.shape);
        part.body = 0;
        part.shape = 0;
        if (part.node.value == 0) return;
        nkscene_transaction transaction = 0;
        if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK) return;
        if (nkscene_tx_destroy_node(transaction, part.node) != NKS_OK) {
            nkscene_transaction_cancel(transaction);
            return;
        }
        nkscene_change_set changes = 0;
        if (nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK &&
            changes != 0)
            nkscene_change_set_destroy(changes);
        part.node = {};
    }

    // Moves a part at rest while not hosted: its body, and a kinematic part's node.
    nksim_result place(Part &part, const nksim_pose &pose, uint32_t motion_type) {
        nksim_body_state state{};
        state.struct_size = sizeof(state);
        auto result = nksim_body_get_state(world_, part.body, &state);
        if (result != NKSIM_OK) return result;
        std::copy_n(pose.position, 3, state.position);
        std::copy_n(pose.rotation, 4, state.rotation);
        std::fill(std::begin(state.linear_velocity), std::end(state.linear_velocity), 0.0);
        std::fill(std::begin(state.angular_velocity), std::end(state.angular_velocity), 0.0);
        state.sleeping = 1;
        if ((result = nksim_body_set_state(world_, part.body, &state)) != NKSIM_OK) return result;
        if (motion_type == NKSIM_MOTION_KINEMATIC) {
            nkscene_transaction transaction = 0;
            if (nkscene_transaction_begin(scene_, &transaction) != NKS_OK) return NKSIM_ERROR_SCENE;
            const auto transform = scene_transform(pose);
            if (nkscene_tx_set_transform(transaction, part.node, &transform) != NKS_OK) {
                nkscene_transaction_cancel(transaction);
                return NKSIM_ERROR_SCENE;
            }
            nkscene_change_set changes = 0;
            if (nkscene_transaction_commit_with_changes(transaction, &changes) != NKS_OK)
                return NKSIM_ERROR_SCENE;
            if (changes != 0) nkscene_change_set_destroy(changes);
        }
        part.pose = part.tick_pose = pose;
        return NKSIM_OK;
    }

    static nksim_result read_snapshot(nksim_snapshot snapshot,
                                      std::unordered_map<nksim_body, nksim_body_state> &out) {
        out.clear();
        uint64_t count = 0;
        auto result = nksim_snapshot_get_body_count(snapshot, &count);
        if (result != NKSIM_OK) return result;
        out.reserve(static_cast<std::size_t>(count));
        for (uint64_t index = 0; index < count; ++index) {
            nksim_body_state state{};
            state.struct_size = sizeof(state);
            if ((result = nksim_snapshot_get_body(snapshot, index, &state)) != NKSIM_OK)
                return result;
            out.emplace(state.body, state);
        }
        return NKSIM_OK;
    }

    nksim_result refresh_snapshot() {
        if (snapshot_ != 0) nksim_snapshot_destroy(snapshot_);
        snapshot_ = 0;
        latest_.clear();
        const auto result = nksim_host_get_snapshot(host_, &snapshot_);
        return result != NKSIM_OK ? result : read_snapshot(snapshot_, latest_);
    }

    nksim_result ensure_host() {
        if (host_ != 0) return NKSIM_OK;
        nksim_host_desc desc{};
        desc.struct_size = sizeof(desc);
        desc.world = world_;
        desc.mode = NKSIM_HOST_MODE_EXTERNAL;
        auto result = nksim_host_create(&desc, &host_);
        if (result != NKSIM_OK) return result;
        if ((result = nksim_host_start(host_)) != NKSIM_OK) {
            nksim_host_destroy(host_);
            host_ = 0;
            return result;
        }
        // Readers use the host's snapshots from here on; direct world reads
        // would come from the wrong thread.
        if ((result = refresh_snapshot()) != NKSIM_OK) {
            release_host();
            return result;
        }
        return NKSIM_OK;
    }

    nksim_result release_host() {
        if (snapshot_ != 0) nksim_snapshot_destroy(snapshot_);
        snapshot_ = 0;
        latest_.clear();
        if (host_ == 0) return NKSIM_OK;
        const auto result = nksim_host_stop(host_);
        nksim_host_destroy(host_);
        host_ = 0;
        return result;
    }

    void destroy() {
        stop();
        std::lock_guard lock(mutex);
        participants_.clear();
        for (auto &[id, object] : objects_) destroy_part(object.part);
        for (auto &[id, actor] : actors_)
            for (auto &part : actor.parts) destroy_part(part);
        objects_.clear();
        actors_.clear();
    }

    nkscene_scene scene_;
    nksim_world world_;
    double fixed_timestep_;
    double gravity_[3]{};
    std::chrono::nanoseconds period_;
    nksim_host host_ = 0;
    nksim_snapshot snapshot_ = 0;
    std::unordered_map<nksim_body, nksim_body_state> latest_;
    std::uint64_t step_index_ = 0;
    double simulation_time_ = 0.0;
    bool sealed_ = false;
    std::mutex state_mutex_;
    bool running_ = false;
    bool stopping_ = false;
    std::thread worker_;
    std::map<nksim_object, Object> objects_;
    std::map<nksim_actor, Actor> actors_;
    // Ordered by registration, so every tick visits participants alike.
    std::map<nksim_participant, nksim_participant_desc> participants_;
    nksim_object next_object_ = 1;
    nksim_actor next_actor_ = 1;
    nksim_participant next_participant_ = 1;
};

template<class T>
class HandleMap {
public:
    uint32_t store(std::shared_ptr<T> value) {
        std::lock_guard lock(mutex_);
        while (next_ == 0 || values_.count(next_) != 0) ++next_;
        const auto handle = next_++;
        values_.emplace(handle, std::move(value));
        return handle;
    }
    std::shared_ptr<T> get(uint32_t handle) {
        std::lock_guard lock(mutex_);
        const auto found = values_.find(handle);
        return found == values_.end() ? nullptr : found->second;
    }
    std::shared_ptr<T> take(uint32_t handle) {
        std::lock_guard lock(mutex_);
        const auto found = values_.find(handle);
        if (found == values_.end()) return nullptr;
        auto value = std::move(found->second);
        values_.erase(found);
        return value;
    }

private:
    std::mutex mutex_;
    std::unordered_map<uint32_t, std::shared_ptr<T>> values_;
    uint32_t next_ = 1;
};

HandleMap<Session> &sessions() {
    static HandleMap<Session> value;
    return value;
}

HandleMap<Frame> &frames() {
    static HandleMap<Frame> value;
    return value;
}

} // namespace
} // namespace nksim

extern "C" {

nksim_result NKSIM_CALL nksim_session_create(const nksim_session_desc *desc,
                                             nksim_session *out_session) {
    if (!desc || !out_session || !nksim::valid_struct_size(desc->struct_size, sizeof(*desc)) ||
        desc->scene == 0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_session = 0;
    const auto world = nksim::resolve_world(desc->world);
    if (!world) return NKSIM_ERROR_INVALID_HANDLE;
    const auto handle = nksim::sessions().store(
        std::make_shared<nksim::Session>(desc->scene, desc->world, world->desc()));
    if (handle == 0) return NKSIM_ERROR_OUT_OF_MEMORY;
    *out_session = handle;
    return NKSIM_OK;
}

void NKSIM_CALL nksim_session_destroy(nksim_session session) {
    // Stopping first joins the owner loop, so the last reference is never
    // released on it; the session is freed when the last in-flight call returns.
    if (const auto value = nksim::sessions().take(session)) value->stop();
}

#define NKSIM_SESSION_OR_FAIL(name)                    \
    const auto name = nksim::sessions().get(session); \
    if (!name) return NKSIM_ERROR_INVALID_HANDLE

nksim_result NKSIM_CALL nksim_session_get_status(nksim_session session,
                                                 nksim_session_status *out_status) {
    if (!out_status || !nksim::valid_struct_size(out_status->struct_size, sizeof(*out_status)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->status(*out_status);
}

nksim_result NKSIM_CALL nksim_session_step(nksim_session session, uint64_t owner_time_ns,
                                           nksim_clock *out_clock) {
    if (out_clock && !nksim::valid_struct_size(out_clock->struct_size, sizeof(*out_clock)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->step(owner_time_ns, out_clock);
}

nksim_result NKSIM_CALL nksim_session_start(nksim_session session) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->start();
}

nksim_result NKSIM_CALL nksim_session_stop(nksim_session session) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->stop();
}

nksim_result NKSIM_CALL nksim_session_reset(nksim_session session) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->reset();
}

nksim_result NKSIM_CALL nksim_session_create_object(nksim_session session,
                                                    const nksim_object_desc *desc,
                                                    nksim_object *out_object) {
    if (!desc || !out_object || !nksim::valid_struct_size(desc->struct_size, sizeof(*desc)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_object = 0;
    NKSIM_SESSION_OR_FAIL(value);
    return value->create_object(*desc, *out_object);
}

nksim_result NKSIM_CALL nksim_session_destroy_object(nksim_session session, nksim_object object) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->destroy_object(object);
}

nksim_result NKSIM_CALL nksim_session_teleport_object(nksim_session session, nksim_object object,
                                                      const nksim_pose *pose) {
    if (!pose) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->teleport_object(object, *pose);
}

nksim_result NKSIM_CALL nksim_session_drive_object(nksim_session session, nksim_object object,
                                                   const nksim_pose *pose) {
    if (!pose) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->drive_object(object, *pose);
}

nksim_result NKSIM_CALL nksim_session_get_object_body(nksim_session session, nksim_object object,
                                                      nksim_body *out_body) {
    if (!out_body) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->object_body(object, *out_body);
}

nksim_result NKSIM_CALL nksim_session_create_actor(nksim_session session,
                                                   const nksim_actor_part *parts,
                                                   uint32_t part_count, nksim_actor *out_actor) {
    if (!out_actor) return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_actor = 0;
    NKSIM_SESSION_OR_FAIL(value);
    return value->create_actor(parts, part_count, *out_actor);
}

nksim_result NKSIM_CALL nksim_session_destroy_actor(nksim_session session, nksim_actor actor) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->destroy_actor(actor);
}

nksim_result NKSIM_CALL nksim_session_push_actor_keyframe(nksim_session session, nksim_actor actor,
                                                          double time, const nksim_pose *poses,
                                                          uint32_t pose_count) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->push_keyframe(actor, time, poses, pose_count);
}

nksim_result NKSIM_CALL nksim_session_get_actor_body(nksim_session session, nksim_actor actor,
                                                     uint32_t part, nksim_body *out_body) {
    if (!out_body) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->actor_body(actor, part, *out_body);
}

nksim_result NKSIM_CALL nksim_session_raycast(nksim_session session, const nksim_ray *ray,
                                              double *out_distance) {
    if (!ray || !out_distance || !nksim::valid_struct_size(ray->struct_size, sizeof(*ray)) ||
        !(ray->max_distance >= 0.0))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->raycast(ray->origin, ray->direction, ray->max_distance, *out_distance);
}

nksim_result NKSIM_CALL nksim_session_capture(nksim_session session, nksim_frame *out_frame) {
    if (!out_frame) return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_frame = 0;
    NKSIM_SESSION_OR_FAIL(value);
    std::shared_ptr<nksim::Frame> frame;
    const auto result = value->capture(frame);
    if (result != NKSIM_OK) return result;
    const auto handle = nksim::frames().store(std::move(frame));
    if (handle == 0) return NKSIM_ERROR_OUT_OF_MEMORY;
    *out_frame = handle;
    return NKSIM_OK;
}

void NKSIM_CALL nksim_frame_destroy(nksim_frame frame) {
    (void)nksim::frames().take(frame);
}

nksim_result NKSIM_CALL nksim_frame_get_clock(nksim_frame frame, nksim_clock *out_clock) {
    if (!out_clock || !nksim::valid_struct_size(out_clock->struct_size, sizeof(*out_clock)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::frames().get(frame);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    out_clock->step_index = value->clock.step_index;
    out_clock->time = value->clock.time;
    out_clock->fixed_timestep = value->clock.fixed_timestep;
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_frame_get_body_state(nksim_frame frame, nksim_body body,
                                                   nksim_body_state *out_state) {
    if (!out_state || !nksim::valid_struct_size(out_state->struct_size, sizeof(*out_state)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::frames().get(frame);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    const auto found = value->bodies.find(body);
    if (found == value->bodies.end()) return NKSIM_ERROR_INVALID_HANDLE;
    *out_state = found->second;
    return NKSIM_OK;
}

namespace {
nksim_result frame_body_pose(const nksim::Frame &frame, nksim_body body, nksim_pose &out) {
    if (!nksim::valid_struct_size(out.struct_size, sizeof(out)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto found = frame.bodies.find(body);
    if (found == frame.bodies.end()) return NKSIM_ERROR_INVALID_HANDLE;
    std::copy_n(found->second.position, 3, out.position);
    std::copy_n(found->second.rotation, 4, out.rotation);
    return NKSIM_OK;
}
} // namespace

nksim_result NKSIM_CALL nksim_frame_get_object_pose(nksim_frame frame, nksim_object object,
                                                    nksim_pose *out_pose) {
    if (!out_pose) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::frames().get(frame);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    const auto found = value->objects.find(object);
    if (found == value->objects.end()) return NKSIM_ERROR_INVALID_HANDLE;
    return frame_body_pose(*value, found->second, *out_pose);
}

nksim_result NKSIM_CALL nksim_frame_get_actor_pose(nksim_frame frame, nksim_actor actor,
                                                   uint32_t part, nksim_pose *out_pose) {
    if (!out_pose) return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::frames().get(frame);
    if (!value) return NKSIM_ERROR_INVALID_HANDLE;
    const auto found = value->actors.find(actor);
    if (found == value->actors.end()) return NKSIM_ERROR_INVALID_HANDLE;
    if (part >= found->second.size()) return NKSIM_ERROR_INVALID_ARGUMENT;
    return frame_body_pose(*value, found->second[part], *out_pose);
}

nksim_snapshot NKSIM_CALL nksim_frame_snapshot(nksim_frame frame) {
    const auto value = nksim::frames().get(frame);
    return value ? value->snapshot : 0;
}

nksim_result NKSIM_CALL nksim_session_add_participant(nksim_session session,
                                                      const nksim_participant_desc *desc,
                                                      nksim_participant *out_participant) {
    if (!desc || !out_participant || desc->struct_size < sizeof(*desc))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_participant = 0;
    NKSIM_SESSION_OR_FAIL(value);
    return value->add_participant(*desc, *out_participant);
}

nksim_result NKSIM_CALL nksim_session_remove_participant(nksim_session session,
                                                         nksim_participant participant) {
    NKSIM_SESSION_OR_FAIL(value);
    return value->remove_participant(participant);
}

void NKSIM_CALL nksim_session_lock(nksim_session session) {
    if (const auto value = nksim::sessions().get(session)) value->mutex.lock();
}

void NKSIM_CALL nksim_session_unlock(nksim_session session) {
    if (const auto value = nksim::sessions().get(session)) value->mutex.unlock();
}

nksim_result NKSIM_CALL nksim_session_get_world(nksim_session session, nksim_world *out_world) {
    if (!out_world) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->world(*out_world);
}

nksim_result NKSIM_CALL nksim_session_get_scene(nksim_session session, nkscene_scene *out_scene) {
    if (!out_scene) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    *out_scene = value->scene();
    return NKSIM_OK;
}

nksim_snapshot NKSIM_CALL nksim_session_latest_snapshot(nksim_session session) {
    const auto value = nksim::sessions().get(session);
    return value ? value->latest_snapshot() : 0;
}

nksim_result NKSIM_CALL nksim_session_get_body_state(nksim_session session, nksim_body body,
                                                     nksim_body_state *out_state) {
    if (!out_state) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->body_state(body, *out_state);
}

nksim_result NKSIM_CALL nksim_session_submit_joint_targets(nksim_session session,
                                                           const nksim_joint_target *targets,
                                                           uint32_t count) {
    if (count != 0 && !targets) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->submit_joint_targets(targets, count);
}

nksim_result NKSIM_CALL nksim_session_drive_bodies(nksim_session session,
                                                   const nksim_body_state *states, uint32_t count) {
    if (count != 0 && !states) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->drive_bodies(states, count);
}

nksim_result NKSIM_CALL nksim_session_set_body_states(nksim_session session,
                                                      const nksim_body_state *states,
                                                      uint32_t count) {
    if (count != 0 && !states) return NKSIM_ERROR_INVALID_ARGUMENT;
    NKSIM_SESSION_OR_FAIL(value);
    return value->set_body_states(states, count);
}

#undef NKSIM_SESSION_OR_FAIL

} // extern "C"
