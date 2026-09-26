#include "internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>

namespace nksim {

namespace {

RuntimeRegistry runtime_registry;

bool valid_motion_type(std::uint32_t motion_type) noexcept {
    return motion_type == NKSIM_MOTION_STATIC || motion_type == NKSIM_MOTION_KINEMATIC ||
        motion_type == NKSIM_MOTION_DYNAMIC;
}

bool valid_inertial_properties(const nksim_body_desc &desc) noexcept {
    if (desc.has_inertial_properties > 1) return false;
    if (!desc.has_inertial_properties) return true;
    for (double value : desc.center_of_mass) if (!std::isfinite(value)) return false;
    const double *m = desc.inertia_tensor;
    for (double value : desc.inertia_tensor) if (!std::isfinite(value)) return false;
    const double scale = std::max({std::abs(m[0]), std::abs(m[1]), std::abs(m[2]),
        std::abs(m[3]), std::abs(m[4]), std::abs(m[5]), std::abs(m[6]),
        std::abs(m[7]), std::abs(m[8])});
    if (scale == 0.0) return false;
    const double epsilon = scale * 1e-10;
    return std::abs(m[1]-m[3]) <= epsilon && std::abs(m[2]-m[6]) <= epsilon &&
        std::abs(m[5]-m[7]) <= epsilon && m[0] > epsilon &&
        m[0]*m[4]-m[1]*m[3] > epsilon*epsilon &&
        m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+
            m[2]*(m[3]*m[7]-m[4]*m[6]) > epsilon*epsilon*epsilon;
}

bool valid_joint_type(std::uint32_t type) noexcept {
    return type == NKSIM_JOINT_FIXED || type == NKSIM_JOINT_REVOLUTE ||
        type == NKSIM_JOINT_PRISMATIC;
}

bool valid_shape_desc(const nksim_shape_desc &desc) noexcept {
    switch (desc.type) {
    case NKSIM_SHAPE_BOX:
        return std::isfinite(desc.parameters[0]) && std::isfinite(desc.parameters[1]) &&
            std::isfinite(desc.parameters[2]) && desc.parameters[0] > 0.0 &&
            desc.parameters[1] > 0.0 && desc.parameters[2] > 0.0;
    case NKSIM_SHAPE_SPHERE:
        return std::isfinite(desc.parameters[0]) && desc.parameters[0] > 0.0;
    case NKSIM_SHAPE_CAPSULE:
        return std::isfinite(desc.parameters[0]) && std::isfinite(desc.parameters[1]) &&
            desc.parameters[0] > 0.0 && desc.parameters[1] > 0.0;
    case NKSIM_SHAPE_PLANE: {
        const double length = std::sqrt(desc.parameters[0] * desc.parameters[0] +
                                         desc.parameters[1] * desc.parameters[1] +
                                         desc.parameters[2] * desc.parameters[2]);
        return std::isfinite(length) && length > 0.0 && std::isfinite(desc.parameters[3]);
    }
    default:
        return false;
    }
}

void normalize_quaternion(std::array<double, 4> &quaternion) noexcept {
    double length = 0.0;
    for (const double component : quaternion)
        length += component * component;
    if (length <= std::numeric_limits<double>::epsilon()) {
        quaternion = {0.0, 0.0, 0.0, 1.0};
        return;
    }
    const double inverse_length = 1.0 / std::sqrt(length);
    for (double &component : quaternion)
        component *= inverse_length;
}

std::array<double, 4> rotation_from_matrix(const float *matrix) noexcept {
    std::array<double, 4> result{};
    const double trace = static_cast<double>(matrix[0]) + matrix[5] + matrix[10];
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        result[3] = 0.25 * scale;
        result[0] = (static_cast<double>(matrix[6]) - matrix[9]) / scale;
        result[1] = (static_cast<double>(matrix[8]) - matrix[2]) / scale;
        result[2] = (static_cast<double>(matrix[1]) - matrix[4]) / scale;
    } else if (matrix[0] > matrix[5] && matrix[0] > matrix[10]) {
        const double scale = std::sqrt(1.0 + matrix[0] - matrix[5] - matrix[10]) * 2.0;
        result[3] = (static_cast<double>(matrix[6]) - matrix[9]) / scale;
        result[0] = 0.25 * scale;
        result[1] = (static_cast<double>(matrix[4]) + matrix[1]) / scale;
        result[2] = (static_cast<double>(matrix[8]) + matrix[2]) / scale;
    } else if (matrix[5] > matrix[10]) {
        const double scale = std::sqrt(1.0 + matrix[5] - matrix[0] - matrix[10]) * 2.0;
        result[3] = (static_cast<double>(matrix[8]) - matrix[2]) / scale;
        result[0] = (static_cast<double>(matrix[4]) + matrix[1]) / scale;
        result[1] = 0.25 * scale;
        result[2] = (static_cast<double>(matrix[9]) + matrix[6]) / scale;
    } else {
        const double scale = std::sqrt(1.0 + matrix[10] - matrix[0] - matrix[5]) * 2.0;
        result[3] = (static_cast<double>(matrix[1]) - matrix[4]) / scale;
        result[0] = (static_cast<double>(matrix[8]) + matrix[2]) / scale;
        result[1] = (static_cast<double>(matrix[9]) + matrix[6]) / scale;
        result[2] = 0.25 * scale;
    }
    normalize_quaternion(result);
    return result;
}

// World-frame angular velocity that rotates `from` onto `to` in `dt`, taking
// the shorter way around. The relative rotation is to * conjugate(from).
std::array<double, 3> angular_velocity_between(const double *from, const double *to,
                                               double dt) noexcept {
    const double ax = -from[0], ay = -from[1], az = -from[2], aw = from[3];
    const double bx = to[0], by = to[1], bz = to[2], bw = to[3];
    double x = bw * ax + bx * aw + by * az - bz * ay;
    double y = bw * ay - bx * az + by * aw + bz * ax;
    double z = bw * az + bx * ay - by * ax + bz * aw;
    double w = bw * aw - bx * ax - by * ay - bz * az;
    if (w < 0.0) {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    const double sine = std::sqrt(x * x + y * y + z * z);
    // angle = 2 atan2(|v|, w); the rotation vector is v * angle / |v|, which
    // tends to 2 v as the relative rotation vanishes.
    const double scale = sine > 1e-12 ? 2.0 * std::atan2(sine, w) / sine : 2.0;
    return {x * scale / dt, y * scale / dt, z * scale / dt};
}

} // namespace

RuntimeRegistry &registry() noexcept {
    return runtime_registry;
}

std::shared_ptr<World> resolve_world(nksim_world world) noexcept {
    std::lock_guard lock(runtime_registry.mutex);
    return runtime_registry.worlds.get(world);
}

std::shared_ptr<Snapshot> resolve_snapshot(nksim_snapshot snapshot) noexcept {
    std::lock_guard lock(runtime_registry.mutex);
    return runtime_registry.snapshots.get(snapshot);
}

bool valid_struct_size(std::uint32_t provided, std::size_t required) noexcept {
    return provided >= required;
}

bool finite_positive(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
}

World::World(const nksim_world_desc &desc, std::unique_ptr<PhysicsBackend> backend)
    : world_desc(desc), owner_thread(std::this_thread::get_id()), backend(std::move(backend)) {
    clock.fixed_timestep = desc.fixed_timestep;
}

// Backend destruction releases the complete world. Calling per-joint and
// per-body destroy here would repeatedly rebuild whole-model backends such as
// MuJoCo while the world itself is already being discarded.
World::~World() = default;

nksim_result World::initialize() {
    nkscene_snapshot scene_snapshot = 0;
    const auto scene_result = nkscene_scene_snapshot(world_desc.scene, &scene_snapshot);
    if (scene_result != NKS_OK)
        return NKSIM_ERROR_INVALID_HANDLE;
    nkscene_snapshot_destroy(scene_snapshot);
    return backend ? backend->initialize(world_desc) : NKSIM_ERROR_BACKEND;
}

nksim_result World::get_clock(nksim_clock *out_clock) const noexcept {
    if (!out_clock || !valid_struct_size(out_clock->struct_size, sizeof(*out_clock)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    clock.write(*out_clock);
    return NKSIM_OK;
}

nksim_result World::begin_topology_update() {
    if (!owns_thread()) return NKSIM_ERROR_WRONG_THREAD;
    if (topology_update_open || clock.step_index != 0)
        return NKSIM_ERROR_INVALID_STATE;
    const auto result = backend->begin_topology_update();
    if (result == NKSIM_OK)
        topology_update_open = true;
    return result;
}

nksim_result World::end_topology_update() {
    if (!owns_thread()) return NKSIM_ERROR_WRONG_THREAD;
    if (!topology_update_open)
        return NKSIM_ERROR_INVALID_STATE;
    topology_update_open = false;
    return backend->end_topology_update();
}

nksim_result World::node_pose(nkscene_node_id node,
                                    std::array<double, 3> &position,
                                    std::array<double, 4> &rotation) const {
    if (!node.value)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    nkscene_snapshot snapshot = 0;
    if (nkscene_scene_snapshot(world_desc.scene, &snapshot) != NKS_OK)
        return NKSIM_ERROR_INVALID_HANDLE;

    uint64_t count = 0;
    if (nkscene_snapshot_get_node_count(snapshot, &count) != NKS_OK) {
        nkscene_snapshot_destroy(snapshot);
        return NKSIM_ERROR_SCENE;
    }
    nkscene_snapshot_node value{};
    value.struct_size = sizeof(value);
    nksim_result result = NKSIM_ERROR_STALE_ID;
    for (uint64_t index = 0; index < count; ++index) {
        if (nkscene_snapshot_get_node(snapshot, index, &value) != NKS_OK)
            continue;
        if (value.node.value != node.value)
            continue;
        position = {value.world_transform.matrix[12], value.world_transform.matrix[13],
                    value.world_transform.matrix[14]};
        rotation = rotation_from_matrix(value.world_transform.matrix);
        result = NKSIM_OK;
        break;
    }
    nkscene_snapshot_destroy(snapshot);
    return result;
}

void World::initialize_body_state(Body &body, const std::array<double, 3> &position,
                                  const std::array<double, 4> &rotation) noexcept {
    body.state = {};
    body.state.struct_size = sizeof(body.state);
    body.state.body = body.handle;
    body.state.node = body.desc.node;
    std::copy(position.begin(), position.end(), std::begin(body.state.position));
    std::copy(rotation.begin(), rotation.end(), std::begin(body.state.rotation));
    body.state.rotation[3] = rotation[3];
}

nksim_result World::set_backend_body_state(const Body &body) {
    return set_backend_body_state(body.backend_body, body.state);
}

nksim_result World::set_backend_body_state(std::uint64_t backend_body,
                                           const nksim_body_state &source) {
    BackendBodyState state{};
    state.backend_body = backend_body;
    std::copy(std::begin(source.position), std::end(source.position), state.position.begin());
    std::copy(std::begin(source.rotation), std::end(source.rotation), state.rotation.begin());
    std::copy(std::begin(source.linear_velocity), std::end(source.linear_velocity),
              state.linear_velocity.begin());
    std::copy(std::begin(source.angular_velocity), std::end(source.angular_velocity),
              state.angular_velocity.begin());
    state.sleeping = source.sleeping;
    return backend->body_set_state(backend_body, state);
}

// Kinematic bodies follow their scene node. Backends pin a kinematic body
// where it is placed (it has no degrees of freedom), so it is placed at the
// node pose before the step and articulations it carries are posed from there.
// A node that moved continuously since the previous step gives the body the
// finite-difference twist over the step. The first step after an explicit
// state write or reset is discontinuous: the body keeps the twist that write
// supplied, so a teleport never reads as a velocity.
nksim_result World::refresh_kinematic_bodies() {
    nksim_result result = NKSIM_OK;
    const double dt = clock.fixed_timestep;
    bodies.for_each([&](nksim_body, Body &body) {
        if (result != NKSIM_OK || body.desc.motion_type != NKSIM_MOTION_KINEMATIC)
            return;
        std::array<double, 3> position{};
        std::array<double, 4> rotation{};
        result = node_pose(body.desc.node, position, rotation);
        if (result != NKSIM_OK)
            return;
        auto target = body.state;
        target.struct_size = sizeof(target);
        target.body = body.handle;
        target.node = body.desc.node;
        std::copy(position.begin(), position.end(), std::begin(target.position));
        std::copy(rotation.begin(), rotation.end(), std::begin(target.rotation));
        target.sleeping = 0;
        if (body.kinematic_continuous) {
            for (int axis = 0; axis < 3; ++axis)
                target.linear_velocity[axis] =
                    (position[axis] - body.state.position[axis]) / dt;
            const auto angular = angular_velocity_between(body.state.rotation,
                                                          target.rotation, dt);
            std::copy(angular.begin(), angular.end(), std::begin(target.angular_velocity));
        }
        body.kinematic_target = target;
        result = set_backend_body_state(body.backend_body, target);
    });
    return result;
}

// A kinematic body's authoritative state is the pose it was driven to and the
// twist that took it there; backends that pin it without degrees of freedom
// report no velocity for it.
void World::commit_kinematic_targets() noexcept {
    bodies.for_each([&](nksim_body, Body &body) {
        if (body.desc.motion_type != NKSIM_MOTION_KINEMATIC)
            return;
        body.state = body.kinematic_target;
        body.kinematic_continuous = true;
    });
}

nksim_result World::read_backend_state() {
    std::vector<BackendBodyState> body_states;
    body_states.reserve(16);
    bodies.for_each([&](nksim_body, const Body &body) {
        BackendBodyState state{};
        state.backend_body = body.backend_body;
        body_states.push_back(state);
    });
    const auto body_result = backend->read_body_states(
        body_states.empty() ? nullptr : body_states.data(),
        static_cast<std::uint32_t>(body_states.size()));
    if (body_result != NKSIM_OK)
        return body_result;
    for (const auto &state : body_states) {
        bodies.for_each([&](nksim_body, Body &body) {
            if (body.backend_body != state.backend_body)
                return;
            body.state.struct_size = sizeof(body.state);
            body.state.body = body.handle;
            body.state.node = body.desc.node;
            std::copy(std::begin(state.position), std::end(state.position),
                      std::begin(body.state.position));
            std::copy(std::begin(state.rotation), std::end(state.rotation),
                      std::begin(body.state.rotation));
            body.state.sleeping = state.sleeping;
            // A kinematic body's twist is prescribed, not simulated: keep the
            // one it was given (backends pinning it report none).
            if (body.desc.motion_type == NKSIM_MOTION_KINEMATIC)
                return;
            std::copy(std::begin(state.linear_velocity), std::end(state.linear_velocity),
                      std::begin(body.state.linear_velocity));
            std::copy(std::begin(state.angular_velocity), std::end(state.angular_velocity),
                      std::begin(body.state.angular_velocity));
        });
    }

    if (joints.empty())
        return NKSIM_OK;
    std::vector<BackendJointState> joint_states;
    joint_states.reserve(16);
    joints.for_each([&](nksim_joint, const Joint &joint) {
        BackendJointState state{};
        state.backend_joint = joint.backend_joint;
        joint_states.push_back(state);
    });
    const auto joint_result = backend->read_joint_states(
        joint_states.data(), static_cast<std::uint32_t>(joint_states.size()));
    if (joint_result != NKSIM_OK)
        return joint_result;
    for (const auto &state : joint_states) {
        joints.for_each([&](nksim_joint, Joint &joint) {
            if (joint.backend_joint != state.backend_joint)
                return;
            joint.state.struct_size = sizeof(joint.state);
            joint.state.joint = joint.handle;
            joint.state.position = state.position;
            joint.state.velocity = state.velocity;
            joint.state.effort = state.effort;
        });
    }
    return NKSIM_OK;
}

nksim_result World::step(nksim_step_result *out_result) {
    if (!out_result || !valid_struct_size(out_result->struct_size, sizeof(*out_result)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    out_result->scene_changes = 0;
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    if (topology_update_open)
        return NKSIM_ERROR_INVALID_STATE;
    auto result = refresh_kinematic_bodies();
    if (result != NKSIM_OK)
        return result;
    result = backend->step(clock.fixed_timestep, world_desc.physics_substeps);
    if (result != NKSIM_OK)
        return result;
    result = read_backend_state();
    if (result != NKSIM_OK)
        return result;
    commit_kinematic_targets();
    nkscene_change_set changes = 0;
    result = synchronize_scene(&changes);
    if (result != NKSIM_OK)
        return result;
    clock.advance();
    out_result->step_index = clock.step_index;
    out_result->simulation_time = clock.time;
    out_result->physics_substeps = world_desc.physics_substeps;
    out_result->scene_changes = changes;
    return NKSIM_OK;
}

nksim_result World::apply_forces(const nksim_body_force *forces, std::uint32_t count) {
    if (!owns_thread() || (count != 0 && !forces))
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    std::vector<BackendBodyForce> backend_forces;
    backend_forces.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index) {
        if (!valid_struct_size(forces[index].struct_size, sizeof(forces[index])))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const auto *body = bodies.get(forces[index].body);
        if (!body)
            return NKSIM_ERROR_INVALID_HANDLE;
        BackendBodyForce force{};
        force.backend_body = body->backend_body;
        std::copy(std::begin(forces[index].force), std::end(forces[index].force), force.force.begin());
        std::copy(std::begin(forces[index].torque), std::end(forces[index].torque), force.torque.begin());
        backend_forces.push_back(force);
    }
    return backend->apply_forces(backend_forces.empty() ? nullptr : backend_forces.data(),
                                 static_cast<std::uint32_t>(backend_forces.size()));
}

nksim_result World::set_joint_targets(const nksim_joint_target *targets, std::uint32_t count) {
    if (!owns_thread() || (count != 0 && !targets))
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    std::vector<BackendJointTarget> backend_targets;
    backend_targets.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index) {
        if (!valid_struct_size(targets[index].struct_size, sizeof(targets[index])))
            return NKSIM_ERROR_INVALID_ARGUMENT;
        if (targets[index].mode < NKSIM_JOINT_TARGET_POSITION ||
            targets[index].mode > NKSIM_JOINT_TARGET_EFFORT)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const auto *joint = joints.get(targets[index].joint);
        if (!joint)
            return NKSIM_ERROR_INVALID_HANDLE;
        BackendJointTarget target{};
        target.backend_joint = joint->backend_joint;
        target.mode = targets[index].mode;
        target.target = targets[index].target;
        target.max_force = targets[index].max_force;
        backend_targets.push_back(target);
    }
    const auto result = backend->set_joint_targets(
        backend_targets.empty() ? nullptr : backend_targets.data(),
        static_cast<std::uint32_t>(backend_targets.size()));
    if (result != NKSIM_OK)
        return result;
    // F4: an instant position-mode target can move a body kinematically in
    // the backend right away; pull that back so an immediate
    // get_body_state/get_joint_state observes it, not just the next step().
    return read_backend_state();
}

nksim_result World::snapshot(std::shared_ptr<Snapshot> &out_snapshot) const {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    auto result = std::make_shared<Snapshot>();
    result->clock = clock;
    bodies.for_each([&](nksim_body, const Body &body) { result->bodies.push_back(body.state); });
    joints.for_each([&](nksim_joint, const Joint &joint) { result->joints.push_back(joint.state); });
    out_snapshot = std::move(result);
    return NKSIM_OK;
}

nksim_result World::create_shape(const nksim_shape_desc &desc, nksim_shape *out_shape) {
    if (!owns_thread() || !out_shape)
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    if (!valid_struct_size(desc.struct_size, sizeof(desc)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!valid_shape_desc(desc))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    auto [handle, shape] = shapes.create();
    if (!shape)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    shape->handle = handle;
    shape->desc = desc;
    *out_shape = handle;
    return NKSIM_OK;
}

nksim_result World::destroy_shape(nksim_shape shape) {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    if (!shapes.get(shape))
        return NKSIM_ERROR_INVALID_HANDLE;
    bool in_use = false;
    bodies.for_each([&](nksim_body, const Body &body) { in_use |= body.desc.shape == shape; });
    if (in_use)
        return NKSIM_ERROR_INVALID_STATE;
    shapes.remove(shape);
    return NKSIM_OK;
}

nksim_result World::create_body(const nksim_body_desc &desc, nksim_body *out_body) {
    if (!owns_thread() || !out_body)
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    if (!valid_struct_size(desc.struct_size, sizeof(desc)) || !valid_motion_type(desc.motion_type) ||
        !valid_inertial_properties(desc))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (desc.motion_type == NKSIM_MOTION_DYNAMIC && !finite_positive(desc.mass))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const Shape *shape = nullptr;
    if (desc.shape) {
        shape = shapes.get(desc.shape);
        if (!shape)
            return NKSIM_ERROR_INVALID_HANDLE;
    }
    std::array<double, 3> position{};
    std::array<double, 4> rotation{};
    auto result = node_pose(desc.node, position, rotation);
    if (result != NKSIM_OK)
        return result;

    auto [handle, body] = bodies.create();
    if (!body)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    body->handle = handle;
    body->desc = desc;
    initialize_body_state(*body, position, rotation);
    BackendBodyDesc backend_desc{};
    backend_desc.motion_type = desc.motion_type;
    backend_desc.mass = desc.mass;
    backend_desc.position = position;
    backend_desc.rotation = rotation;
    backend_desc.collision_layer = desc.collision_layer;
    backend_desc.collision_mask = desc.collision_mask;
    backend_desc.has_inertial_properties = desc.has_inertial_properties != 0;
    std::copy_n(desc.center_of_mass, 3, backend_desc.center_of_mass.begin());
    std::copy_n(desc.inertia_tensor, 9, backend_desc.inertia_tensor.begin());
    if (shape) {
        backend_desc.shape_type = shape->desc.type;
        std::copy(std::begin(shape->desc.parameters), std::end(shape->desc.parameters),
                  backend_desc.shape_parameters.begin());
    }
    std::uint64_t backend_body = 0;
    result = backend->body_create(backend_desc, &backend_body);
    if (result != NKSIM_OK) {
        bodies.remove(handle);
        return result;
    }
    body->backend_body = backend_body;
    body->initial_state = body->state;
    *out_body = handle;
    return NKSIM_OK;
}

nksim_result World::destroy_body(nksim_body body) {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    if (!bodies.get(body))
        return NKSIM_ERROR_INVALID_HANDLE;
    bool in_use = false;
    joints.for_each([&](nksim_joint, const Joint &joint) {
        in_use |= joint.desc.body_a == body || joint.desc.body_b == body;
    });
    if (in_use)
        return NKSIM_ERROR_INVALID_STATE;
    const auto *value = bodies.get(body);
    const auto result = backend->body_destroy(value->backend_body);
    if (result != NKSIM_OK)
        return result;
    bodies.remove(body);
    return NKSIM_OK;
}

nksim_result World::get_body_state(nksim_body body, nksim_body_state *out_state) const {
    if (!out_state || !valid_struct_size(out_state->struct_size, sizeof(*out_state)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    const auto *value = bodies.get(body);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    *out_state = value->state;
    return NKSIM_OK;
}

nksim_result World::set_body_state(nksim_body body, const nksim_body_state &state) {
    if (!owns_thread() || !valid_struct_size(state.struct_size, sizeof(state)))
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    auto *value = bodies.get(body);
    if (!value || state.body != body)
        return NKSIM_ERROR_INVALID_HANDLE;
    const auto previous = value->state;
    value->state = state;
    value->state.struct_size = sizeof(value->state);
    value->state.body = body;
    value->state.node = value->desc.node;
    const auto result = set_backend_body_state(*value);
    if (result != NKSIM_OK) { value->state = previous; return result; }
    value->kinematic_continuous = false;
    // F4: a root's new pose (or a reset child's zeroed joint) can move other
    // bodies kinematically in the backend; pull every body/joint's resulting
    // state back so an immediate get_body_state/get_joint_state observes it,
    // not just the next step().
    return read_backend_state();
}

nksim_result World::reset_body(nksim_body body) {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    auto *value = bodies.get(body);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    const auto previous = value->state;
    value->state = value->initial_state;
    const auto result = set_backend_body_state(*value);
    if (result != NKSIM_OK) { value->state = previous; return result; }
    value->kinematic_continuous = false;
    return read_backend_state();
}

nksim_result World::reset() {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    nksim_result result = NKSIM_OK;
    bodies.for_each([&](nksim_body, Body &body) {
        if (result != NKSIM_OK)
            return;
        body.state = body.initial_state;
        body.kinematic_continuous = false;
        result = set_backend_body_state(body);
    });
    if (result != NKSIM_OK)
        return result;
    joints.for_each([&](nksim_joint, Joint &joint) {
        joint.state = {};
        joint.state.struct_size = sizeof(joint.state);
        joint.state.joint = joint.handle;
    });
    clock.time = 0.0;
    clock.step_index = 0;
    // F4: re-derive every body's pose from the now-reset backend joint/body
    // state (a joint-connected body's rest pose may depend on another
    // body's, so this is not simply each body's own initial_state again).
    return read_backend_state();
}

namespace {
// Bytes a caller must provide for the fields that predate rotation_a/rotation_b;
// a struct_size at least this large is accepted, but rotation_a/rotation_b are
// only read (never touched, so never over-read) when struct_size covers the
// full current struct. This is the "old prefixes remain valid" convention
// nk_init_options documents.
constexpr std::size_t joint_desc_legacy_size = offsetof(nksim_joint_desc, rotation_a);
} // namespace

nksim_result World::create_joint(const nksim_joint_desc &desc, nksim_joint *out_joint) {
    if (!owns_thread() || !out_joint)
        return !owns_thread() ? NKSIM_ERROR_WRONG_THREAD : NKSIM_ERROR_INVALID_ARGUMENT;
    if (!valid_struct_size(desc.struct_size, joint_desc_legacy_size) || !valid_joint_type(desc.type))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto *body_a = bodies.get(desc.body_a);
    const auto *body_b = bodies.get(desc.body_b);
    if (!body_a || !body_b)
        return NKSIM_ERROR_INVALID_HANDLE;
    auto [handle, joint] = joints.create();
    if (!joint)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    joint->handle = handle;
    joint->desc = {};
    joint->desc.struct_size = sizeof(joint->desc);
    joint->desc.type = desc.type;
    joint->desc.body_a = desc.body_a;
    joint->desc.body_b = desc.body_b;
    std::copy(std::begin(desc.anchor_a), std::end(desc.anchor_a), std::begin(joint->desc.anchor_a));
    std::copy(std::begin(desc.anchor_b), std::end(desc.anchor_b), std::begin(joint->desc.anchor_b));
    std::copy(std::begin(desc.axis_a), std::end(desc.axis_a), std::begin(joint->desc.axis_a));
    joint->desc.lower_limit = desc.lower_limit;
    joint->desc.upper_limit = desc.upper_limit;
    joint->desc.max_force = desc.max_force;
    joint->desc.rotation_a[3] = 1.0; // identity default for legacy-sized callers
    joint->desc.rotation_b[3] = 1.0;
    if (desc.struct_size >= sizeof(nksim_joint_desc)) {
        std::copy(std::begin(desc.rotation_a), std::end(desc.rotation_a), std::begin(joint->desc.rotation_a));
        std::copy(std::begin(desc.rotation_b), std::end(desc.rotation_b), std::begin(joint->desc.rotation_b));
    }
    // A zero-initialized struct (a common `Type{}` idiom, e.g. in tests
    // predating these fields) reads back an unnormalizable rotation, not a
    // deliberately invalid one; treat that exactly like a legacy-sized
    // caller and default to identity, matching normalize_quaternion's own
    // zero-length fallback elsewhere in this file.
    std::array<double, 4> normalized_rotation_a{
        joint->desc.rotation_a[0], joint->desc.rotation_a[1],
        joint->desc.rotation_a[2], joint->desc.rotation_a[3]};
    std::array<double, 4> normalized_rotation_b{
        joint->desc.rotation_b[0], joint->desc.rotation_b[1],
        joint->desc.rotation_b[2], joint->desc.rotation_b[3]};
    normalize_quaternion(normalized_rotation_a);
    normalize_quaternion(normalized_rotation_b);
    std::copy(normalized_rotation_a.begin(), normalized_rotation_a.end(), std::begin(joint->desc.rotation_a));
    std::copy(normalized_rotation_b.begin(), normalized_rotation_b.end(), std::begin(joint->desc.rotation_b));
    joint->state = {};
    joint->state.struct_size = sizeof(joint->state);
    joint->state.joint = handle;
    BackendJointDesc backend_desc{};
    backend_desc.type = joint->desc.type;
    backend_desc.body_a = body_a->backend_body;
    backend_desc.body_b = body_b->backend_body;
    std::copy(std::begin(joint->desc.anchor_a), std::end(joint->desc.anchor_a), backend_desc.anchor_a.begin());
    std::copy(std::begin(joint->desc.anchor_b), std::end(joint->desc.anchor_b), backend_desc.anchor_b.begin());
    std::copy(std::begin(joint->desc.axis_a), std::end(joint->desc.axis_a), backend_desc.axis_a.begin());
    backend_desc.lower_limit = joint->desc.lower_limit;
    backend_desc.upper_limit = joint->desc.upper_limit;
    backend_desc.max_force = joint->desc.max_force;
    std::copy(std::begin(joint->desc.rotation_a), std::end(joint->desc.rotation_a), backend_desc.rotation_a.begin());
    std::copy(std::begin(joint->desc.rotation_b), std::end(joint->desc.rotation_b), backend_desc.rotation_b.begin());
    std::uint64_t backend_joint = 0;
    const auto result = backend->joint_create(backend_desc, &backend_joint);
    if (result != NKSIM_OK) {
        joints.remove(handle);
        return result;
    }
    joint->backend_joint = backend_joint;
    *out_joint = handle;
    return NKSIM_OK;
}

nksim_result World::destroy_joint(nksim_joint joint) {
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    const auto *value = joints.get(joint);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    const auto result = backend->joint_destroy(value->backend_joint);
    if (result != NKSIM_OK)
        return result;
    joints.remove(joint);
    return NKSIM_OK;
}

nksim_result World::get_joint_state(nksim_joint joint, nksim_joint_state *out_state) const {
    if (!out_state || !valid_struct_size(out_state->struct_size, sizeof(*out_state)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!owns_thread())
        return NKSIM_ERROR_WRONG_THREAD;
    const auto *value = joints.get(joint);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    *out_state = value->state;
    return NKSIM_OK;
}

} // namespace nksim

namespace nksim {

nksim_result create_world_with_backend(const nksim_world_desc &desc,
                                       std::unique_ptr<PhysicsBackend> backend,
                                       nksim_world *out_world) {
    if (!out_world || !valid_struct_size(desc.struct_size, sizeof(desc)) ||
        !finite_positive(desc.fixed_timestep) || desc.physics_substeps == 0 ||
        !std::isfinite(desc.gravity[0]) || !std::isfinite(desc.gravity[1]) ||
        !std::isfinite(desc.gravity[2]))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!backend)
        return NKSIM_ERROR_BACKEND;
    *out_world = 0;
    auto world = std::make_shared<World>(desc, std::move(backend));
    const auto init_result = world->initialize();
    if (init_result != NKSIM_OK)
        return init_result;
    auto &state = registry();
    std::lock_guard lock(state.mutex);
    const auto handle = state.worlds.create(std::move(world));
    if (!handle)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    *out_world = handle;
    return NKSIM_OK;
}

} // namespace nksim

extern "C" {

nksim_result NKSIM_CALL nksim_world_create(const nksim_world_desc *desc,
                                           nksim_world *out_world) {
    if (!desc || !out_world)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (!nksim::valid_struct_size(desc->struct_size, sizeof(*desc)) ||
        !nksim::finite_positive(desc->fixed_timestep) || desc->physics_substeps == 0 ||
        !std::isfinite(desc->gravity[0]) || !std::isfinite(desc->gravity[1]) ||
        !std::isfinite(desc->gravity[2]))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    return nksim::create_world_with_backend(
        *desc, nksim::make_test_physics_backend(), out_world);
}

void NKSIM_CALL nksim_world_destroy(nksim_world world) {
    auto &state = nksim::registry();
    std::lock_guard lock(state.mutex);
    state.worlds.remove(world);
}

nksim_result NKSIM_CALL nksim_world_step(nksim_world world, nksim_step_result *out_result) {
    const auto value = nksim::resolve_world(world);
    return value ? value->step(out_result) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_begin_topology_update(nksim_world world) {
    const auto value = nksim::resolve_world(world);
    return value ? value->begin_topology_update() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_end_topology_update(nksim_world world) {
    const auto value = nksim::resolve_world(world);
    return value ? value->end_topology_update() : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_apply_forces(nksim_world world,
                                                 const nksim_body_force *forces,
                                                 uint32_t count) {
    const auto value = nksim::resolve_world(world);
    return value ? value->apply_forces(forces, count) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_set_joint_targets(nksim_world world,
                                                      const nksim_joint_target *targets,
                                                      uint32_t count) {
    const auto value = nksim::resolve_world(world);
    return value ? value->set_joint_targets(targets, count) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_world_snapshot(nksim_world world, nksim_snapshot *out_snapshot) {
    if (!out_snapshot)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_world(world);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    std::shared_ptr<nksim::Snapshot> snapshot;
    const auto result = value->snapshot(snapshot);
    if (result != NKSIM_OK)
        return result;
    auto &state = nksim::registry();
    std::lock_guard lock(state.mutex);
    const auto handle = state.snapshots.create(std::move(snapshot));
    if (!handle)
        return NKSIM_ERROR_OUT_OF_MEMORY;
    *out_snapshot = handle;
    return NKSIM_OK;
}

} // extern "C"
