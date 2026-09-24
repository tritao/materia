#pragma once

#include "nativekit_sim.h"

#include <array>
#include <cstdint>
#include <memory>

namespace nksim {

struct BackendBodyDesc {
    std::uint32_t motion_type = NKSIM_MOTION_STATIC;
    double mass = 0.0;
    std::uint32_t shape_type = 0;
    std::array<double, 4> shape_parameters{};
    std::array<double, 3> position{};
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};
    std::uint32_t collision_layer = 0;
    std::uint32_t collision_mask = 0;
    bool has_inertial_properties = false;
    std::array<double, 3> center_of_mass{};
    std::array<double, 9> inertia_tensor{};
};

struct BackendBodyState {
    std::uint64_t backend_body = 0;
    std::array<double, 3> position{};
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> linear_velocity{};
    std::array<double, 3> angular_velocity{};
    std::uint32_t sleeping = 0;
};

struct BackendBodyForce {
    std::uint64_t backend_body = 0;
    std::array<double, 3> force{};
    std::array<double, 3> torque{};
};

struct BackendJointDesc {
    std::uint32_t type = 0;
    std::uint64_t body_a = 0;
    std::uint64_t body_b = 0;
    std::array<double, 3> anchor_a{};
    std::array<double, 3> anchor_b{};
    std::array<double, 3> axis_a{};
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    double max_force = 0.0;
};

struct BackendJointState {
    std::uint64_t backend_joint = 0;
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
};

struct BackendJointTarget {
    std::uint64_t backend_joint = 0;
    std::uint32_t mode = 0;
    double target = 0.0;
    double max_force = 0.0;
};

/** Internal backend contract. It is intentionally not part of the C ABI. */
class PhysicsBackend {
public:
    virtual ~PhysicsBackend() = default;

    virtual nksim_result initialize(const nksim_world_desc &desc) = 0;

    virtual nksim_result body_create(const BackendBodyDesc &desc,
                                     std::uint64_t *out_body) = 0;
    virtual nksim_result body_destroy(std::uint64_t body) = 0;
    virtual nksim_result body_set_state(std::uint64_t body,
                                        const BackendBodyState &state) = 0;
    virtual nksim_result apply_forces(const BackendBodyForce *forces,
                                      std::uint32_t count) = 0;

    virtual nksim_result joint_create(const BackendJointDesc &desc,
                                      std::uint64_t *out_joint) = 0;
    virtual nksim_result joint_destroy(std::uint64_t joint) = 0;
    virtual nksim_result set_joint_targets(const BackendJointTarget *targets,
                                           std::uint32_t count) = 0;

    // Runtime owners can stage a connected topology and ask backends that
    // compile whole models (such as MuJoCo) to rebuild only once.
    virtual nksim_result begin_topology_update() { return NKSIM_OK; }
    virtual nksim_result end_topology_update() { return NKSIM_OK; }

    virtual nksim_result step(double dt, std::uint32_t substeps) = 0;
    virtual nksim_result read_body_states(BackendBodyState *states,
                                          std::uint32_t count) = 0;
    virtual nksim_result read_joint_states(BackendJointState *states,
                                           std::uint32_t count) = 0;
};

std::unique_ptr<PhysicsBackend> make_test_physics_backend();

} // namespace nksim
