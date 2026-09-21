#include "PhysicsBackend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace nksim {
namespace {

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
        body->state = state;
        body->state.backend_body = id;
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
        return joints.size() == before ? NKSIM_ERROR_INVALID_HANDLE : NKSIM_OK;
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
        }
        return NKSIM_OK;
    }

    nksim_result step(double dt, std::uint32_t substeps) override {
        if (!std::isfinite(dt) || dt <= 0.0 || substeps == 0)
            return NKSIM_ERROR_INVALID_ARGUMENT;
        const double substep = dt / static_cast<double>(substeps);
        for (std::uint32_t iteration = 0; iteration < substeps; ++iteration) {
            for (auto &body : bodies) {
                if (body.desc.motion_type != NKSIM_MOTION_DYNAMIC)
                    continue;
                const double inverse_mass = body.desc.mass > 0.0 ? 1.0 / body.desc.mass : 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    body.state.linear_velocity[axis] +=
                        (gravity[axis] + body.force[axis] * inverse_mass) * substep;
                    body.state.position[axis] += body.state.linear_velocity[axis] * substep;
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
    std::uint64_t next_body = 1;
    std::uint64_t next_joint = 1;
};

} // namespace

std::unique_ptr<PhysicsBackend> make_test_physics_backend() {
    return std::make_unique<TestPhysicsBackend>();
}

} // namespace nksim
