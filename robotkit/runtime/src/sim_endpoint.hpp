#ifndef ROBOTKIT_SIM_ENDPOINT_HPP
#define ROBOTKIT_SIM_ENDPOINT_HPP

#include "robotkit_runtime.hpp"
#include "nativekit_sim.h"
#include "nativekit_sim_host.h"

#include <memory>
#include <vector>

namespace robotkit {

class SimRobotBinding;

class SimWorld final {
public:
    explicit SimWorld(double fixed_timestep = 0.01, uint32_t physics_substeps = 1);
    ~SimWorld();
    SimWorld(const SimWorld &) = delete;
    SimWorld &operator=(const SimWorld &) = delete;

    std::shared_ptr<SimRobotBinding> add_robot(const rk_runtime_blueprint &blueprint);
    rk_result step();
    rk_result start();
    rk_result stop();
    uint64_t step_index() const noexcept { return step_index_; }
    double simulation_time() const noexcept { return simulation_time_; }

private:
    friend class SimRobotBinding;
    void cleanup() noexcept;

    nkscene_scene scene_ = 0;
    nksim_world world_ = 0;
    nksim_shape shape_ = 0;
    nksim_host host_ = 0;
    nksim_snapshot snapshot_ = 0;
    double fixed_timestep_ = 0.01;
    uint32_t physics_substeps_ = 1;
    uint64_t step_index_ = 0;
    double simulation_time_ = 0.0;
    bool topology_frozen_ = false;
    std::vector<nkscene_occurrence_id> occurrences_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<std::weak_ptr<SimRobotBinding>> bindings_;
};

class SimRobotBinding final {
public:
    rk_result apply(const rk_robot_command &command);
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) const;
    void discard_pending() noexcept { pending_targets_.clear(); }

private:
    friend class SimWorld;
    explicit SimRobotBinding(SimWorld &world) : world_(world) {}
    std::vector<nksim_joint_target> take_pending_targets();

    SimWorld &world_;
    std::vector<nkscene_occurrence_id> occurrences_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<nksim_joint_target> pending_targets_;
    bool stopped_ = false;
};

/** Per-robot adapter; ownership and advancement remain in SimWorld. */
class SimEndpoint final : public Endpoint {
public:
    explicit SimEndpoint(std::shared_ptr<SimRobotBinding> binding)
        : binding_(std::move(binding)) {}
    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;
    void discard_pending() noexcept override { binding_->discard_pending(); }

private:
    std::shared_ptr<SimRobotBinding> binding_;
};

} // namespace robotkit
#endif
