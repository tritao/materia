#ifndef ROBOTKIT_SIMULATION_HPP
#define ROBOTKIT_SIMULATION_HPP

#include "runtime_registry.hpp"
#include "sim_endpoint.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace robotkit {

class Simulation final : public internal::RuntimeCoordinator,
                         public std::enable_shared_from_this<Simulation> {
public:
    Simulation(double fixed_timestep, uint32_t physics_substeps);
    ~Simulation() override;

    rk_result add_robot(const rk_runtime_blueprint &blueprint, rk_runtime &out_runtime);
    rk_result start() override;
    rk_result stop() override;
    rk_result step(uint64_t timestamp_ns) override;
    uint64_t step_index() const;
    double simulation_time() const;

private:
    void run();

    std::shared_ptr<SimWorld> world_;
    std::vector<std::shared_ptr<Runtime>> runtimes_;
    std::vector<rk_runtime> handles_;
    std::chrono::nanoseconds period_;
    mutable std::mutex tick_mutex_;
    mutable std::mutex state_mutex_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool sealed_ = false;
};

} // namespace robotkit
#endif
