#ifndef ROBOTKIT_SIMULATION_HPP
#define ROBOTKIT_SIMULATION_HPP

#include "robotkit_runtime.hpp"
#include "nativekit_sim.h"
#include "nativekit_sim_host.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace robotkit {

class SimulationRobot;

/**
 * Owns one shared simulated universe and coordinates all RobotRuntime objects
 * attached to it. Commands are collected for every robot before this object
 * advances the physics clock exactly once.
 */
class Simulation final {
public:
    /** Creates an empty shared universe; robots must be added before stepping. */
    Simulation(double fixed_timestep, uint32_t physics_substeps);
    ~Simulation();

    Simulation(const Simulation &) = delete;
    Simulation &operator=(const Simulation &) = delete;

    /** Adds one simulation-owned runtime before the topology is sealed. */
    rk_result add_robot(const rk_robot_runtime_blueprint &blueprint, rk_robot_runtime &out_runtime);
    /** Starts the shared realtime owner thread after all robots are attached. */
    rk_result start();
    /** Stops the shared realtime owner thread without destroying the universe. */
    rk_result stop();
    /** Applies all robot mailboxes and advances the physics host exactly once. */
    rk_result step(uint64_t timestamp_ns);
    /** Returns the number of completed shared physics steps. */
    uint64_t step_index() const;
    /** Returns the fixed-step simulation time in seconds. */
    double simulation_time() const;

private:
    friend class SimulationRobot;
    void cleanup() noexcept;
    rk_result ensure_host();
    rk_result advance(uint64_t timestamp_ns);
    void run();

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
    std::vector<std::weak_ptr<SimulationRobot>> bindings_;
    std::vector<std::shared_ptr<RobotRuntime>> runtimes_;
    std::vector<rk_robot_runtime> handles_;
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
