#ifndef ROBOTKIT_SIMULATION_HPP
#define ROBOTKIT_SIMULATION_HPP

#include "robotkit_runtime.hpp"
#include "robotkit_simkit.h"
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
    Simulation(double fixed_timestep, uint32_t physics_substeps, uint32_t backend = 0);
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
    rk_result reset();
    rk_result reset_robot(uint32_t robot_index);
    rk_result teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose);
    rk_result get_robot_pose(uint32_t robot_index, rk_simulation_pose &out_pose) const;
    rk_result get_link_pose(uint32_t robot_index, uint32_t link_index,
                            rk_simulation_pose &out_pose) const;
    rk_result spawn_object(const rk_simulation_object_desc &desc,
                           rk_simulation_object &out_object);
    rk_result remove_object(rk_simulation_object object);
    rk_result teleport_object(rk_simulation_object object, const rk_simulation_pose &pose);
    rk_result get_object_pose(rk_simulation_object object, rk_simulation_pose &out_pose) const;
    /** Returns the number of completed shared physics steps. */
    uint64_t step_index() const;
    /** Returns the fixed-step simulation time in seconds. */
    double simulation_time() const;

private:
    friend class SimulationRobot;
    void cleanup() noexcept;
    rk_result ensure_host();
    rk_result advance(uint64_t timestamp_ns);
    rk_result read_body_pose(nksim_body body, rk_simulation_pose &out_pose) const;
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
    double gravity_[3] = {0.0, 0.0, -9.81};
    bool topology_frozen_ = false;
    std::vector<nkscene_occurrence_id> occurrences_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<std::weak_ptr<SimulationRobot>> bindings_;
    std::vector<std::shared_ptr<RobotRuntime>> runtimes_;
    std::vector<rk_robot_runtime> handles_;
    std::vector<nksim_body> robot_base_bodies_;
    std::vector<rk_simulation_pose> robot_initial_poses_;
    struct EnvironmentObject {
        nkscene_occurrence_id occurrence{};
        nksim_shape shape = 0;
        nksim_body body = 0;
        bool active = false;
        double half_extents[3]{};
        rk_simulation_pose initial_pose{};
    };
    std::vector<EnvironmentObject> objects_;
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
