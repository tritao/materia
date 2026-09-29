#ifndef ROBOTKIT_SIMULATION_HPP
#define ROBOTKIT_SIMULATION_HPP

#include "robotkit_runtime.hpp"
#include "robotkit_simkit.h"
#include "nativekit_sim.h"
#include "nativekit_sim_session.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace robotkit {

class SimulationRobot;
class VirtualDeviceEndpoint;

/**
 * The robots taking part in one SimKit session. Every tick the session asks
 * this participant to apply every robot's commands (all or none), submit
 * their joint targets and base drives, and publish their samples from the
 * tick's snapshot.
 *
 * A Simulation attaches to a session its caller owns and outlives. The
 * session's owner controls the clock (step, start, stop, reset) and the
 * environment (session objects and actors), not this object.
 */
class Simulation final {
public:
    /** Takes part in a session the caller owns and outlives this object. */
    explicit Simulation(nksim_session session);
    ~Simulation();

    Simulation(const Simulation &) = delete;
    Simulation &operator=(const Simulation &) = delete;

    /** Adds one simulation-owned runtime before the topology is sealed. */
    rk_result add_robot(const rk_robot_runtime_blueprint &blueprint, rk_robot_runtime &out_runtime,
                        const rk_simulation_robot_desc *robot_desc = nullptr);
    rk_result cut_virtual_device_link(uint32_t robot_index, bool cut);
    rk_result reset_robot(uint32_t robot_index);
    rk_result teleport_robot(uint32_t robot_index, const rk_simulation_pose &pose);
    rk_result set_joint_positions(uint32_t robot_index, const double *positions, uint32_t count);
    /**
     * Moves one kinematic robot base for the next tick without stopping the
     * owner, resetting sensors, or replacing the reset pose.
     */
    rk_result drive_robot_base(uint32_t robot_index, const rk_simulation_pose &pose);
    /**
     * Jumps one robot base to a pose for the next tick without stopping the
     * owner, resetting sensors, or replacing the reset pose. The jump carries
     * the base's body-frame twist instead of reading as a velocity.
     */
    rk_result place_robot_base(uint32_t robot_index, const rk_simulation_pose &pose);
    /** Couples one robot's applied wheel velocity targets to its base pose. */
    rk_result set_differential_drive(uint32_t robot_index,
                                     const rk_simulation_differential_drive_desc &desc);
    rk_result clear_differential_drive(uint32_t robot_index);
    rk_result get_differential_drive_state(
        uint32_t robot_index, rk_simulation_differential_drive_state &out_state) const;
    /** Couples one robot's three applied omni-wheel velocity targets to its base pose. */
    rk_result set_omni_drive(uint32_t robot_index, const rk_simulation_omni_drive_desc &desc);
    rk_result clear_omni_drive(uint32_t robot_index);
    rk_result get_omni_drive_state(uint32_t robot_index,
                                   rk_simulation_omni_drive_state &out_state) const;
    rk_result get_robot_pose(uint32_t robot_index, rk_simulation_pose &out_pose) const;
    /** Reads one robot base's world-frame twist from the latest physics state. */
    rk_result get_robot_base_velocity(uint32_t robot_index, rk_simulation_twist &out_twist) const;
    rk_result apply_robot_force(uint32_t robot_index, const rk_simulation_wrench &wrench);
    rk_result get_link_pose(uint32_t robot_index, uint32_t link_index,
                            rk_simulation_pose &out_pose) const;
    rk_result get_robot_contacts(rk_robot_runtime runtime,
                                 std::vector<rk_robot_contact> &out) const;
    rk_result capture_presentation(rk_simulation_presentation_info &out_info,
        std::vector<rk_simulation_presentation_pose> &out_poses) const;
    /** Robot poses from a frame captured from this session. */
    rk_result present_frame(nksim_frame frame, rk_simulation_presentation_info &out_info,
                            std::vector<rk_simulation_presentation_pose> &out_poses) const;
    /** Returns the number of completed shared physics steps. */
    uint64_t step_index() const;
    /** Returns the fixed-step simulation time in seconds. */
    double simulation_time() const;
    /** Returns the RobotKit error that failed the latest tick, or RK_OK. */
    rk_result rejection() const { return rejected_; }

private:
    friend class SimulationRobot;
    /** Holds the recursive session lock, which orders every edit against ticks. */
    class Lock {
    public:
        explicit Lock(nksim_session session) : session_(session) { nksim_session_lock(session_); }
        ~Lock() { nksim_session_unlock(session_); }
        Lock(const Lock &) = delete;
        Lock &operator=(const Lock &) = delete;
    private:
        nksim_session session_;
    };
    void attach();
    void cleanup() noexcept;
    /** The world, while the session is not hosted, for topology and direct edits. */
    nksim_world stopped_world() const;
    // Session participant callbacks.
    rk_result prepare(const nksim_tick &tick);
    void discard() noexcept;
    rk_result submit(const nksim_tick &tick);
    rk_result publish(const nksim_tick &tick);
    rk_result reset_robots();
    rk_result read_body_pose(nksim_body body, rk_simulation_pose &out_pose) const;
    rk_result read_body_state(nksim_body body, nksim_body_state &state) const;
    /**
     * Writes one robot base's scene node, which its kinematic body follows on
     * the next tick, and re-seeds any drive plant from it.
     */
    rk_result set_robot_base_node_pose(uint32_t robot_index, const rk_simulation_pose &pose);
    rk_result write_robot_base_node(uint32_t robot_index, const rk_simulation_pose &pose);
    /** Re-seeds a robot's drive plant (pose, heading, tilt). */
    void seed_drive(uint32_t robot_index, const rk_simulation_pose &pose);
    /**
     * Moves a robot base continuously to `pose` over the next tick with the
     * exact double-precision twist given (world frame), writing its scene node
     * and driving its kinematic body (nksim_body_drive), so the body's reported
     * velocity carries no single-precision scene-node rounding.
     */
    rk_result drive_robot_base_body(uint32_t robot_index, const rk_simulation_pose &pose,
                                    const double linear_velocity[3],
                                    const double angular_velocity[3]);
    /** Integrates every enabled drive plant from its robot's applied wheel targets. */
    rk_result advance_drives();
    rk_result valid_wheel_joints(uint32_t robot_index, const uint32_t *joints,
                                 uint32_t count) const;
    /** Removes a robot's drive plant when it is of `kind`; its base stays put. */
    rk_result clear_drive(uint32_t robot_index, int kind);

    /**
     * Ideal rolling drive plant for one robot's kinematic base. Each tick it
     * turns the wheel velocity targets the robot applied for that tick into a
     * constant body twist and rolls the base along it, before physics
     * advances, so the base responds with zero latency and follows every stop
     * the robot applies. A robot has at most one plant, of either kind.
     */
    struct DrivePlant {
        enum class Kind { None, Differential, Omni };
        Kind kind = Kind::None;
        uint32_t wheel_count = 0;
        uint32_t joints[3] = {0, 0, 0};
        double wheel_radius = 0.0;
        double track_width = 0.0; // Differential only.
        // Omni only: maps wheel rim speeds to the body twist (vx, vy, omega).
        double inverse[3][3] = {};
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0; // Unwrapped, continuous across re-seeding.
        double height = 0.0;
        /**
         * The base's authored roll and pitch: its rotation with the heading
         * removed, Rz(-yaw) * rotation. The base's rotation is Rz(yaw) * tilt,
         * so the tilt turns with the heading while the base rolls on the
         * level floor.
         */
        double tilt[4] = {0.0, 0.0, 0.0, 1.0};
        double rates[3] = {0.0, 0.0, 0.0};
    };

    nksim_session session_ = 0;
    nksim_participant participant_ = 0;
    nkscene_scene scene_ = 0;
    nksim_shape shape_ = 0;
    std::vector<nksim_shape> link_shapes_;
    double fixed_timestep_ = 0.01;
    double gravity_[3] = {0.0, 0.0, -9.81};
    /** The robot command error that rejected the latest tick. */
    rk_result rejected_ = RK_OK;
    // Virtual device samples taken while submitting, published after physics.
    std::vector<rk_robot_state> virtual_samples_;
    std::vector<rk_result> virtual_sample_results_;
    std::vector<nkscene_node_id> nodes_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    std::vector<std::weak_ptr<SimulationRobot>> bindings_;
    std::vector<std::shared_ptr<SimulationRobot>> virtual_bindings_;
    std::vector<std::shared_ptr<VirtualDeviceEndpoint>> virtual_devices_;
    std::vector<std::shared_ptr<RobotRuntime>> runtimes_;
    std::vector<rk_robot_runtime> handles_;
    std::vector<nksim_body> robot_base_bodies_;
    // True when the robot's root link is a free dynamic body (floating base),
    // which kinematic base drives and couplings must not move.
    std::vector<bool> robot_floating_;
    std::vector<rk_simulation_pose> robot_initial_poses_;
    std::vector<rk_simulation_pose> robot_base_poses_; // Last pose written to each base node.
    // Base pose each robot held at the end of the latest completed tick
    // (or after a placement, teleport, or reset); a drive's motion is
    // measured from it.
    std::vector<rk_simulation_pose> robot_tick_poses_;
    std::vector<DrivePlant> drives_;
    std::chrono::nanoseconds period_;
};

} // namespace robotkit
#endif
