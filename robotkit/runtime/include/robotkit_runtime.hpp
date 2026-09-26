#ifndef ROBOTKIT_RUNTIME_HPP
#define ROBOTKIT_RUNTIME_HPP

#include "robotkit_runtime.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

namespace robotkit {

/**
 * Backend adapter used by one RobotRuntime.
 *
 * RobotEndpoint is the native boundary between RobotRuntime's command mailbox and
 * the state-producing mechanism behind it. Implementations may represent a
 * physical driver, a shared Simulation binding, a replay source, or a small
 * test double. RobotEndpoint does not own the RobotRuntime, its public handle, or
 * a simulation clock.
 *
 * RobotRuntime invokes the methods on its owner thread. For a shared
 * Simulation, the simulation invokes the same phases for every endpoint in a
 * single tick: all commands are applied first, the world advances once, and
 * every endpoint is sampled afterward. Implementations should therefore stage
 * command effects in apply() and publish only observed state from sample().
 */
class RK_API RobotEndpoint {
public:
    virtual ~RobotEndpoint() = default;

    /**
     * Applies one validated command or per-cycle controller output batch.
     * A runtime owner phase may apply an intermediate lifecycle command before
     * its final generated setpoint when a mailbox drain contains an ordered
     * reset/stop followed by a new motion request.
     *
     * The endpoint should not advance shared time here. A simulation can call
     * discard_pending() if a different robot rejects its command, so staged
     * changes must remain rollback-safe until the world tick is committed.
     *
     * @param command Runtime-arbitrated intent or the generated actuator
     * setpoints for the current owner tick.
     * @return RK_OK when accepted, or a backend/safety/argument error.
     */
    virtual rk_result apply(const rk_robot_command &command) = 0;

    /**
     * Samples the backend's latest state into a caller-owned value.
     *
     * A shared Simulation calls this after its one world step; the endpoint
     * itself never owns shared time. timestamp_ns is an owner-clock hint, not
     * a receive timestamp or a request to advance the backend. Endpoints with
     * their own clock publish it (including epoch zero) as source time.
     *
     * @param timestamp_ns Owner-clock sampling hint; not Runtime acceptance time.
     * @param state Destination state, including arrays sized by the runtime
     * layout. The runtime preserves its mode. Safety is preserved by default;
     * an endpoint that reports safety state may provide its observed value.
     * @return RK_OK when a complete state was copied, or a backend error.
     */
    virtual rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) = 0;

    /** Returns true when sample() supplies the endpoint's observed safety state. */
    virtual bool reports_safety_state() const noexcept { return false; }

    /** Initial endpoint safety state, used before the first state sample. */
    virtual rk_safety_state initial_safety_state() const noexcept { return RK_SAFETY_READY; }

    /** Returns true when the endpoint accepts timestamped trajectory chunks. */
    virtual bool supports_trajectory_queue() const noexcept { return true; }

    /**
     * Rolls back command effects staged during a failed simulation tick.
     *
     * The default implementation is appropriate for endpoints whose apply()
     * operation is already transactional or has no deferred backend state.
     * This method is noexcept because it is used while unwinding a rejected
     * multi-robot tick.
     */
    virtual void discard_pending() noexcept {}
};

/**
 * Small loopback robot adapter used by runtime tests and initial host bring-up.
 *
 * It is intentionally not a simulator and does not model bodies, contacts,
 * or a physics clock. It tracks the most recent position setpoints exactly,
 * making it useful for deterministic controller tests before a physical or
 * simulated endpoint is available.
 */
class RK_API InMemoryRobot final : public RobotEndpoint {
public:
    explicit InMemoryRobot(uint32_t joint_count);

    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;

private:
    uint32_t joint_count_ = 0;
    rk_joint_target targets_[RK_MAX_JOINTS]{};
    bool has_target_[RK_MAX_JOINTS]{};
    bool stopped_ = false;
    uint64_t last_sample_timestamp_ns_ = 0;
    bool has_sample_timestamp_ = false;
};

/**
 * One robot's command mailbox, endpoint adapter, and published state.
 *
 * RobotRuntime deliberately does not own a simulation clock. A standalone
 * instance may run its endpoint worker with start()/stop(); a runtime created
 * by Simulation is marked externally driven and is advanced only during the
 * simulation's shared tick.
 */
class RK_API RobotRuntime final {
public:
    RobotRuntime(const rk_robot_runtime_blueprint &blueprint, std::shared_ptr<RobotEndpoint> endpoint,
            std::chrono::nanoseconds period = std::chrono::milliseconds(10));
    ~RobotRuntime();

    RobotRuntime(const RobotRuntime &) = delete;
    RobotRuntime &operator=(const RobotRuntime &) = delete;

    /** Starts the private endpoint worker; invalid for externally driven runtimes. */
    rk_result start();
    /** Stops the private endpoint worker, if one is running. */
    rk_result stop();
    /** Queues command metadata and joint-target payload for the next owner-thread phase. */
    rk_result submit(const rk_robot_command &command);
    /** Queues trajectory metadata and its separately allocated payload. */
    rk_result submit_trajectory(const rk_robot_command &command,
                                const rk_trajectory_chunk &chunk);
    /** Copies the latest robot state without advancing endpoint time. */
    rk_result snapshot(rk_robot_state &out_state) const;
    /** Copies the latest state plus revision, endpoint, and fault metadata. */
    rk_result snapshot_full(rk_robot_snapshot &out_snapshot) const;
    /** Reports whether the endpoint accepts buffered trajectory chunks. */
    bool supports_trajectory_queue() const noexcept {
        return endpoint_ != nullptr && endpoint_->supports_trajectory_queue();
    }

    bool running() const;

    /** Internal phases used by Simulation to coordinate multiple runtimes. */
    rk_result apply_pending_commands();
    rk_result publish_sample(uint64_t timestamp_ns);
    void discard_pending_commands() noexcept;
    void reset_state() noexcept;
    void set_externally_driven(bool value) noexcept;

private:
    struct ControlState {
        rk_joint_target targets[RK_MAX_JOINTS]{};
        double position_reference[RK_MAX_JOINTS]{};
        bool active[RK_MAX_JOINTS]{};
        bool reference_initialized[RK_MAX_JOINTS]{};
        std::deque<rk_trajectory_point> trajectory;
        uint64_t trajectory_time_ns = 0;
        bool trajectory_active = false;
        uint64_t stop_ramp_time_ns = 0;
        uint64_t stop_ramp_duration_ns = 0;
        double stop_ramp_positions[RK_MAX_JOINTS]{};
        double stop_ramp_velocities[RK_MAX_JOINTS]{};
        bool stop_ramp_active = false;
    };

    struct QueuedCommand {
        rk_robot_command command{};
        std::shared_ptr<rk_trajectory_chunk> trajectory;
    };

    void run();
    rk_result step_owner(uint64_t timestamp_ns);
    void latch_fault();

    rk_robot_runtime_blueprint blueprint_{};
    std::shared_ptr<RobotEndpoint> endpoint_;
    std::chrono::nanoseconds period_;
    mutable std::mutex state_mutex_;
    rk_robot_state state_{};
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::deque<QueuedCommand> commands_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool externally_driven_ = false;
    uint64_t last_command_sequence_ = 0;
    rk_robot_state state_backup_{};
    ControlState control_{};
    ControlState control_backup_{};
    uint64_t endpoint_command_sequence_ = 0;
    bool state_backup_valid_ = false;
};

} // namespace robotkit

#endif
