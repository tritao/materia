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
     * Applies one validated command batch to the backend.
     *
     * The endpoint should not advance shared time here. A simulation can call
     * discard_pending() if a different robot rejects its command, so staged
     * changes must remain rollback-safe until the world tick is committed.
     *
     * @param command Complete command batch removed from the runtime mailbox.
     * @return RK_OK when accepted, or a backend/safety/argument error.
     */
    virtual rk_result apply(const rk_robot_command &command) = 0;

    /**
     * Samples the backend's latest state into a caller-owned value.
     *
     * A shared Simulation calls this after its one world step; the endpoint
     * itself never owns shared time. timestamp_ns labels the observation and
     * is not a request to advance the backend.
     *
     * @param timestamp_ns Observation timestamp chosen by the owner.
     * @param state Destination state, including arrays sized by the runtime
     * layout.
     * @return RK_OK when a complete state was copied, or a backend error.
     */
    virtual rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) = 0;

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
 * or a physics clock. It only moves joints toward the most recent position
 * targets, making it useful for checking mailbox and snapshot plumbing before
 * a real physical or simulated endpoint is available.
 */
class RK_API InMemoryRobot final : public RobotEndpoint {
public:
    explicit InMemoryRobot(uint32_t joint_count);

    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) override;

private:
    uint32_t joint_count_ = 0;
    double targets_[RK_MAX_JOINTS]{};
    bool has_target_[RK_MAX_JOINTS]{};
    bool stopped_ = false;
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
    /** Queues one complete command batch for the next owner-thread phase. */
    rk_result submit(const rk_robot_command &command);
    /** Copies the latest robot state without advancing endpoint time. */
    rk_result snapshot(rk_robot_state &out_state) const;
    /** Copies the latest state plus revision, endpoint, and fault metadata. */
    rk_result snapshot_full(rk_robot_snapshot &out_snapshot) const;

    bool running() const;

    /** Internal phases used by Simulation to coordinate multiple runtimes. */
    rk_result apply_pending_commands();
    rk_result publish_sample(uint64_t timestamp_ns);
    void discard_pending_commands() noexcept;
    void reset_state() noexcept;
    void set_externally_driven(bool value) noexcept;

private:
    void run();
    rk_result step_owner(uint64_t timestamp_ns);

    rk_robot_runtime_blueprint blueprint_{};
    std::shared_ptr<RobotEndpoint> endpoint_;
    std::chrono::nanoseconds period_;
    mutable std::mutex state_mutex_;
    rk_robot_state state_{};
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::deque<rk_robot_command> commands_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool externally_driven_ = false;
    uint64_t last_command_sequence_ = 0;
};

} // namespace robotkit

#endif
