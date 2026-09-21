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

class RK_API Endpoint {
public:
    virtual ~Endpoint() = default;

    virtual rk_result apply(const rk_robot_command &command) = 0;
    /**
     * Read the endpoint's latest state. A shared Simulation calls this after
     * its one world step; the endpoint itself never owns shared time.
     */
    virtual rk_result sample(uint64_t timestamp_ns, rk_robot_state &state) = 0;
    /** Discard commands staged during a simulation tick that will not advance. */
    virtual void discard_pending() noexcept {}
};

/**
 * Small deterministic endpoint used by the runtime tests and initial host
 * bring-up. It is intentionally not a simulator; it only moves joints toward
 * the most recent position targets.
 */
class RK_API InMemoryEndpoint final : public Endpoint {
public:
    explicit InMemoryEndpoint(uint32_t joint_count);

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
    RobotRuntime(const rk_robot_runtime_layout &layout, std::shared_ptr<Endpoint> endpoint,
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
    void set_externally_driven(bool value) noexcept;

private:
    void run();
    rk_result step_owner(uint64_t timestamp_ns);

    rk_robot_runtime_layout layout_{};
    std::shared_ptr<Endpoint> endpoint_;
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
};

} // namespace robotkit

#endif
