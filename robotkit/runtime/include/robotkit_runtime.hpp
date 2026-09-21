#ifndef ROBOTKIT_RUNTIME_HPP
#define ROBOTKIT_RUNTIME_HPP

#include "robotkit_core.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

namespace robotkit {

class Endpoint {
public:
    virtual ~Endpoint() = default;

    virtual rk_result apply(const rk_robot_command &command) = 0;
    virtual rk_result step(uint64_t timestamp_ns, rk_robot_state &state) = 0;
};

/**
 * Small deterministic endpoint used by the runtime tests and initial host
 * bring-up. It is intentionally not a simulator; it only moves joints toward
 * the most recent position targets.
 */
class InMemoryEndpoint final : public Endpoint {
public:
    explicit InMemoryEndpoint(uint32_t joint_count);

    rk_result apply(const rk_robot_command &command) override;
    rk_result step(uint64_t timestamp_ns, rk_robot_state &state) override;

private:
    uint32_t joint_count_ = 0;
    double targets_[RK_MAX_JOINTS]{};
    bool has_target_[RK_MAX_JOINTS]{};
    bool stopped_ = false;
};

class Runtime final {
public:
    Runtime(const rk_runtime_layout &layout, std::unique_ptr<Endpoint> endpoint,
            std::chrono::nanoseconds period = std::chrono::milliseconds(10));
    ~Runtime();

    Runtime(const Runtime &) = delete;
    Runtime &operator=(const Runtime &) = delete;

    rk_result start();
    rk_result stop();
    rk_result submit(const rk_robot_command &command);
    /** Run one owner-thread tick. The realtime worker must be stopped. */
    rk_result step_once(uint64_t timestamp_ns);
    rk_result snapshot(rk_robot_state &out_state) const;
    rk_result snapshot_full(rk_robot_snapshot &out_snapshot) const;

    bool running() const;

private:
    void run();
    rk_result step_owner(uint64_t timestamp_ns);

    rk_runtime_layout layout_{};
    std::unique_ptr<Endpoint> endpoint_;
    std::chrono::nanoseconds period_;
    mutable std::mutex state_mutex_;
    rk_robot_state state_{};
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::deque<rk_robot_command> commands_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
};

} // namespace robotkit

#endif
