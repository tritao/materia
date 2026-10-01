#ifndef ROBOTKIT_RUNTIME_HPP
#define ROBOTKIT_RUNTIME_HPP

#include "robotkit_runtime.h"
#include "motionkit.hpp"

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

    /**
     * Relative precision of the positions sample() reports: 0 for full double
     * precision, the float epsilon for a link that carries 32-bit floats. A
     * joint resting exactly on a limit may read that far past it, so the
     * runtime's observed-limit check allows it on top of the blueprint's
     * tolerance.
     */
    virtual double observed_position_precision() const noexcept { return 0.0; }

    /** Initial endpoint safety state, used before the first state sample. */
    virtual rk_safety_state initial_safety_state() const noexcept { return RK_SAFETY_READY; }

    /** Returns true when the endpoint accepts timestamped trajectory chunks. */
    virtual bool supports_trajectory_queue() const noexcept { return true; }

    /** Device-owned polynomial queue: the runtime does not emit sampled targets. */
    virtual bool executes_trajectory_queue() const noexcept { return false; }
    virtual int32_t diagnostic_code() const noexcept { return 0; }

    /** Called after host validation, before a submitted plan becomes visible. */
    virtual rk_result submit_device_plan(const rk_plan_submission &, uint64_t,
        uint64_t, uint64_t, const rk_robot_runtime_blueprint &) { return RK_ERROR_UNSUPPORTED; }

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
    rk_result submit_segments(const rk_robot_command &command,
                              const rk_trajectory_segment_chunk &chunk);
    rk_result submit_plan(const rk_plan_submission &plan);
    /** Copies the latest robot state without advancing endpoint time. */
    rk_result snapshot(rk_robot_state &out_state) const;
    /** Copies the latest state plus revision, endpoint, and fault metadata. */
    rk_result snapshot_full(rk_robot_snapshot &out_snapshot) const;
    rk_result poll_events(rk_event_record_batch &out_batch);
    /** Reports whether the endpoint accepts buffered trajectory chunks. */
    bool supports_trajectory_queue() const noexcept {
        return endpoint_ != nullptr && endpoint_->supports_trajectory_queue();
    }

    /** The compiled topology and limits this runtime executes. */
    const rk_robot_runtime_blueprint &blueprint() const noexcept { return blueprint_; }

    bool running() const;

    struct RuntimeTrajectoryPoint {
        struct {
            uint64_t time_from_start_ns = 0;
            uint32_t joint_count = 0;
            double positions[RK_MAX_TRAJECTORY_JOINTS]{};
        } point{};
        mk_segment segment{}; /**< Valid when has_segment; starts at point time. */
        bool has_segment = false;
        uint64_t chunk_base_time_ns = 0;
        uint64_t tag = 0;
        uint64_t plan_id = 0;
        bool ends_at_rest = true;
    };

    /** Internal phases used by Simulation to coordinate multiple runtimes. */
    rk_result apply_pending_commands(uint64_t owner_time_ns = 0);
    rk_result publish_sample(uint64_t timestamp_ns);
    rk_result publish_presampled(uint64_t timestamp_ns, const rk_robot_state &sample,
                                rk_result sample_result = RK_OK);
    void discard_pending_commands() noexcept;
    /**
     * Confines a failed tick to this robot: rolls back what the tick had
     * applied and leaves the robot faulted, which its own state reports. The
     * shared Simulation calls it so one robot's rejected command or
     * out-of-limit reading does not fail the tick of every other robot.
     */
    void fail_tick() noexcept;
    void reset_state() noexcept;
    void set_externally_driven(bool value) noexcept;

private:
    rk_result publish_sample_impl(uint64_t timestamp_ns, const rk_robot_state *sample,
                                  rk_result sample_result);
    struct QueuedEvent {
        uint64_t time_ns = 0;
        uint64_t plan_id = 0;
        rk_timed_event event{};
    };
    struct ControlState {
        rk_joint_target targets[RK_MAX_JOINTS]{};
        rk_joint_servo servos[RK_MAX_SERVO_JOINTS]{}; // Terms of RK_TARGET_SERVO targets, by joint.
        double position_reference[RK_MAX_JOINTS]{};
        bool active[RK_MAX_JOINTS]{};
        bool reference_initialized[RK_MAX_JOINTS]{};
        /** Source-clock time a velocity target lapses (0: never); see rk_robot_command.expires_at_ns. */
        uint64_t velocity_expiry_ns[RK_MAX_JOINTS]{};
        std::deque<RuntimeTrajectoryPoint> trajectory;
        std::deque<QueuedEvent> events;
        rk_event_value channel_values[RK_MAX_PROCESS_CHANNELS]{};
        rk_event_value last_fired_values[RK_MAX_PROCESS_CHANNELS]{};
        rk_event_hold_policy channel_hold_policies[RK_MAX_PROCESS_CHANNELS]{};
        bool channel_has_fired[RK_MAX_PROCESS_CHANNELS]{};
        /** The last point dropped from the front of the queue, for looking back. */
        RuntimeTrajectoryPoint trajectory_history{};
        bool trajectory_history_valid = false;
        uint64_t trajectory_time_ns = 0;
        bool trajectory_active = false;
        bool plan_just_submitted = false;
        uint64_t trajectory_tag = 0;
        uint64_t trajectory_tag_time_ns = 0;
        uint64_t active_plan_id = 0;
        /** Trajectory clock rate; below 1 only while a path-following stop runs. */
        double trajectory_rate = 1.0;
        double trajectory_time_remainder_ns = 0.0;
        uint64_t stop_ramp_time_ns = 0;
        uint64_t stop_ramp_duration_ns = 0;
        double stop_ramp_positions[RK_MAX_JOINTS]{};
        double stop_ramp_velocities[RK_MAX_JOINTS]{};
        /** Last forward estimate of the queued path's acceleration during a stop. */
        double stop_path_accelerations[RK_MAX_TRAJECTORY_JOINTS]{};
        bool stop_ramp_active = false;
        bool hold_requested = false;
        bool resume_requested = false;
        int32_t diagnostic_code = 0;
        /** Set when the unclamped straight ramp reaches a joint travel limit. */
        bool stop_ramp_hits_limit = false;
    };

    struct QueuedCommand {
        rk_robot_command command{};
        std::shared_ptr<rk_trajectory_segment_chunk> segments;
    };

    void run();
    rk_result step_owner(uint64_t timestamp_ns);
    void latch_fault(bool clear_control = true, int32_t fault_code = 1);

    rk_robot_runtime_blueprint blueprint_{};
    std::shared_ptr<RobotEndpoint> endpoint_;
    std::chrono::nanoseconds period_;
    uint64_t last_owner_timestamp_ns_ = 0;
    mutable std::mutex state_mutex_;
    rk_robot_state state_{};
    mutable std::mutex queue_mutex_;
    /** Serializes plan submission with owner queue/clock mutations. */
    mutable std::mutex owner_mutex_;
    std::condition_variable queue_condition_;
    std::deque<QueuedCommand> commands_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool externally_driven_ = false;
    uint64_t last_command_sequence_ = 0;
    rk_robot_state state_backup_{};
    // apply_pending_commands' working copies, kept off the stack (about 17 KiB each); guarded by owner_mutex_.
    rk_robot_state owner_state_{};
    rk_robot_state owner_target_state_{};
    rk_robot_command owner_output_{};
    ControlState control_{};
    std::deque<rk_event_record> event_records_;
    bool event_records_overflow_ = false;
    uint64_t current_owner_time_ns_ = 0;
    void record_event(uint32_t channel_index, const rk_event_value &value,
        uint64_t plan_id, uint64_t scheduled_ns, uint64_t owner_ns, rk_event_cause cause);
    void safe_channels(uint64_t owner_ns, rk_event_cause cause, bool hold_only = false);
    int32_t latched_fault_code_ = 1;
    ControlState control_backup_{};
    /** Last position sent to the endpoint, retained after a trajectory drains. */
    double commanded_position_[RK_MAX_JOINTS]{};
    double commanded_position_backup_[RK_MAX_JOINTS]{};
    bool velocity_anchor_pending_[RK_MAX_JOINTS]{};
    bool velocity_anchor_pending_backup_[RK_MAX_JOINTS]{};
    uint64_t endpoint_command_sequence_ = 0;
    bool state_backup_valid_ = false;
};

} // namespace robotkit

#endif
