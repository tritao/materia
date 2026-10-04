#ifndef ROBOTKIT_RUNTIME_HPP
#define ROBOTKIT_RUNTIME_HPP

#include "robotkit_runtime.h"
#include "trajectory_core.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <optional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace robotkit {

/** One joint's coefficients in a segment, in powers of seconds from its start. */
struct SegmentCoefficients {
    double value[RK_TRAJECTORY_COEFFICIENT_STRIDE]{}; /**< Degree zero through five. */
};

/** A polynomial segment over every robot joint, timed from the start of its batch. */
struct TrajectorySegment {
    uint64_t time_from_start_ns = 0;
    uint64_t duration_ns = 0;
    uint32_t degree = 0;
    uint32_t joint_count = 0;
    SegmentCoefficients coefficients[RK_MAX_TRAJECTORY_JOINTS];
};

/** Contiguous segments starting at time zero, reported under one trajectory tag. */
struct SegmentBatch {
    std::vector<TrajectorySegment> segments;
    uint64_t tag = 0;
};

/**
 * A plan as RobotRuntime::submit_plan accepts it: rk_plan_header's fields, its
 * segments over every robot joint, and its events.
 */
struct PlanRequest {
    uint64_t sequence = 0;
    uint64_t plan_id = 0;
    uint64_t model_revision = 0;
    uint64_t calibration_revision = 0;
    uint32_t required_capabilities = 0;
    uint32_t flags = 0;
    uint64_t replace_after_plan_id = 0;
    uint64_t replace_after_time_ns = 0;
    bool ends_at_rest = false;
    double start_position[RK_MAX_TRAJECTORY_JOINTS]{};
    double start_velocity[RK_MAX_TRAJECTORY_JOINTS]{};
    double start_acceleration[RK_MAX_TRAJECTORY_JOINTS]{};
    double position_tolerance[RK_MAX_TRAJECTORY_JOINTS]{};
    double velocity_tolerance[RK_MAX_TRAJECTORY_JOINTS]{};
    double acceleration_tolerance[RK_MAX_TRAJECTORY_JOINTS]{};
    double control_acceleration[RK_MAX_TRAJECTORY_JOINTS]{};
    SegmentBatch segments;
    std::vector<rk_timed_event> events;
};

/** Checks segments: contiguous from zero, finite, degree at most five, all over one joint count. */
RK_API rk_result validate_segments(const SegmentBatch &batch);
/** validate_segments, over the blueprint's joints and honouring its couplings. */
RK_API rk_result validate_segments_for_blueprint(const SegmentBatch &batch,
    const rk_robot_runtime_blueprint &blueprint);
/** Checks a plan's identity, start state, segments, and events against a blueprint. */
RK_API rk_result validate_plan_for_blueprint(const PlanRequest &plan,
    const rk_robot_runtime_blueprint &blueprint);

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
    virtual rk_result device_homing_control(const rk_device_homing_control &) { return RK_ERROR_UNSUPPORTED; }
    virtual rk_result device_homing_status(uint64_t) const { return RK_ERROR_UNSUPPORTED; }
    virtual rk_result device_input(const char *, rk_device_input_observation &) const {
        return RK_ERROR_UNSUPPORTED;
    }
    virtual bool reports_safety_state() const noexcept { return false; }
    /** Atomically shift motor counter origins without changing physical targets or poses.
     * Implementations must validate the complete batch before modifying any motor. */
    virtual rk_result rebase_counters(const uint32_t *, const double *, uint32_t) {
        return RK_ERROR_UNSUPPORTED;
    }
    /** Physical coordinate for sensor synthesis, distinct from a power-up counter origin. */
    virtual double physical_position(uint32_t, double counter_position) const noexcept {
        return counter_position;
    }

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
    virtual rk_result submit_device_plan(const PlanRequest &, uint64_t,
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
    /** Queues segments; the command's kind is RK_COMMAND_TRAJECTORY_SEGMENTS. */
    rk_result submit_segments(const rk_robot_command &command, SegmentBatch batch);
    rk_result submit_plan(const PlanRequest &plan);
    rk_result require_reference(uint32_t joint, bool required);
    rk_result limit_input(uint32_t joint, bool active);
    rk_result device_input(const char *switch_id, rk_device_input_observation &out) const;
    rk_result device_homing_control(const rk_device_homing_control &control);
    rk_result device_homing_status(uint64_t sequence);
    /** Atomically establish logical = endpoint position + offset at rest. */
    rk_result calibrate_coordinates(const double *offsets, uint32_t count,
                                   const uint32_t *reference_joints = nullptr, uint32_t reference_count = 0);
    rk_result calibrate_home_drives(const uint32_t *joints, const double *side_zeros, uint32_t count);
    rk_result latch_reference(uint32_t joint);
    rk_result reference_status(uint32_t joint, uint32_t &out_referenced) const;
    /** Copies the latest robot state without advancing endpoint time. */
    rk_result snapshot(rk_robot_state &out_state) const;
    /** Copies the latest state plus revision, endpoint, and fault metadata. */
    rk_result snapshot_full(rk_robot_snapshot &out_snapshot, bool endpoint_coordinates = false) const;
    rk_result poll_events(rk_event_record_batch &out_batch);
    /** The current output value of a declared process channel. */
    rk_result channel_value(const char *channel, rk_event_value &out_value) const;
    /** Reports whether the endpoint accepts buffered trajectory chunks. */
    bool supports_trajectory_queue() const noexcept {
        return endpoint_ != nullptr && endpoint_->supports_trajectory_queue();
    }

    /** The compiled topology and limits this runtime executes. */
    const rk_robot_runtime_blueprint &blueprint() const noexcept { return blueprint_; }

    bool running() const;

    /**
      A queued segment holding only the coefficients its joints and degree use:
      MotionKit's `mk_segment` reserves room for every joint and degree, about
      3 KB, where a three-axis machine uses about a hundred bytes.
    **/
    struct RuntimeSegment {
        int64_t t0_ns = 0;
        int64_t duration_ns = 0;
        uint32_t degree = 0;
        uint32_t joint_count = 0;
        std::vector<double> coefficients; ///< Joint-major: `joint * (degree + 1) + power`.
        /** The segment in MotionKit's full layout, for its API. */
        mk_segment native() const;
    };

    struct RuntimeTrajectoryPoint {
        struct {
            uint64_t time_from_start_ns = 0;
            uint32_t joint_count = 0;
            std::vector<double> positions; ///< Held by end markers; a segment is evaluated instead.
        } point{};
        RuntimeSegment segment{}; /**< Valid when has_segment; starts at point time. */
        bool has_segment = false;
        uint64_t chunk_base_time_ns = 0;
        uint64_t tag = 0;
        uint64_t plan_id = 0;
        uint32_t plan_flags = 0;
        bool ends_at_rest = true;
        double control_acceleration[RK_MAX_TRAJECTORY_JOINTS]{};
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
        std::deque<QueuedEvent> events; ///< The trajectory queue itself is `trajectory_`.
        /** End of the queue a device that executes it was running at its latest report. */
        uint64_t device_queue_end_ns = 0;
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
        double control_acceleration[RK_MAX_TRAJECTORY_JOINTS]{};
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
        /** Preserve the interrupted homing bounds after its queue is cleared. */
        bool stop_ramp_homing = false;
    };

    struct QueuedCommand {
        rk_robot_command command{};
        std::shared_ptr<const SegmentBatch> segments;
    };

    void run();
    rk_result step_owner(uint64_t timestamp_ns);
    void latch_fault(bool clear_control = true, int32_t fault_code = 1);
    std::array<bool, RK_MAX_JOINTS> limit_inputs_{};

    rk_robot_runtime_blueprint blueprint_{};
    std::shared_ptr<RobotEndpoint> endpoint_;
    std::chrono::nanoseconds period_;
    uint64_t last_owner_timestamp_ns_ = 0;
    /** Whether a device that executes the queue reported it running in its latest sample. */
    bool device_queue_active_ = false;
    mutable std::mutex state_mutex_;
    rk_robot_state state_{};
    mutable std::mutex queue_mutex_;
    struct PendingDriveCalibration {
        std::array<uint32_t, RK_MAX_JOINTS> joints{};
        std::array<double, RK_MAX_JOINTS> zeros{};
        std::array<double, RK_MAX_JOINTS> deltas{};
        uint32_t count = 0;
        bool acknowledged = false;
    };
    std::optional<PendingDriveCalibration> pending_drive_calibration_;
    std::optional<uint64_t> pending_homing_stop_;
    std::array<double, RK_MAX_JOINTS> coordinate_offsets_{};
    std::array<bool, RK_MAX_JOINTS> reference_required_{};
    std::array<bool, RK_MAX_JOINTS> reference_latched_{};
    std::array<bool, RK_MAX_JOINTS> references_locked() const;
    /** Serializes plan submission with owner queue/clock mutations. */
    mutable std::mutex owner_mutex_;
    /**
     * Each channel's output as its device sees it: the safe value until an event fires on it, then
     * the last value fired. Kept apart from the control state, which stops and resets clear.
     */
    rk_event_value channel_outputs_[RK_MAX_PROCESS_CHANNELS]{};
    rk_event_value channel_outputs_backup_[RK_MAX_PROCESS_CHANNELS]{};
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
    /**
     * Takes channels to their safe values; a commanded stop or abort leaves those declared
     * RK_CHANNEL_KEEP_ON_STOP as they are.
     */
    void safe_channels(uint64_t owner_ns, rk_event_cause cause, bool hold_only = false, bool commanded_stop = false);
    int32_t latched_fault_code_ = 1;
    /** The control state at the start of the world tick, without its trajectory queue. */
    ControlState control_backup_{};
    /**
      The trajectory queue. Knots leave its front as the clock passes them and
      join its back as plans arrive; reads are open, and every change is
      journalled so a discarded world tick is undone in the size of its changes
      rather than by copying the queue. It cannot be copied, so it is kept out
      of the control state the tick backs up.
    **/
    class TrajectoryQueue {
    public:
        using Knots = std::deque<RuntimeTrajectoryPoint>;
        TrajectoryQueue() = default;
        TrajectoryQueue(const TrajectoryQueue &) = delete;
        TrajectoryQueue &operator=(const TrajectoryQueue &) = delete;

        const Knots &knots() const noexcept { return knots_; }
        bool empty() const noexcept { return knots_.empty(); }
        std::size_t size() const noexcept { return knots_.size(); }
        const RuntimeTrajectoryPoint &front() const { return knots_.front(); }
        const RuntimeTrajectoryPoint &back() const { return knots_.back(); }
        const RuntimeTrajectoryPoint &operator[](std::size_t index) const { return knots_[index]; }
        Knots::const_iterator begin() const noexcept { return knots_.begin(); }
        Knots::const_iterator end() const noexcept { return knots_.end(); }
        Knots::const_reverse_iterator rbegin() const noexcept { return knots_.rbegin(); }
        Knots::const_reverse_iterator rend() const noexcept { return knots_.rend(); }

        void pop_front();
        void clear();
        /** Appends knots ending in their own end marker, which replaces the queue's. */
        void append(std::vector<RuntimeTrajectoryPoint> &&added);
        void replace(Knots &&knots);
        /** Starts a world tick: its changes can be undone until the next one starts. */
        void begin_tick();
        /** Undoes the changes since `begin_tick`. */
        void undo_tick();
        /** Empties the queue and forgets the journal. */
        void reset();

    private:
        Knots knots_;
        std::vector<RuntimeTrajectoryPoint> removed_; ///< Knots of the tick's start taken from the front, in order.
        std::optional<RuntimeTrajectoryPoint> end_marker_; ///< The starting end marker an append replaced.
        std::size_t originals_ = 0; ///< Knots of the tick's start still at the front.
        std::size_t appended_ = 0; ///< Knots added this tick, at the back.
    };
    TrajectoryQueue trajectory_;
    /** Clears the trajectory queue, then every control field. */
    void reset_control();
    /** Last position sent to the endpoint, retained after a trajectory drains. */
    double commanded_position_[RK_MAX_JOINTS]{};
    /** A queued device's counters establish its first held anchor, before any plan. */
    bool device_anchor_initialized_ = false;
    double commanded_position_backup_[RK_MAX_JOINTS]{};
    bool velocity_anchor_pending_[RK_MAX_JOINTS]{};
    bool velocity_anchor_pending_backup_[RK_MAX_JOINTS]{};
    uint64_t endpoint_command_sequence_ = 0;
    bool state_backup_valid_ = false;
};

} // namespace robotkit

#endif
