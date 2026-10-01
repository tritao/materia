#pragma once

#include "robotkit_runtime.hpp"
#include "device_wire6.hpp"
#include "clock_estimator6.hpp"
#include "device_compiler6.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace robotkit {

/** Complete-frame transport. The virtual link and serial adapter implement this. */
class RK_API Rkd6Transport {
public:
    virtual ~Rkd6Transport() = default;
    virtual bool send(std::span<const std::uint8_t> frame) = 0;
    virtual bool receive(std::vector<std::uint8_t> &frame) = 0;
    virtual unsigned baud() const noexcept = 0;
    virtual std::uint64_t received_at_ns() const noexcept { return 0; }
    /** Bytes handed to the transport and not yet on the line, when it can tell. */
    virtual std::optional<std::size_t> queued_output_bytes() const noexcept { return std::nullopt; }
};

class RK_API Rkd6Endpoint final : public RobotEndpoint {
public:
    static std::uint64_t minimum_baud(std::uint8_t actuator_count,
        std::uint64_t minimum_segment_ns, std::uint64_t processing_allowance_ns);
    static std::uint16_t minimum_queue_depth(unsigned baud, std::uint8_t actuator_count,
        std::uint64_t minimum_segment_ns, std::uint64_t link_latency_ns,
        std::uint64_t clock_uncertainty_ns);
    static std::shared_ptr<Rkd6Endpoint> attach(std::unique_ptr<Rkd6Transport> transport,
        const rk_robot_runtime_blueprint &blueprint, std::array<std::uint8_t, 16> fingerprint,
        std::uint64_t session, double target_error, std::uint64_t clock_bound_ns,
        std::uint64_t link_latency_ns, std::uint32_t step_tick_hz = 40'000,
        std::uint64_t link_loss_timeout_ns = 500'000'000,
        std::span<const DeviceActuator6> layout = {}, rk_result *error = nullptr);

    rk_result apply(const rk_robot_command &command) override;
    rk_result sample(std::uint64_t timestamp_ns, rk_robot_state &state) override;
    bool reports_safety_state() const noexcept override { return true; }
    rk_safety_state initial_safety_state() const noexcept override { return RK_SAFETY_EMERGENCY_STOP; }
    bool executes_trajectory_queue() const noexcept override { return true; }
    int32_t diagnostic_code() const noexcept override {
        return queue_revision_mismatch_ ? RK_FAULT_QUEUE_REVISION_MISMATCH :
            clock_.clock_sync_lost() ? RK_FAULT_CLOCK_SYNC_LOST :
            status_.fault == 4 ? RK_FAULT_DUAL_DRIVE_SKEW : 0;
    }
    rk_result submit_device_plan(const PlanRequest &plan, std::uint64_t base_time_ns,
        std::uint64_t owner_now_ns, std::uint64_t committed_through_ns,
        const rk_robot_runtime_blueprint &blueprint) override;
    const char *fault_reason() const noexcept {
        return queue_revision_mismatch_ ? "queue_revision_mismatch" : clock_.fault_reason();
    }
    std::uint64_t committed_until_ticks() const noexcept { return committed_until_ticks_; }
    std::pair<std::size_t, std::size_t> bookkeeping_counts() const noexcept {
        return {sent_.size(), chunk_timings_.size()};
    }

private:
    Rkd6Endpoint(std::unique_ptr<Rkd6Transport>, device_wire6::SessionAck6,
        double target_error, std::uint64_t clock_bound_ns, std::uint64_t link_latency_ns,
        std::vector<DeviceActuator6> layout, std::uint32_t joint_count);
    bool send_record(std::uint8_t kind, std::span<const std::uint8_t> payload);
    bool send_segment(const DeviceSegment6 &segment);
    bool send_commit(std::uint64_t through_ticks);
    void poll_frames(std::uint64_t owner_now_ns);
    void pump_queue();
    /** How long the line takes to send what it already holds. */
    std::uint64_t link_drain_ns() const noexcept;
    /** How far ahead of the device's path clock a commit is sent. */
    std::uint64_t commit_margin_ns() const noexcept;
    /**
      The most the line may hold when segments are sent: a commit due then is
      seen up to an owner period late, waits for the line, and crosses the link
      within the margin it was due at.
    **/
    std::uint64_t segment_backlog_budget_ns() const noexcept;
    /** Whether `segment` can go now and the line still clear within the budget. */
    bool link_has_room(const DeviceSegment6 &segment) const noexcept;
    void send_due_commit();

    std::unique_ptr<Rkd6Transport> transport_;
    device_wire6::SessionAck6 ack_{};
    std::vector<DeviceActuator6> layout_;
    std::uint32_t joint_count_ = 0;
    ClockEstimator6 clock_;
    double target_error_;
    std::uint64_t link_latency_ns_;
    std::uint64_t owner_period_ns_ = 10'000'000;
    /** The owner time last seen, and when the line finishes sending what was given to it. */
    std::uint64_t now_ns_ = 0;
    std::uint64_t link_free_at_ns_ = 0;
    std::uint64_t host_epoch_ns_ = 0;
    std::uint64_t device_epoch_ticks_ = 0;
    bool epoch_set_ = false;
    std::uint64_t revision_ = 0;
    std::uint64_t revision_boundary_ticks_ = 0;
    bool queue_revision_mismatch_ = false;
    std::uint64_t committed_until_ticks_ = 0;
    device_wire6::QueueStatus6 status_{};
    device_wire6::State6Header state_header_{};
    std::array<device_wire6::ActuatorState6, device_wire6::MAX_ACTUATORS> actuators_{};
    bool has_state_ = false;
    std::deque<DeviceSegment6> pending_;
    std::vector<DeviceSegment6> sent_;
    std::size_t next_commit_ = 0;
    /**
      How one submitted chunk's path time maps to device ticks: through the
      clock mapping it was compiled with, shifted to meet the queued path. A
      boundary or commit inside it maps to exactly the ticks its segments
      carry, however the clock estimate has moved since.
    **/
    struct ChunkTiming {
        std::uint64_t host_path_start_ns;
        std::uint64_t device_start_ticks;
        std::uint64_t host_epoch_ns;
        ClockMap6 clock;
        std::int64_t shift_ticks;
    };
    std::vector<ChunkTiming> chunk_timings_;
    /** Device ticks of the queued path at `path_ns`, or 0 when no chunk covers it. */
    std::uint64_t device_ticks_at(std::uint64_t path_ns) const noexcept;
    struct PlanTag {
        std::uint64_t plan_id;
        std::uint64_t start_ticks;
        std::uint64_t end_ticks;
        std::uint64_t duration_ns;
    };
    std::vector<PlanTag> plan_tags_;
    std::uint64_t path_time_ns(std::uint64_t device_ticks) const noexcept;
};

} // namespace robotkit
