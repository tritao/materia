#include "rkd6_endpoint.hpp"
#include "device_frame6.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>

namespace robotkit {

std::uint64_t Rkd6Endpoint::minimum_baud(std::uint8_t actuator_count,
    std::uint64_t minimum_segment_ns, std::uint64_t processing_allowance_ns) {
    if (actuator_count == 0 || minimum_segment_ns <= processing_allowance_ns) return UINT64_MAX;
    const auto bytes = device_frame6::HEADER_SIZE + device_frame6::CRC_SIZE +
        device_wire6::Segment6Header::SIZE +
        actuator_count * device_wire6::Segment6Coefficients::SIZE;
    return static_cast<std::uint64_t>(std::ceil(
        10.0L * bytes * 1e9L / (minimum_segment_ns - processing_allowance_ns)));
}

std::uint16_t Rkd6Endpoint::minimum_queue_depth(unsigned baud, std::uint8_t actuator_count,
    std::uint64_t minimum_segment_ns, std::uint64_t link_latency_ns,
    std::uint64_t clock_uncertainty_ns) {
    if (baud == 0 || minimum_segment_ns == 0) return UINT16_MAX;
    const auto bytes = device_frame6::HEADER_SIZE + device_frame6::CRC_SIZE +
        device_wire6::Segment6Header::SIZE +
        actuator_count * device_wire6::Segment6Coefficients::SIZE;
    const auto frame_ns = std::ceil(10.0L * bytes * 1e9L / baud);
    const auto depth = std::ceil((frame_ns + link_latency_ns +
        2.0L * clock_uncertainty_ns) / minimum_segment_ns) + 1;
    return static_cast<std::uint16_t>(std::min<long double>(depth, UINT16_MAX));
}

Rkd6Endpoint::Rkd6Endpoint(std::unique_ptr<Rkd6Transport> transport,
    device_wire6::SessionAck6 ack, double target_error, std::uint64_t clock_bound_ns,
    std::uint64_t link_latency_ns, std::vector<DeviceActuator6> layout,
    std::uint32_t joint_count)
    : transport_(std::move(transport)), ack_(ack), clock_(ack.device_tick_hz, clock_bound_ns),
      layout_(std::move(layout)), joint_count_(joint_count), target_error_(target_error),
      link_latency_ns_(link_latency_ns) {}

std::shared_ptr<Rkd6Endpoint> Rkd6Endpoint::attach(std::unique_ptr<Rkd6Transport> transport,
    const rk_robot_runtime_blueprint &blueprint, std::array<std::uint8_t, 16> fingerprint,
    std::uint64_t session, double target_error, std::uint64_t clock_bound_ns,
    std::uint64_t link_latency_ns, std::uint32_t step_tick_hz,
    std::uint64_t link_loss_timeout_ns, std::span<const DeviceActuator6> layout,
    rk_result *error) {
    if (error) *error = RK_ERROR_BACKEND;
    const auto actuator_count = layout.empty() ? blueprint.joint_count : layout.size();
    if (!transport || session == 0 || blueprint.joint_count == 0 ||
        blueprint.joint_count > device_wire6::MAX_ACTUATORS ||
        actuator_count == 0 || actuator_count > device_wire6::MAX_ACTUATORS ||
        !std::isfinite(target_error) || target_error < 0 || clock_bound_ns == 0 ||
        step_tick_hz == 0 || link_loss_timeout_ns == 0 ||
        blueprint.channel_count > RK_MAX_PROCESS_CHANNELS) {
        if (error) *error = RK_ERROR_INVALID_ARGUMENT;
        return {};
    }
    fingerprint = fingerprint_device_layout6(fingerprint, layout,
        std::span(blueprint.channels, blueprint.channel_count));
    device_wire6::SessionBegin6 begin{};
    begin.session = session;
    begin.protocol_version = device_wire6::PROTOCOL_VERSION;
    begin.model_fingerprint = fingerprint;
    begin.actuator_count = static_cast<std::uint8_t>(actuator_count);
    begin.max_degree = 5;
    begin.step_tick_hz = step_tick_hz;
    begin.link_loss_timeout_ns = link_loss_timeout_ns;
    begin.channel_count = static_cast<std::uint8_t>(blueprint.channel_count);
    for (std::uint32_t i = 0; i < blueprint.channel_count; ++i) {
        const auto &channel = blueprint.channels[i];
        begin.channel_kind[i] = static_cast<std::uint8_t>(channel.kind);
        begin.safe_digital[i] = static_cast<std::uint8_t>(channel.safe_value.digital);
        if (!std::isfinite(channel.safe_value.analog) ||
            !std::isfinite(channel.safe_value.argument) ||
            std::abs(channel.safe_value.analog) > std::numeric_limits<float>::max() ||
            std::abs(channel.safe_value.argument) > std::numeric_limits<float>::max()) return {};
        begin.safe_analog[i] = static_cast<float>(channel.safe_value.analog);
        begin.safe_argument[i] = static_cast<float>(channel.safe_value.argument);
        std::memcpy(begin.channel_id.data() + i * RK_PROCESS_CHANNEL_ID_BYTES,
            channel.id, RK_PROCESS_CHANNEL_ID_BYTES);
        std::memcpy(begin.safe_command.data() + i * RK_PROCESS_COMMAND_BYTES,
            channel.safe_value.command, RK_PROCESS_COMMAND_BYTES);
    }
    for (std::size_t i = 0; i < actuator_count; ++i) {
        const auto mapping = layout.empty() ? DeviceActuator6{static_cast<std::uint8_t>(i)} : layout[i];
        if (mapping.joint >= blueprint.joint_count || !std::isfinite(mapping.ratio) ||
            mapping.ratio == 0 || !std::isfinite(mapping.offset) ||
            !std::isfinite(mapping.steps_per_unit) || mapping.steps_per_unit <= 0 ||
            !std::isfinite(mapping.max_rate) || mapping.max_rate < 0 ||
            !std::isfinite(mapping.dual_drive_skew_bound) || mapping.dual_drive_skew_bound < 0)
            return {};
        begin.actuator_max_acceleration[i] = static_cast<float>(
            std::abs(mapping.ratio) * blueprint.joints[mapping.joint].max_acceleration);
        begin.steps_per_unit[i] = static_cast<float>(mapping.steps_per_unit);
        begin.max_rate[i] = static_cast<float>(mapping.max_rate);
        begin.direction_setup_ticks[i] = mapping.direction_setup_ticks;
        begin.actuator_joint[i] = mapping.joint;
        begin.actuator_ratio[i] = static_cast<float>(mapping.ratio);
        begin.dual_drive_skew_bound[i] = static_cast<float>(mapping.dual_drive_skew_bound);
        begin.max_acceleration = std::max(begin.max_acceleration,
            begin.actuator_max_acceleration[i]);
    }
    if (begin.max_acceleration <= 0 || !std::isfinite(begin.max_acceleration)) return {};
    std::vector<std::uint8_t> payload(begin.SIZE);
    if (!device_wire6::encode(begin, payload)) return {};
    std::vector<std::uint8_t> frame;
    if (!device_frame6::encode(1, payload, frame) || !transport->send(frame)) return {};
    std::vector<std::uint8_t> reply;
    device_wire6::SessionAck6 ack{};
    bool acknowledged = false;
    while (transport->receive(reply)) {
        device_frame6::Frame decoded{};
        if (!device_frame6::decode(reply, decoded)) return {};
        if (decoded.kind == 2) {
            acknowledged = device_wire6::decode(decoded.payload, ack);
            break;
        }
    }
    if (acknowledged && ack.device_fingerprint != fingerprint) {
        if (error) *error = RK_ERROR_MODEL_MISMATCH;
        return {};
    }
    if (!acknowledged || ack.session != session || ack.protocol_version != device_wire6::PROTOCOL_VERSION ||
        ack.status != 1 ||
        ack.actuator_count != actuator_count || ack.device_tick_hz == 0 ||
        ack.step_tick_hz == 0 || ack.segment_capacity == 0 || ack.max_degree > 5 ||
        (ack.profile != 1 && ack.profile != 2) ||
        (ack.profile == 2 && ack.max_degree != 1)) {
        if (error) *error = RK_ERROR_UNSUPPORTED;
        return {};
    }
    const auto period_ns = blueprint.owner_period_ns ? blueprint.owner_period_ns : 10'000'000ULL;
    const auto allowance_ns = blueprint.serial_processing_allowance_ns
        ? blueprint.serial_processing_allowance_ns : 2'000'000ULL;
    const auto required_baud = minimum_baud(ack.actuator_count, period_ns, allowance_ns);
    const auto required_depth = minimum_queue_depth(transport->baud(), ack.actuator_count,
        period_ns, link_latency_ns, clock_bound_ns);
    const auto segment_bytes = device_frame6::HEADER_SIZE + device_frame6::CRC_SIZE +
        device_wire6::Segment6Header::SIZE +
        ack.actuator_count * device_wire6::Segment6Coefficients::SIZE;
    const auto minimum_period_ns = transport->baud() == 0 ? UINT64_MAX :
        allowance_ns + static_cast<std::uint64_t>(std::ceil(
            10.0L * segment_bytes * 1e9L / transport->baud()));
    if (transport->baud() < required_baud || ack.segment_capacity < required_depth) {
        std::fprintf(stderr, "Rkd6Endpoint: unqualified serial link: minimum_baud=%llu "
            "minimum_queue_depth=%u minimum_period_ns=%llu configured_period_ns=%llu configured_baud=%u configured_queue_depth=%u\n",
            static_cast<unsigned long long>(required_baud), required_depth,
            static_cast<unsigned long long>(minimum_period_ns),
            static_cast<unsigned long long>(period_ns),
            transport->baud(), ack.segment_capacity);
        if (error) *error = RK_ERROR_UNSUPPORTED;
        return {};
    }
    auto endpoint = std::shared_ptr<Rkd6Endpoint>(new Rkd6Endpoint(
        std::move(transport), ack, target_error, clock_bound_ns, link_latency_ns,
        std::vector<DeviceActuator6>(layout.begin(), layout.end()), blueprint.joint_count));
    endpoint->owner_period_ns_ = period_ns;
    if (error) *error = RK_OK;
    return endpoint;
}

bool Rkd6Endpoint::send_record(std::uint8_t kind, std::span<const std::uint8_t> payload) {
    std::vector<std::uint8_t> frame;
    return device_frame6::encode(kind, payload, frame) && transport_->send(frame);
}

bool Rkd6Endpoint::send_segment(const DeviceSegment6 &segment) {
    std::vector<std::uint8_t> body(segment.header.SIZE +
        segment.coefficients.size() * device_wire6::Segment6Coefficients::SIZE);
    if (!device_wire6::encode(segment.header, std::span(body).first(segment.header.SIZE)))
        return false;
    for (std::size_t i = 0; i < segment.coefficients.size(); ++i)
        if (!device_wire6::encode(segment.coefficients[i], std::span(body).subspan(
            segment.header.SIZE + i * device_wire6::Segment6Coefficients::SIZE,
            device_wire6::Segment6Coefficients::SIZE))) return false;
    return send_record(6, body);
}

bool Rkd6Endpoint::send_commit(std::uint64_t through_ticks) {
    device_wire6::Commit6 commit{through_ticks};
    std::array<std::uint8_t, device_wire6::Commit6::SIZE> body{};
    if (!device_wire6::encode(commit, body) || !send_record(7, body)) return false;
    committed_until_ticks_ = through_ticks;
    return true;
}

rk_result Rkd6Endpoint::submit_device_plan(const rk_plan_submission &plan,
    std::uint64_t base_time_ns, std::uint64_t owner_now_ns,
    std::uint64_t committed_through_ns, const rk_robot_runtime_blueprint &blueprint) {
    if (!clock_.may_commit()) return RK_ERROR_INVALID_STATE;
    if (base_time_ns == 0 && plan.replace_after_plan_id == 0 &&
        status_.executing_plan_id == 0 &&
        status_.remaining_segments == ack_.segment_capacity && pending_.empty()) {
        epoch_set_ = false;
        sent_.clear();
        next_commit_ = 0;
        committed_until_ticks_ = 0;
    }
    if (!epoch_set_) {
        const auto frame_bytes = device_frame6::HEADER_SIZE + device_frame6::CRC_SIZE +
            device_wire6::Segment6Header::SIZE +
            ack_.actuator_count * device_wire6::Segment6Coefficients::SIZE;
        const auto startup_bytes = 512u +
            std::min<std::uint32_t>(plan.segments.segment_count,
                ack_.segment_capacity) * frame_bytes;
        const auto startup_ns = static_cast<std::uint64_t>(std::ceil(
            10.0L * startup_bytes * 1e9L / transport_->baud()));
        const auto delay = link_latency_ns_ + 2 * clock_.uncertainty_ns() +
            (blueprint.owner_period_ns ? blueprint.owner_period_ns : 10'000'000ULL) +
            startup_ns;
        host_epoch_ns_ = owner_now_ns + delay;
        device_epoch_ticks_ = clock_.map_host_ns(host_epoch_ns_);
        epoch_set_ = true;
    }
    if (host_epoch_ns_ > UINT64_MAX - base_time_ns) return RK_ERROR_LIMIT;
    const auto replace_ticks = clock_.map_host_ns(host_epoch_ns_ + base_time_ns);
    if (plan.replace_after_plan_id && replace_ticks < committed_until_ticks_)
        return RK_ERROR_INVALID_STATE;
    if (plan.replace_after_plan_id) {
        const auto received_until = sent_.empty() ? 0 :
            sent_.back().header.t0_ticks + sent_.back().header.duration_ticks;
        // A reported commit proves that its segment boundary reached the
        // device. Transport send completion alone does not prove reception.
        if (replace_ticks > received_until ||
            replace_ticks > status_.committed_until_ticks ||
            std::none_of(sent_.begin(), sent_.end(), [&](const auto &segment) {
                return segment.header.t0_ticks + segment.header.duration_ticks == replace_ticks;
            })) return RK_ERROR_INVALID_STATE;
    }
    auto compiled = compile_device_segments6(
        std::span(plan.segments.segments, plan.segments.segment_count), plan.plan_id,
        plan.ends_at_rest != 0, host_epoch_ns_ + base_time_ns, clock_, blueprint,
        ack_.device_tick_hz, ack_.step_tick_hz, ack_.max_degree, target_error_, layout_);
    if (!compiled.ok) return RK_ERROR_LIMIT;
    const auto event_count = plan.struct_size >= sizeof(plan) ? plan.event_count : 0u;
    if (event_count > ack_.event_capacity || event_count > RK_MAX_PLAN_EVENTS)
        return RK_ERROR_LIMIT;
    std::vector<device_wire6::Event6> wire_events;
    wire_events.reserve(event_count);
    for (std::uint32_t i = 0; i < event_count; ++i) {
        const auto &source = plan.events[i];
        if (base_time_ns > UINT64_MAX - source.time_ns ||
            host_epoch_ns_ > UINT64_MAX - base_time_ns - source.time_ns ||
            !std::isfinite(source.value.analog) || !std::isfinite(source.value.argument) ||
            std::abs(source.value.analog) > std::numeric_limits<float>::max() ||
            std::abs(source.value.argument) > std::numeric_limits<float>::max())
            return RK_ERROR_LIMIT;
        std::uint32_t channel = blueprint.channel_count;
        for (std::uint32_t j = 0; j < blueprint.channel_count; ++j)
            if (std::strcmp(source.channel, blueprint.channels[j].id) == 0) {
                channel = j; break;
            }
        if (channel == blueprint.channel_count ||
            source.value.kind != blueprint.channels[channel].kind) return RK_ERROR_INVALID_ARGUMENT;
        device_wire6::Event6 event{};
        event.plan_id = plan.plan_id;
        event.path_ticks = clock_.map_host_ns(host_epoch_ns_ + base_time_ns + source.time_ns);
        event.channel = static_cast<std::uint8_t>(channel);
        event.kind = static_cast<std::uint8_t>(source.value.kind);
        event.hold_policy = static_cast<std::uint8_t>(source.hold_policy);
        event.digital = static_cast<std::uint8_t>(source.value.digital);
        event.analog = static_cast<float>(source.value.analog);
        event.argument = static_cast<float>(source.value.argument);
        std::memcpy(event.command.data(), source.value.command, RK_PROCESS_COMMAND_BYTES);
        wire_events.push_back(event);
    }
    const auto first_host_duration = ack_.profile == 2
        ? std::min<std::uint64_t>(owner_period_ns_, plan.segments.segments[0].duration_ns)
        : plan.segments.segments[0].duration_ns;
    const auto path_rate = static_cast<double>(compiled.segments.front().header.duration_ticks) /
        static_cast<double>(first_host_duration);
    if (!std::isfinite(path_rate) || path_rate <= 0) return RK_ERROR_LIMIT;
    std::uint64_t shortest_ns = UINT64_MAX;
    for (const auto &segment : compiled.segments) {
        const auto duration_ns = static_cast<std::uint64_t>(std::ceil(
            static_cast<long double>(segment.header.duration_ticks) * 1e9L /
            ack_.device_tick_hz));
        shortest_ns = std::min(shortest_ns, duration_ns);
    }
    const auto allowance_ns = blueprint.serial_processing_allowance_ns
        ? blueprint.serial_processing_allowance_ns : 2'000'000ULL;
    const auto required_baud = minimum_baud(ack_.actuator_count, shortest_ns, allowance_ns);
    const auto required_depth = minimum_queue_depth(transport_->baud(), ack_.actuator_count,
        shortest_ns, link_latency_ns_, clock_.uncertainty_ns());
    if (transport_->baud() < required_baud || ack_.segment_capacity < required_depth) {
        std::fprintf(stderr, "Rkd6Endpoint: plan exceeds serial qualification: "
            "minimum_baud=%llu minimum_queue_depth=%u\n",
            static_cast<unsigned long long>(required_baud), required_depth);
        return RK_ERROR_LIMIT;
    }
    device_wire6::QueueBegin6 begin{};
    begin.queue_revision = ++revision_;
    revision_boundary_ticks_ = replace_ticks;
    begin.replace_after_ticks = replace_ticks;
    begin.actuator_count = ack_.actuator_count;
    for (std::size_t i = 0; i < ack_.actuator_count; ++i) {
        const auto mapping = layout_.empty() ? DeviceActuator6{static_cast<std::uint8_t>(i)} : layout_[i];
        begin.expected_position[i] = static_cast<float>(mapping.ratio *
            (plan.start_position[mapping.joint] - mapping.offset));
        begin.expected_velocity[i] = static_cast<float>(mapping.ratio * plan.start_velocity[mapping.joint]);
    }
    // A continuation starts at the previous wire polynomial's endpoint.
    // Re-evaluate that f32 polynomial exactly as the device does: converting
    // an independently rounded joint anchor can exceed its 1e-4 actuator
    // check even when both chunks meet the target-error bound.
    auto anchor_from = [&](const auto &segments) {
        for (const auto &segment : segments) {
            if (segment.header.t0_ticks + segment.header.duration_ticks != replace_ticks)
                continue;
            const auto tau = static_cast<float>(segment.header.duration_ticks) /
                static_cast<float>(ack_.device_tick_hz);
            const auto degree = static_cast<std::size_t>(segment.header.degree);
            for (std::size_t i = 0; i < ack_.actuator_count; ++i) {
                const auto &row = segment.coefficients[i];
                const float c[]{row.c0, row.c1, row.c2, row.c3, row.c4, row.c5};
                float position = c[degree];
                for (std::size_t k = degree; k-- > 0;)
                    position = position * tau + c[k];
                float velocity = 0.0f;
                if (degree != 0) {
                    velocity = static_cast<float>(degree) * c[degree];
                    for (std::size_t k = degree; k-- > 1;)
                        velocity = velocity * tau + static_cast<float>(k) * c[k];
                }
                begin.expected_position[i] = position;
                begin.expected_velocity[i] = segment.header.ends_at_rest ? 0.0f : velocity;
            }
            return true;
        }
        return false;
    };
    if (!anchor_from(sent_)) anchor_from(pending_);
    std::array<std::uint8_t, device_wire6::QueueBegin6::SIZE> body{};
    if (!device_wire6::encode(begin, body) || !send_record(5, body)) return RK_ERROR_BACKEND;
    for (auto &event : wire_events) {
        event.queue_revision = revision_;
        std::array<std::uint8_t, device_wire6::Event6::SIZE> event_body{};
        if (!device_wire6::encode(event, event_body) || !send_record(16, event_body))
            return RK_ERROR_BACKEND;
    }
    path_maps_.push_back({compiled.segments.front().header.t0_ticks, base_time_ns, path_rate});
    if (plan.replace_after_plan_id) {
        while (!plan_tags_.empty() && plan_tags_.back().start_ticks >= replace_ticks)
            plan_tags_.pop_back();
        if (!plan_tags_.empty() && plan_tags_.back().end_ticks > replace_ticks) {
            auto &previous = plan_tags_.back();
            previous.duration_ns = static_cast<std::uint64_t>(std::llround(
                static_cast<long double>(previous.duration_ns) *
                (replace_ticks - previous.start_ticks) /
                (previous.end_ticks - previous.start_ticks)));
            previous.end_ticks = replace_ticks;
        }
        while (!pending_.empty() && pending_.back().header.t0_ticks >= replace_ticks)
            pending_.pop_back();
        while (!sent_.empty() && sent_.back().header.t0_ticks >= replace_ticks)
            sent_.pop_back();
        next_commit_ = std::min(next_commit_, sent_.size());
        for (auto &segment : pending_)
            segment.header.queue_revision = revision_;
    }
    const auto plan_start_ticks = compiled.segments.front().header.t0_ticks;
    const auto plan_end_ticks = compiled.segments.back().header.t0_ticks +
        compiled.segments.back().header.duration_ticks;
    for (auto &segment : compiled.segments) {
        segment.header.queue_revision = revision_;
        pending_.push_back(std::move(segment));
    }
    const auto &last_host_segment = plan.segments.segments[plan.segments.segment_count - 1];
    plan_tags_.push_back({plan.plan_id, plan_start_ticks, plan_end_ticks,
        last_host_segment.time_from_start_ns + last_host_segment.duration_ns});
    const auto occupied = std::count_if(sent_.begin(), sent_.end(), [&](const auto &row) {
        return row.header.t0_ticks + row.header.duration_ticks >= status_.path_clock_ticks;
    });
    std::size_t available = ack_.segment_capacity > occupied ? ack_.segment_capacity - occupied : 0;
    while (available > 0 && !pending_.empty()) {
        if (!send_segment(pending_.front())) return RK_ERROR_BACKEND;
        sent_.push_back(std::move(pending_.front()));
        pending_.pop_front();
        --available;
    }
    if (next_commit_ < sent_.size()) {
        const auto target = clock_.map_host_ns(host_epoch_ns_ + committed_through_ns);
        std::size_t chosen = next_commit_;
        while (chosen + 1 < sent_.size() &&
               sent_[chosen + 1].header.t0_ticks + sent_[chosen + 1].header.duration_ticks <= target)
            ++chosen;
        const auto end = sent_[chosen].header.t0_ticks + sent_[chosen].header.duration_ticks;
        if (!send_commit(end)) return RK_ERROR_BACKEND;
        next_commit_ = chosen + 1;
    }
    return RK_OK;
}

rk_result Rkd6Endpoint::apply(const rk_robot_command &command) {
    std::uint8_t kind = 0;
    switch (command.kind) {
        case RK_COMMAND_HOLD: kind = 8; break;
        case RK_COMMAND_RESUME: kind = 9; break;
        case RK_COMMAND_ABORT: kind = 10; break;
        case RK_COMMAND_STOP: kind = 11; break;
        case RK_COMMAND_EMERGENCY_STOP: kind = 12; break;
        case RK_COMMAND_RESET_SAFETY: kind = 13; break;
        case RK_COMMAND_NONE: return RK_OK;
        default: return RK_ERROR_UNSUPPORTED;
    }
    return send_record(kind, {}) ? RK_OK : RK_ERROR_BACKEND;
}

void Rkd6Endpoint::poll_frames(std::uint64_t owner_now_ns) {
    std::vector<std::uint8_t> frame;
    while (transport_->receive(frame)) {
        device_frame6::Frame decoded{};
        if (!device_frame6::decode(frame, decoded)) continue;
        if (decoded.kind == 4) {
            device_wire6::TimeSyncReply reply{};
            if (device_wire6::decode(decoded.payload, reply))
                clock_.observe(reply.host_send_ns,
                    transport_->received_at_ns() ? transport_->received_at_ns() : owner_now_ns,
                    reply.device_rx_ticks, reply.device_tx_ticks);
        } else if (decoded.kind == 14) {
            if (device_wire6::decode(decoded.payload, status_)) {
                queue_revision_mismatch_ = status_.queue_revision > revision_ ||
                    (revision_ != 0 && status_.queue_revision < revision_ &&
                     status_.path_clock_ticks >= revision_boundary_ticks_);
                std::size_t retired = 0;
                while (retired < sent_.size() &&
                       sent_[retired].header.t0_ticks +
                           sent_[retired].header.duration_ticks < status_.path_clock_ticks)
                    ++retired;
                if (retired != 0) {
                    sent_.erase(sent_.begin(), sent_.begin() + retired);
                    next_commit_ = next_commit_ > retired ? next_commit_ - retired : 0;
                }
                while (path_maps_.size() > 1 &&
                       path_maps_[1].device_start_ticks <= status_.path_clock_ticks)
                    path_maps_.erase(path_maps_.begin());
                while (plan_tags_.size() > 1 &&
                       plan_tags_[1].start_ticks <= state_header_.path_clock_ticks)
                    plan_tags_.erase(plan_tags_.begin());
            }
        } else if (decoded.kind == 15) {
            if (decoded.payload.size() < device_wire6::State6Header::SIZE ||
                !device_wire6::decode(decoded.payload.first(device_wire6::State6Header::SIZE),
                    state_header_)) continue;
            for (std::size_t i = 0; i < state_header_.actuator_count; ++i)
                device_wire6::decode(decoded.payload.subspan(device_wire6::State6Header::SIZE +
                    i * device_wire6::ActuatorState6::SIZE,
                    device_wire6::ActuatorState6::SIZE), actuators_[i]);
            has_state_ = true;
        }
    }
}

void Rkd6Endpoint::pump_queue() {
    if (!clock_.may_commit()) return;
    const auto occupied = std::count_if(sent_.begin(), sent_.end(), [&](const auto &row) {
        return row.header.t0_ticks + row.header.duration_ticks >= status_.path_clock_ticks;
    });
    auto available = ack_.segment_capacity > occupied ? ack_.segment_capacity - occupied : 0;
    while (available > 0 && !pending_.empty()) {
        if (!send_segment(pending_.front())) return;
        sent_.push_back(std::move(pending_.front()));
        pending_.pop_front();
        --available;
    }
    if (next_commit_ < sent_.size() &&
        status_.path_clock_ticks + static_cast<std::uint64_t>(
            (link_latency_ns_ + 2 * clock_.uncertainty_ns() +
                2 * owner_period_ns_) *
            static_cast<double>(ack_.device_tick_hz) / 1e9) >= committed_until_ticks_) {
        const auto &segment = sent_.back();
        if (send_commit(segment.header.t0_ticks + segment.header.duration_ticks))
            next_commit_ = sent_.size();
    }
}

rk_result Rkd6Endpoint::sample(std::uint64_t timestamp_ns, rk_robot_state &state) {
    if (clock_.sync_due(timestamp_ns, 100'000'000)) {
        device_wire6::TimeSyncRequest request{timestamp_ns};
        std::array<std::uint8_t, request.SIZE> body{};
        if (!device_wire6::encode(request, body) || !send_record(3, body)) return RK_ERROR_BACKEND;
        clock_.note_sync_request(timestamp_ns);
    }
    poll_frames(timestamp_ns);
    pump_queue();
    if (!has_state_) return RK_ERROR_STALE_STATE;
    state.struct_size = sizeof(state);
    state.joint_count = joint_count_;
    state.source_timestamp_ns = static_cast<std::uint64_t>(
        state_header_.timestamp_ticks * (1e9 / ack_.device_tick_hz));
    state.safety = status_.fault ? RK_SAFETY_FAULT :
        static_cast<rk_safety_state>(state_header_.safety);
    for (std::size_t joint = 0; joint < joint_count_; ++joint)
        state.effort[joint] = 0.0;
    for (std::size_t i = 0; i < state_header_.actuator_count; ++i) {
        const auto mapping = layout_.empty() ? DeviceActuator6{static_cast<std::uint8_t>(i)} : layout_[i];
        state.position[mapping.joint] = actuators_[i].position / mapping.ratio + mapping.offset;
        state.velocity[mapping.joint] = actuators_[i].velocity / mapping.ratio;
        state.effort[mapping.joint] += actuators_[i].effort * mapping.ratio;
    }
    state.trajectory_queue_depth = ack_.segment_capacity - status_.remaining_segments;
    // Motion is still to come while segments wait to be sent, or were sent
    // and end after the device's path clock: the device reports its own
    // queue only, so a queue it has not received or started yet counts too.
    state.trajectory_active = status_.executing_plan_id != 0 || state.trajectory_queue_depth != 0 ||
        !pending_.empty() || (!sent_.empty() &&
            sent_.back().header.t0_ticks + sent_.back().header.duration_ticks > status_.path_clock_ticks);
    state.trajectory_time_ns = state.trajectory_active
        ? path_time_ns(state_header_.path_clock_ticks) : 0;
    state.active_plan_id = status_.executing_plan_id;
    state.committed_until_ns = path_time_ns(committed_until_ticks_);
    state.trajectory_duration_ns = state.trajectory_active
        ? std::max(state.trajectory_time_ns, state.committed_until_ns) : 0;
    if (!plan_tags_.empty()) {
        const auto path_ticks = state_header_.path_clock_ticks;
        const PlanTag *tag = nullptr;
        for (const auto &candidate : plan_tags_) {
            if (candidate.start_ticks > path_ticks) break;
            tag = &candidate;
        }
        if (tag) {
            state.trajectory_tag = tag->plan_id;
            const auto elapsed_ticks = std::min(path_ticks, tag->end_ticks) - tag->start_ticks;
            const auto total_ticks = tag->end_ticks - tag->start_ticks;
            state.trajectory_tag_time_ns = total_ticks == 0 ? 0 :
                static_cast<std::uint64_t>(std::llround(
                    static_cast<long double>(tag->duration_ns) * elapsed_ticks / total_ticks));
        }
    }
    state.session_state = status_.rate == 0.0f ? RK_SESSION_HELD :
        (state.trajectory_active ? RK_SESSION_EXECUTING : RK_SESSION_IDLE);
    return RK_OK;
}

std::uint64_t Rkd6Endpoint::path_time_ns(std::uint64_t device_ticks) const noexcept {
    for (auto it = path_maps_.rbegin(); it != path_maps_.rend(); ++it) {
        if (device_ticks >= it->device_start_ticks) {
            const auto elapsed = static_cast<long double>(device_ticks - it->device_start_ticks) /
                it->ticks_per_host_ns;
            if (elapsed >= static_cast<long double>(UINT64_MAX - it->host_path_start_ns))
                return UINT64_MAX;
            return it->host_path_start_ns + static_cast<std::uint64_t>(std::llround(elapsed));
        }
    }
    return 0;
}

} // namespace robotkit
