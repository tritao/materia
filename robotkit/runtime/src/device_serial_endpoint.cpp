#include "robotkit_device_serial_endpoint.hpp"

#include <array>
#include <cmath>
#include <cstdio>

namespace robotkit {

std::shared_ptr<DeviceSerialEndpoint> DeviceSerialEndpoint::open(const char *path, unsigned baud,
    std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count,
    double max_target_error, std::uint8_t *session_status,
    std::chrono::nanoseconds owner_period,
    std::chrono::nanoseconds processing_allowance) {
    if (session_status) *session_status = 0;
    if (!std::isfinite(max_target_error) || max_target_error < 0.0 ||
        joint_count > device_wire::MAX_JOINTS) return {};
    const auto timing_error = qualification_error(baud, joint_count, owner_period,
        processing_allowance);
    if (!timing_error.empty()) {
        std::fprintf(stderr, "%s\n", timing_error.c_str());
        return {};
    }
    return attach(device::HostLink::open(path, baud, fingerprint, joint_count, session_status),
        joint_count, max_target_error, owner_period, processing_allowance);
}

std::shared_ptr<DeviceSerialEndpoint> DeviceSerialEndpoint::attach(
    std::unique_ptr<device::HostLink> link, std::uint8_t joint_count, double max_target_error,
    std::chrono::nanoseconds owner_period,
    std::chrono::nanoseconds processing_allowance) {
    if (!link || !link->ready() || joint_count > device_wire::MAX_JOINTS ||
        !std::isfinite(max_target_error) || max_target_error < 0.0) return {};
    const auto timing_error = qualification_error(link->baud(), joint_count, owner_period,
        processing_allowance);
    if (!timing_error.empty()) {
        std::fprintf(stderr, "%s\n", timing_error.c_str());
        return {};
    }
    device::HostState initial{};
    if (!link->read_state(initial) || initial.header.accepted_sequence != 0 ||
        (initial.header.safety != RK_SAFETY_EMERGENCY_STOP &&
         initial.header.safety != RK_SAFETY_FAULT) ||
        initial.header.joint_count != joint_count) return {};
    return std::shared_ptr<DeviceSerialEndpoint>(new DeviceSerialEndpoint(
        std::move(link), joint_count, max_target_error, initial, owner_period,
        processing_allowance));
}

std::uint64_t DeviceSerialEndpoint::command_frame_time_ns(unsigned baud,
    std::uint8_t joint_count) {
    if (baud == 0) return 0;
    const std::uint64_t frame_bytes = 12 + device_wire::CommandHeader_SIZE +
        static_cast<std::uint64_t>(joint_count) * device_wire::JointTarget_SIZE;
    return (frame_bytes * 10'000'000'000ULL + baud - 1) / baud;
}

std::uint64_t DeviceSerialEndpoint::minimum_owner_period_ns(unsigned baud,
    std::uint8_t joint_count, std::chrono::nanoseconds processing_allowance) {
    if (baud == 0 || processing_allowance.count() < 0) return 0;
    return command_frame_time_ns(baud, joint_count) +
        static_cast<std::uint64_t>(processing_allowance.count());
}

std::string DeviceSerialEndpoint::qualification_error(unsigned baud,
    std::uint8_t joint_count, std::chrono::nanoseconds owner_period,
    std::chrono::nanoseconds processing_allowance) {
    const auto frame_ns = command_frame_time_ns(baud, joint_count);
    const auto minimum_ns = minimum_owner_period_ns(baud, joint_count,
        processing_allowance);
    if (baud != 0 && processing_allowance.count() >= 0 &&
        owner_period.count() > 0 &&
        static_cast<std::uint64_t>(owner_period.count()) >= minimum_ns)
        return {};
    return "DeviceSerialEndpoint: unsupported serial queue timing: baud=" +
        std::to_string(baud) + " joint_count=" + std::to_string(joint_count) +
        " frame_time_ns=" + std::to_string(frame_ns) +
        " processing_allowance_ns=" +
        std::to_string(processing_allowance.count()) +
        " minimum_owner_period_ns=" + std::to_string(minimum_ns) +
        " configured_owner_period_ns=" + std::to_string(owner_period.count());
}

DeviceSerialEndpoint::DeviceSerialEndpoint(std::unique_ptr<device::HostLink> link,
    std::uint8_t joint_count, double max_target_error, device::HostState initial_state,
    std::chrono::nanoseconds owner_period,
    std::chrono::nanoseconds processing_allowance)
    : link_(std::move(link)), joint_count_(joint_count), max_target_error_(max_target_error),
      initial_state_(initial_state) {
    (void)owner_period;
    (void)processing_allowance;
}

rk_result DeviceSerialEndpoint::apply(const rk_robot_command &command) {
    if (!link_ || !link_->ready()) return RK_ERROR_BACKEND;
    if (command.struct_size != sizeof(command)) return RK_ERROR_INVALID_ARGUMENT;
    if (command.sequence == 0 || command.sequence <= last_command_sequence_)
        return RK_ERROR_STALE_COMMAND;
    if (command.target_count > joint_count_) return RK_ERROR_LIMIT;
    bool sent = false;
    if (command.kind == RK_COMMAND_JOINT_TARGETS) {
        if (command.target_count == 0) return RK_ERROR_INVALID_ARGUMENT;
        std::array<device::HostTarget, device_wire::MAX_JOINTS> targets{};
        for (std::size_t index = 0; index < command.target_count; ++index) {
            const auto &source = command.targets[index];
            if (source.joint >= joint_count_ || source.mode < RK_TARGET_POSITION ||
                source.mode > RK_TARGET_EFFORT) return RK_ERROR_LIMIT;
            targets[index] = {static_cast<std::uint16_t>(source.joint),
                static_cast<std::uint8_t>(source.mode), source.target};
        }
        sent = link_->send_targets(std::span(targets).first(command.target_count), max_target_error_);
    } else {
        if (command.target_count != 0) return RK_ERROR_INVALID_ARGUMENT;
        // A runtime no-op heartbeat has no device command kind. A normal stop is safe and
        // gives the device an accepted command to refresh its watchdog.
        const auto kind = command.kind == RK_COMMAND_NONE ? RK_COMMAND_STOP : command.kind;
        if (kind < RK_COMMAND_STOP || kind > RK_COMMAND_RESET_SAFETY)
            return RK_ERROR_UNSUPPORTED;
        sent = link_->send_command(static_cast<std::uint8_t>(kind));
    }
    if (!sent) return RK_ERROR_BACKEND;
    last_command_sequence_ = command.sequence;
    has_initial_state_ = false;
    return RK_OK;
}

rk_result DeviceSerialEndpoint::sample(std::uint64_t, rk_robot_state &state) {
    if (!link_ || !link_->ready()) return RK_ERROR_BACKEND;
    device::HostState observed{};
    if (has_initial_state_) {
        observed = initial_state_;
        has_initial_state_ = false;
    } else if (!link_->read_state(observed)) return RK_ERROR_BACKEND;
    state = {};
    state.struct_size = sizeof(state);
    state.source_timestamp_ns = observed.header.timestamp_ns;
    state.safety = observed.header.fault ? RK_SAFETY_FAULT : observed.header.safety;
    state.joint_count = observed.header.joint_count;
    for (std::size_t index = 0; index < state.joint_count; ++index) {
        state.position[index] = observed.joints[index].position;
        state.velocity[index] = observed.joints[index].velocity;
        state.effort[index] = observed.joints[index].effort;
    }
    return RK_OK;
}

} // namespace robotkit
