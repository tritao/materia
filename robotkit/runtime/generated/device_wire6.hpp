#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace robotkit::device_wire6 {

inline constexpr std::uint8_t PROTOCOL_VERSION = 14;
inline constexpr std::uint8_t MAX_ACTUATORS = 64;

enum class MessageType6 : std::uint8_t {
    session_begin6 = 1,
    session_ack6 = 2,
    time_sync_request = 3,
    time_sync_reply = 4,
    queue_begin = 5,
    segment = 6,
    commit = 7,
    hold = 8,
    resume = 9,
    abort = 10,
    stop = 11,
    emergency_stop = 12,
    reset_safety = 13,
    queue_status = 14,
    state6 = 15,
    event = 16,
    sensor6 = 17,
    homing_scope = 21,
    homing_side = 22,
    homing_control_ack = 23,
    homing_counter_batch = 24,
};

inline constexpr std::size_t SessionBegin6_SIZE = 5013;
struct SessionBegin6 {
    static constexpr std::size_t SIZE = SessionBegin6_SIZE;
    std::uint64_t session{};
    std::uint8_t protocol_version{};
    std::array<std::uint8_t, 16> expected_controller{};
    std::uint8_t actuator_count{};
    std::uint8_t max_degree{};
    std::uint32_t step_tick_hz{};
    float max_acceleration{};
    std::array<float, 64> actuator_max_acceleration{};
    std::array<float, 64> steps_per_unit{};
    std::array<std::uint32_t, 64> min_step_ticks{};
    std::array<std::uint16_t, 64> direction_setup_ticks{};
    std::array<std::uint8_t, 64> actuator_joint{};
    std::array<float, 64> actuator_ratio{};
    std::array<float, 64> dual_drive_skew_bound{};
    std::uint64_t link_loss_timeout_ns{};
    std::uint8_t channel_count{};
    std::array<std::uint8_t, 1536> channel_id{};
    std::array<std::uint8_t, 32> channel_kind{};
    std::array<std::uint8_t, 32> safe_digital{};
    std::array<float, 32> safe_analog{};
    std::array<float, 32> safe_argument{};
    std::array<std::uint8_t, 1536> safe_command{};
    std::array<std::uint8_t, 32> channel_stop_policy{};
    std::uint8_t input_count{};
    std::array<std::uint8_t, 64> input_actuator{};
    std::uint64_t input_active_high{};
};

inline bool encode(const SessionBegin6 &value, std::span<std::uint8_t> out) {
    if (out.size() < SessionBegin6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint8_t bits_protocol_version = static_cast<std::uint8_t>(value.protocol_version);
    out[offset++] = static_cast<std::uint8_t>(bits_protocol_version >> 0);
    for (std::size_t i = 0; i < 16; ++i) {
        const std::uint8_t bits_expected_controller = static_cast<std::uint8_t>(value.expected_controller[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_controller >> 0);
    }
    const std::uint8_t bits_actuator_count = static_cast<std::uint8_t>(value.actuator_count);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator_count >> 0);
    const std::uint8_t bits_max_degree = static_cast<std::uint8_t>(value.max_degree);
    out[offset++] = static_cast<std::uint8_t>(bits_max_degree >> 0);
    const std::uint32_t bits_step_tick_hz = static_cast<std::uint32_t>(value.step_tick_hz);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 24);
    const std::uint32_t bits_max_acceleration = std::bit_cast<std::uint32_t>(value.max_acceleration);
    out[offset++] = static_cast<std::uint8_t>(bits_max_acceleration >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_max_acceleration >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_max_acceleration >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_max_acceleration >> 24);
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_actuator_max_acceleration = std::bit_cast<std::uint32_t>(value.actuator_max_acceleration[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_max_acceleration >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_max_acceleration >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_max_acceleration >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_max_acceleration >> 24);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_steps_per_unit = std::bit_cast<std::uint32_t>(value.steps_per_unit[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_steps_per_unit >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_steps_per_unit >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_steps_per_unit >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_steps_per_unit >> 24);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_min_step_ticks = static_cast<std::uint32_t>(value.min_step_ticks[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_min_step_ticks >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_min_step_ticks >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_min_step_ticks >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_min_step_ticks >> 24);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint16_t bits_direction_setup_ticks = static_cast<std::uint16_t>(value.direction_setup_ticks[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_direction_setup_ticks >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_direction_setup_ticks >> 8);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint8_t bits_actuator_joint = static_cast<std::uint8_t>(value.actuator_joint[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_joint >> 0);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_actuator_ratio = std::bit_cast<std::uint32_t>(value.actuator_ratio[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_ratio >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_ratio >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_ratio >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_actuator_ratio >> 24);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_dual_drive_skew_bound = std::bit_cast<std::uint32_t>(value.dual_drive_skew_bound[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_dual_drive_skew_bound >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_dual_drive_skew_bound >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_dual_drive_skew_bound >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_dual_drive_skew_bound >> 24);
    }
    const std::uint64_t bits_link_loss_timeout_ns = static_cast<std::uint64_t>(value.link_loss_timeout_ns);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_link_loss_timeout_ns >> 56);
    const std::uint8_t bits_channel_count = static_cast<std::uint8_t>(value.channel_count);
    out[offset++] = static_cast<std::uint8_t>(bits_channel_count >> 0);
    for (std::size_t i = 0; i < 1536; ++i) {
        const std::uint8_t bits_channel_id = static_cast<std::uint8_t>(value.channel_id[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_channel_id >> 0);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        const std::uint8_t bits_channel_kind = static_cast<std::uint8_t>(value.channel_kind[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_channel_kind >> 0);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        const std::uint8_t bits_safe_digital = static_cast<std::uint8_t>(value.safe_digital[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_digital >> 0);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        const std::uint32_t bits_safe_analog = std::bit_cast<std::uint32_t>(value.safe_analog[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_analog >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_analog >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_analog >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_analog >> 24);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        const std::uint32_t bits_safe_argument = std::bit_cast<std::uint32_t>(value.safe_argument[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_argument >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_argument >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_argument >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_argument >> 24);
    }
    for (std::size_t i = 0; i < 1536; ++i) {
        const std::uint8_t bits_safe_command = static_cast<std::uint8_t>(value.safe_command[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_safe_command >> 0);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        const std::uint8_t bits_channel_stop_policy = static_cast<std::uint8_t>(value.channel_stop_policy[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_channel_stop_policy >> 0);
    }
    const std::uint8_t bits_input_count = static_cast<std::uint8_t>(value.input_count);
    out[offset++] = static_cast<std::uint8_t>(bits_input_count >> 0);
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint8_t bits_input_actuator = static_cast<std::uint8_t>(value.input_actuator[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_input_actuator >> 0);
    }
    const std::uint64_t bits_input_active_high = static_cast<std::uint64_t>(value.input_active_high);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_input_active_high >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, SessionBegin6 &value) {
    if (input.size() != SessionBegin6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint8_t bits_protocol_version = 0;
    bits_protocol_version |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.protocol_version = bits_protocol_version;
    for (std::size_t i = 0; i < 16; ++i) {
        std::uint8_t bits_expected_controller = 0;
        bits_expected_controller |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.expected_controller[i] = bits_expected_controller;
    }
    std::uint8_t bits_actuator_count = 0;
    bits_actuator_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator_count = bits_actuator_count;
    std::uint8_t bits_max_degree = 0;
    bits_max_degree |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.max_degree = bits_max_degree;
    std::uint32_t bits_step_tick_hz = 0;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.step_tick_hz = bits_step_tick_hz;
    std::uint32_t bits_max_acceleration = 0;
    bits_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.max_acceleration = std::bit_cast<float>(bits_max_acceleration);
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_actuator_max_acceleration = 0;
        bits_actuator_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_actuator_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_actuator_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_actuator_max_acceleration |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.actuator_max_acceleration[i] = std::bit_cast<float>(bits_actuator_max_acceleration);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_steps_per_unit = 0;
        bits_steps_per_unit |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_steps_per_unit |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_steps_per_unit |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_steps_per_unit |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.steps_per_unit[i] = std::bit_cast<float>(bits_steps_per_unit);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_min_step_ticks = 0;
        bits_min_step_ticks |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_min_step_ticks |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_min_step_ticks |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_min_step_ticks |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.min_step_ticks[i] = bits_min_step_ticks;
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint16_t bits_direction_setup_ticks = 0;
        bits_direction_setup_ticks |= static_cast<std::uint16_t>(input[offset++]) << 0;
        bits_direction_setup_ticks |= static_cast<std::uint16_t>(input[offset++]) << 8;
        value.direction_setup_ticks[i] = bits_direction_setup_ticks;
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint8_t bits_actuator_joint = 0;
        bits_actuator_joint |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.actuator_joint[i] = bits_actuator_joint;
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_actuator_ratio = 0;
        bits_actuator_ratio |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_actuator_ratio |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_actuator_ratio |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_actuator_ratio |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.actuator_ratio[i] = std::bit_cast<float>(bits_actuator_ratio);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_dual_drive_skew_bound = 0;
        bits_dual_drive_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_dual_drive_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_dual_drive_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_dual_drive_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.dual_drive_skew_bound[i] = std::bit_cast<float>(bits_dual_drive_skew_bound);
    }
    std::uint64_t bits_link_loss_timeout_ns = 0;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_link_loss_timeout_ns |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.link_loss_timeout_ns = bits_link_loss_timeout_ns;
    std::uint8_t bits_channel_count = 0;
    bits_channel_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.channel_count = bits_channel_count;
    for (std::size_t i = 0; i < 1536; ++i) {
        std::uint8_t bits_channel_id = 0;
        bits_channel_id |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.channel_id[i] = bits_channel_id;
    }
    for (std::size_t i = 0; i < 32; ++i) {
        std::uint8_t bits_channel_kind = 0;
        bits_channel_kind |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.channel_kind[i] = bits_channel_kind;
    }
    for (std::size_t i = 0; i < 32; ++i) {
        std::uint8_t bits_safe_digital = 0;
        bits_safe_digital |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.safe_digital[i] = bits_safe_digital;
    }
    for (std::size_t i = 0; i < 32; ++i) {
        std::uint32_t bits_safe_analog = 0;
        bits_safe_analog |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_safe_analog |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_safe_analog |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_safe_analog |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.safe_analog[i] = std::bit_cast<float>(bits_safe_analog);
    }
    for (std::size_t i = 0; i < 32; ++i) {
        std::uint32_t bits_safe_argument = 0;
        bits_safe_argument |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_safe_argument |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_safe_argument |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_safe_argument |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.safe_argument[i] = std::bit_cast<float>(bits_safe_argument);
    }
    for (std::size_t i = 0; i < 1536; ++i) {
        std::uint8_t bits_safe_command = 0;
        bits_safe_command |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.safe_command[i] = bits_safe_command;
    }
    for (std::size_t i = 0; i < 32; ++i) {
        std::uint8_t bits_channel_stop_policy = 0;
        bits_channel_stop_policy |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.channel_stop_policy[i] = bits_channel_stop_policy;
    }
    std::uint8_t bits_input_count = 0;
    bits_input_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.input_count = bits_input_count;
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint8_t bits_input_actuator = 0;
        bits_input_actuator |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.input_actuator[i] = bits_input_actuator;
    }
    std::uint64_t bits_input_active_high = 0;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_input_active_high |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.input_active_high = bits_input_active_high;
    return true;
}

inline constexpr std::size_t Event6_SIZE = 84;
struct Event6 {
    static constexpr std::size_t SIZE = Event6_SIZE;
    std::uint64_t queue_revision{};
    std::uint64_t plan_id{};
    std::uint64_t path_ticks{};
    std::uint8_t channel{};
    std::uint8_t kind{};
    std::uint8_t hold_policy{};
    std::uint8_t digital{};
    float analog{};
    float argument{};
    std::array<std::uint8_t, 48> command{};
};

inline bool encode(const Event6 &value, std::span<std::uint8_t> out) {
    if (out.size() < Event6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_queue_revision = static_cast<std::uint64_t>(value.queue_revision);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 56);
    const std::uint64_t bits_plan_id = static_cast<std::uint64_t>(value.plan_id);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 56);
    const std::uint64_t bits_path_ticks = static_cast<std::uint64_t>(value.path_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_path_ticks >> 56);
    const std::uint8_t bits_channel = static_cast<std::uint8_t>(value.channel);
    out[offset++] = static_cast<std::uint8_t>(bits_channel >> 0);
    const std::uint8_t bits_kind = static_cast<std::uint8_t>(value.kind);
    out[offset++] = static_cast<std::uint8_t>(bits_kind >> 0);
    const std::uint8_t bits_hold_policy = static_cast<std::uint8_t>(value.hold_policy);
    out[offset++] = static_cast<std::uint8_t>(bits_hold_policy >> 0);
    const std::uint8_t bits_digital = static_cast<std::uint8_t>(value.digital);
    out[offset++] = static_cast<std::uint8_t>(bits_digital >> 0);
    const std::uint32_t bits_analog = std::bit_cast<std::uint32_t>(value.analog);
    out[offset++] = static_cast<std::uint8_t>(bits_analog >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_analog >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_analog >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_analog >> 24);
    const std::uint32_t bits_argument = std::bit_cast<std::uint32_t>(value.argument);
    out[offset++] = static_cast<std::uint8_t>(bits_argument >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_argument >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_argument >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_argument >> 24);
    for (std::size_t i = 0; i < 48; ++i) {
        const std::uint8_t bits_command = static_cast<std::uint8_t>(value.command[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_command >> 0);
    }
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Event6 &value) {
    if (input.size() != Event6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_queue_revision = 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.queue_revision = bits_queue_revision;
    std::uint64_t bits_plan_id = 0;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.plan_id = bits_plan_id;
    std::uint64_t bits_path_ticks = 0;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_path_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.path_ticks = bits_path_ticks;
    std::uint8_t bits_channel = 0;
    bits_channel |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.channel = bits_channel;
    std::uint8_t bits_kind = 0;
    bits_kind |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.kind = bits_kind;
    std::uint8_t bits_hold_policy = 0;
    bits_hold_policy |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.hold_policy = bits_hold_policy;
    std::uint8_t bits_digital = 0;
    bits_digital |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.digital = bits_digital;
    std::uint32_t bits_analog = 0;
    bits_analog |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_analog |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_analog |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_analog |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.analog = std::bit_cast<float>(bits_analog);
    std::uint32_t bits_argument = 0;
    bits_argument |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_argument |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_argument |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_argument |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.argument = std::bit_cast<float>(bits_argument);
    for (std::size_t i = 0; i < 48; ++i) {
        std::uint8_t bits_command = 0;
        bits_command |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.command[i] = bits_command;
    }
    return true;
}

inline constexpr std::size_t SessionAck6_SIZE = 53;
struct SessionAck6 {
    static constexpr std::size_t SIZE = SessionAck6_SIZE;
    std::uint64_t session{};
    std::uint8_t protocol_version{};
    std::array<std::uint8_t, 16> controller{};
    std::uint8_t status{};
    std::uint64_t device_tick_hz{};
    std::uint16_t segment_capacity{};
    std::uint16_t event_capacity{};
    std::uint32_t step_tick_hz{};
    std::uint8_t max_degree{};
    std::uint8_t actuator_count{};
    std::uint8_t profile{};
    std::uint64_t config_digest{};
};

inline bool encode(const SessionAck6 &value, std::span<std::uint8_t> out) {
    if (out.size() < SessionAck6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint8_t bits_protocol_version = static_cast<std::uint8_t>(value.protocol_version);
    out[offset++] = static_cast<std::uint8_t>(bits_protocol_version >> 0);
    for (std::size_t i = 0; i < 16; ++i) {
        const std::uint8_t bits_controller = static_cast<std::uint8_t>(value.controller[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_controller >> 0);
    }
    const std::uint8_t bits_status = static_cast<std::uint8_t>(value.status);
    out[offset++] = static_cast<std::uint8_t>(bits_status >> 0);
    const std::uint64_t bits_device_tick_hz = static_cast<std::uint64_t>(value.device_tick_hz);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tick_hz >> 56);
    const std::uint16_t bits_segment_capacity = static_cast<std::uint16_t>(value.segment_capacity);
    out[offset++] = static_cast<std::uint8_t>(bits_segment_capacity >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_segment_capacity >> 8);
    const std::uint16_t bits_event_capacity = static_cast<std::uint16_t>(value.event_capacity);
    out[offset++] = static_cast<std::uint8_t>(bits_event_capacity >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_event_capacity >> 8);
    const std::uint32_t bits_step_tick_hz = static_cast<std::uint32_t>(value.step_tick_hz);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_step_tick_hz >> 24);
    const std::uint8_t bits_max_degree = static_cast<std::uint8_t>(value.max_degree);
    out[offset++] = static_cast<std::uint8_t>(bits_max_degree >> 0);
    const std::uint8_t bits_actuator_count = static_cast<std::uint8_t>(value.actuator_count);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator_count >> 0);
    const std::uint8_t bits_profile = static_cast<std::uint8_t>(value.profile);
    out[offset++] = static_cast<std::uint8_t>(bits_profile >> 0);
    const std::uint64_t bits_config_digest = static_cast<std::uint64_t>(value.config_digest);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_config_digest >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, SessionAck6 &value) {
    if (input.size() != SessionAck6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint8_t bits_protocol_version = 0;
    bits_protocol_version |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.protocol_version = bits_protocol_version;
    for (std::size_t i = 0; i < 16; ++i) {
        std::uint8_t bits_controller = 0;
        bits_controller |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.controller[i] = bits_controller;
    }
    std::uint8_t bits_status = 0;
    bits_status |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.status = bits_status;
    std::uint64_t bits_device_tick_hz = 0;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_device_tick_hz |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.device_tick_hz = bits_device_tick_hz;
    std::uint16_t bits_segment_capacity = 0;
    bits_segment_capacity |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_segment_capacity |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.segment_capacity = bits_segment_capacity;
    std::uint16_t bits_event_capacity = 0;
    bits_event_capacity |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_event_capacity |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.event_capacity = bits_event_capacity;
    std::uint32_t bits_step_tick_hz = 0;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_step_tick_hz |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.step_tick_hz = bits_step_tick_hz;
    std::uint8_t bits_max_degree = 0;
    bits_max_degree |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.max_degree = bits_max_degree;
    std::uint8_t bits_actuator_count = 0;
    bits_actuator_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator_count = bits_actuator_count;
    std::uint8_t bits_profile = 0;
    bits_profile |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.profile = bits_profile;
    std::uint64_t bits_config_digest = 0;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_config_digest |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.config_digest = bits_config_digest;
    return true;
}

inline constexpr std::size_t TimeSyncRequest_SIZE = 8;
struct TimeSyncRequest {
    static constexpr std::size_t SIZE = TimeSyncRequest_SIZE;
    std::uint64_t host_send_ns{};
};

inline bool encode(const TimeSyncRequest &value, std::span<std::uint8_t> out) {
    if (out.size() < TimeSyncRequest_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_host_send_ns = static_cast<std::uint64_t>(value.host_send_ns);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, TimeSyncRequest &value) {
    if (input.size() != TimeSyncRequest_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_host_send_ns = 0;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.host_send_ns = bits_host_send_ns;
    return true;
}

inline constexpr std::size_t TimeSyncReply_SIZE = 24;
struct TimeSyncReply {
    static constexpr std::size_t SIZE = TimeSyncReply_SIZE;
    std::uint64_t host_send_ns{};
    std::uint64_t device_rx_ticks{};
    std::uint64_t device_tx_ticks{};
};

inline bool encode(const TimeSyncReply &value, std::span<std::uint8_t> out) {
    if (out.size() < TimeSyncReply_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_host_send_ns = static_cast<std::uint64_t>(value.host_send_ns);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_host_send_ns >> 56);
    const std::uint64_t bits_device_rx_ticks = static_cast<std::uint64_t>(value.device_rx_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_device_rx_ticks >> 56);
    const std::uint64_t bits_device_tx_ticks = static_cast<std::uint64_t>(value.device_tx_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_device_tx_ticks >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, TimeSyncReply &value) {
    if (input.size() != TimeSyncReply_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_host_send_ns = 0;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_host_send_ns |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.host_send_ns = bits_host_send_ns;
    std::uint64_t bits_device_rx_ticks = 0;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_device_rx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.device_rx_ticks = bits_device_rx_ticks;
    std::uint64_t bits_device_tx_ticks = 0;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_device_tx_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.device_tx_ticks = bits_device_tx_ticks;
    return true;
}

inline constexpr std::size_t QueueBegin6_SIZE = 529;
struct QueueBegin6 {
    static constexpr std::size_t SIZE = QueueBegin6_SIZE;
    std::uint64_t queue_revision{};
    std::uint64_t replace_after_ticks{};
    std::array<float, 64> expected_position{};
    std::array<float, 64> expected_velocity{};
    std::uint8_t actuator_count{};
};

inline bool encode(const QueueBegin6 &value, std::span<std::uint8_t> out) {
    if (out.size() < QueueBegin6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_queue_revision = static_cast<std::uint64_t>(value.queue_revision);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 56);
    const std::uint64_t bits_replace_after_ticks = static_cast<std::uint64_t>(value.replace_after_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_replace_after_ticks >> 56);
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_expected_position = std::bit_cast<std::uint32_t>(value.expected_position[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_position >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_position >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_position >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_position >> 24);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        const std::uint32_t bits_expected_velocity = std::bit_cast<std::uint32_t>(value.expected_velocity[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_velocity >> 0);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_velocity >> 8);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_velocity >> 16);
        out[offset++] = static_cast<std::uint8_t>(bits_expected_velocity >> 24);
    }
    const std::uint8_t bits_actuator_count = static_cast<std::uint8_t>(value.actuator_count);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator_count >> 0);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, QueueBegin6 &value) {
    if (input.size() != QueueBegin6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_queue_revision = 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.queue_revision = bits_queue_revision;
    std::uint64_t bits_replace_after_ticks = 0;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_replace_after_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.replace_after_ticks = bits_replace_after_ticks;
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_expected_position = 0;
        bits_expected_position |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_expected_position |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_expected_position |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_expected_position |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.expected_position[i] = std::bit_cast<float>(bits_expected_position);
    }
    for (std::size_t i = 0; i < 64; ++i) {
        std::uint32_t bits_expected_velocity = 0;
        bits_expected_velocity |= static_cast<std::uint32_t>(input[offset++]) << 0;
        bits_expected_velocity |= static_cast<std::uint32_t>(input[offset++]) << 8;
        bits_expected_velocity |= static_cast<std::uint32_t>(input[offset++]) << 16;
        bits_expected_velocity |= static_cast<std::uint32_t>(input[offset++]) << 24;
        value.expected_velocity[i] = std::bit_cast<float>(bits_expected_velocity);
    }
    std::uint8_t bits_actuator_count = 0;
    bits_actuator_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator_count = bits_actuator_count;
    return true;
}

inline constexpr std::size_t Segment6Header_SIZE = 36;
struct Segment6Header {
    static constexpr std::size_t SIZE = Segment6Header_SIZE;
    std::uint64_t queue_revision{};
    std::uint64_t plan_id{};
    std::uint64_t t0_ticks{};
    std::uint64_t duration_ticks{};
    std::uint8_t degree{};
    std::uint8_t actuator_count{};
    std::uint8_t ends_at_rest{};
    std::uint8_t purpose{};
};

inline bool encode(const Segment6Header &value, std::span<std::uint8_t> out) {
    if (out.size() < Segment6Header_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_queue_revision = static_cast<std::uint64_t>(value.queue_revision);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 56);
    const std::uint64_t bits_plan_id = static_cast<std::uint64_t>(value.plan_id);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_plan_id >> 56);
    const std::uint64_t bits_t0_ticks = static_cast<std::uint64_t>(value.t0_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_t0_ticks >> 56);
    const std::uint64_t bits_duration_ticks = static_cast<std::uint64_t>(value.duration_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_duration_ticks >> 56);
    const std::uint8_t bits_degree = static_cast<std::uint8_t>(value.degree);
    out[offset++] = static_cast<std::uint8_t>(bits_degree >> 0);
    const std::uint8_t bits_actuator_count = static_cast<std::uint8_t>(value.actuator_count);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator_count >> 0);
    const std::uint8_t bits_ends_at_rest = static_cast<std::uint8_t>(value.ends_at_rest);
    out[offset++] = static_cast<std::uint8_t>(bits_ends_at_rest >> 0);
    const std::uint8_t bits_purpose = static_cast<std::uint8_t>(value.purpose);
    out[offset++] = static_cast<std::uint8_t>(bits_purpose >> 0);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Segment6Header &value) {
    if (input.size() != Segment6Header_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_queue_revision = 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.queue_revision = bits_queue_revision;
    std::uint64_t bits_plan_id = 0;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.plan_id = bits_plan_id;
    std::uint64_t bits_t0_ticks = 0;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_t0_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.t0_ticks = bits_t0_ticks;
    std::uint64_t bits_duration_ticks = 0;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_duration_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.duration_ticks = bits_duration_ticks;
    std::uint8_t bits_degree = 0;
    bits_degree |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.degree = bits_degree;
    std::uint8_t bits_actuator_count = 0;
    bits_actuator_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator_count = bits_actuator_count;
    std::uint8_t bits_ends_at_rest = 0;
    bits_ends_at_rest |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.ends_at_rest = bits_ends_at_rest;
    std::uint8_t bits_purpose = 0;
    bits_purpose |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.purpose = bits_purpose;
    return true;
}

inline constexpr std::size_t Segment6Coefficients_SIZE = 25;
struct Segment6Coefficients {
    static constexpr std::size_t SIZE = Segment6Coefficients_SIZE;
    std::uint8_t actuator{};
    float c0{};
    float c1{};
    float c2{};
    float c3{};
    float c4{};
    float c5{};
};

inline bool encode(const Segment6Coefficients &value, std::span<std::uint8_t> out) {
    if (out.size() < Segment6Coefficients_SIZE) return false;
    std::size_t offset = 0;
    const std::uint8_t bits_actuator = static_cast<std::uint8_t>(value.actuator);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator >> 0);
    const std::uint32_t bits_c0 = std::bit_cast<std::uint32_t>(value.c0);
    out[offset++] = static_cast<std::uint8_t>(bits_c0 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c0 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c0 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c0 >> 24);
    const std::uint32_t bits_c1 = std::bit_cast<std::uint32_t>(value.c1);
    out[offset++] = static_cast<std::uint8_t>(bits_c1 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c1 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c1 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c1 >> 24);
    const std::uint32_t bits_c2 = std::bit_cast<std::uint32_t>(value.c2);
    out[offset++] = static_cast<std::uint8_t>(bits_c2 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c2 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c2 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c2 >> 24);
    const std::uint32_t bits_c3 = std::bit_cast<std::uint32_t>(value.c3);
    out[offset++] = static_cast<std::uint8_t>(bits_c3 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c3 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c3 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c3 >> 24);
    const std::uint32_t bits_c4 = std::bit_cast<std::uint32_t>(value.c4);
    out[offset++] = static_cast<std::uint8_t>(bits_c4 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c4 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c4 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c4 >> 24);
    const std::uint32_t bits_c5 = std::bit_cast<std::uint32_t>(value.c5);
    out[offset++] = static_cast<std::uint8_t>(bits_c5 >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_c5 >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_c5 >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_c5 >> 24);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Segment6Coefficients &value) {
    if (input.size() != Segment6Coefficients_SIZE) return false;
    std::size_t offset = 0;
    std::uint8_t bits_actuator = 0;
    bits_actuator |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator = bits_actuator;
    std::uint32_t bits_c0 = 0;
    bits_c0 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c0 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c0 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c0 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c0 = std::bit_cast<float>(bits_c0);
    std::uint32_t bits_c1 = 0;
    bits_c1 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c1 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c1 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c1 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c1 = std::bit_cast<float>(bits_c1);
    std::uint32_t bits_c2 = 0;
    bits_c2 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c2 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c2 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c2 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c2 = std::bit_cast<float>(bits_c2);
    std::uint32_t bits_c3 = 0;
    bits_c3 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c3 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c3 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c3 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c3 = std::bit_cast<float>(bits_c3);
    std::uint32_t bits_c4 = 0;
    bits_c4 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c4 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c4 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c4 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c4 = std::bit_cast<float>(bits_c4);
    std::uint32_t bits_c5 = 0;
    bits_c5 |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_c5 |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_c5 |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_c5 |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.c5 = std::bit_cast<float>(bits_c5);
    return true;
}

inline constexpr std::size_t Commit6_SIZE = 8;
struct Commit6 {
    static constexpr std::size_t SIZE = Commit6_SIZE;
    std::uint64_t through_ticks{};
};

inline bool encode(const Commit6 &value, std::span<std::uint8_t> out) {
    if (out.size() < Commit6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_through_ticks = static_cast<std::uint64_t>(value.through_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_through_ticks >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Commit6 &value) {
    if (input.size() != Commit6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_through_ticks = 0;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_through_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.through_ticks = bits_through_ticks;
    return true;
}

inline constexpr std::size_t QueueStatus6_SIZE = 60;
struct QueueStatus6 {
    static constexpr std::size_t SIZE = QueueStatus6_SIZE;
    std::uint64_t queue_revision{};
    std::uint64_t committed_until_ticks{};
    std::uint64_t executing_plan_id{};
    std::uint16_t executing_segment{};
    std::uint64_t path_clock_ticks{};
    float rate{};
    std::uint16_t remaining_segments{};
    std::uint16_t remaining_events{};
    std::uint8_t underflow{};
    std::uint8_t fault{};
    std::uint64_t received_until_ticks{};
    std::uint64_t received_bytes{};
};

inline bool encode(const QueueStatus6 &value, std::span<std::uint8_t> out) {
    if (out.size() < QueueStatus6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_queue_revision = static_cast<std::uint64_t>(value.queue_revision);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_queue_revision >> 56);
    const std::uint64_t bits_committed_until_ticks = static_cast<std::uint64_t>(value.committed_until_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_committed_until_ticks >> 56);
    const std::uint64_t bits_executing_plan_id = static_cast<std::uint64_t>(value.executing_plan_id);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_plan_id >> 56);
    const std::uint16_t bits_executing_segment = static_cast<std::uint16_t>(value.executing_segment);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_segment >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_executing_segment >> 8);
    const std::uint64_t bits_path_clock_ticks = static_cast<std::uint64_t>(value.path_clock_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 56);
    const std::uint32_t bits_rate = std::bit_cast<std::uint32_t>(value.rate);
    out[offset++] = static_cast<std::uint8_t>(bits_rate >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_rate >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_rate >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_rate >> 24);
    const std::uint16_t bits_remaining_segments = static_cast<std::uint16_t>(value.remaining_segments);
    out[offset++] = static_cast<std::uint8_t>(bits_remaining_segments >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_remaining_segments >> 8);
    const std::uint16_t bits_remaining_events = static_cast<std::uint16_t>(value.remaining_events);
    out[offset++] = static_cast<std::uint8_t>(bits_remaining_events >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_remaining_events >> 8);
    const std::uint8_t bits_underflow = static_cast<std::uint8_t>(value.underflow);
    out[offset++] = static_cast<std::uint8_t>(bits_underflow >> 0);
    const std::uint8_t bits_fault = static_cast<std::uint8_t>(value.fault);
    out[offset++] = static_cast<std::uint8_t>(bits_fault >> 0);
    const std::uint64_t bits_received_until_ticks = static_cast<std::uint64_t>(value.received_until_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_received_until_ticks >> 56);
    const std::uint64_t bits_received_bytes = static_cast<std::uint64_t>(value.received_bytes);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_received_bytes >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, QueueStatus6 &value) {
    if (input.size() != QueueStatus6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_queue_revision = 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_queue_revision |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.queue_revision = bits_queue_revision;
    std::uint64_t bits_committed_until_ticks = 0;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_committed_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.committed_until_ticks = bits_committed_until_ticks;
    std::uint64_t bits_executing_plan_id = 0;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_executing_plan_id |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.executing_plan_id = bits_executing_plan_id;
    std::uint16_t bits_executing_segment = 0;
    bits_executing_segment |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_executing_segment |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.executing_segment = bits_executing_segment;
    std::uint64_t bits_path_clock_ticks = 0;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.path_clock_ticks = bits_path_clock_ticks;
    std::uint32_t bits_rate = 0;
    bits_rate |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_rate |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_rate |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_rate |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.rate = std::bit_cast<float>(bits_rate);
    std::uint16_t bits_remaining_segments = 0;
    bits_remaining_segments |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_remaining_segments |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.remaining_segments = bits_remaining_segments;
    std::uint16_t bits_remaining_events = 0;
    bits_remaining_events |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_remaining_events |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.remaining_events = bits_remaining_events;
    std::uint8_t bits_underflow = 0;
    bits_underflow |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.underflow = bits_underflow;
    std::uint8_t bits_fault = 0;
    bits_fault |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.fault = bits_fault;
    std::uint64_t bits_received_until_ticks = 0;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_received_until_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.received_until_ticks = bits_received_until_ticks;
    std::uint64_t bits_received_bytes = 0;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_received_bytes |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.received_bytes = bits_received_bytes;
    return true;
}

inline constexpr std::size_t State6Header_SIZE = 45;
struct State6Header {
    static constexpr std::size_t SIZE = State6Header_SIZE;
    std::uint64_t session{};
    std::uint64_t timestamp_ticks{};
    std::uint64_t accepted_sequence{};
    std::uint8_t safety{};
    std::uint8_t fault{};
    std::uint8_t actuator_count{};
    std::uint8_t reserved{};
    std::uint64_t path_clock_ticks{};
    std::uint8_t input_count{};
    std::uint64_t input_bits{};
};

inline bool encode(const State6Header &value, std::span<std::uint8_t> out) {
    if (out.size() < State6Header_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_timestamp_ticks = static_cast<std::uint64_t>(value.timestamp_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 56);
    const std::uint64_t bits_accepted_sequence = static_cast<std::uint64_t>(value.accepted_sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted_sequence >> 56);
    const std::uint8_t bits_safety = static_cast<std::uint8_t>(value.safety);
    out[offset++] = static_cast<std::uint8_t>(bits_safety >> 0);
    const std::uint8_t bits_fault = static_cast<std::uint8_t>(value.fault);
    out[offset++] = static_cast<std::uint8_t>(bits_fault >> 0);
    const std::uint8_t bits_actuator_count = static_cast<std::uint8_t>(value.actuator_count);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator_count >> 0);
    const std::uint8_t bits_reserved = static_cast<std::uint8_t>(value.reserved);
    out[offset++] = static_cast<std::uint8_t>(bits_reserved >> 0);
    const std::uint64_t bits_path_clock_ticks = static_cast<std::uint64_t>(value.path_clock_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_path_clock_ticks >> 56);
    const std::uint8_t bits_input_count = static_cast<std::uint8_t>(value.input_count);
    out[offset++] = static_cast<std::uint8_t>(bits_input_count >> 0);
    const std::uint64_t bits_input_bits = static_cast<std::uint64_t>(value.input_bits);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_input_bits >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, State6Header &value) {
    if (input.size() != State6Header_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_timestamp_ticks = 0;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.timestamp_ticks = bits_timestamp_ticks;
    std::uint64_t bits_accepted_sequence = 0;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_accepted_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.accepted_sequence = bits_accepted_sequence;
    std::uint8_t bits_safety = 0;
    bits_safety |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.safety = bits_safety;
    std::uint8_t bits_fault = 0;
    bits_fault |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.fault = bits_fault;
    std::uint8_t bits_actuator_count = 0;
    bits_actuator_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator_count = bits_actuator_count;
    std::uint8_t bits_reserved = 0;
    bits_reserved |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.reserved = bits_reserved;
    std::uint64_t bits_path_clock_ticks = 0;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_path_clock_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.path_clock_ticks = bits_path_clock_ticks;
    std::uint8_t bits_input_count = 0;
    bits_input_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.input_count = bits_input_count;
    std::uint64_t bits_input_bits = 0;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_input_bits |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.input_bits = bits_input_bits;
    return true;
}

inline constexpr std::size_t ActuatorState6_SIZE = 20;
struct ActuatorState6 {
    static constexpr std::size_t SIZE = ActuatorState6_SIZE;
    float position{};
    float velocity{};
    float effort{};
    std::int64_t step_count{};
};

inline bool encode(const ActuatorState6 &value, std::span<std::uint8_t> out) {
    if (out.size() < ActuatorState6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint32_t bits_position = std::bit_cast<std::uint32_t>(value.position);
    out[offset++] = static_cast<std::uint8_t>(bits_position >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_position >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_position >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_position >> 24);
    const std::uint32_t bits_velocity = std::bit_cast<std::uint32_t>(value.velocity);
    out[offset++] = static_cast<std::uint8_t>(bits_velocity >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_velocity >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_velocity >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_velocity >> 24);
    const std::uint32_t bits_effort = std::bit_cast<std::uint32_t>(value.effort);
    out[offset++] = static_cast<std::uint8_t>(bits_effort >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_effort >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_effort >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_effort >> 24);
    const std::uint64_t bits_step_count = std::bit_cast<std::uint64_t>(value.step_count);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_step_count >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, ActuatorState6 &value) {
    if (input.size() != ActuatorState6_SIZE) return false;
    std::size_t offset = 0;
    std::uint32_t bits_position = 0;
    bits_position |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_position |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_position |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_position |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.position = std::bit_cast<float>(bits_position);
    std::uint32_t bits_velocity = 0;
    bits_velocity |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_velocity |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_velocity |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_velocity |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.velocity = std::bit_cast<float>(bits_velocity);
    std::uint32_t bits_effort = 0;
    bits_effort |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_effort |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_effort |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_effort |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.effort = std::bit_cast<float>(bits_effort);
    std::uint64_t bits_step_count = 0;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_step_count |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.step_count = std::bit_cast<std::int64_t>(bits_step_count);
    return true;
}

inline constexpr std::size_t Sensor6Header_SIZE = 27;
struct Sensor6Header {
    static constexpr std::size_t SIZE = Sensor6Header_SIZE;
    std::uint64_t session{};
    std::uint64_t timestamp_ticks{};
    std::uint64_t sequence{};
    std::uint8_t slot{};
    std::uint16_t value_count{};
};

inline bool encode(const Sensor6Header &value, std::span<std::uint8_t> out) {
    if (out.size() < Sensor6Header_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_timestamp_ticks = static_cast<std::uint64_t>(value.timestamp_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ticks >> 56);
    const std::uint64_t bits_sequence = static_cast<std::uint64_t>(value.sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 56);
    const std::uint8_t bits_slot = static_cast<std::uint8_t>(value.slot);
    out[offset++] = static_cast<std::uint8_t>(bits_slot >> 0);
    const std::uint16_t bits_value_count = static_cast<std::uint16_t>(value.value_count);
    out[offset++] = static_cast<std::uint8_t>(bits_value_count >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_value_count >> 8);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Sensor6Header &value) {
    if (input.size() != Sensor6Header_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_timestamp_ticks = 0;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_timestamp_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.timestamp_ticks = bits_timestamp_ticks;
    std::uint64_t bits_sequence = 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.sequence = bits_sequence;
    std::uint8_t bits_slot = 0;
    bits_slot |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.slot = bits_slot;
    std::uint16_t bits_value_count = 0;
    bits_value_count |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_value_count |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.value_count = bits_value_count;
    return true;
}

inline constexpr std::size_t Sensor6Value_SIZE = 4;
struct Sensor6Value {
    static constexpr std::size_t SIZE = Sensor6Value_SIZE;
    float value{};
};

inline bool encode(const Sensor6Value &value, std::span<std::uint8_t> out) {
    if (out.size() < Sensor6Value_SIZE) return false;
    std::size_t offset = 0;
    const std::uint32_t bits_value = std::bit_cast<std::uint32_t>(value.value);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 24);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, Sensor6Value &value) {
    if (input.size() != Sensor6Value_SIZE) return false;
    std::size_t offset = 0;
    std::uint32_t bits_value = 0;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.value = std::bit_cast<float>(bits_value);
    return true;
}

inline constexpr std::size_t InputState6_SIZE = 32;
struct InputState6 {
    static constexpr std::size_t SIZE = InputState6_SIZE;
    std::uint64_t closing_count{};
    std::uint64_t opening_count{};
    std::int64_t captured_steps{};
    std::uint64_t captured_ticks{};
};

inline bool encode(const InputState6 &value, std::span<std::uint8_t> out) {
    if (out.size() < InputState6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_closing_count = static_cast<std::uint64_t>(value.closing_count);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_closing_count >> 56);
    const std::uint64_t bits_opening_count = static_cast<std::uint64_t>(value.opening_count);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_opening_count >> 56);
    const std::uint64_t bits_captured_steps = std::bit_cast<std::uint64_t>(value.captured_steps);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_steps >> 56);
    const std::uint64_t bits_captured_ticks = static_cast<std::uint64_t>(value.captured_ticks);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_captured_ticks >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, InputState6 &value) {
    if (input.size() != InputState6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_closing_count = 0;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_closing_count |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.closing_count = bits_closing_count;
    std::uint64_t bits_opening_count = 0;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_opening_count |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.opening_count = bits_opening_count;
    std::uint64_t bits_captured_steps = 0;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_captured_steps |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.captured_steps = std::bit_cast<std::int64_t>(bits_captured_steps);
    std::uint64_t bits_captured_ticks = 0;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_captured_ticks |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.captured_ticks = bits_captured_ticks;
    return true;
}

inline constexpr std::size_t HomingScope6_SIZE = 31;
struct HomingScope6 {
    static constexpr std::size_t SIZE = HomingScope6_SIZE;
    std::uint64_t session{};
    std::uint64_t sequence{};
    std::uint64_t scope{};
    std::uint8_t action{};
    std::uint8_t first{};
    std::uint8_t second{};
    float skew_bound{};
};

inline bool encode(const HomingScope6 &value, std::span<std::uint8_t> out) {
    if (out.size() < HomingScope6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_sequence = static_cast<std::uint64_t>(value.sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 56);
    const std::uint64_t bits_scope = static_cast<std::uint64_t>(value.scope);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 56);
    const std::uint8_t bits_action = static_cast<std::uint8_t>(value.action);
    out[offset++] = static_cast<std::uint8_t>(bits_action >> 0);
    const std::uint8_t bits_first = static_cast<std::uint8_t>(value.first);
    out[offset++] = static_cast<std::uint8_t>(bits_first >> 0);
    const std::uint8_t bits_second = static_cast<std::uint8_t>(value.second);
    out[offset++] = static_cast<std::uint8_t>(bits_second >> 0);
    const std::uint32_t bits_skew_bound = std::bit_cast<std::uint32_t>(value.skew_bound);
    out[offset++] = static_cast<std::uint8_t>(bits_skew_bound >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_skew_bound >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_skew_bound >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_skew_bound >> 24);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, HomingScope6 &value) {
    if (input.size() != HomingScope6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_sequence = 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.sequence = bits_sequence;
    std::uint64_t bits_scope = 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.scope = bits_scope;
    std::uint8_t bits_action = 0;
    bits_action |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.action = bits_action;
    std::uint8_t bits_first = 0;
    bits_first |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.first = bits_first;
    std::uint8_t bits_second = 0;
    bits_second |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.second = bits_second;
    std::uint32_t bits_skew_bound = 0;
    bits_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_skew_bound |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.skew_bound = std::bit_cast<float>(bits_skew_bound);
    return true;
}

inline constexpr std::size_t HomingSide6_SIZE = 26;
struct HomingSide6 {
    static constexpr std::size_t SIZE = HomingSide6_SIZE;
    std::uint64_t session{};
    std::uint64_t sequence{};
    std::uint64_t scope{};
    std::uint8_t actuator{};
    std::uint8_t hold{};
};

inline bool encode(const HomingSide6 &value, std::span<std::uint8_t> out) {
    if (out.size() < HomingSide6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_sequence = static_cast<std::uint64_t>(value.sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 56);
    const std::uint64_t bits_scope = static_cast<std::uint64_t>(value.scope);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 56);
    const std::uint8_t bits_actuator = static_cast<std::uint8_t>(value.actuator);
    out[offset++] = static_cast<std::uint8_t>(bits_actuator >> 0);
    const std::uint8_t bits_hold = static_cast<std::uint8_t>(value.hold);
    out[offset++] = static_cast<std::uint8_t>(bits_hold >> 0);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, HomingSide6 &value) {
    if (input.size() != HomingSide6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_sequence = 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.sequence = bits_sequence;
    std::uint64_t bits_scope = 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.scope = bits_scope;
    std::uint8_t bits_actuator = 0;
    bits_actuator |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.actuator = bits_actuator;
    std::uint8_t bits_hold = 0;
    bits_hold |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.hold = bits_hold;
    return true;
}

inline constexpr std::size_t HomingControlAck6_SIZE = 25;
struct HomingControlAck6 {
    static constexpr std::size_t SIZE = HomingControlAck6_SIZE;
    std::uint64_t session{};
    std::uint64_t sequence{};
    std::uint64_t scope{};
    std::uint8_t accepted{};
};

inline bool encode(const HomingControlAck6 &value, std::span<std::uint8_t> out) {
    if (out.size() < HomingControlAck6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_sequence = static_cast<std::uint64_t>(value.sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 56);
    const std::uint64_t bits_scope = static_cast<std::uint64_t>(value.scope);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 56);
    const std::uint8_t bits_accepted = static_cast<std::uint8_t>(value.accepted);
    out[offset++] = static_cast<std::uint8_t>(bits_accepted >> 0);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, HomingControlAck6 &value) {
    if (input.size() != HomingControlAck6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_sequence = 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.sequence = bits_sequence;
    std::uint64_t bits_scope = 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.scope = bits_scope;
    std::uint8_t bits_accepted = 0;
    bits_accepted |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.accepted = bits_accepted;
    return true;
}

inline constexpr std::size_t HomingCounterBatch6_SIZE = 42;
struct HomingCounterBatch6 {
    static constexpr std::size_t SIZE = HomingCounterBatch6_SIZE;
    std::uint64_t session{};
    std::uint64_t sequence{};
    std::uint64_t scope{};
    std::uint8_t first{};
    std::uint8_t second{};
    double first_delta{};
    double second_delta{};
};

inline bool encode(const HomingCounterBatch6 &value, std::span<std::uint8_t> out) {
    if (out.size() < HomingCounterBatch6_SIZE) return false;
    std::size_t offset = 0;
    const std::uint64_t bits_session = static_cast<std::uint64_t>(value.session);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_session >> 56);
    const std::uint64_t bits_sequence = static_cast<std::uint64_t>(value.sequence);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_sequence >> 56);
    const std::uint64_t bits_scope = static_cast<std::uint64_t>(value.scope);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_scope >> 56);
    const std::uint8_t bits_first = static_cast<std::uint8_t>(value.first);
    out[offset++] = static_cast<std::uint8_t>(bits_first >> 0);
    const std::uint8_t bits_second = static_cast<std::uint8_t>(value.second);
    out[offset++] = static_cast<std::uint8_t>(bits_second >> 0);
    const std::uint64_t bits_first_delta = std::bit_cast<std::uint64_t>(value.first_delta);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_first_delta >> 56);
    const std::uint64_t bits_second_delta = std::bit_cast<std::uint64_t>(value.second_delta);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_second_delta >> 56);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, HomingCounterBatch6 &value) {
    if (input.size() != HomingCounterBatch6_SIZE) return false;
    std::size_t offset = 0;
    std::uint64_t bits_session = 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_session |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.session = bits_session;
    std::uint64_t bits_sequence = 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_sequence |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.sequence = bits_sequence;
    std::uint64_t bits_scope = 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_scope |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.scope = bits_scope;
    std::uint8_t bits_first = 0;
    bits_first |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.first = bits_first;
    std::uint8_t bits_second = 0;
    bits_second |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.second = bits_second;
    std::uint64_t bits_first_delta = 0;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_first_delta |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.first_delta = std::bit_cast<double>(bits_first_delta);
    std::uint64_t bits_second_delta = 0;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_second_delta |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.second_delta = std::bit_cast<double>(bits_second_delta);
    return true;
}

} // namespace robotkit::device_wire6
