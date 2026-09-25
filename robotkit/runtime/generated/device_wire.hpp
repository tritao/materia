#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace robotkit::device_wire {

inline constexpr std::uint8_t PROTOCOL_VERSION = 5;
inline constexpr std::uint8_t MAX_JOINTS = 64;

enum class MessageType : std::uint8_t {
    session_begin = 1,
    session_ack = 2,
    command = 3,
    state = 4,
};

enum class SessionStatus : std::uint8_t {
    latched_safe = 1,
    model_mismatch = 2,
};

enum class CommandKind : std::uint8_t {
    targets = 1,
    stop = 2,
    emergency_stop = 3,
    reset_safety = 4,
};

enum class TargetMode : std::uint8_t {
    position = 1,
    velocity = 2,
    effort = 3,
};

inline constexpr std::size_t SessionBegin_SIZE = 24;
struct SessionBegin {
    static constexpr std::size_t SIZE = SessionBegin_SIZE;
    std::uint64_t session{};
    std::array<std::uint8_t, 16> model_fingerprint{};
};

inline bool encode(const SessionBegin &value, std::span<std::uint8_t> out) {
    if (out.size() < SessionBegin_SIZE) return false;
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
    for (std::size_t i = 0; i < 16; ++i) {
        const std::uint8_t bits_model_fingerprint = static_cast<std::uint8_t>(value.model_fingerprint[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_model_fingerprint >> 0);
    }
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, SessionBegin &value) {
    if (input.size() != SessionBegin_SIZE) return false;
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
    for (std::size_t i = 0; i < 16; ++i) {
        std::uint8_t bits_model_fingerprint = 0;
        bits_model_fingerprint |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.model_fingerprint[i] = bits_model_fingerprint;
    }
    return true;
}

inline constexpr std::size_t SessionAck_SIZE = 28;
struct SessionAck {
    static constexpr std::size_t SIZE = SessionAck_SIZE;
    std::uint64_t session{};
    std::array<std::uint8_t, 16> device_fingerprint{};
    std::uint8_t status{};
    std::array<std::uint8_t, 3> reserved{};
};

inline bool encode(const SessionAck &value, std::span<std::uint8_t> out) {
    if (out.size() < SessionAck_SIZE) return false;
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
    for (std::size_t i = 0; i < 16; ++i) {
        const std::uint8_t bits_device_fingerprint = static_cast<std::uint8_t>(value.device_fingerprint[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_device_fingerprint >> 0);
    }
    const std::uint8_t bits_status = static_cast<std::uint8_t>(value.status);
    out[offset++] = static_cast<std::uint8_t>(bits_status >> 0);
    for (std::size_t i = 0; i < 3; ++i) {
        const std::uint8_t bits_reserved = static_cast<std::uint8_t>(value.reserved[i]);
        out[offset++] = static_cast<std::uint8_t>(bits_reserved >> 0);
    }
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, SessionAck &value) {
    if (input.size() != SessionAck_SIZE) return false;
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
    for (std::size_t i = 0; i < 16; ++i) {
        std::uint8_t bits_device_fingerprint = 0;
        bits_device_fingerprint |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.device_fingerprint[i] = bits_device_fingerprint;
    }
    std::uint8_t bits_status = 0;
    bits_status |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.status = bits_status;
    for (std::size_t i = 0; i < 3; ++i) {
        std::uint8_t bits_reserved = 0;
        bits_reserved |= static_cast<std::uint8_t>(input[offset++]) << 0;
        value.reserved[i] = bits_reserved;
    }
    return true;
}

inline constexpr std::size_t CommandHeader_SIZE = 20;
struct CommandHeader {
    static constexpr std::size_t SIZE = CommandHeader_SIZE;
    std::uint64_t session{};
    std::uint64_t sequence{};
    std::uint8_t kind{};
    std::uint8_t target_count{};
    std::uint16_t reserved{};
};

inline bool encode(const CommandHeader &value, std::span<std::uint8_t> out) {
    if (out.size() < CommandHeader_SIZE) return false;
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
    const std::uint8_t bits_kind = static_cast<std::uint8_t>(value.kind);
    out[offset++] = static_cast<std::uint8_t>(bits_kind >> 0);
    const std::uint8_t bits_target_count = static_cast<std::uint8_t>(value.target_count);
    out[offset++] = static_cast<std::uint8_t>(bits_target_count >> 0);
    const std::uint16_t bits_reserved = static_cast<std::uint16_t>(value.reserved);
    out[offset++] = static_cast<std::uint8_t>(bits_reserved >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_reserved >> 8);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, CommandHeader &value) {
    if (input.size() != CommandHeader_SIZE) return false;
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
    std::uint8_t bits_kind = 0;
    bits_kind |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.kind = bits_kind;
    std::uint8_t bits_target_count = 0;
    bits_target_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.target_count = bits_target_count;
    std::uint16_t bits_reserved = 0;
    bits_reserved |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_reserved |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.reserved = bits_reserved;
    return true;
}

inline constexpr std::size_t JointTarget_SIZE = 8;
struct JointTarget {
    static constexpr std::size_t SIZE = JointTarget_SIZE;
    std::uint16_t joint{};
    std::uint8_t mode{};
    std::uint8_t flags{};
    float value{};
};

inline bool encode(const JointTarget &value, std::span<std::uint8_t> out) {
    if (out.size() < JointTarget_SIZE) return false;
    std::size_t offset = 0;
    const std::uint16_t bits_joint = static_cast<std::uint16_t>(value.joint);
    out[offset++] = static_cast<std::uint8_t>(bits_joint >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_joint >> 8);
    const std::uint8_t bits_mode = static_cast<std::uint8_t>(value.mode);
    out[offset++] = static_cast<std::uint8_t>(bits_mode >> 0);
    const std::uint8_t bits_flags = static_cast<std::uint8_t>(value.flags);
    out[offset++] = static_cast<std::uint8_t>(bits_flags >> 0);
    const std::uint32_t bits_value = std::bit_cast<std::uint32_t>(value.value);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_value >> 24);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, JointTarget &value) {
    if (input.size() != JointTarget_SIZE) return false;
    std::size_t offset = 0;
    std::uint16_t bits_joint = 0;
    bits_joint |= static_cast<std::uint16_t>(input[offset++]) << 0;
    bits_joint |= static_cast<std::uint16_t>(input[offset++]) << 8;
    value.joint = bits_joint;
    std::uint8_t bits_mode = 0;
    bits_mode |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.mode = bits_mode;
    std::uint8_t bits_flags = 0;
    bits_flags |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.flags = bits_flags;
    std::uint32_t bits_value = 0;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 0;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 8;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 16;
    bits_value |= static_cast<std::uint32_t>(input[offset++]) << 24;
    value.value = std::bit_cast<float>(bits_value);
    return true;
}

inline constexpr std::size_t StateHeader_SIZE = 28;
struct StateHeader {
    static constexpr std::size_t SIZE = StateHeader_SIZE;
    std::uint64_t session{};
    std::uint64_t timestamp_ns{};
    std::uint64_t accepted_sequence{};
    std::uint8_t safety{};
    std::uint8_t fault{};
    std::uint8_t joint_count{};
    std::uint8_t reserved{};
};

inline bool encode(const StateHeader &value, std::span<std::uint8_t> out) {
    if (out.size() < StateHeader_SIZE) return false;
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
    const std::uint64_t bits_timestamp_ns = static_cast<std::uint64_t>(value.timestamp_ns);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 0);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 8);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 16);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 24);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 32);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 40);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 48);
    out[offset++] = static_cast<std::uint8_t>(bits_timestamp_ns >> 56);
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
    const std::uint8_t bits_joint_count = static_cast<std::uint8_t>(value.joint_count);
    out[offset++] = static_cast<std::uint8_t>(bits_joint_count >> 0);
    const std::uint8_t bits_reserved = static_cast<std::uint8_t>(value.reserved);
    out[offset++] = static_cast<std::uint8_t>(bits_reserved >> 0);
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, StateHeader &value) {
    if (input.size() != StateHeader_SIZE) return false;
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
    std::uint64_t bits_timestamp_ns = 0;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 0;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 8;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 16;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 24;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 32;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 40;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 48;
    bits_timestamp_ns |= static_cast<std::uint64_t>(input[offset++]) << 56;
    value.timestamp_ns = bits_timestamp_ns;
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
    std::uint8_t bits_joint_count = 0;
    bits_joint_count |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.joint_count = bits_joint_count;
    std::uint8_t bits_reserved = 0;
    bits_reserved |= static_cast<std::uint8_t>(input[offset++]) << 0;
    value.reserved = bits_reserved;
    return true;
}

inline constexpr std::size_t JointState_SIZE = 12;
struct JointState {
    static constexpr std::size_t SIZE = JointState_SIZE;
    float position{};
    float velocity{};
    float effort{};
};

inline bool encode(const JointState &value, std::span<std::uint8_t> out) {
    if (out.size() < JointState_SIZE) return false;
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
    return true;
}

inline bool decode(std::span<const std::uint8_t> input, JointState &value) {
    if (input.size() != JointState_SIZE) return false;
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
    return true;
}

} // namespace robotkit::device_wire
