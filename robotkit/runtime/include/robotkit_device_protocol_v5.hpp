#pragma once

#include "device_wire.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace robotkit::v5 {

namespace wire = robotkit::device_wire;
inline constexpr std::array<std::uint8_t, 4> magic{'R', 'K', 'D', '5'};
inline constexpr std::size_t header_size = 8;
inline constexpr std::size_t crc_size = 4;
inline constexpr std::size_t max_command_payload = wire::CommandHeader::SIZE + wire::MAX_JOINTS * wire::JointTarget::SIZE;
inline constexpr std::size_t max_state_payload = wire::StateHeader::SIZE + wire::MAX_JOINTS * wire::JointState::SIZE;
inline constexpr std::size_t max_frame_size = header_size + max_state_payload + crc_size;

inline constexpr std::uint64_t response_deadline_us(unsigned baud) noexcept {
    if (baud != 115200 && baud != 230400 && baud != 460800 && baud != 921600) return 0;
    constexpr std::uint64_t exchange_bytes =
        header_size + max_command_payload + crc_size + max_frame_size;
    return (10 * exchange_bytes * 1000000ULL + baud - 1) / baud + 30000;
}

inline std::uint32_t crc32(std::span<const std::uint8_t> bytes) noexcept {
    std::uint32_t value = 0xffffffffu;
    for (auto byte : bytes) {
        value ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
    }
    return ~value;
}

inline std::uint32_t read_u32(const std::uint8_t *bytes) noexcept {
    return static_cast<std::uint32_t>(bytes[0]) |
        (static_cast<std::uint32_t>(bytes[1]) << 8) |
        (static_cast<std::uint32_t>(bytes[2]) << 16) |
        (static_cast<std::uint32_t>(bytes[3]) << 24);
}

inline bool encode_frame(std::uint8_t type, std::span<const std::uint8_t> payload,
                         std::span<std::uint8_t> output, std::size_t &written) noexcept {
    written = 0;
    if (type < 1 || type > 4 || payload.size() > max_state_payload ||
        output.size() < header_size + payload.size() + crc_size) return false;
    std::copy(magic.begin(), magic.end(), output.begin());
    output[4] = type;
    output[5] = 0;
    output[6] = static_cast<std::uint8_t>(payload.size());
    output[7] = static_cast<std::uint8_t>(payload.size() >> 8);
    std::copy(payload.begin(), payload.end(), output.begin() + header_size);
    const auto checksum = crc32(output.first(header_size + payload.size()));
    for (unsigned index = 0; index < 4; ++index)
        output[header_size + payload.size() + index] = static_cast<std::uint8_t>(checksum >> (8 * index));
    written = header_size + payload.size() + crc_size;
    return true;
}

struct Command {
    wire::CommandHeader header{};
    std::array<wire::JointTarget, wire::MAX_JOINTS> targets{};
};

class CommandDecoder final {
public:
    // on_session must synchronously stop actuators, clear targets, and latch
    // safety before returning true. The prior session is invalidated first.
    using SessionHandler = bool (*)(void *, std::uint64_t);
    // Return true only after the application has accepted the complete command.
    using CommandHandler = bool (*)(void *, const Command &);

    struct Statistics {
        std::uint64_t bad_length = 0;
        std::uint64_t bad_crc = 0;
        std::uint64_t invalid = 0;
        std::uint64_t rejected = 0;
    };

    CommandDecoder(std::uint8_t joint_count, std::array<std::uint8_t, 16> model_fingerprint) noexcept
        : joint_count_(joint_count), fingerprint_(model_fingerprint) {}

    bool valid_configuration() const noexcept { return joint_count_ <= wire::MAX_JOINTS; }
    std::uint64_t session_id() const noexcept { return session_id_; }
    std::uint64_t last_sequence() const noexcept { return last_sequence_; }
    bool latched() const noexcept { return latched_; }
    bool model_matches() const noexcept { return model_matches_; }
    Statistics statistics() const noexcept { return statistics_; }

    bool watchdog_expired(std::uint64_t now_ns, std::uint64_t timeout_ns) const noexcept {
        // The device loop must stop and latch hardware when this becomes true.
        return !has_command_ || !timeout_ns || now_ns < last_command_ns_ ||
            now_ns - last_command_ns_ >= timeout_ns;
    }

    std::size_t feed(std::span<const std::uint8_t> bytes, std::uint64_t received_at_ns,
                     SessionHandler on_session, CommandHandler on_command, void *context) noexcept {
        if (!valid_configuration() || !on_session || !on_command) return 0;
        std::size_t accepted = 0;
        for (auto byte : bytes) {
            if (size_ == buffer_.size()) {
                discard(1);
                ++statistics_.bad_length;
            }
            buffer_[size_++] = byte;
            process(received_at_ns, on_session, on_command, context, accepted);
        }
        return accepted;
    }

private:
    void discard(std::size_t count) noexcept {
        if (count >= size_) { size_ = 0; return; }
        std::move(buffer_.begin() + count, buffer_.begin() + size_, buffer_.begin());
        size_ -= count;
    }

    bool accept_command(std::span<const std::uint8_t> payload, std::uint64_t received_at_ns,
                        CommandHandler on_command, void *context) noexcept {
        Command decoded{};
        if (payload.size() < wire::CommandHeader::SIZE ||
            !wire::decode(payload.first(wire::CommandHeader::SIZE), decoded.header)) return false;
        const auto &head = decoded.header;
        if (head.reserved || head.target_count > joint_count_ ||
            payload.size() != wire::CommandHeader::SIZE + head.target_count * wire::JointTarget::SIZE ||
            head.kind < 1 || head.kind > 4 ||
            (head.kind == 1 && head.target_count == 0) ||
            (head.kind != 1 && head.target_count != 0)) return false;
        if (!session_id_ || !model_matches_ || head.session != session_id_ ||
            !head.sequence || head.sequence <= last_sequence_ ||
            (latched_ && head.kind == 1)) return false;
        std::array<bool, wire::MAX_JOINTS> used{};
        for (std::size_t index = 0; index < head.target_count; ++index) {
            const auto start = wire::CommandHeader::SIZE + index * wire::JointTarget::SIZE;
            auto &target = decoded.targets[index];
            if (!wire::decode(payload.subspan(start, wire::JointTarget::SIZE), target) ||
                target.joint >= joint_count_ || used[target.joint] ||
                target.mode < 1 || target.mode > 3 || target.flags ||
                !std::isfinite(target.value)) return false;
            used[target.joint] = true;
        }
        if (!on_command(context, decoded)) return false;
        last_sequence_ = head.sequence;
        last_command_ns_ = received_at_ns;
        has_command_ = true;
        if (head.kind == 3) latched_ = true;
        if (head.kind == 4) latched_ = false;
        return true;
    }

    void process(std::uint64_t received_at_ns, SessionHandler on_session,
                 CommandHandler on_command, void *context, std::size_t &accepted) noexcept {
        while (size_) {
            auto found = std::search(buffer_.begin(), buffer_.begin() + size_, magic.begin(), magic.end());
            if (found == buffer_.begin() + size_) {
                std::size_t keep = 0;
                for (std::size_t count = std::min(size_, magic.size() - 1); count; --count) {
                    if (std::equal(buffer_.begin() + size_ - count, buffer_.begin() + size_, magic.begin())) {
                        keep = count;
                        break;
                    }
                }
                discard(size_ - keep);
                return;
            }
            if (found != buffer_.begin()) { discard(static_cast<std::size_t>(found - buffer_.begin())); continue; }
            if (size_ < header_size) return;
            const auto payload_size = static_cast<std::size_t>(buffer_[6]) |
                (static_cast<std::size_t>(buffer_[7]) << 8);
            if (payload_size > max_state_payload) {
                ++statistics_.bad_length;
                discard(1);
                continue;
            }
            const auto frame_size = header_size + payload_size + crc_size;
            if (size_ < frame_size) return;
            if (read_u32(buffer_.data() + header_size + payload_size) !=
                crc32(std::span<const std::uint8_t>(buffer_.data(), header_size + payload_size))) {
                ++statistics_.bad_crc;
                discard(1);
                continue;
            }
            const auto type = buffer_[4];
            const auto payload = std::span<const std::uint8_t>(buffer_.data() + header_size, payload_size);
            if (buffer_[5]) {
                ++statistics_.invalid;
            } else if (type == 1) {
                wire::SessionBegin request{};
                if (!wire::decode(payload, request) || !request.session) {
                    ++statistics_.invalid;
                } else {
                    // Invalidate the old session before application stop/latch runs.
                    session_id_ = 0;
                    model_matches_ = false;
                    latched_ = true;
                    last_sequence_ = 0;
                    has_command_ = false;
                    last_command_ns_ = 0;
                    if (on_session(context, request.session)) {
                        session_id_ = request.session;
                        model_matches_ = request.model_fingerprint == fingerprint_;
                    } else {
                        ++statistics_.rejected;
                    }
                }
            } else if (type == 3) {
                if (accept_command(payload, received_at_ns, on_command, context)) ++accepted;
                else ++statistics_.rejected;
            } else {
                ++statistics_.invalid;
            }
            discard(frame_size);
        }
    }

    std::uint8_t joint_count_;
    std::array<std::uint8_t, 16> fingerprint_;
    std::array<std::uint8_t, max_frame_size> buffer_{};
    std::size_t size_ = 0;
    std::uint64_t session_id_ = 0;
    std::uint64_t last_sequence_ = 0;
    std::uint64_t last_command_ns_ = 0;
    bool has_command_ = false;
    bool latched_ = true;
    bool model_matches_ = false;
    Statistics statistics_{};
};

} // namespace robotkit::v5
