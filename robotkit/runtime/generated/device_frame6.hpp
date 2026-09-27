#pragma once

#include "device_wire6.hpp"
#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace robotkit::device_frame6 {

inline constexpr std::size_t HEADER_SIZE = 8;
inline constexpr std::size_t CRC_SIZE = 4;
inline constexpr std::size_t MAX_PAYLOAD_SIZE = device_wire6::Segment6Header::SIZE +
    device_wire6::MAX_ACTUATORS * device_wire6::Segment6Coefficients::SIZE;
inline constexpr std::size_t MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

inline std::uint32_t crc32(std::span<const std::uint8_t> bytes) {
    std::uint32_t value = 0xffffffffu;
    for (auto byte : bytes) {
        value ^= byte;
        for (int i = 0; i < 8; ++i)
            value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1)));
    }
    return ~value;
}

struct Frame {
    std::uint8_t kind{};
    std::span<const std::uint8_t> payload{};
};

inline bool decode(std::span<const std::uint8_t> bytes, Frame &frame) {
    if (bytes.size() < HEADER_SIZE + CRC_SIZE || bytes.size() > MAX_FRAME_SIZE ||
        bytes[0] != 'R' || bytes[1] != 'K' || bytes[2] != 'D' || bytes[3] != '6' ||
        bytes[5] != 0 || bytes[4] == 0 || bytes[4] > 15) return false;
    const auto length = std::size_t(bytes[6]) | (std::size_t(bytes[7]) << 8);
    if (length > MAX_PAYLOAD_SIZE || bytes.size() != HEADER_SIZE + length + CRC_SIZE) return false;
    const auto expected = std::uint32_t(bytes[8 + length]) |
        (std::uint32_t(bytes[9 + length]) << 8) |
        (std::uint32_t(bytes[10 + length]) << 16) |
        (std::uint32_t(bytes[11 + length]) << 24);
    if (crc32(bytes.first(HEADER_SIZE + length)) != expected) return false;
    if ((bytes[4] >= 8 && bytes[4] <= 13 && length != 0) ||
        (bytes[4] == 1 && length != device_wire6::SessionBegin6::SIZE) ||
        (bytes[4] == 2 && length != device_wire6::SessionAck6::SIZE) ||
        (bytes[4] == 3 && length != device_wire6::TimeSyncRequest::SIZE) ||
        (bytes[4] == 4 && length != device_wire6::TimeSyncReply::SIZE) ||
        (bytes[4] == 5 && length != device_wire6::QueueBegin6::SIZE) ||
        (bytes[4] == 7 && length != device_wire6::Commit6::SIZE) ||
        (bytes[4] == 14 && length != device_wire6::QueueStatus6::SIZE)) return false;
    if (bytes[4] == 6) {
        if (length < device_wire6::Segment6Header::SIZE) return false;
        device_wire6::Segment6Header header{};
        if (!device_wire6::decode(bytes.subspan(HEADER_SIZE, device_wire6::Segment6Header::SIZE), header) ||
            header.degree > 5 || header.actuator_count == 0 ||
            header.actuator_count > device_wire6::MAX_ACTUATORS || header.ends_at_rest > 1 ||
            header.reserved != 0 || header.duration_ticks == 0 ||
            length != device_wire6::Segment6Header::SIZE +
                header.actuator_count * device_wire6::Segment6Coefficients::SIZE) return false;
    }
    if (bytes[4] == 15) {
        if (length < device_wire6::State6Header::SIZE) return false;
        device_wire6::State6Header header{};
        if (!device_wire6::decode(bytes.subspan(HEADER_SIZE, device_wire6::State6Header::SIZE), header) ||
            header.actuator_count > device_wire6::MAX_ACTUATORS || header.reserved != 0 ||
            length != device_wire6::State6Header::SIZE +
                header.actuator_count * device_wire6::ActuatorState6::SIZE) return false;
    }
    frame = {bytes[4], bytes.subspan(HEADER_SIZE, length)};
    return true;
}

inline bool encode(std::uint8_t kind, std::span<const std::uint8_t> payload,
                   std::vector<std::uint8_t> &out) {
    if (kind == 0 || kind > 15 || payload.size() > MAX_PAYLOAD_SIZE) return false;
    out.resize(HEADER_SIZE + payload.size() + CRC_SIZE);
    out[0] = 'R'; out[1] = 'K'; out[2] = 'D'; out[3] = '6';
    out[4] = kind; out[5] = 0;
    out[6] = static_cast<std::uint8_t>(payload.size());
    out[7] = static_cast<std::uint8_t>(payload.size() >> 8);
    for (std::size_t i = 0; i < payload.size(); ++i) out[HEADER_SIZE + i] = payload[i];
    const auto crc = crc32(std::span<const std::uint8_t>(out.data(), HEADER_SIZE + payload.size()));
    for (int i = 0; i < 4; ++i)
        out[HEADER_SIZE + payload.size() + i] = static_cast<std::uint8_t>(crc >> (8 * i));
    Frame check{};
    return decode(out, check);
}

} // namespace robotkit::device_frame6
