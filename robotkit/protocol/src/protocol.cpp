#include "robotkit_protocol.hpp"

#include <limits>

namespace robotkit::protocol {

namespace {

void put_u16(std::vector<uint8_t> &output, uint16_t value) {
    output.push_back(static_cast<uint8_t>(value));
    output.push_back(static_cast<uint8_t>(value >> 8));
}

void put_u32(std::vector<uint8_t> &output, uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        output.push_back(static_cast<uint8_t>(value >> shift));
}

void put_u64(std::vector<uint8_t> &output, uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        output.push_back(static_cast<uint8_t>(value >> shift));
}

uint16_t get_u16(std::span<const uint8_t> input, std::size_t offset) {
    return static_cast<uint16_t>(input[offset]) |
           static_cast<uint16_t>(input[offset + 1] << 8);
}

uint32_t get_u32(std::span<const uint8_t> input, std::size_t offset) {
    uint32_t value = 0;
    for (unsigned shift = 0; shift < 32; shift += 8)
        value |= static_cast<uint32_t>(input[offset + shift / 8]) << shift;
    return value;
}

uint64_t get_u64(std::span<const uint8_t> input, std::size_t offset) {
    uint64_t value = 0;
    for (unsigned shift = 0; shift < 64; shift += 8)
        value |= static_cast<uint64_t>(input[offset + shift / 8]) << shift;
    return value;
}

} // namespace

bool encode_frame(const Frame &frame, std::vector<uint8_t> &out) {
    if (frame.version != kVersion || frame.payload.size() > std::numeric_limits<uint32_t>::max())
        return false;

    out.clear();
    out.reserve(kHeaderSize + frame.payload.size());
    put_u32(out, kMagic);
    put_u16(out, frame.version);
    put_u16(out, static_cast<uint16_t>(frame.type));
    put_u64(out, frame.sequence);
    put_u32(out, static_cast<uint32_t>(frame.payload.size()));
    put_u32(out, frame.flags);
    out.insert(out.end(), frame.payload.begin(), frame.payload.end());
    return true;
}

DecodeStatus decode_frame(std::span<const uint8_t> input, Frame &out,
                          std::size_t &consumed, std::size_t max_payload) {
    consumed = 0;
    if (input.size() < kHeaderSize)
        return DecodeStatus::NeedMore;
    if (get_u32(input, 0) != kMagic || get_u16(input, 4) != kVersion)
        return DecodeStatus::Malformed;

    const auto payload_size = static_cast<std::size_t>(get_u32(input, 16));
    if (payload_size > max_payload || payload_size > std::numeric_limits<std::size_t>::max() - kHeaderSize)
        return DecodeStatus::Malformed;
    const auto frame_size = kHeaderSize + payload_size;
    if (input.size() < frame_size)
        return DecodeStatus::NeedMore;

    out.version = get_u16(input, 4);
    out.type = static_cast<MessageType>(get_u16(input, 6));
    out.sequence = get_u64(input, 8);
    out.flags = get_u32(input, 20);
    out.payload.assign(input.begin() + kHeaderSize, input.begin() + frame_size);
    consumed = frame_size;
    return DecodeStatus::Complete;
}

} // namespace robotkit::protocol
