#include "robotkit_serial_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace robotkit {
namespace {

constexpr std::array<uint8_t, 4> command_magic{'R', 'K', 'C', '3'};
constexpr size_t envelope_header_bytes = 8;
constexpr size_t checksum_bytes = 4;
constexpr size_t command_header_bytes = 20;
constexpr size_t target_bytes = 16;
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);

uint32_t read_u32(const uint8_t *bytes) noexcept {
    uint32_t value = 0;
    for (unsigned index = 0; index < 4; ++index)
        value |= static_cast<uint32_t>(bytes[index]) << (index * 8);
    return value;
}

uint64_t read_u64(const uint8_t *bytes) noexcept {
    uint64_t value = 0;
    for (unsigned index = 0; index < 8; ++index)
        value |= static_cast<uint64_t>(bytes[index]) << (index * 8);
    return value;
}

double read_double(const uint8_t *bytes) noexcept {
    const uint64_t bits = read_u64(bytes);
    double value;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t crc32(const uint8_t *bytes, size_t size) noexcept {
    uint32_t value = 0xffffffffu;
    for (size_t index = 0; index < size; ++index) {
        value ^= bytes[index];
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
    }
    return ~value;
}

enum class DecodeResult { valid, invalid, stale };

DecodeResult decode_command(const uint8_t *payload, size_t payload_size,
                            uint32_t joint_count, uint64_t last_sequence,
                            SerialCommandFrame &out) noexcept {
    if (payload_size < command_header_bytes)
        return DecodeResult::invalid;

    const uint32_t kind = read_u32(payload);
    const uint64_t sequence = read_u64(payload + 4);
    const uint32_t target_count = read_u32(payload + 12);
    if (read_u32(payload + 16) != 0 || kind > RK_COMMAND_RESET_SAFETY ||
        target_count > RK_MAX_SERIAL_JOINTS ||
        payload_size != command_header_bytes + static_cast<size_t>(target_count) * target_bytes)
        return DecodeResult::invalid;
    if (!sequence || sequence <= last_sequence)
        return DecodeResult::stale;
    if (target_count > joint_count ||
        (kind != RK_COMMAND_JOINT_TARGETS && target_count != 0))
        return DecodeResult::invalid;

    SerialCommandFrame decoded{};
    decoded.kind = kind;
    decoded.sequence = sequence;
    decoded.target_count = target_count;
    bool targeted[RK_MAX_SERIAL_JOINTS]{};
    size_t offset = command_header_bytes;
    for (uint32_t index = 0; index < target_count; ++index) {
        auto &target = decoded.targets[index];
        target.joint = read_u32(payload + offset);
        target.mode = read_u32(payload + offset + 4);
        target.target = read_double(payload + offset + 8);
        offset += target_bytes;
        if (target.joint >= joint_count || targeted[target.joint] ||
            target.mode < RK_TARGET_POSITION || target.mode > RK_TARGET_EFFORT ||
            !std::isfinite(target.target))
            return DecodeResult::invalid;
        targeted[target.joint] = true;
    }

    out = decoded;
    return DecodeResult::valid;
}

} // namespace

SerialCommandDecoder::SerialCommandDecoder(uint32_t joint_count) noexcept
    : joint_count_(joint_count) {}

size_t SerialCommandDecoder::feed(const uint8_t *bytes, size_t size,
                                  uint64_t received_at_ns, CommandHandler handler,
                                  void *context) noexcept {
    if ((!bytes && size) || !valid_configuration() || !handler)
        return 0;

    size_t accepted = 0;
    for (size_t index = 0; index < size; ++index) {
        if (buffer_size_ == buffer_.size()) {
            discard_prefix(1);
            ++statistics_.bad_length;
        }
        buffer_[buffer_size_++] = bytes[index];
        process_buffer(received_at_ns, handler, context, accepted);
    }
    return accepted;
}

bool SerialCommandDecoder::watchdog_expired(uint64_t now_ns,
                                           uint64_t timeout_ns) const noexcept {
    if (!has_received_command_ || !timeout_ns || now_ns < last_received_at_ns_)
        return true;
    return now_ns - last_received_at_ns_ >= timeout_ns;
}

void SerialCommandDecoder::process_buffer(uint64_t received_at_ns,
                                         CommandHandler handler, void *context,
                                         size_t &accepted) noexcept {
    while (buffer_size_) {
        const auto begin = buffer_.begin();
        const auto end = begin + static_cast<ptrdiff_t>(buffer_size_);
        const auto magic = std::search(begin, end, command_magic.begin(), command_magic.end());
        if (magic == end) {
            size_t keep = 0;
            const size_t maximum_prefix = std::min(buffer_size_, command_magic.size() - 1);
            for (size_t count = maximum_prefix; count > 0; --count) {
                if (std::equal(buffer_.begin() + static_cast<ptrdiff_t>(buffer_size_ - count),
                               buffer_.begin() + static_cast<ptrdiff_t>(buffer_size_),
                               command_magic.begin())) {
                    keep = count;
                    break;
                }
            }
            discard_prefix(buffer_size_ - keep);
            return;
        }
        const size_t prefix_size = static_cast<size_t>(magic - begin);
        if (prefix_size) {
            discard_prefix(prefix_size);
            continue;
        }
        if (buffer_size_ < envelope_header_bytes)
            return;

        const uint32_t payload_size = read_u32(buffer_.data() + 4);
        if (payload_size > max_payload_bytes) {
            ++statistics_.bad_length;
            discard_prefix(1);
            continue;
        }
        const size_t frame_size = envelope_header_bytes + payload_size + checksum_bytes;
        if (buffer_size_ < frame_size)
            return;

        const uint32_t expected_crc = read_u32(buffer_.data() + envelope_header_bytes + payload_size);
        const uint32_t actual_crc = crc32(buffer_.data(), envelope_header_bytes + payload_size);
        if (actual_crc != expected_crc) {
            ++statistics_.bad_crc;
            discard_prefix(1);
            continue;
        }

        SerialCommandFrame command{};
        const auto result = decode_command(buffer_.data() + envelope_header_bytes, payload_size,
                                           joint_count_, last_sequence_, command);
        if (result == DecodeResult::stale) {
            ++statistics_.stale_sequence;
        } else if (result == DecodeResult::invalid) {
            ++statistics_.invalid_command;
        } else {
            last_sequence_ = command.sequence;
            last_received_at_ns_ = received_at_ns;
            has_received_command_ = true;
            handler(context, command);
            ++accepted;
        }
        discard_prefix(frame_size);
    }
}

void SerialCommandDecoder::discard_prefix(size_t count) noexcept {
    if (count >= buffer_size_) {
        buffer_size_ = 0;
        return;
    }
    std::move(buffer_.begin() + static_cast<ptrdiff_t>(count),
              buffer_.begin() + static_cast<ptrdiff_t>(buffer_size_), buffer_.begin());
    buffer_size_ -= count;
}

} // namespace robotkit
