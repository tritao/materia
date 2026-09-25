#include "robotkit_serial_protocol.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

void append_u32(std::vector<uint8_t> &bytes, uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<uint8_t>(value >> shift));
}

void append_u64(std::vector<uint8_t> &bytes, uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes.push_back(static_cast<uint8_t>(value >> shift));
}

void append_double(std::vector<uint8_t> &bytes, double value) {
    uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(bytes, bits);
}

uint32_t read_u32(const uint8_t *bytes) {
    uint32_t value = 0;
    for (unsigned index = 0; index < 4; ++index)
        value |= static_cast<uint32_t>(bytes[index]) << (index * 8);
    return value;
}

uint32_t crc32(const uint8_t *bytes, size_t size) {
    uint32_t value = 0xffffffffu;
    for (size_t index = 0; index < size; ++index) {
        value ^= bytes[index];
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
    }
    return ~value;
}

std::vector<uint8_t> frame(const std::array<uint8_t, 4> &magic,
                           const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> bytes(magic.begin(), magic.end());
    append_u32(bytes, static_cast<uint32_t>(payload.size()));
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    append_u32(bytes, crc32(bytes.data(), bytes.size()));
    return bytes;
}

struct Target {
    uint32_t joint;
    uint32_t mode;
    double value;
};

std::vector<uint8_t> session_frame(uint64_t session_id) {
    std::vector<uint8_t> payload;
    append_u64(payload, session_id);
    return frame({'R', 'K', 'H', '4'}, payload);
}

std::vector<uint8_t> command(uint32_t kind, uint64_t session_id, uint64_t sequence,
                             const std::vector<Target> &targets = {},
                             uint32_t reserved = 0) {
    std::vector<uint8_t> payload;
    append_u32(payload, kind);
    append_u64(payload, session_id);
    append_u64(payload, sequence);
    append_u32(payload, static_cast<uint32_t>(targets.size()));
    append_u32(payload, reserved);
    for (const auto &target : targets) {
        append_u32(payload, target.joint);
        append_u32(payload, target.mode);
        append_double(payload, target.value);
    }
    return frame({'R', 'K', 'C', '4'}, payload);
}

struct Collector {
    std::vector<robotkit::SerialCommandFrame> commands;
    std::vector<uint64_t> sessions;
    bool accept_sessions = true;

    static bool begin_session(void *context, uint64_t session_id) {
        auto &self = *static_cast<Collector *>(context);
        if (!self.accept_sessions) return false;
        self.sessions.push_back(session_id);
        return true;
    }

    static void accept(void *context, const robotkit::SerialCommandFrame &command) {
        static_cast<Collector *>(context)->commands.push_back(command);
    }
};

size_t feed(robotkit::SerialCommandDecoder &decoder, const std::vector<uint8_t> &bytes,
            uint64_t received_at_ns, Collector &collector) {
    return decoder.feed(bytes.data(), bytes.size(), received_at_ns,
                        Collector::begin_session, Collector::accept, &collector);
}

} // namespace

int main() {
    constexpr uint8_t crc_test_vector[] = "123456789";
    assert(crc32(crc_test_vector, sizeof(crc_test_vector) - 1) == 0xcbf43926u);

    robotkit::SerialCommandDecoder decoder(6);
    Collector collector;
    assert(feed(decoder, session_frame(0x12345678), 50, collector) == 0);
    assert(decoder.session_id() == 0x12345678 && collector.sessions.size() == 1);
    assert(decoder.watchdog_expired(50, 100));

    const auto mixed = command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 1, {
        {0, RK_TARGET_VELOCITY, -1.25},
        {2, RK_TARGET_POSITION, 0.75},
        {5, RK_TARGET_EFFORT, 3.5}
    });
    assert(decoder.watchdog_expired(100, 100));
    size_t accepted = 0;
    for (const uint8_t byte : mixed)
        accepted += decoder.feed(&byte, 1, 100, Collector::begin_session,
                                 Collector::accept, &collector);
    assert(accepted == 1 && collector.commands.size() == 1);
    const auto &decoded = collector.commands.front();
    assert(decoded.kind == RK_COMMAND_JOINT_TARGETS && decoded.sequence == 1);
    assert(decoded.target_count == 3);
    assert(decoded.targets[0].joint == 0 && decoded.targets[0].mode == RK_TARGET_VELOCITY);
    assert(decoded.targets[0].target == -1.25);
    assert(decoded.targets[1].joint == 2 && decoded.targets[1].mode == RK_TARGET_POSITION);
    assert(decoded.targets[1].target == 0.75);
    assert(decoded.targets[2].joint == 5 && decoded.targets[2].mode == RK_TARGET_EFFORT);
    assert(decoded.targets[2].target == 3.5);
    assert(decoder.last_sequence() == 1);
    assert(!decoder.watchdog_expired(199, 100));
    assert(decoder.watchdog_expired(200, 100));

    // A corrupted frame must not refresh the watchdog or block the next frame.
    auto damaged = command(RK_COMMAND_NONE, 0x12345678, 2);
    damaged.back() ^= 0x40;
    auto heartbeat = command(RK_COMMAND_NONE, 0x12345678, 2);
    damaged.insert(damaged.end(), heartbeat.begin(), heartbeat.end());
    assert(feed(decoder, damaged, 250, collector) == 1);
    assert(collector.commands.size() == 2 && collector.commands.back().sequence == 2);
    assert(decoder.statistics().bad_crc == 1);
    assert(!decoder.watchdog_expired(349, 100));
    assert(decoder.watchdog_expired(350, 100));

    // Invalid batches and replayed sequence numbers leave the sequence watermark unchanged.
    assert(feed(decoder, command(RK_COMMAND_NONE, 0x12345678, 2), 400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 3, {
        {1, RK_TARGET_POSITION, 0.1}, {1, RK_TARGET_EFFORT, 0.2}
    }), 400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 3, {
        {6, RK_TARGET_POSITION, 0.1}
    }), 400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 3, {
        {1, RK_TARGET_POSITION, std::nan("")}
    }), 400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 3, {
        {1, 99, 0.1}
    }), 400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_STOP, 0x12345678, 3,
                                 {{0, RK_TARGET_POSITION, 0.0}}),
                400, collector) == 0);
    assert(feed(decoder, command(RK_COMMAND_NONE, 0x12345678, 3, {}, 1),
                400, collector) == 0);
    assert(decoder.last_sequence() == 2);
    assert(decoder.statistics().stale_sequence == 1);
    assert(decoder.statistics().invalid_command == 6);
    assert(decoder.watchdog_expired(400, 100));

    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 3, {
        {1, RK_TARGET_VELOCITY, 0.5}
    }), 401, collector) == 1);
    assert(decoder.last_sequence() == 3);
    assert(!decoder.watchdog_expired(500, 100));

    // Oversized lengths are discarded, then parsing resumes at the next magic.
    std::vector<uint8_t> oversized{'R', 'K', 'C', '3'};
    oversized[3] = '4';
    append_u32(oversized, static_cast<uint32_t>(28 + 16 * RK_MAX_SERIAL_JOINTS + 1));
    const auto after_oversized = command(RK_COMMAND_EMERGENCY_STOP, 0x12345678, 4);
    oversized.insert(oversized.end(), after_oversized.begin(), after_oversized.end());
    assert(feed(decoder, oversized, 501, collector) == 1);
    assert(collector.commands.back().kind == RK_COMMAND_EMERGENCY_STOP);
    assert(decoder.statistics().bad_length == 1);
    assert(!decoder.watchdog_expired(600, 100));

    robotkit::SerialCommandDecoder invalid_model(RK_MAX_SERIAL_JOINTS + 1);
    assert(!invalid_model.valid_configuration());
    assert(invalid_model.watchdog_expired(0, 1));
    assert(feed(invalid_model, command(RK_COMMAND_NONE, 1, 1), 1, collector) == 0);

    // A missing command handler must not count traffic as a valid heartbeat.
    robotkit::SerialCommandDecoder no_handler(2);
    const auto unhandled_session = session_frame(1);
    const auto unhandled = command(RK_COMMAND_NONE, 1, 1);
    assert(no_handler.feed(unhandled_session.data(), unhandled_session.size(), 100,
                           nullptr, Collector::accept, nullptr) == 0);
    assert(no_handler.feed(unhandled.data(), unhandled.size(), 100,
                           Collector::begin_session, nullptr, nullptr) == 0);
    assert(no_handler.watchdog_expired(100, 100));

    // A host restart establishes a distinct session and resets only that session's sequence.
    assert(feed(decoder, session_frame(0xabcdef), 700, collector) == 0);
    assert(decoder.session_id() == 0xabcdef && decoder.last_sequence() == 0);
    assert(decoder.watchdog_expired(700, 100));
    assert(feed(decoder, command(RK_COMMAND_JOINT_TARGETS, 0x12345678, 5, {
        {0, RK_TARGET_POSITION, 0.0}
    }), 701, collector) == 0);
    assert(decoder.statistics().rejected_session == 1);
    assert(feed(decoder, command(RK_COMMAND_RESET_SAFETY, 0xabcdef, 1),
                702, collector) == 1);
    assert(decoder.last_sequence() == 1 && !decoder.watchdog_expired(801, 100));
    collector.accept_sessions = false;
    assert(feed(decoder, session_frame(0x999999), 900, collector) == 0);
    assert(decoder.session_id() == 0xabcdef);
    assert(decoder.statistics().rejected_session == 2);
}
