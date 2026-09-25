#ifndef ROBOTKIT_SERIAL_PROTOCOL_HPP
#define ROBOTKIT_SERIAL_PROTOCOL_HPP

#include "robotkit_runtime.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace robotkit {

/** One target decoded from an RKC4 command frame. */
struct SerialJointTarget {
    uint32_t joint;
    uint32_t mode;
    double target;
};

/** Complete atomic target batch decoded from an RKC4 command frame. */
struct SerialCommandFrame {
    uint32_t kind;
    uint64_t session_id;
    uint64_t sequence;
    uint32_t target_count;
    std::array<SerialJointTarget, RK_MAX_SERIAL_JOINTS> targets;
};

/**
 * Fixed-capacity stream decoder for the RKH4 session and RKC4 command formats.
 * It has no POSIX or heap dependency and can be ported into a device loop. A
 * session handler must stop and latch the device before it accepts a new host
 * session. Command handlers receive complete batches synchronously only after
 * their envelope, CRC, session, sequence, and every target validate.
 */
class RK_API SerialCommandDecoder final {
public:
    using SessionHandler = bool (*)(void *context, uint64_t session_id);
    using CommandHandler = void (*)(void *context, const SerialCommandFrame &command);

    struct Statistics {
        uint64_t bad_length = 0;
        uint64_t bad_crc = 0;
        uint64_t invalid_command = 0;
        uint64_t stale_sequence = 0;
        uint64_t rejected_session = 0;
    };

    /** joint_count must match the device's compiled model and be at most 64. */
    explicit SerialCommandDecoder(uint32_t joint_count) noexcept;

    /**
     * Feed any number of bytes from the serial stream. The session handler is
     * called for each valid RKH4 request and must synchronously stop and latch
     * the device before returning true. Returns the number of RKC4 commands
     * delivered to handler. Null handlers consume no data or watchdog time.
     */
    size_t feed(const uint8_t *bytes, size_t size, uint64_t received_at_ns,
                SessionHandler session_handler, CommandHandler handler,
                void *context) noexcept;

    /**
     * Returns true until a valid command arrives, or once accepted command
     * traffic has been absent for timeout_ns. Call this from the local device
     * control loop and stop actuators when it returns true.
     */
    bool watchdog_expired(uint64_t now_ns, uint64_t timeout_ns) const noexcept;

    uint64_t last_sequence() const noexcept { return last_sequence_; }
    uint64_t session_id() const noexcept { return session_id_; }
    Statistics statistics() const noexcept { return statistics_; }
    bool valid_configuration() const noexcept { return joint_count_ <= RK_MAX_SERIAL_JOINTS; }

private:
    static constexpr size_t max_payload_bytes = 28 + 16 * RK_MAX_SERIAL_JOINTS;
    static constexpr size_t max_frame_bytes = 8 + max_payload_bytes + 4;

    void process_buffer(uint64_t received_at_ns, SessionHandler session_handler,
                        CommandHandler handler, void *context, size_t &accepted) noexcept;
    void discard_prefix(size_t count) noexcept;

    uint32_t joint_count_ = 0;
    std::array<uint8_t, max_frame_bytes> buffer_{};
    size_t buffer_size_ = 0;
    uint64_t session_id_ = 0;
    uint64_t last_sequence_ = 0;
    uint64_t last_received_at_ns_ = 0;
    bool has_received_command_ = false;
    Statistics statistics_{};
};

} // namespace robotkit

#endif
