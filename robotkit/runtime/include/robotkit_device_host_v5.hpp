#pragma once

#include "device_wire.hpp"
#include "robotkit_runtime.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace robotkit::v5 {

struct HostState {
    device_wire::StateHeader header{};
    std::array<device_wire::JointState, device_wire::MAX_JOINTS> joints{};
};

/** POSIX v5 link used beside the active v4 SerialRobotEndpoint. */
class RK_API HostLink final {
public:
    HostLink(int descriptor, bool take_ownership, unsigned baud,
             std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count);
    ~HostLink();
    HostLink(const HostLink &) = delete;
    HostLink &operator=(const HostLink &) = delete;

    static std::unique_ptr<HostLink> open(const char *path, unsigned baud,
        std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count);

    bool begin_session();
    bool send_command(std::uint8_t kind, std::span<const device_wire::JointTarget> targets = {});
    bool read_state(HostState &state);
    std::uint64_t session_id() const noexcept { return session_id_; }
    std::uint64_t last_sent_sequence() const noexcept { return last_sent_sequence_; }
    std::uint8_t last_session_status() const noexcept { return last_session_status_; }
    bool ready() const noexcept { return ready_; }

private:
    bool write_frame(std::uint8_t type, std::span<const std::uint8_t> payload);
    bool next_frame(std::uint8_t &type, std::vector<std::uint8_t> &payload,
                    std::chrono::steady_clock::time_point deadline);
    int descriptor_ = -1;
    bool owns_descriptor_ = false;
    unsigned baud_ = 0;
    std::array<std::uint8_t, 16> fingerprint_{};
    std::uint8_t joint_count_ = 0;
    std::vector<std::uint8_t> input_;
    std::uint64_t session_id_ = 0;
    std::uint64_t last_sent_sequence_ = 0;
    std::uint64_t last_timestamp_ns_ = 0;
    std::uint8_t last_session_status_ = 0;
    bool ready_ = false;
};

} // namespace robotkit::v5
