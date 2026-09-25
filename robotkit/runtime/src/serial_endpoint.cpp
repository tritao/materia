#include "robotkit_serial_endpoint.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <random>
#include <vector>

#if defined(_WIN32)
#error "SerialRobotEndpoint currently targets POSIX serial devices"
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace robotkit {
namespace {

constexpr std::array<std::uint8_t, 4> session_magic{'R', 'K', 'H', '4'};
constexpr std::array<std::uint8_t, 4> command_magic{'R', 'K', 'C', '4'};
constexpr std::array<std::uint8_t, 4> state_magic{'R', 'K', 'S', '4'};
constexpr std::size_t frame_header_bytes = 8;
constexpr std::size_t frame_checksum_bytes = 4;
constexpr std::size_t state_header_payload_bytes = 4 + 8 + 4 + 8 + 8 + 4;
constexpr std::size_t command_header_payload_bytes = 4 + 8 + 8 + 4 + 4;
constexpr std::size_t joint_state_bytes = 3 * sizeof(double);
constexpr std::size_t sensor_header_payload_bytes = 8 + 8 + 4;
constexpr std::size_t maximum_frame_payload = state_header_payload_bytes +
    RK_MAX_SERIAL_JOINTS * joint_state_bytes + RK_MAX_SENSORS *
    (sensor_header_payload_bytes + RK_MAX_SENSOR_VALUES * sizeof(double));
constexpr auto response_timeout = std::chrono::milliseconds(250);

speed_t baud_value(uint32_t baud) {
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
    }
}

void append_u32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

void append_u64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

void append_double(std::vector<std::uint8_t> &bytes, double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t));
    append_u64(bytes, std::bit_cast<std::uint64_t>(value));
}

std::uint32_t read_u32(const std::uint8_t *bytes) {
    std::uint32_t value = 0;
    for (unsigned index = 0; index < 4; ++index)
        value |= static_cast<std::uint32_t>(bytes[index]) << (index * 8);
    return value;
}

std::uint64_t read_u64(const std::uint8_t *bytes) {
    std::uint64_t value = 0;
    for (unsigned index = 0; index < 8; ++index)
        value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8);
    return value;
}

double read_double(const std::uint8_t *bytes) {
    return std::bit_cast<double>(read_u64(bytes));
}

std::uint32_t crc32(const std::uint8_t *bytes, std::size_t size) {
    std::uint32_t value = 0xffffffffu;
    for (std::size_t index = 0; index < size; ++index) {
        value ^= bytes[index];
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
    }
    return ~value;
}

std::vector<std::uint8_t> frame(const std::array<std::uint8_t, 4> &magic,
                                std::vector<std::uint8_t> payload) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(frame_header_bytes + payload.size() + frame_checksum_bytes);
    bytes.insert(bytes.end(), magic.begin(), magic.end());
    append_u32(bytes, static_cast<std::uint32_t>(payload.size()));
    bytes.insert(bytes.end(), payload.begin(), payload.end());
    append_u32(bytes, crc32(bytes.data(), bytes.size()));
    return bytes;
}

bool extract_frame(std::vector<std::uint8_t> &input,
                  const std::array<std::uint8_t, 4> &magic,
                  std::size_t maximum_payload,
                  std::vector<std::uint8_t> &payload) {
    while (!input.empty()) {
        auto start = std::search(input.begin(), input.end(), magic.begin(), magic.end());
        if (start == input.end()) {
            const auto keep = std::min<std::size_t>(input.size(), magic.size() - 1);
            input.erase(input.begin(), input.end() - keep);
            return false;
        }
        if (start != input.begin()) input.erase(input.begin(), start);
        if (input.size() < frame_header_bytes) return false;
        const auto payload_size = read_u32(input.data() + 4);
        if (payload_size > maximum_payload) {
            input.erase(input.begin());
            continue;
        }
        const auto frame_size = frame_header_bytes + payload_size + frame_checksum_bytes;
        if (input.size() < frame_size) return false;
        const auto expected_crc = read_u32(input.data() + frame_header_bytes + payload_size);
        if (crc32(input.data(), frame_header_bytes + payload_size) != expected_crc) {
            input.erase(input.begin());
            continue;
        }
        payload.assign(input.begin() + frame_header_bytes,
                       input.begin() + frame_header_bytes + payload_size);
        input.erase(input.begin(), input.begin() + frame_size);
        return true;
    }
    return false;
}

struct StateMetadata {
    std::uint64_t session_id = 0;
    std::uint64_t last_command_sequence = 0;
};

bool decode_state_payload(const std::uint8_t *payload, std::size_t payload_size,
                          rk_robot_state &decoded, StateMetadata &metadata) {
    if (payload_size < state_header_payload_bytes) return false;
    const auto joint_count = read_u32(payload);
    const auto source_timestamp_ns = read_u64(payload + 4);
    const auto sensor_count = read_u32(payload + 12);
    metadata.session_id = read_u64(payload + 16);
    metadata.last_command_sequence = read_u64(payload + 24);
    const auto safety = read_u32(payload + 32);
    if (joint_count > RK_MAX_SERIAL_JOINTS || sensor_count > RK_MAX_SENSORS ||
        safety > RK_SAFETY_FAULT)
        return false;

    std::size_t offset = state_header_payload_bytes;
    const auto joints_bytes = static_cast<std::size_t>(joint_count) * joint_state_bytes;
    if (offset + joints_bytes > payload_size) return false;
    decoded = {};
    decoded.struct_size = sizeof(decoded);
    decoded.joint_count = joint_count;
    decoded.source_timestamp_ns = source_timestamp_ns;
    decoded.safety = safety;
    for (uint32_t joint = 0; joint < joint_count; ++joint) {
        decoded.position[joint] = read_double(payload + offset); offset += 8;
        decoded.velocity[joint] = read_double(payload + offset); offset += 8;
        decoded.effort[joint] = read_double(payload + offset); offset += 8;
    }
    decoded.sensor_count = sensor_count;
    for (uint32_t index = 0; index < sensor_count; ++index) {
        if (offset + sensor_header_payload_bytes > payload_size) return false;
        auto &sensor = decoded.sensors[index];
        sensor.sequence = read_u64(payload + offset); offset += 8;
        sensor.source_timestamp_ns = read_u64(payload + offset); offset += 8;
        sensor.value_count = read_u32(payload + offset); offset += 4;
        if (sensor.value_count > RK_MAX_SENSOR_VALUES ||
            offset + static_cast<std::size_t>(sensor.value_count) * 8 > payload_size)
            return false;
        for (uint32_t value = 0; value < sensor.value_count; ++value) {
            sensor.values[value] = read_double(payload + offset);
            offset += 8;
        }
    }
    return offset == payload_size;
}

std::uint64_t new_session_id() {
    std::random_device random;
    const auto value = (static_cast<std::uint64_t>(random()) << 32) | random();
    return value ? value : 1;
}

bool write_all(int descriptor, const std::uint8_t *bytes, std::size_t size) {
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + response_timeout;
    while (written < size) {
        const auto result = ::write(descriptor, bytes + written, size - written);
        if (result > 0) { written += static_cast<std::size_t>(result); continue; }
        if (result < 0 && errno == EINTR) continue;
        if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return false;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        pollfd writable{descriptor, POLLOUT, 0};
        const int ready = ::poll(&writable, 1,
            static_cast<int>(std::max<int64_t>(1, remaining.count())));
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || (writable.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    }
    return true;
}

} // namespace

std::shared_ptr<SerialRobotEndpoint> SerialRobotEndpoint::open(const char *path, uint32_t baud) {
    if (!path || baud_value(baud) == 0)
        return {};
    const int descriptor = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0)
        return {};
    termios settings{};
    if (tcgetattr(descriptor, &settings) != 0) {
        ::close(descriptor);
        return {};
    }
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    cfsetispeed(&settings, baud_value(baud));
    cfsetospeed(&settings, baud_value(baud));
    if (tcsetattr(descriptor, TCSANOW, &settings) != 0) {
        ::close(descriptor);
        return {};
    }
    bool endpoint_owns_descriptor = false;
    std::unique_ptr<SerialRobotEndpoint> endpoint;
    try {
        endpoint.reset(new SerialRobotEndpoint(
            descriptor, true, new_session_id(), RK_SAFETY_EMERGENCY_STOP, false));
        endpoint_owns_descriptor = true;
        if (endpoint->begin_session() != RK_OK)
            return {};
        return std::shared_ptr<SerialRobotEndpoint>(std::move(endpoint));
    } catch (...) {
        if (!endpoint_owns_descriptor) ::close(descriptor);
        return {};
    }
}

SerialRobotEndpoint::SerialRobotEndpoint(int descriptor, bool take_ownership,
                                         uint64_t session_id,
                                         rk_safety_state initial_safety)
    : SerialRobotEndpoint(descriptor, take_ownership, session_id, initial_safety, true) {}

SerialRobotEndpoint::SerialRobotEndpoint(int descriptor, bool take_ownership,
                                         uint64_t session_id,
                                         rk_safety_state initial_safety,
                                         bool session_ready)
    : descriptor_(descriptor), owns_descriptor_(take_ownership),
      session_id_(session_id), initial_safety_(initial_safety),
      session_ready_(session_ready && session_id != 0 && initial_safety <= RK_SAFETY_FAULT) {
    if (descriptor_ >= 0) {
        const int flags = fcntl(descriptor_, F_GETFL, 0);
        if (flags >= 0) fcntl(descriptor_, F_SETFL, flags | O_NONBLOCK);
    }
}

SerialRobotEndpoint::~SerialRobotEndpoint() {
    if (owns_descriptor_ && descriptor_ >= 0)
        ::close(descriptor_);
}

rk_result SerialRobotEndpoint::reconnect(int descriptor, bool take_ownership) noexcept {
    if (descriptor < 0)
        return RK_ERROR_INVALID_ARGUMENT;
    if (owns_descriptor_ && descriptor_ >= 0)
        ::close(descriptor_);
    descriptor_ = descriptor;
    owns_descriptor_ = take_ownership;
    input_.clear();
    has_pending_state_ = false;
    last_source_timestamp_ns_ = 0;
    has_source_timestamp_ = false;
    last_command_sequence_ = 0;
    session_ready_ = false;
    const int flags = fcntl(descriptor_, F_GETFL, 0);
    if (flags >= 0 && fcntl(descriptor_, F_SETFL, flags | O_NONBLOCK) != 0)
        return RK_ERROR_BACKEND;
    try {
        session_id_ = new_session_id();
        return begin_session();
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

rk_result SerialRobotEndpoint::begin_session() {
    if (descriptor_ < 0 || !session_id_)
        return RK_ERROR_INVALID_STATE;
    session_ready_ = false;
    initial_safety_ = RK_SAFETY_EMERGENCY_STOP;
    last_command_sequence_ = 0;
    input_.clear();
    has_pending_state_ = false;
    has_source_timestamp_ = false;
    std::vector<std::uint8_t> payload;
    append_u64(payload, session_id_);
    auto packet = frame(session_magic, std::move(payload));
    if (!write_all(descriptor_, packet.data(), packet.size()))
        return RK_ERROR_BACKEND;
    return wait_for_session_state(session_id_);
}

rk_result SerialRobotEndpoint::wait_for_session_state(uint64_t requested_session_id) {
    const auto deadline = std::chrono::steady_clock::now() + response_timeout;
    std::array<std::uint8_t, 4096> chunk{};
    std::vector<std::uint8_t> payload;
    while (true) {
        while (extract_frame(input_, state_magic, maximum_frame_payload, payload)) {
            rk_robot_state decoded{};
            StateMetadata metadata{};
            if (!decode_state_payload(payload.data(), payload.size(), decoded, metadata))
                return RK_ERROR_BACKEND;
            if (metadata.session_id != requested_session_id)
                continue;
            if (metadata.last_command_sequence != 0 ||
                (decoded.safety != RK_SAFETY_EMERGENCY_STOP &&
                 decoded.safety != RK_SAFETY_FAULT))
                return RK_ERROR_SAFETY_STOPPED;
            initial_safety_ = decoded.safety;
            pending_state_ = decoded;
            pending_state_sequence_ = metadata.last_command_sequence;
            has_pending_state_ = true;
            session_ready_ = true;
            return RK_OK;
        }

        bool received_bytes = false;
        for (;;) {
            const auto result = ::read(descriptor_, chunk.data(), chunk.size());
            if (result > 0) {
                input_.insert(input_.end(), chunk.begin(), chunk.begin() + result);
                received_bytes = true;
                continue;
            }
            if (result < 0 && errno == EINTR) continue;
            if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            return RK_ERROR_BACKEND;
        }
        if (received_bytes) continue;

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return RK_ERROR_BACKEND;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        pollfd descriptor{descriptor_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1,
            static_cast<int>(std::max<int64_t>(1, remaining.count())));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))))
            return RK_ERROR_BACKEND;
        if (ready == 0) return RK_ERROR_BACKEND;
    }
}

rk_result SerialRobotEndpoint::apply(const rk_robot_command &command) {
    if (descriptor_ < 0 || !session_ready_)
        return RK_ERROR_BACKEND;
    if (!command.sequence || command.sequence <= last_command_sequence_)
        return RK_ERROR_STALE_COMMAND;
    if (command.target_count > RK_MAX_SERIAL_JOINTS)
        return RK_ERROR_LIMIT;
    for (uint32_t index = 0; index < command.target_count; ++index)
        if (command.targets[index].joint >= RK_MAX_SERIAL_JOINTS)
            return RK_ERROR_LIMIT;
    std::vector<std::uint8_t> payload;
    payload.reserve(command_header_payload_bytes + command.target_count * 16);
    append_u32(payload, command.kind);
    append_u64(payload, session_id_);
    append_u64(payload, command.sequence);
    append_u32(payload, command.target_count);
    append_u32(payload, 0);
    for (uint32_t index = 0; index < command.target_count; ++index) {
        append_u32(payload, command.targets[index].joint);
        append_u32(payload, command.targets[index].mode);
        append_double(payload, command.targets[index].target);
    }
    auto packet = frame(command_magic, std::move(payload));
    if (!write_all(descriptor_, packet.data(), packet.size()))
        return RK_ERROR_BACKEND;
    last_command_sequence_ = command.sequence;
    return RK_OK;
}

rk_result SerialRobotEndpoint::sample(uint64_t, rk_robot_state &state) {
    if (descriptor_ < 0 || !session_ready_)
        return RK_ERROR_BACKEND;
    const auto deadline = std::chrono::steady_clock::now() + response_timeout;
    std::array<std::uint8_t, 4096> chunk{};
    std::vector<std::uint8_t> payload;
    while (true) {
        if (has_pending_state_) {
            const auto pending_sequence = pending_state_sequence_;
            has_pending_state_ = false;
            if (pending_state_.source_timestamp_ns > last_source_timestamp_ns_ ||
                !has_source_timestamp_) {
                if (pending_sequence >= last_command_sequence_ ||
                    pending_state_.safety == RK_SAFETY_EMERGENCY_STOP ||
                    pending_state_.safety == RK_SAFETY_FAULT) {
                    last_source_timestamp_ns_ = pending_state_.source_timestamp_ns;
                    has_source_timestamp_ = true;
                    state = pending_state_;
                    return RK_OK;
                }
            }
        }

        while (extract_frame(input_, state_magic, maximum_frame_payload, payload)) {
            rk_robot_state decoded{};
            StateMetadata metadata{};
            if (!decode_state_payload(payload.data(), payload.size(), decoded, metadata))
                return RK_ERROR_BACKEND;
            if (metadata.session_id != session_id_)
                continue;
            if (metadata.last_command_sequence > last_command_sequence_)
                return RK_ERROR_BACKEND;
            if (has_source_timestamp_ && decoded.source_timestamp_ns <= last_source_timestamp_ns_)
                continue;
            if (metadata.last_command_sequence < last_command_sequence_ &&
                decoded.safety != RK_SAFETY_EMERGENCY_STOP &&
                decoded.safety != RK_SAFETY_FAULT)
                continue;
            last_source_timestamp_ns_ = decoded.source_timestamp_ns;
            has_source_timestamp_ = true;
            state = decoded;
            return RK_OK;
        }

        bool received_bytes = false;
        for (;;) {
            const auto result = ::read(descriptor_, chunk.data(), chunk.size());
            if (result > 0) {
                input_.insert(input_.end(), chunk.begin(), chunk.begin() + result);
                received_bytes = true;
                continue;
            }
            if (result < 0 && errno == EINTR) continue;
            if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            return RK_ERROR_BACKEND;
        }
        if (received_bytes) continue;

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return RK_ERROR_BACKEND;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        pollfd descriptor{descriptor_, POLLIN, 0};
        const int ready = ::poll(&descriptor, 1, static_cast<int>(std::max<int64_t>(1, remaining.count())));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))))
            return RK_ERROR_BACKEND;
        if (ready == 0) return RK_ERROR_BACKEND;
    }
}

} // namespace robotkit
