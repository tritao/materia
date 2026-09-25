#include "robotkit_serial_endpoint.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
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

constexpr std::array<std::uint8_t, 4> command_magic{'R', 'K', 'C', '3'};
constexpr std::array<std::uint8_t, 4> state_magic{'R', 'K', 'S', '3'};
constexpr std::size_t frame_header_bytes = 8;
constexpr std::size_t frame_checksum_bytes = 4;
constexpr std::size_t state_header_payload_bytes = 4 + 8 + 4;
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
    return std::make_shared<SerialRobotEndpoint>(descriptor, true);
}

SerialRobotEndpoint::SerialRobotEndpoint(int descriptor, bool take_ownership)
    : descriptor_(descriptor), owns_descriptor_(take_ownership) {
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
    last_source_timestamp_ns_ = 0;
    has_source_timestamp_ = false;
    const int flags = fcntl(descriptor_, F_GETFL, 0);
    if (flags >= 0 && fcntl(descriptor_, F_SETFL, flags | O_NONBLOCK) != 0)
        return RK_ERROR_BACKEND;
    return RK_OK;
}

rk_result SerialRobotEndpoint::apply(const rk_robot_command &command) {
    if (descriptor_ < 0)
        return RK_ERROR_BACKEND;
    if (command.target_count > RK_MAX_SERIAL_JOINTS)
        return RK_ERROR_LIMIT;
    for (uint32_t index = 0; index < command.target_count; ++index)
        if (command.targets[index].joint >= RK_MAX_SERIAL_JOINTS)
            return RK_ERROR_LIMIT;
    std::vector<std::uint8_t> payload;
    payload.reserve(20 + command.target_count * 16);
    append_u32(payload, command.kind);
    append_u64(payload, command.sequence);
    append_u32(payload, command.target_count);
    append_u32(payload, 0);
    for (uint32_t index = 0; index < command.target_count; ++index) {
        append_u32(payload, command.targets[index].joint);
        append_u32(payload, command.targets[index].mode);
        append_double(payload, command.targets[index].target);
    }
    auto packet = frame(command_magic, std::move(payload));
    return write_all(descriptor_, packet.data(), packet.size()) ? RK_OK : RK_ERROR_BACKEND;
}

rk_result SerialRobotEndpoint::sample(uint64_t, rk_robot_state &state) {
    if (descriptor_ < 0)
        return RK_ERROR_BACKEND;
    const auto deadline = std::chrono::steady_clock::now() + response_timeout;
    std::array<std::uint8_t, 4096> chunk{};
    while (true) {
        for (;;) {
            const auto result = ::read(descriptor_, chunk.data(), chunk.size());
            if (result > 0) {
                input_.insert(input_.end(), chunk.begin(), chunk.begin() + result);
                continue;
            }
            if (result < 0 && errno == EINTR) continue;
            if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
            if (result == 0) return RK_ERROR_BACKEND;
            return RK_ERROR_BACKEND;
        }

        while (!input_.empty()) {
            auto magic = std::search(input_.begin(), input_.end(),
                state_magic.begin(), state_magic.end());
            if (magic == input_.end()) {
                const auto keep = std::min<std::size_t>(input_.size(), state_magic.size() - 1);
                input_.erase(input_.begin(), input_.end() - keep);
                break;
            }
            if (magic != input_.begin()) input_.erase(input_.begin(), magic);
            if (input_.size() < frame_header_bytes) break;

            const auto payload_size = read_u32(input_.data() + 4);
            if (payload_size > maximum_frame_payload) {
                input_.erase(input_.begin());
                continue;
            }
            const auto frame_size = frame_header_bytes + payload_size + frame_checksum_bytes;
            if (input_.size() < frame_size) break;
            const auto expected_crc = read_u32(input_.data() + frame_header_bytes + payload_size);
            if (crc32(input_.data(), frame_header_bytes + payload_size) != expected_crc) {
                input_.erase(input_.begin());
                continue;
            }

            const auto *payload = input_.data() + frame_header_bytes;
            if (payload_size < state_header_payload_bytes) return RK_ERROR_BACKEND;
            const auto joint_count = read_u32(payload);
            const auto source_timestamp_ns = read_u64(payload + 4);
            const auto sensor_count = read_u32(payload + 12);
            if (joint_count > RK_MAX_SERIAL_JOINTS || sensor_count > RK_MAX_SENSORS)
                return RK_ERROR_BACKEND;
            std::size_t offset = state_header_payload_bytes;
            const auto joints_bytes = static_cast<std::size_t>(joint_count) * joint_state_bytes;
            if (offset + joints_bytes > payload_size) return RK_ERROR_BACKEND;

            rk_robot_state decoded{};
            decoded.struct_size = sizeof(decoded);
            decoded.joint_count = joint_count;
            decoded.source_timestamp_ns = source_timestamp_ns;
            for (uint32_t joint = 0; joint < joint_count; ++joint) {
                decoded.position[joint] = read_double(payload + offset); offset += 8;
                decoded.velocity[joint] = read_double(payload + offset); offset += 8;
                decoded.effort[joint] = read_double(payload + offset); offset += 8;
            }
            decoded.sensor_count = sensor_count;
            for (uint32_t index = 0; index < sensor_count; ++index) {
                if (offset + sensor_header_payload_bytes > payload_size) return RK_ERROR_BACKEND;
                auto &sensor = decoded.sensors[index];
                sensor.sequence = read_u64(payload + offset); offset += 8;
                sensor.source_timestamp_ns = read_u64(payload + offset); offset += 8;
                sensor.value_count = read_u32(payload + offset); offset += 4;
                if (sensor.value_count > RK_MAX_SENSOR_VALUES ||
                    offset + static_cast<std::size_t>(sensor.value_count) * 8 > payload_size)
                    return RK_ERROR_BACKEND;
                for (uint32_t value = 0; value < sensor.value_count; ++value) {
                    sensor.values[value] = read_double(payload + offset);
                    offset += 8;
                }
            }
            if (offset != payload_size) return RK_ERROR_BACKEND;
            input_.erase(input_.begin(), input_.begin() + frame_size);
            if (has_source_timestamp_ && source_timestamp_ns <= last_source_timestamp_ns_)
                continue;
            last_source_timestamp_ns_ = source_timestamp_ns;
            has_source_timestamp_ = true;
            state = decoded;
            return RK_OK;
        }

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
