#include "robotkit_serial_endpoint.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <memory>

#if defined(_WIN32)
#error "SerialRobotEndpoint currently targets POSIX serial devices"
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace robotkit {
namespace {

constexpr std::array<std::uint8_t, 4> command_magic{'R', 'K', 'C', '1'};
constexpr std::array<std::uint8_t, 4> state_magic{'R', 'K', 'S', '1'};
constexpr std::size_t command_bytes = 4 + 4 + 8 + 4 + 4 + RK_MAX_SERIAL_JOINTS * (4 + 8);
constexpr std::size_t state_bytes = 4 + 4 + 8 + 4 + RK_MAX_SERIAL_JOINTS * 8 * 3;

#pragma pack(push, 1)
struct WireCommand {
    std::uint8_t magic[4];
    std::uint32_t kind;
    std::uint64_t sequence;
    std::uint32_t target_count;
    std::uint32_t reserved;
    struct Target { std::uint32_t mode; double value; } targets[RK_MAX_SERIAL_JOINTS];
};
struct WireState {
    std::uint8_t magic[4];
    std::uint32_t joint_count;
    std::uint64_t source_timestamp_ns;
    std::uint32_t mode;
    double position[RK_MAX_SERIAL_JOINTS];
    double velocity[RK_MAX_SERIAL_JOINTS];
    double effort[RK_MAX_SERIAL_JOINTS];
};
#pragma pack(pop)

static_assert(sizeof(WireCommand) == command_bytes);
static_assert(sizeof(WireState) == state_bytes);

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

bool write_all(int descriptor, const void *data, std::size_t size) {
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    std::size_t written = 0;
    while (written < size) {
        const auto result = ::write(descriptor, bytes + written, size - written);
        if (result > 0) { written += static_cast<std::size_t>(result); continue; }
        if (result < 0 && (errno == EINTR)) continue;
        return false;
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
    cfsetispeed(&settings, baud_value(baud));
    cfsetospeed(&settings, baud_value(baud));
    settings.c_cflag |= CLOCAL | CREAD;
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
    WireCommand packet{};
    std::copy(command_magic.begin(), command_magic.end(), packet.magic);
    packet.kind = command.kind;
    packet.sequence = command.sequence;
    packet.target_count = command.target_count;
    for (uint32_t index = 0; index < command.target_count; ++index) {
        packet.targets[index].mode = command.targets[index].mode;
        packet.targets[index].value = command.targets[index].target;
    }
    return write_all(descriptor_, &packet, sizeof(packet)) ? RK_OK : RK_ERROR_BACKEND;
}

rk_result SerialRobotEndpoint::sample(uint64_t timestamp_ns, rk_robot_state &state) {
    if (descriptor_ < 0)
        return RK_ERROR_BACKEND;
    std::array<std::uint8_t, 4096> chunk{};
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
    bool saw_stale = false;
    while (input_.size() >= state_bytes) {
        if (!std::equal(state_magic.begin(), state_magic.end(), input_.begin())) {
            input_.erase(input_.begin());
            continue;
        }
        WireState packet{};
        std::memcpy(&packet, input_.data(), sizeof(packet));
        input_.erase(input_.begin(), input_.begin() + sizeof(packet));
        if (packet.joint_count > RK_MAX_SERIAL_JOINTS)
            return RK_ERROR_BACKEND;
        if (has_source_timestamp_ && packet.source_timestamp_ns <= last_source_timestamp_ns_) {
            saw_stale = true;
            continue;
        }
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = packet.source_timestamp_ns;
        state.received_timestamp_ns = timestamp_ns;
        state.joint_count = packet.joint_count;
        state.mode = packet.mode;
        for (uint32_t index = 0; index < packet.joint_count; ++index) {
            state.position[index] = packet.position[index];
            state.velocity[index] = packet.velocity[index];
            state.effort[index] = packet.effort[index];
        }
        state.safety = RK_SAFETY_READY;
        last_source_timestamp_ns_ = packet.source_timestamp_ns;
        has_source_timestamp_ = true;
        return RK_OK;
    }
    return saw_stale ? RK_ERROR_STALE_STATE : RK_ERROR_INVALID_STATE;
}

} // namespace robotkit
