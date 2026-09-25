#include "robotkit_device_host_v5.hpp"
#include "robotkit_device_protocol_v5.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

#if defined(_WIN32)
#error "RobotKit v5 HostLink currently targets POSIX serial devices"
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace robotkit::v5 {
namespace {

speed_t baud_value(unsigned baud) {
    switch (baud) {
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return 0;
    }
}

std::uint64_t random_session() {
    std::random_device random;
    const auto value = (static_cast<std::uint64_t>(random()) << 32) | random();
    return value ? value : 1;
}

int remaining_ms(std::chrono::steady_clock::time_point deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return 0;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    return static_cast<int>(std::max<std::int64_t>(1, ms));
}

} // namespace

HostLink::HostLink(int descriptor, bool take_ownership, unsigned baud,
                   std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count)
    : descriptor_(descriptor), owns_descriptor_(take_ownership), baud_(baud),
      fingerprint_(fingerprint), joint_count_(joint_count) {
    input_.reserve(max_frame_size);
}

HostLink::~HostLink() {
    if (owns_descriptor_ && descriptor_ >= 0) ::close(descriptor_);
}

std::unique_ptr<HostLink> HostLink::open(const char *path, unsigned baud,
    std::array<std::uint8_t, 16> fingerprint, std::uint8_t joint_count) {
    if (!path || !baud_value(baud) || joint_count > device_wire::MAX_JOINTS) return {};
    const int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return {};
    termios settings{};
    if (tcgetattr(fd, &settings) != 0) { ::close(fd); return {}; }
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    cfsetispeed(&settings, baud_value(baud));
    cfsetospeed(&settings, baud_value(baud));
    if (tcsetattr(fd, TCSANOW, &settings) != 0) { ::close(fd); return {}; }
    auto link = std::unique_ptr<HostLink>(new HostLink(fd, true, baud, fingerprint, joint_count));
    if (!link->begin_session()) return {};
    return link;
}

bool HostLink::write_frame(std::uint8_t type, std::span<const std::uint8_t> payload) {
    if (descriptor_ < 0 || !response_deadline_us(baud_)) return false;
    std::array<std::uint8_t, max_frame_size> bytes{};
    std::size_t size = 0;
    if (!encode_frame(type, payload, bytes, size)) return false;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(response_deadline_us(baud_));
    std::size_t offset = 0;
    while (offset < size) {
        const auto count = ::write(descriptor_, bytes.data() + offset, size - offset);
        if (count > 0) { offset += static_cast<std::size_t>(count); continue; }
        if (count < 0 && errno == EINTR) continue;
        if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        const auto wait = remaining_ms(deadline);
        if (!wait) return false;
        pollfd writable{descriptor_, POLLOUT, 0};
        const auto result = ::poll(&writable, 1, wait);
        if (result < 0 && errno == EINTR) continue;
        if (result <= 0 || (writable.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    }
    return true;
}

bool HostLink::next_frame(std::uint8_t &type, std::vector<std::uint8_t> &payload,
                          std::chrono::steady_clock::time_point deadline) {
    while (true) {
        const auto start = std::search(input_.begin(), input_.end(), magic.begin(), magic.end());
        if (start == input_.end()) {
            std::size_t keep = 0;
            for (std::size_t count = std::min(input_.size(), magic.size() - 1); count; --count) {
                if (std::equal(input_.end() - count, input_.end(), magic.begin())) {
                    keep = count;
                    break;
                }
            }
            input_.erase(input_.begin(), input_.end() - keep);
        } else if (start != input_.begin()) {
            input_.erase(input_.begin(), start);
        }
        if (input_.size() >= header_size) {
            const auto length = static_cast<std::size_t>(input_[6]) |
                (static_cast<std::size_t>(input_[7]) << 8);
            if (length > max_state_payload) { input_.erase(input_.begin()); continue; }
            const auto size = header_size + length + crc_size;
            if (input_.size() >= size) {
                if (read_u32(input_.data() + header_size + length) !=
                    crc32(std::span<const std::uint8_t>(input_.data(), header_size + length))) {
                    input_.erase(input_.begin());
                    continue;
                }
                type = input_[4];
                const auto flags = input_[5];
                payload.assign(input_.begin() + header_size, input_.begin() + header_size + length);
                input_.erase(input_.begin(), input_.begin() + size);
                if (flags || type < 1 || type > 4) continue;
                return true;
            }
        }
        const auto wait = remaining_ms(deadline);
        if (!wait) return false;
        pollfd readable{descriptor_, POLLIN, 0};
        const auto ready = ::poll(&readable, 1, wait);
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || (readable.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
        std::array<std::uint8_t, 256> chunk{};
        const auto count = ::read(descriptor_, chunk.data(), chunk.size());
        if (count > 0) input_.insert(input_.end(), chunk.begin(), chunk.begin() + count);
        else if (count == 0) return false;
        else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return false;
    }
}

bool HostLink::begin_session() {
    ready_ = false;
    session_id_ = 0;
    last_sent_sequence_ = 0;
    last_timestamp_ns_ = 0;
    has_timestamp_ = false;
    last_session_status_ = 0;
    input_.clear();
    if (joint_count_ > device_wire::MAX_JOINTS || !response_deadline_us(baud_)) return false;
    const auto requested = random_session();
    const device_wire::SessionBegin begin{requested, fingerprint_};
    std::array<std::uint8_t, device_wire::SessionBegin::SIZE> body{};
    if (!device_wire::encode(begin, body) || !write_frame(1, body)) return false;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(response_deadline_us(baud_));
    std::uint8_t type = 0;
    std::vector<std::uint8_t> payload;
    while (next_frame(type, payload, deadline)) {
        if (type != 2) continue;
        device_wire::SessionAck ack{};
        if (!device_wire::decode(payload, ack) || ack.session != requested ||
            ack.reserved != std::array<std::uint8_t, 3>{} ||
            (ack.status != 1 && ack.status != 2)) continue;
        last_session_status_ = ack.status;
        if (ack.status != 1) return false;
        if (ack.device_fingerprint != fingerprint_) continue;
        session_id_ = requested;
        ready_ = true;
        return true;
    }
    return false;
}

bool HostLink::send_command(std::uint8_t kind, std::span<const device_wire::JointTarget> targets) {
    if (!ready_ || last_sent_sequence_ == UINT64_MAX || kind < 1 || kind > 4 ||
        targets.size() > joint_count_ || (kind == 1) != !targets.empty()) return false;
    std::array<bool, device_wire::MAX_JOINTS> used{};
    for (const auto &target : targets) {
        if (target.joint >= joint_count_ || used[target.joint] || target.mode < 1 ||
            target.mode > 3 || target.flags || !std::isfinite(target.value)) return false;
        used[target.joint] = true;
    }
    const auto sequence = last_sent_sequence_ + 1;
    const device_wire::CommandHeader header{session_id_, sequence, kind,
        static_cast<std::uint8_t>(targets.size()), 0};
    std::array<std::uint8_t, max_command_payload> body{};
    if (!device_wire::encode(header, std::span(body).first(device_wire::CommandHeader::SIZE))) return false;
    for (std::size_t index = 0; index < targets.size(); ++index) {
        const auto offset = device_wire::CommandHeader::SIZE + index * device_wire::JointTarget::SIZE;
        if (!device_wire::encode(targets[index], std::span(body).subspan(offset, device_wire::JointTarget::SIZE))) return false;
    }
    if (!write_frame(3, std::span(body).first(device_wire::CommandHeader::SIZE +
        targets.size() * device_wire::JointTarget::SIZE))) {
        ready_ = false;
        return false;
    }
    last_sent_sequence_ = sequence;
    return true;
}

bool HostLink::send_targets(std::span<const HostTarget> targets, double max_absolute_error) {
    if (targets.empty() || targets.size() > joint_count_ ||
        !std::isfinite(max_absolute_error) || max_absolute_error < 0.0) return false;
    std::array<device_wire::JointTarget, device_wire::MAX_JOINTS> wire_targets{};
    for (std::size_t index = 0; index < targets.size(); ++index) {
        const auto &target = targets[index];
        if (!std::isfinite(target.value) ||
            std::abs(target.value) > std::numeric_limits<float>::max()) return false;
        const auto value = static_cast<float>(target.value);
        if (!std::isfinite(value) || (target.value != 0.0 && value == 0.0f) ||
            std::abs(static_cast<double>(value) - target.value) > max_absolute_error) return false;
        wire_targets[index] = {target.joint, target.mode, 0, value};
    }
    return send_command(1, std::span(wire_targets).first(targets.size()));
}

bool HostLink::read_state(HostState &state) {
    if (!ready_) return false;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::microseconds(response_deadline_us(baud_));
    std::uint8_t type = 0;
    std::vector<std::uint8_t> payload;
    while (next_frame(type, payload, deadline)) {
        if (type != 4 || payload.size() < device_wire::StateHeader::SIZE) continue;
        HostState decoded{};
        if (!device_wire::decode(std::span(payload).first(device_wire::StateHeader::SIZE), decoded.header)) continue;
        const auto &head = decoded.header;
        if (head.session != session_id_ || head.reserved || head.joint_count != joint_count_ ||
            head.safety > 3 || payload.size() != device_wire::StateHeader::SIZE +
            head.joint_count * device_wire::JointState::SIZE ||
            (has_timestamp_ && head.timestamp_ns <= last_timestamp_ns_) ||
            head.accepted_sequence > last_sent_sequence_ ||
            (head.accepted_sequence < last_sent_sequence_ && head.safety < 2)) continue;
        bool valid = true;
        for (std::size_t index = 0; index < head.joint_count; ++index) {
            const auto offset = device_wire::StateHeader::SIZE + index * device_wire::JointState::SIZE;
            auto &joint = decoded.joints[index];
            if (!device_wire::decode(std::span(payload).subspan(offset, device_wire::JointState::SIZE), joint) ||
                !std::isfinite(joint.position) || !std::isfinite(joint.velocity) ||
                !std::isfinite(joint.effort)) { valid = false; break; }
        }
        if (!valid) continue;
        last_timestamp_ns_ = head.timestamp_ns;
        has_timestamp_ = true;
        state = decoded;
        return true;
    }
    return false;
}

} // namespace robotkit::v5
