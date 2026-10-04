#include "robotkit_device_serial_endpoint.hpp"
#include "rkd6_endpoint.hpp"
#include "device_compiler6.hpp"
#include "device_frame6.hpp"
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <random>
#include <termios.h>
#include <unistd.h>

namespace robotkit {
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

class PosixRkd6Transport final : public Rkd6Transport {
public:
    PosixRkd6Transport(int fd, unsigned baud) : fd_(fd), baud_(baud) {}
    ~PosixRkd6Transport() override { if (fd_ >= 0) ::close(fd_); }
    unsigned baud() const noexcept override { return baud_; }
    std::optional<std::size_t> queued_output_bytes() const noexcept override {
        int bytes = 0;
        if (::ioctl(fd_, TIOCOUTQ, &bytes) != 0 || bytes < 0) return std::nullopt;
        return static_cast<std::size_t>(bytes);
    }
    bool send(std::span<const std::uint8_t> frame) override {
        while (!frame.empty()) {
            const auto n = ::write(fd_, frame.data(), frame.size());
            if (n > 0) { frame = frame.subspan(static_cast<std::size_t>(n)); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd poller{fd_, POLLOUT, 0};
                if (::poll(&poller, 1, 1000) > 0) continue;
            }
            return false;
        }
        return true;
    }
    bool receive(std::vector<std::uint8_t> &frame) override {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (true) {
            while (buffer_.size() >= device_frame6::HEADER_SIZE) {
                if (!std::equal(buffer_.begin(), buffer_.begin() + 4, "RKD6")) {
                    buffer_.erase(buffer_.begin()); continue;
                }
                const auto payload = static_cast<std::size_t>(buffer_[6]) |
                    (static_cast<std::size_t>(buffer_[7]) << 8);
                if (payload > device_frame6::MAX_PAYLOAD_SIZE) {
                    buffer_.erase(buffer_.begin()); continue;
                }
                const auto size = device_frame6::HEADER_SIZE + payload + device_frame6::CRC_SIZE;
                if (buffer_.size() < size) break;
                frame.assign(buffer_.begin(), buffer_.begin() + size);
                buffer_.erase(buffer_.begin(), buffer_.begin() + size);
                device_frame6::Frame decoded{};
                if (device_frame6::decode(frame, decoded)) {
                    if (decoded.kind == 2) initial_ = false;
                    return true;
                }
            }
            std::uint8_t bytes[4096];
            const auto n = ::read(fd_, bytes, sizeof(bytes));
            if (n > 0) { buffer_.insert(buffer_.end(), bytes, bytes + n); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (!initial_) return false;
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0) return false;
            pollfd poller{fd_, POLLIN, 0};
            if (::poll(&poller, 1, static_cast<int>(remaining)) <= 0) return false;
        }
    }
private:
    int fd_;
    unsigned baud_;
    bool initial_ = true;
    std::vector<std::uint8_t> buffer_;
};
} // namespace

namespace {
/** Opens and configures a raw 8N1 serial port; -1 with `error` set on failure. */
int open_port(const char *path, unsigned baud, rk_result *error) {
    if (error) *error = RK_ERROR_BACKEND;
    const auto speed = baud_value(baud);
    if (!path || !*path || !speed) {
        if (error) *error = RK_ERROR_UNSUPPORTED;
        return -1;
    }
    const auto fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;
    termios settings{};
    if (::tcgetattr(fd, &settings) != 0) { ::close(fd); return -1; }
    ::cfmakeraw(&settings);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cflag |= CLOCAL | CREAD;
    if (::tcsetattr(fd, TCSANOW, &settings) != 0) { ::close(fd); return -1; }
    return fd;
}

std::uint64_t random_session() {
    std::random_device random;
    auto session = (static_cast<std::uint64_t>(random()) << 32) | random();
    return session == 0 ? 1 : session;
}
} // namespace

std::shared_ptr<Rkd6Endpoint> DeviceSerialEndpoint::open(const char *path, unsigned baud,
    const rk_robot_runtime_blueprint &blueprint, std::array<std::uint8_t, 16> controller,
    double target_error, std::uint32_t step_tick_hz,
    std::uint64_t link_loss_timeout_ns, std::uint64_t clock_bound_ns,
    std::uint64_t link_latency_ns, std::span<const DeviceActuator6> layout, rk_result *error, std::span<const DeviceInput6> inputs) {
    const auto fd = open_port(path, baud, error);
    if (fd < 0) return {};
    return Rkd6Endpoint::attach(std::make_unique<PosixRkd6Transport>(fd, baud), blueprint,
        controller, random_session(), target_error, clock_bound_ns, link_latency_ns,
        step_tick_hz, link_loss_timeout_ns, layout, error, inputs);
}

rk_result DeviceSerialEndpoint::identify(const char *path, unsigned baud,
    std::array<std::uint8_t, 16> &controller) {
    rk_result error = RK_ERROR_BACKEND;
    const auto fd = open_port(path, baud, &error);
    if (fd < 0) return error;
    return Rkd6Endpoint::identify(std::make_unique<PosixRkd6Transport>(fd, baud),
        random_session(), controller);
}
} // namespace robotkit
