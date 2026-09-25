#include "robotkit_device_host_v5.hpp"
#include "robotkit_device_protocol_v5.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <unistd.h>

#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); std::abort(); } } while (false)

namespace v5 = robotkit::v5;
namespace wire = robotkit::device_wire;

struct DeviceSim {
    int master;
    std::array<std::uint8_t, 16> fingerprint;
    v5::CommandDecoder decoder;
    std::atomic<bool> done{false};
    std::atomic<bool> reject_model{false};
    std::size_t stops = 0;
    std::size_t commands = 0;
    std::uint64_t timestamp = 0;

    DeviceSim(int fd, std::array<std::uint8_t, 16> id)
        : master(fd), fingerprint(id), decoder(1, id) {}

    void send(std::uint8_t type, std::span<const std::uint8_t> payload) {
        std::array<std::uint8_t, v5::max_frame_size> frame{};
        std::size_t size = 0;
        CHECK(v5::encode_frame(type, payload, frame, size));
        std::size_t offset = 0;
        while (offset < size) {
            const auto count = ::write(master, frame.data() + offset, std::min<std::size_t>(3, size - offset));
            if (count > 0) { offset += static_cast<std::size_t>(count); continue; }
            if (count < 0 && (errno == EAGAIN || errno == EINTR)) continue;
            CHECK(false);
        }
    }

    static bool on_session(void *context, std::uint64_t session) {
        auto &self = *static_cast<DeviceSim *>(context);
        ++self.stops;
        const wire::SessionAck ack{session, self.fingerprint,
            static_cast<std::uint8_t>(self.reject_model ? 2 : 1), {0, 0, 0}};
        std::array<std::uint8_t, wire::SessionAck::SIZE> body{};
        CHECK(wire::encode(ack, body));
        self.send(2, body);
        return true;
    }

    static bool on_command(void *context, const v5::Command &command) {
        auto &self = *static_cast<DeviceSim *>(context);
        ++self.commands;
        CHECK(command.header.kind == 4 || command.header.kind == 1);
        wire::StateHeader state{command.header.session, ++self.timestamp,
            command.header.sequence, 0, 0, 1, 0};
        std::array<std::uint8_t, wire::StateHeader::SIZE + wire::JointState::SIZE> body{};
        CHECK(wire::encode(state, std::span(body).first(wire::StateHeader::SIZE)));
        CHECK(wire::encode(wire::JointState{1.0f, -2.0f, 0.5f},
            std::span(body).subspan(wire::StateHeader::SIZE)));
        self.send(4, body);
        return true;
    }

    void run() {
        while (!done.load()) {
            pollfd readable{master, POLLIN, 0};
            if (::poll(&readable, 1, 10) <= 0 || !(readable.revents & POLLIN)) continue;
            std::array<std::uint8_t, 17> bytes{};
            const auto count = ::read(master, bytes.data(), bytes.size());
            if (count > 0) decoder.feed(std::span(bytes).first(static_cast<std::size_t>(count)),
                100, on_session, on_command, this);
        }
    }
};

int main() {
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    CHECK(master >= 0);
    CHECK(::grantpt(master) == 0 && ::unlockpt(master) == 0);
    const char *slave = ::ptsname(master);
    CHECK(slave != nullptr);
    std::array<std::uint8_t, 16> fingerprint{};
    for (std::size_t i = 0; i < fingerprint.size(); ++i)
        fingerprint[i] = static_cast<std::uint8_t>(i);
    DeviceSim sim(master, fingerprint);
    std::thread thread([&] { sim.run(); });
    {
        auto link = v5::HostLink::open(slave, 115200, fingerprint, 1);
        CHECK(link && link->ready() && link->last_session_status() == 1);
        CHECK(link->send_command(4));
        v5::HostState state{};
        CHECK(link->read_state(state));
        CHECK(state.header.safety == 0 && state.header.accepted_sequence == 1);
        const std::array<wire::JointTarget, 1> targets{{{0, 2, 0, 1.5f}}};
        CHECK(link->send_command(1, targets));
        CHECK(link->read_state(state));
        CHECK(state.header.accepted_sequence == 2 && state.joints[0].velocity == -2.0f);
        const auto previous_session = link->session_id();
        CHECK(previous_session != 0 && link->last_sent_sequence() == 2);
        CHECK(link->begin_session());
        CHECK(link->session_id() != previous_session && link->last_sent_sequence() == 0);
        CHECK(link->send_command(4) && link->read_state(state));
        CHECK(state.header.accepted_sequence == 1 && state.header.session == link->session_id());
    }
    auto wrong = fingerprint;
    wrong[0] ^= 1;
    sim.reject_model = true;
    const int rejected_fd = ::open(slave, O_RDWR | O_NOCTTY | O_NONBLOCK);
    CHECK(rejected_fd >= 0);
    v5::HostLink rejected(rejected_fd, true, 115200, wrong, 1);
    CHECK(!rejected.begin_session() && rejected.last_session_status() == 2);
    sim.done = true;
    thread.join();
    CHECK(sim.stops == 3 && sim.commands == 3);
    ::close(master);
}
