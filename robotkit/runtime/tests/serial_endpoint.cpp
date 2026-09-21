#include "robotkit_serial_endpoint.hpp"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

namespace {
#pragma pack(push, 1)
struct WireCommand {
    std::uint8_t magic[4];
    std::uint32_t kind;
    std::uint64_t sequence;
    std::uint32_t target_count;
    std::uint32_t reserved;
    struct Target { std::uint32_t mode; double value; } targets[RK_MAX_JOINTS];
};
struct WireState {
    std::uint8_t magic[4];
    std::uint32_t joint_count;
    std::uint64_t source_timestamp_ns;
    std::uint32_t mode;
    double position[RK_MAX_JOINTS];
    double velocity[RK_MAX_JOINTS];
    double effort[RK_MAX_JOINTS];
};
#pragma pack(pop)
}

int main() {
    int sockets[2]{};
    assert(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    auto endpoint = std::make_shared<robotkit::SerialRobotEndpoint>(sockets[0]);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 7;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 1;
    command.targets[0] = {0, RK_TARGET_POSITION, 0.5, 0.0, 0.0};
    assert(endpoint->apply(command) == RK_OK);
    WireCommand received{};
    assert(::read(sockets[1], &received, sizeof(received)) == sizeof(received));
    assert(received.sequence == 7 && received.target_count == 1);
    assert(received.targets[0].value == 0.5);

    WireState state_packet{};
    state_packet.magic[0] = 'R'; state_packet.magic[1] = 'K';
    state_packet.magic[2] = 'S'; state_packet.magic[3] = '1';
    state_packet.joint_count = 1;
    state_packet.source_timestamp_ns = 1234;
    state_packet.position[0] = 0.25;
    assert(::write(sockets[1], &state_packet, sizeof(state_packet)) == sizeof(state_packet));
    rk_robot_state state{};
    assert(endpoint->sample(5678, state) == RK_OK);
    assert(state.source_timestamp_ns == 1234);
    assert(state.received_timestamp_ns == 5678);
    assert(state.position[0] == 0.25);
    ::close(sockets[1]);
    assert(endpoint->sample(5679, state) == RK_ERROR_BACKEND);
}
