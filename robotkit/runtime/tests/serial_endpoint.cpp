#include "robotkit_serial_endpoint.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {

void append_u32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}
void append_u64(std::vector<std::uint8_t> &bytes, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}
void append_double(std::vector<std::uint8_t> &bytes, double value) {
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(bytes, bits);
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
    const auto bits = read_u64(bytes);
    double value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
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
                                const std::vector<std::uint8_t> &payload) {
    std::vector<std::uint8_t> result(magic.begin(), magic.end());
    append_u32(result, static_cast<std::uint32_t>(payload.size()));
    result.insert(result.end(), payload.begin(), payload.end());
    append_u32(result, crc32(result.data(), result.size()));
    return result;
}
void write_all(int descriptor, const std::uint8_t *bytes, std::size_t size) {
    std::size_t written = 0;
    while (written < size) {
        const auto result = ::write(descriptor, bytes + written, size - written);
        assert(result > 0);
        written += static_cast<std::size_t>(result);
    }
}
void read_all(int descriptor, std::uint8_t *bytes, std::size_t size) {
    std::size_t received = 0;
    while (received < size) {
        const auto result = ::read(descriptor, bytes + received, size - received);
        assert(result > 0);
        received += static_cast<std::size_t>(result);
    }
}
std::vector<std::uint8_t> state_frame(std::uint64_t timestamp_ns, std::uint32_t joint_count = 1) {
    const auto sensor_timestamp = timestamp_ns >= 10 ? timestamp_ns - 10 : timestamp_ns;
    const auto later_sensor_timestamp = timestamp_ns >= 5 ? timestamp_ns - 5 : timestamp_ns;
    std::vector<std::uint8_t> payload;
    append_u32(payload, joint_count);
    append_u64(payload, timestamp_ns);
    append_u32(payload, 2); // IMU and LiDAR slots
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
        append_double(payload, 0.25 + joint);
        append_double(payload, 0.5 + joint);
        append_double(payload, 1.5 + joint);
    }
    append_u64(payload, 3); append_u64(payload, sensor_timestamp); append_u32(payload, 6);
    for (double value : {0.0, 0.1, 0.2, 1.0, 2.0, 9.81}) append_double(payload, value);
    append_u64(payload, 4); append_u64(payload, later_sensor_timestamp); append_u32(payload, 3);
    for (double value : {1.2, 1.4, 1.6}) append_double(payload, value);
    return frame({'R', 'K', 'S', '3'}, payload);
}

} // namespace

int main() {
    int sockets[2]{};
    assert(::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    auto endpoint = std::make_shared<robotkit::SerialRobotEndpoint>(sockets[0]);

    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = 7;
    command.kind = RK_COMMAND_JOINT_TARGETS;
    command.target_count = 2;
    command.targets[0] = {2, RK_TARGET_POSITION, 0.5, 0.0, 0.0};
    command.targets[1] = {5, RK_TARGET_EFFORT, 3.0, 0.0, 0.0};
    assert(endpoint->apply(command) == RK_OK);
    std::array<std::uint8_t, 8> command_header{};
    read_all(sockets[1], command_header.data(), command_header.size());
    assert(command_header[0] == 'R' && command_header[1] == 'K'
        && command_header[2] == 'C' && command_header[3] == '3');
    assert(read_u32(command_header.data() + 4) == 20 + 2 * 16);
    std::vector<std::uint8_t> command_tail(read_u32(command_header.data() + 4) + 4);
    read_all(sockets[1], command_tail.data(), command_tail.size());
    auto command_payload = command_tail.data();
    assert(read_u32(command_payload) == RK_COMMAND_JOINT_TARGETS);
    assert(read_u64(command_payload + 4) == 7);
    assert(read_u32(command_payload + 12) == 2);
    assert(read_u32(command_payload + 20) == 2);
    assert(read_u32(command_payload + 24) == RK_TARGET_POSITION);
    assert(read_double(command_payload + 28) == 0.5);
    assert(read_u32(command_payload + 36) == 5);
    assert(read_u32(command_payload + 40) == RK_TARGET_EFFORT);
    assert(read_double(command_payload + 44) == 3.0);
    std::vector<std::uint8_t> checksum_input(command_header.begin(), command_header.end());
    checksum_input.insert(checksum_input.end(), command_tail.begin(), command_tail.end() - 4);
    assert(read_u32(command_tail.data() + command_tail.size() - 4) ==
        crc32(checksum_input.data(), checksum_input.size()));
    command.targets[0].joint = RK_MAX_SERIAL_JOINTS;
    assert(endpoint->apply(command) == RK_ERROR_LIMIT);

    auto state_packet = state_frame(1234);
    const auto split = state_packet.size() / 2;
    write_all(sockets[1], state_packet.data(), split);
    std::thread finish_packet([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        write_all(sockets[1], state_packet.data() + split, state_packet.size() - split);
    });
    rk_robot_state state{};
    assert(endpoint->sample(5678, state) == RK_OK);
    finish_packet.join();
    assert(state.source_timestamp_ns == 1234);
    assert(state.position[0] == 0.25 && state.velocity[0] == 0.5 && state.effort[0] == 1.5);
    assert(state.sensor_count == 2);
    assert(state.sensors[0].sequence == 3 && state.sensors[0].source_timestamp_ns == 1224);
    assert(state.sensors[0].value_count == 6 && state.sensors[0].values[5] == 9.81);
    assert(state.sensors[1].sequence == 4 && state.sensors[1].source_timestamp_ns == 1229);
    assert(state.sensors[1].value_count == 3 && state.sensors[1].values[2] == 1.6);

    auto damaged = state_frame(1235);
    damaged.back() ^= 0x80;
    auto fresh = state_frame(1236);
    write_all(sockets[1], damaged.data(), damaged.size());
    write_all(sockets[1], fresh.data(), fresh.size());
    assert(endpoint->sample(5680, state) == RK_OK);
    assert(state.source_timestamp_ns == 1236);

    ::close(sockets[1]);
    assert(endpoint->sample(5681, state) == RK_ERROR_BACKEND);

    int replacement[2]{};
    assert(::socketpair(AF_UNIX, SOCK_STREAM, 0, replacement) == 0);
    assert(endpoint->reconnect(replacement[0]) == RK_OK);
    auto restarted = state_frame(1);
    write_all(replacement[1], restarted.data(), restarted.size());
    assert(endpoint->sample(5682, state) == RK_OK);
    assert(state.source_timestamp_ns == 1);
    ::close(replacement[1]);

    const int serial_master = ::posix_openpt(O_RDWR | O_NOCTTY);
    assert(serial_master >= 0);
    assert(::grantpt(serial_master) == 0);
    assert(::unlockpt(serial_master) == 0);
    const char *serial_path = ::ptsname(serial_master);
    assert(serial_path != nullptr);
    assert(!robotkit::SerialRobotEndpoint::open(serial_path, 12345));
    auto serial = robotkit::SerialRobotEndpoint::open(serial_path, 115200);
    assert(serial);

    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
    blueprint.joint_count = 6;
    blueprint.link_count = 7;
    for (std::uint32_t joint = 0; joint < blueprint.joint_count; ++joint)
        blueprint.joints[joint] = {joint, RK_RUNTIME_JOINT_REVOLUTE, joint, joint + 1,
            -10.0, 10.0, 10.0};
    blueprint.sensor_count = 2;
    blueprint.sensors[0].kind = RK_SENSOR_IMU;
    blueprint.sensors[0].link = 0;
    blueprint.sensors[0].rotation[3] = 1.0;
    blueprint.sensors[1].kind = RK_SENSOR_LIDAR;
    blueprint.sensors[1].link = 0;
    blueprint.sensors[1].rotation[3] = 1.0;
    blueprint.sensors[1].ray_count = 3;
    blueprint.sensors[1].max_range = 10.0;
    blueprint.sensors[1].field_of_view = 1.0;

    robotkit::RobotRuntime serial_runtime(blueprint, serial, std::chrono::milliseconds(100));
    assert(serial_runtime.submit(command) == RK_OK);
    assert(serial_runtime.apply_pending_commands() == RK_OK);
    read_all(serial_master, command_header.data(), command_header.size());
    assert(command_header[0] == 'R' && command_header[1] == 'K'
        && command_header[2] == 'C' && command_header[3] == '3');
    command_tail.resize(read_u32(command_header.data() + 4) + 4);
    read_all(serial_master, command_tail.data(), command_tail.size());
    const auto runtime_command = command_tail.data();
    assert(read_u32(runtime_command) == RK_COMMAND_JOINT_TARGETS);
    assert(read_u64(runtime_command + 4) == 1);
    assert(read_u32(runtime_command + 12) == 2);
    assert(read_u32(runtime_command + 20) == 2);
    assert(read_u32(runtime_command + 24) == RK_TARGET_POSITION);
    assert(read_double(runtime_command + 28) == 0.5);
    assert(read_u32(runtime_command + 36) == 5);
    assert(read_u32(runtime_command + 40) == RK_TARGET_EFFORT);
    assert(read_double(runtime_command + 44) == 3.0);

    auto serial_state = state_frame(2000, blueprint.joint_count);
    write_all(serial_master, serial_state.data(), serial_state.size());
    assert(serial_runtime.publish_sample(6000) == RK_OK);
    assert(serial_runtime.snapshot(state) == RK_OK);
    assert(state.sequence == 1 && state.source_timestamp_ns == 2000);
    assert(state.mode == RK_ROBOT_MODE_TRACKING && state.safety == RK_SAFETY_READY);
    assert(state.joint_count == blueprint.joint_count && state.position[5] == 5.25);
    assert(state.sensor_count == 2 && state.sensors[0].sequence == 3);
    assert(state.sensors[1].sequence == 4 && state.sensors[1].values[2] == 1.6);
    serial.reset();
    ::close(serial_master);
}
