#include "device_wire6.hpp"
#include "device_frame6.hpp"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

using namespace robotkit::device_wire6;

static std::vector<std::uint8_t> from_hex(const std::string &hex) {
    std::vector<std::uint8_t> result;
    for (std::size_t i = 0; i < hex.size(); i += 2)
        result.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    return result;
}

int main(int argc, char **argv) {
    assert(argc == 2);
    TimeSyncRequest request{123456789};
    std::vector<std::uint8_t> payload(request.SIZE);
    assert(encode(request, payload));
    TimeSyncRequest decoded{};
    assert(decode(payload, decoded) && decoded.host_send_ns == request.host_send_ns);
    SessionBegin6 begin{};
    begin.session = 7;
    begin.protocol_version = PROTOCOL_VERSION;
    begin.actuator_count = 2;
    begin.max_degree = 5;
    begin.step_tick_hz = 40'000;
    begin.max_acceleration = 4.0f;
    begin.actuator_max_acceleration[0] = 2.0f;
    begin.actuator_max_acceleration[1] = 4.0f;
    begin.steps_per_unit[0] = begin.steps_per_unit[1] = 400.0f;
    begin.min_step_ticks[0] = begin.min_step_ticks[1] = 1;
    begin.actuator_ratio[0] = begin.actuator_ratio[1] = 1.0f;
    begin.link_loss_timeout_ns = 500'000'000;
    std::vector<std::uint8_t> session(begin.SIZE);
    assert(encode(begin, session));
    std::vector<std::uint8_t> session_frame;
    assert(robotkit::device_frame6::encode(1, session, session_frame));
    robotkit::device_frame6::Frame decoded_session{};
    assert(robotkit::device_frame6::decode(session_frame, decoded_session));
    // Physical homing controls and acknowledgments cross the same frame gate
    // as ordinary queue records, including the highest assigned message kind.
    auto check_homing_frame = [](std::uint8_t kind, const auto &record) {
        std::vector<std::uint8_t> body(record.SIZE), bytes;
        assert(encode(record, body));
        assert(robotkit::device_frame6::encode(kind, body, bytes));
        robotkit::device_frame6::Frame frame{};
        assert(robotkit::device_frame6::decode(bytes, frame));
        assert(frame.kind == kind && frame.payload.size() == record.SIZE);
    };
    check_homing_frame(21, HomingScope6{7, 1, 1, 0, 0, 1, 0.01f});
    check_homing_frame(22, HomingSide6{7, 2, 1, 0, 1});
    check_homing_frame(23, HomingControlAck6{7, 2, 1, 1});
    check_homing_frame(24, HomingCounterBatch6{7, 3, 1, 0, 1, 0.1, -0.1});
    for (std::uint8_t kind = 18; kind <= 20; ++kind) {
        std::vector<std::uint8_t> rejected;
        assert(!robotkit::device_frame6::encode(kind, {}, rejected));
    }
    std::ifstream input(argv[1]);
    assert(input.good());
    std::string line;
    bool seen = false;
    while (std::getline(input, line)) {
        const auto tab = line.find('\t');
        assert(tab != std::string::npos);
        auto bytes = from_hex(line.substr(tab + 1));
        assert(bytes.size() >= 12 && bytes[0] == 'R' && bytes[3] == '6');
        robotkit::device_frame6::Frame frame{};
        assert(robotkit::device_frame6::decode(bytes, frame));
        std::vector<std::uint8_t> encoded;
        assert(robotkit::device_frame6::encode(frame.kind, frame.payload, encoded));
        assert(encoded == bytes);
        auto corrupt = bytes;
        corrupt.back() ^= 1;
        assert(!robotkit::device_frame6::decode(corrupt, frame));
        if (line.substr(0, tab) == "time_sync_request") {
            assert(bytes[4] == 3 && bytes[6] == request.SIZE);
            assert(std::equal(payload.begin(), payload.end(), bytes.begin() + 8));
            seen = true;
        }
    }
    assert(seen);
}
