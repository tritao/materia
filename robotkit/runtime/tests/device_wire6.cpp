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
