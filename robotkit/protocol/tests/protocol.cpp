#include "robotkit_protocol.hpp"

#include <cassert>
#include <cstdint>
#include <vector>

int main() {
    using namespace robotkit::protocol;

    Frame frame;
    frame.type = MessageType::Command;
    frame.flags = 3;
    frame.sequence = 42;
    frame.payload = {1, 2, 3, 4};

    std::vector<uint8_t> encoded;
    assert(encode_frame(frame, encoded));
    assert(encoded.size() == kHeaderSize + frame.payload.size());

    Frame decoded;
    std::size_t consumed = 0;
    assert(decode_frame(std::span<const uint8_t>(encoded).first(5), decoded, consumed) ==
           DecodeStatus::NeedMore);
    assert(decode_frame(encoded, decoded, consumed) == DecodeStatus::Complete);
    assert(consumed == encoded.size());
    assert(decoded.type == frame.type);
    assert(decoded.flags == frame.flags);
    assert(decoded.sequence == frame.sequence);
    assert(decoded.payload == frame.payload);

    auto malformed = encoded;
    malformed[0] = 0;
    assert(decode_frame(malformed, decoded, consumed) == DecodeStatus::Malformed);

    auto oversized = encoded;
    oversized[16] = 5;
    assert(decode_frame(oversized, decoded, consumed, 4) == DecodeStatus::Malformed);
    return 0;
}
