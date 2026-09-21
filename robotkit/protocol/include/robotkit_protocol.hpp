#ifndef ROBOTKIT_PROTOCOL_HPP
#define ROBOTKIT_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#if defined(_WIN32)
#if defined(RK_STATIC)
#define RK_PROTOCOL_API
#elif defined(RK_BUILDING_LIBRARY)
#define RK_PROTOCOL_API __declspec(dllexport)
#else
#define RK_PROTOCOL_API __declspec(dllimport)
#endif
#else
#define RK_PROTOCOL_API __attribute__((visibility("default")))
#endif

namespace robotkit::protocol {

constexpr uint32_t kMagic = 0x314B4252; // "RBK1" in little-endian storage.
constexpr uint16_t kVersion = 1;
constexpr std::size_t kHeaderSize = 24;
constexpr std::size_t kDefaultMaxPayload = 1024 * 1024;

enum class MessageType : uint16_t {
    Hello = 1,
    Capabilities = 2,
    Command = 3,
    CommandAck = 4,
    Snapshot = 5,
    RuntimeEvent = 6,
    Error = 7,
    Ping = 8
};

struct Frame {
    uint16_t version = kVersion;
    MessageType type = MessageType::Error;
    uint32_t flags = 0;
    uint64_t sequence = 0;
    std::vector<uint8_t> payload;
};

enum class DecodeStatus {
    Complete,
    NeedMore,
    Malformed
};

RK_PROTOCOL_API bool encode_frame(const Frame &frame, std::vector<uint8_t> &out);
RK_PROTOCOL_API DecodeStatus decode_frame(std::span<const uint8_t> input, Frame &out,
                                          std::size_t &consumed,
                                          std::size_t max_payload = kDefaultMaxPayload);

} // namespace robotkit::protocol

#endif
