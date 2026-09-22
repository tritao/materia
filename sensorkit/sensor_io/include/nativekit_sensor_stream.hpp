#pragma once

/* ------------------------------------------------------------------------- */
/* Dependencies                                                              */
/* ------------------------------------------------------------------------- */

#include "nativekit_sensor_wire.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

/* ------------------------------------------------------------------------- */
/* Export visibility                                                         */
/* ------------------------------------------------------------------------- */

#if defined(_WIN32)
#if defined(NKSENSOR_STREAM_STATIC)
#define NKSENSOR_STREAM_API
#elif defined(NKSENSOR_STREAM_BUILDING_LIBRARY)
#define NKSENSOR_STREAM_API __declspec(dllexport)
#else
#define NKSENSOR_STREAM_API __declspec(dllimport)
#endif
#else
#define NKSENSOR_STREAM_API __attribute__((visibility("default")))
#endif

namespace nksensor::stream {

/* ------------------------------------------------------------------------- */
/* Packet types                                                              */
/* ------------------------------------------------------------------------- */

/**
 * One immutable encoded sensor packet. All fanout subscribers receive the
 * same shared byte storage, so a recorder and live transport do not duplicate
 * the SensorWire payload.
 */
struct SensorPacket {
    SensorSampleHeader header;
    wire::MessageType type = wire::MessageType::camera_frame;
    std::shared_ptr<const std::vector<std::uint8_t>> encoded;

    bool valid() const noexcept { return encoded != nullptr; }

    std::span<const std::uint8_t> bytes() const noexcept {
        return encoded ? std::span<const std::uint8_t>(*encoded)
                       : std::span<const std::uint8_t>{};
    }
};

/* ------------------------------------------------------------------------- */
/* Packet creation                                                           */
/* ------------------------------------------------------------------------- */

/** Encode a core measurement once and attach its routing metadata. */
NKSENSOR_STREAM_API std::optional<SensorPacket> make_sensor_packet(
    const SensorMeasurement &measurement, std::string *error = nullptr,
    std::size_t max_messagepack_bytes = wire::default_max_messagepack_bytes);

/* ------------------------------------------------------------------------- */
/* Packet fanout                                                             */
/* ------------------------------------------------------------------------- */

using PacketSink = std::function<void(const SensorPacket &)>;
using SubscriptionId = std::uint64_t;
constexpr SubscriptionId invalid_subscription = 0;

/**
 * Synchronous packet multiplexer. Sinks may be live transports, recorders,
 * UI callbacks, or tests; none of those concerns enter SensorWire.
 */
class NKSENSOR_STREAM_API PacketFanout {
public:
    SubscriptionId subscribe(PacketSink sink);
    bool unsubscribe(SubscriptionId id);
    void publish(const SensorPacket &packet) const;
    std::size_t size() const noexcept { return sinks_.size(); }

private:
    struct SinkEntry {
        SubscriptionId id = invalid_subscription;
        PacketSink sink;
    };

    std::vector<SinkEntry> sinks_;
    SubscriptionId next_id_ = 1;
};

/* ------------------------------------------------------------------------- */
/* Packet buffer                                                             */
/* ------------------------------------------------------------------------- */

/**
 * Small deterministic recorder/replay source used by tests and local
 * playback. It stores shared packet bytes, preserving the exact arrival
 * order and wire representation without imposing a file format.
 */
class NKSENSOR_STREAM_API PacketBuffer {
public:
    void record(const SensorPacket &packet);
    PacketSink sink();
    void clear() noexcept { packets_.clear(); }

    std::size_t size() const noexcept { return packets_.size(); }
    std::span<const SensorPacket> packets() const noexcept { return packets_; }

    /** Immediately replay packets in their original order. */
    void replay(PacketFanout &fanout) const;

private:
    std::vector<SensorPacket> packets_;
};

} // namespace nksensor::stream
