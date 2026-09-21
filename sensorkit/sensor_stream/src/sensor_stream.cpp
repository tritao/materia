#include "nativekit_sensor_stream.hpp"

#include <algorithm>
#include <type_traits>
#include <utility>

namespace nksensor::stream {
namespace {

template <typename Value>
wire::MessageType message_type_for() noexcept {
    if constexpr (std::is_same_v<Value, ImuSample>)
        return wire::MessageType::imu_sample;
    else if constexpr (std::is_same_v<Value, LidarScan>)
        return wire::MessageType::lidar_scan;
    else if constexpr (std::is_same_v<Value, CameraFrame>)
        return wire::MessageType::camera_frame;
    else if constexpr (std::is_same_v<Value, DepthFrame>)
        return wire::MessageType::depth_frame;
    else
        return wire::MessageType::segmentation_frame;
}

} // namespace

std::optional<SensorPacket> make_sensor_packet(
    const SensorMeasurement &measurement, std::string *error,
    std::size_t max_messagepack_bytes) {
    auto encoded = wire::encode_sensor_measurement(
        measurement, error, max_messagepack_bytes);
    if (!encoded.has_value())
        return std::nullopt;

    SensorPacket packet;
    std::visit(
        [&packet](const auto &value) {
            using Value = std::decay_t<decltype(value)>;
            packet.header = value.header;
            packet.type = message_type_for<Value>();
        },
        measurement);
    /* Move the codec's one allocation into shared immutable storage so the
     * fanout does not create another packet copy before dispatch. */
    packet.encoded = std::make_shared<std::vector<std::uint8_t>>(std::move(*encoded));
    return packet;
}

SubscriptionId PacketFanout::subscribe(PacketSink sink) {
    if (!sink)
        return invalid_subscription;

    const auto id = next_id_++;
    if (next_id_ == invalid_subscription)
        next_id_ = 1;
    sinks_.push_back({id, std::move(sink)});
    return id;
}

bool PacketFanout::unsubscribe(SubscriptionId id) {
    const auto found = std::remove_if(sinks_.begin(), sinks_.end(),
                                      [id](const SinkEntry &entry) {
                                          return entry.id == id;
                                      });
    if (found == sinks_.end())
        return false;
    sinks_.erase(found, sinks_.end());
    return true;
}

void PacketFanout::publish(const SensorPacket &packet) const {
    /* Copy the callback list so a sink can subscribe or unsubscribe while it
     * handles a packet without invalidating this dispatch. Every callback
     * still receives the same packet and shared encoded byte buffer. */
    const auto sinks = sinks_;
    for (const auto &entry : sinks)
        entry.sink(packet);
}

void PacketBuffer::record(const SensorPacket &packet) {
    if (packet.valid())
        packets_.push_back(packet);
}

PacketSink PacketBuffer::sink() {
    return [this](const SensorPacket &packet) { record(packet); };
}

void PacketBuffer::replay(PacketFanout &fanout) const {
    for (const auto &packet : packets_)
        fanout.publish(packet);
}

} // namespace nksensor::stream
