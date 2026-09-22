#include "nativekit_sensor_stream.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace nksensor;
using namespace nksensor::stream;

CameraFrame make_camera_frame() {
    CameraFrame frame;
    frame.header.sensor = 70;
    frame.header.sequence = 1;
    frame.header.capture_time = 1.0;
    frame.header.delivery_time = 1.02;
    frame.header.frame = 20;
    frame.width = 1;
    frame.height = 1;
    frame.rgba8 = {1, 2, 3, 255};
    return frame;
}

ImuSample make_imu_sample() {
    ImuSample sample;
    sample.header.sensor = 71;
    sample.header.sequence = 2;
    sample.header.capture_time = 1.01;
    sample.header.delivery_time = 1.03;
    sample.header.frame = 20;
    sample.angular_velocity = {0.1, 0.2, 0.3};
    sample.linear_acceleration = {1.0, 2.0, 3.0};
    sample.angular_velocity_covariance = Matrix3::diagonal({0.01, 0.01, 0.01});
    sample.linear_acceleration_covariance = Matrix3::diagonal({0.1, 0.1, 0.1});
    return sample;
}

void measurement_becomes_one_shared_packet() {
    const SensorMeasurement measurement{make_camera_frame()};
    std::string error;
    const auto packet = make_sensor_packet(measurement, &error);
    assert(packet.has_value());
    assert(error.empty());
    assert(packet->valid());
    assert(packet->type == wire::MessageType::camera_frame);
    assert(packet->header.sensor == 70);

    const auto decoded = wire::view_camera_frame(packet->bytes(), &error);
    assert(decoded.has_value());
    assert(decoded->data.size() == 4);
    assert(decoded->data[0] == 1 && decoded->data[3] == 255);
}

void fanout_reuses_packet_for_multiple_sinks() {
    const auto packet = *make_sensor_packet(SensorMeasurement{make_camera_frame()});
    PacketFanout fanout;
    int first_count = 0;
    int second_count = 0;
    std::shared_ptr<const std::vector<std::uint8_t>> first_bytes;
    std::shared_ptr<const std::vector<std::uint8_t>> second_bytes;

    const auto first = fanout.subscribe([&](const SensorPacket &value) {
        ++first_count;
        first_bytes = value.encoded;
    });
    const auto second = fanout.subscribe([&](const SensorPacket &value) {
        ++second_count;
        second_bytes = value.encoded;
    });
    assert(first != invalid_subscription && second != invalid_subscription);
    assert(fanout.size() == 2);

    fanout.publish(packet);
    assert(first_count == 1 && second_count == 1);
    assert(first_bytes == packet.encoded);
    assert(second_bytes == packet.encoded);
    assert(first_bytes == second_bytes);

    assert(fanout.unsubscribe(first));
    assert(!fanout.unsubscribe(first));
    fanout.publish(packet);
    assert(first_count == 1 && second_count == 2);
}

void buffer_replay_preserves_order_and_exact_bytes() {
    const auto camera = *make_sensor_packet(SensorMeasurement{make_camera_frame()});
    const auto imu = *make_sensor_packet(SensorMeasurement{make_imu_sample()});

    PacketBuffer buffer;
    buffer.record(camera);
    buffer.record(imu);
    assert(buffer.size() == 2);

    std::vector<SensorPacket> replayed;
    PacketFanout fanout;
    fanout.subscribe([&](const SensorPacket &packet) { replayed.push_back(packet); });
    buffer.replay(fanout);

    assert(replayed.size() == 2);
    assert(replayed[0].type == wire::MessageType::camera_frame);
    assert(replayed[1].type == wire::MessageType::imu_sample);
    assert(replayed[0].encoded == buffer.packets()[0].encoded);
    assert(replayed[1].encoded == buffer.packets()[1].encoded);
    assert(*replayed[0].encoded == *camera.encoded);
    assert(*replayed[1].encoded == *imu.encoded);
}

void runtime_output_can_feed_a_recorder_sink() {
    SensorConfig config;
    config.id = 72;
    config.frame = 21;
    config.timing.update_rate_hz = 100.0;
    auto sensor = std::make_shared<ImuSensor>(config);

    ImuTruth truth;
    truth.linear_acceleration = {0.0, 0.0, 9.81};
    truth.gravity = {0.0, 0.0, -9.81};

    SensorRuntime runtime;
    assert(runtime.add(sensor, [sensor, truth](const SensorTick &tick)
                       -> std::optional<SensorMeasurement> {
        const auto sample = sensor->sample(truth, tick);
        return sample.has_value() ? std::optional<SensorMeasurement>{*sample}
                                   : std::nullopt;
    }));

    PacketBuffer buffer;
    PacketFanout fanout;
    fanout.subscribe(buffer.sink());
    for (const auto &measurement : runtime.poll(0.0)) {
        const auto packet = make_sensor_packet(measurement);
        assert(packet.has_value());
        fanout.publish(*packet);
    }
    assert(buffer.size() == 1);
    assert(buffer.packets()[0].type == wire::MessageType::imu_sample);
}

} // namespace

int main() {
    measurement_becomes_one_shared_packet();
    fanout_reuses_packet_for_multiple_sinks();
    buffer_replay_preserves_order_and_exact_bytes();
    runtime_output_can_feed_a_recorder_sink();
    return 0;
}
