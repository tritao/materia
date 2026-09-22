#include "nativekit_sensor_wire.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace {

using namespace nksensor;
using namespace nksensor::wire;

CameraFrame make_frame() {
    CameraFrame frame;
    frame.header.sensor = 40;
    frame.header.sequence = 7;
    frame.header.capture_time = 3.0;
    frame.header.delivery_time = 3.125;
    frame.header.frame = 9;
    frame.width = 2;
    frame.height = 1;
    frame.rgba8 = {255, 0, 0, 255, 0, 255, 0, 255};
    return frame;
}

ImuSample make_imu_sample() {
    ImuSample sample;
    sample.header.sensor = 61;
    sample.header.sequence = 14;
    sample.header.capture_time = 8.0;
    sample.header.delivery_time = 8.04;
    sample.header.frame = 17;
    sample.angular_velocity = {1.0, -2.5, 3.25};
    sample.linear_acceleration = {-4.0, 5.5, -6.75};
    sample.angular_velocity_covariance.values = {
        0.1, 0.2, 0.3,
        0.4, 0.5, 0.6,
        0.7, 0.8, 0.9};
    sample.linear_acceleration_covariance.values = {
        1.1, 1.2, 1.3,
        1.4, 1.5, 1.6,
        1.7, 1.8, 1.9};
    return sample;
}

LidarScan make_lidar_scan() {
    LidarScan scan;
    scan.header.sensor = 63;
    scan.header.sequence = 15;
    scan.header.capture_time = 9.0;
    scan.header.delivery_time = 9.05;
    scan.header.frame = 19;
    scan.horizontal_count = 3;
    scan.vertical_count = 1;
    scan.returns.resize(3);
    scan.returns[0].hit = true;
    scan.returns[0].range = 2.5;
    scan.returns[0].intensity = 0.75;
    scan.returns[1].hit = false;
    scan.returns[1].range = 100.0;
    scan.returns[1].intensity = 0.0;
    scan.returns[2].hit = true;
    scan.returns[2].range = 12.0;
    scan.returns[2].intensity = 4.5;
    return scan;
}

std::vector<std::uint8_t> sensor_wire_vector(std::string_view name) {
    std::ifstream input(NKSENSOR_SENSOR_WIRE_VECTORS_PATH);
    assert(input.good());
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        const auto separator = line.find('\t');
        assert(separator != std::string::npos);
        if (line.substr(0, separator) != name)
            continue;
        const auto hex = std::string_view(line).substr(separator + 1);
        assert(hex.size() % 2 == 0);
        std::vector<std::uint8_t> bytes;
        bytes.reserve(hex.size() / 2);
        for (std::size_t index = 0; index < hex.size(); index += 2)
            bytes.push_back(static_cast<std::uint8_t>(std::stoul(std::string(hex.substr(index, 2)), nullptr, 16)));
        return bytes;
    }
    assert(false && "missing generated SensorKit wire vector");
    return {};
}

void encodes_haxeon_hmpk_and_messagepack_binary() {
    std::string error;
    const auto encoded = encode_camera_frame(make_frame(), &error);
    assert(encoded.has_value());
    assert(error.empty());
    assert(encoded->size() > frame_header_size);
    assert((*encoded)[0] == 'H');
    assert((*encoded)[1] == 'M');
    assert((*encoded)[2] == 'P');
    assert((*encoded)[3] == 'K');
    assert((*encoded)[4] == current_frame_version);
    assert((*encoded)[5] == 0);
    const auto payload_size = (static_cast<std::uint32_t>((*encoded)[6]) << 24) |
                              (static_cast<std::uint32_t>((*encoded)[7]) << 16) |
                              (static_cast<std::uint32_t>((*encoded)[8]) << 8) |
                              (*encoded)[9];
    assert(payload_size == encoded->size() - frame_header_size);
    assert((*encoded)[frame_header_size] == 0x8c); // fixmap(12), Haxeon map shape
    assert(*encoded == sensor_wire_vector("PackedFrameMessage"));
}

void generated_codec_validates_constants_and_field_id_semantics() {
    std::string error;
    auto bad_version = encode_camera_frame(make_frame());
    assert(bad_version.has_value());
    (*bad_version)[frame_header_size + 2] = 2;
    assert(!view_packed_frame(*bad_version, &error).has_value());
    assert(error.find("schema version") != std::string::npos);

    const auto missing_version = sensor_wire_vector("PackedFrameMessage.invalid.missing_schema_version");
    assert(!view_packed_frame(missing_version, &error).has_value());
    assert(error.find("missing a required field") != std::string::npos);

    auto bad_type = encode_imu_sample(make_imu_sample());
    assert(bad_type.has_value());
    (*bad_type)[frame_header_size + 4] = static_cast<std::uint8_t>(MessageType::lidar_scan);
    assert(!view_imu_sample(*bad_type, &error).has_value());
    assert(error.find("not a ImuSampleMessage") != std::string::npos);

    auto missing_data = encode_camera_frame(make_frame());
    assert(missing_data.has_value());
    (*missing_data)[frame_header_size] = 0x8b; // map now omits its final field
    missing_data->resize(missing_data->size() - 11); // key plus bin8(8) and data
    const auto shortened_payload_size =
        static_cast<std::uint32_t>(missing_data->size() - frame_header_size);
    (*missing_data)[6] = static_cast<std::uint8_t>(shortened_payload_size >> 24);
    (*missing_data)[7] = static_cast<std::uint8_t>(shortened_payload_size >> 16);
    (*missing_data)[8] = static_cast<std::uint8_t>(shortened_payload_size >> 8);
    (*missing_data)[9] = static_cast<std::uint8_t>(shortened_payload_size);
    assert(!view_packed_frame(*missing_data, &error).has_value());
    assert(error.find("missing a required field") != std::string::npos);

    const auto reserved = sensor_wire_vector("PackedFrameMessage.reserved");
    assert(!view_camera_frame(reserved, &error).has_value());
    assert(error.find("reserved field ID") != std::string::npos);

    const auto extended = sensor_wire_vector("PackedFrameMessage.extension");
    const auto view = view_camera_frame(extended, &error);
    assert(view.has_value());
    assert(view->data.size() == make_frame().rgba8.size());

    const auto bad_width = sensor_wire_vector("PackedFrameMessage.invalid.width_range");
    assert(!view_packed_frame(bad_width, &error).has_value());
    assert(error.find("width is out of range") != std::string::npos);

    const auto negative_identifier = sensor_wire_vector("ImuSampleMessage.invalid.sensor");
    assert(!view_imu_sample(negative_identifier, &error).has_value());
    assert(error.find("sensor is out of range") != std::string::npos);
}

void view_decoder_is_zero_copy_and_owned_decoder_copies() {
    const auto encoded = encode_camera_frame(make_frame());
    assert(encoded.has_value());

    std::string error;
    const auto view = view_camera_frame(*encoded, &error);
    assert(view.has_value());
    assert(error.empty());
    assert(view->header.sensor == 40);
    assert(view->header.sequence == 7);
    assert(view->header.capture_time == 3.0);
    assert(view->header.delivery_time == 3.125);
    assert(view->width == 2 && view->height == 1 && view->stride == 8);
    assert(view->format == PixelFormat::rgba8);
    assert(view->data.size() == 8);
    assert(view->data[0] == 255 && view->data[4] == 0 && view->data[5] == 255);
    assert(view->data.data() >= encoded->data());
    assert(view->data.data() < encoded->data() + encoded->size());

    const auto decoded = decode_camera_frame(*encoded, &error);
    assert(decoded.has_value());
    assert(decoded->rgba8 == make_frame().rgba8);
    assert(decoded->rgba8.data() != view->data.data());
}

void generic_packed_codec_supports_depth_and_segmentation_formats() {
    PackedFrameView depth;
    depth.header.sensor = 41;
    depth.header.sequence = 8;
    depth.header.capture_time = 4.0;
    depth.header.delivery_time = 4.01;
    depth.type = MessageType::depth_frame;
    depth.format = PixelFormat::r32f_le;
    depth.width = 2;
    depth.height = 1;
    depth.stride = 8;
    const std::vector<std::uint8_t> depth_bytes(8, 0xaa);
    depth.data = depth_bytes;

    const auto encoded_depth = encode_packed_frame(depth);
    assert(encoded_depth.has_value());
    const auto decoded_depth = view_packed_frame(*encoded_depth);
    assert(decoded_depth.has_value());
    assert(decoded_depth->type == MessageType::depth_frame);
    assert(decoded_depth->format == PixelFormat::r32f_le);
    assert(decoded_depth->data.size() == 8);
    const auto owned_depth = decode_packed_frame(*encoded_depth);
    assert(owned_depth.has_value());
    assert(owned_depth->data == depth_bytes);
    assert(owned_depth->data.data() != decoded_depth->data.data());

    PackedFrameView segmentation = depth;
    segmentation.type = MessageType::segmentation_frame;
    segmentation.format = PixelFormat::u64_le;
    segmentation.stride = 16;
    segmentation.width = 2;
    const std::vector<std::uint8_t> labels(16, 0xbb);
    segmentation.data = labels;
    const auto encoded_segmentation = encode_packed_frame(segmentation);
    assert(encoded_segmentation.has_value());
    const auto decoded_segmentation = view_packed_frame(*encoded_segmentation);
    assert(decoded_segmentation.has_value());
    assert(decoded_segmentation->type == MessageType::segmentation_frame);
    assert(decoded_segmentation->format == PixelFormat::u64_le);
    assert(decoded_segmentation->data.size() == 16);

    auto invalid = depth;
    invalid.type = MessageType::camera_frame;
    std::string error;
    assert(!encode_packed_frame(invalid, &error).has_value());
    assert(error.find("incompatible") != std::string::npos);
}

void typed_depth_codec_preserves_little_endian_r32f_values() {
    DepthFrame source;
    source.header.sensor = 51;
    source.header.sequence = 12;
    source.header.capture_time = 6.0;
    source.header.delivery_time = 6.02;
    source.header.frame = 15;
    source.width = 2;
    source.height = 1;
    source.meters = {1.0f, 2.5f};

    const auto encoded = encode_depth_frame(source);
    assert(encoded.has_value());
    const auto view = view_packed_frame(*encoded);
    assert(view.has_value());
    assert(view->type == MessageType::depth_frame);
    assert(view->format == PixelFormat::r32f_le);
    const std::vector<std::uint8_t> expected_bytes{
        0x00, 0x00, 0x80, 0x3f,
        0x00, 0x00, 0x20, 0x40};
    assert(std::vector<std::uint8_t>(view->data.begin(), view->data.end()) == expected_bytes);

    const auto decoded = decode_depth_frame(*encoded);
    assert(decoded.has_value());
    assert(decoded->header.sensor == source.header.sensor);
    assert(decoded->meters == source.meters);
}

void typed_segmentation_codec_preserves_little_endian_u64_labels() {
    SegmentationFrame source;
    source.header.sensor = 52;
    source.header.sequence = 13;
    source.header.capture_time = 7.0;
    source.header.delivery_time = 7.03;
    source.header.frame = 16;
    source.width = 2;
    source.height = 1;
    source.labels = {0x0102030405060708ULL, 0xffffffffffffffffULL};

    const auto encoded = encode_segmentation_frame(source);
    assert(encoded.has_value());
    const auto view = view_packed_frame(*encoded);
    assert(view.has_value());
    assert(view->type == MessageType::segmentation_frame);
    assert(view->format == PixelFormat::u64_le);
    const std::vector<std::uint8_t> expected_bytes{
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    assert(std::vector<std::uint8_t>(view->data.begin(), view->data.end()) == expected_bytes);

    const auto decoded = decode_segmentation_frame(*encoded);
    assert(decoded.has_value());
    assert(decoded->header.sensor == source.header.sensor);
    assert(decoded->labels == source.labels);
}

void imu_codec_is_packed_zero_copy_and_lossless() {
    const auto source = make_imu_sample();
    std::string error;
    const auto encoded = encode_imu_sample(source, &error);
    assert(encoded.has_value());
    assert(error.empty());
    assert((*encoded)[frame_header_size] == 0x88); // fixmap(8)
    assert(*encoded == sensor_wire_vector("ImuSampleMessage"));

    const auto view = view_imu_sample(*encoded, &error);
    assert(view.has_value());
    assert(error.empty());
    assert(view->header.sensor == source.header.sensor);
    assert(view->header.sequence == source.header.sequence);
    assert(view->header.capture_time == source.header.capture_time);
    assert(view->header.delivery_time == source.header.delivery_time);
    assert(view->header.frame == source.header.frame);
    assert(view->data.size() == imu_packed_data_size);
    assert(view->data.data() >= encoded->data());
    assert(view->data.data() < encoded->data() + encoded->size());

    /* 1.0 and -2.5 prove the payload is little-endian binary64, not a
     * platform-native struct dump or a sequence of MessagePack scalars. */
    const std::vector<std::uint8_t> expected_prefix{
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xc0};
    assert(std::equal(expected_prefix.begin(), expected_prefix.end(), view->data.begin()));

    const auto decoded = decode_imu_sample(*encoded, &error);
    assert(decoded.has_value());
    assert(decoded->header.sensor == source.header.sensor);
    assert(decoded->angular_velocity == source.angular_velocity);
    assert(decoded->linear_acceleration == source.linear_acceleration);
    assert(decoded->angular_velocity_covariance.values ==
           source.angular_velocity_covariance.values);
    assert(decoded->linear_acceleration_covariance.values ==
           source.linear_acceleration_covariance.values);

    auto non_finite = source;
    non_finite.linear_acceleration.z = std::numeric_limits<double>::quiet_NaN();
    assert(!encode_imu_sample(non_finite, &error).has_value());
    assert(error.find("finite") != std::string::npos);
}

void lidar_codec_is_packed_zero_copy_and_round_trips() {
    const auto source = make_lidar_scan();
    std::string error;
    const auto encoded = encode_lidar_scan(source, &error);
    assert(encoded.has_value());
    assert(error.empty());
    assert((*encoded)[frame_header_size] == 0x8b); // fixmap(11)
    assert(*encoded == sensor_wire_vector("LidarScanMessage"));

    const auto view = view_lidar_scan(*encoded, &error);
    assert(view.has_value());
    assert(error.empty());
    assert(view->header.sensor == source.header.sensor);
    assert(view->horizontal_count == 3 && view->vertical_count == 1);
    assert(view->return_stride == lidar_packed_return_size);
    assert(view->data.size() == 3 * lidar_packed_return_size);
    assert(view->data.data() >= encoded->data());
    assert(view->data.data() < encoded->data() + encoded->size());

    /* 2.5f, 0.75f, and hit=true in the first fixed-size return. */
    const std::vector<std::uint8_t> expected_prefix{
        0x00, 0x00, 0x20, 0x40,
        0x00, 0x00, 0x40, 0x3f,
        0x01, 0x00, 0x00, 0x00};
    assert(std::equal(expected_prefix.begin(), expected_prefix.end(), view->data.begin()));

    const auto decoded = decode_lidar_scan(*encoded, &error);
    assert(decoded.has_value());
    assert(decoded->header.sensor == source.header.sensor);
    assert(decoded->horizontal_count == source.horizontal_count);
    assert(decoded->vertical_count == source.vertical_count);
    assert(decoded->returns.size() == source.returns.size());
    for (std::size_t index = 0; index < source.returns.size(); ++index) {
        assert(decoded->returns[index].hit == source.returns[index].hit);
        assert(std::abs(decoded->returns[index].range - source.returns[index].range) < 1e-6);
        assert(std::abs(decoded->returns[index].intensity - source.returns[index].intensity) <
               1e-6);
    }

    auto invalid_flag = *encoded;
    const auto data_offset = static_cast<std::size_t>(view->data.data() - encoded->data());
    invalid_flag[data_offset + 8] = 2;
    assert(!view_lidar_scan(invalid_flag, &error).has_value());
    assert(error.find("hit flag") != std::string::npos);

    auto invalid_count = source;
    invalid_count.returns.pop_back();
    assert(!encode_lidar_scan(invalid_count, &error).has_value());
    assert(error.find("return count") != std::string::npos);
}

void runtime_measurements_have_wire_adapter() {
    SensorConfig config;
    config.id = 62;
    config.frame = 18;
    config.timing.update_rate_hz = 100.0;
    auto sensor = std::make_shared<ImuSensor>(config);

    ImuTruth truth;
    truth.linear_acceleration = {0.0, 0.0, 9.81};
    truth.gravity = {0.0, 0.0, -9.81};

    SensorRuntime runtime;
    assert(runtime.add(sensor, [sensor, truth](const SensorTick &tick)
                       -> std::optional<SensorMeasurement> {
        const auto sample = sensor->sample(truth, tick);
        if (!sample.has_value())
            return std::nullopt;
        return SensorMeasurement{*sample};
    }));

    const auto measurements = runtime.poll(0.0);
    assert(measurements.size() == 1);
    std::string error;
    const auto encoded = encode_sensor_measurement(measurements.front(), &error);
    assert(encoded.has_value());
    assert(error.empty());
    assert(decode_imu_sample(*encoded, &error).has_value());

    const SensorMeasurement lidar_measurement{make_lidar_scan()};
    const auto encoded_lidar = encode_sensor_measurement(lidar_measurement, &error);
    assert(encoded_lidar.has_value());
    assert(decode_lidar_scan(*encoded_lidar, &error).has_value());
}

void rejects_bad_framing_and_invalid_camera_payloads() {
    const auto encoded = encode_camera_frame(make_frame());
    assert(encoded.has_value());

    auto bad_magic = *encoded;
    bad_magic[0] = 'X';
    assert(!view_camera_frame(bad_magic).has_value());

    auto truncated = *encoded;
    truncated.pop_back();
    assert(!view_camera_frame(truncated).has_value());

    auto bad_flags = *encoded;
    bad_flags[5] = 1;
    assert(!view_camera_frame(bad_flags).has_value());

    CameraFrame bad_size = make_frame();
    bad_size.rgba8.pop_back();
    std::string error;
    assert(!encode_camera_frame(bad_size, &error).has_value());
    assert(error.find("payload size") != std::string::npos);
}

void enforces_messagepack_size_limit() {
    auto frame = make_frame();
    std::string error;
    assert(!encode_camera_frame(frame, &error, 8).has_value());
    assert(error.find("configured limit") != std::string::npos);
}

} // namespace

int main() {
    encodes_haxeon_hmpk_and_messagepack_binary();
    generated_codec_validates_constants_and_field_id_semantics();
    view_decoder_is_zero_copy_and_owned_decoder_copies();
    generic_packed_codec_supports_depth_and_segmentation_formats();
    typed_depth_codec_preserves_little_endian_r32f_values();
    typed_segmentation_codec_preserves_little_endian_u64_labels();
    imu_codec_is_packed_zero_copy_and_lossless();
    lidar_codec_is_packed_zero_copy_and_round_trips();
    runtime_measurements_have_wire_adapter();
    rejects_bad_framing_and_invalid_camera_payloads();
    enforces_messagepack_size_limit();
    return 0;
}
