#include "nativekit_sensor_wire.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <string>
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

    const std::vector<std::uint8_t> expected_payload{
        0x8c,
        0x01, 0x01,
        0x02, 0x01,
        0x03, 0x28,
        0x04, 0x07,
        0x05, 0xcb, 0x40, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x06, 0xcb, 0x40, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x07, 0x09,
        0x08, 0x02,
        0x09, 0x01,
        0x0a, 0x08,
        0x0b, 0x01,
        0x0c, 0xc4, 0x08, 0xff, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff};
    assert(std::vector<std::uint8_t>(encoded->begin() + frame_header_size,
                                     encoded->end()) == expected_payload);

    /* The final field is bin8(8), followed by the two RGBA pixels. */
    const auto data_marker = std::find(encoded->begin() + frame_header_size,
                                       encoded->end(), static_cast<std::uint8_t>(0xc4));
    assert(data_marker != encoded->end());
    assert(data_marker + 1 != encoded->end() && *(data_marker + 1) == 8);
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
    view_decoder_is_zero_copy_and_owned_decoder_copies();
    generic_packed_codec_supports_depth_and_segmentation_formats();
    typed_depth_codec_preserves_little_endian_r32f_values();
    typed_segmentation_codec_preserves_little_endian_u64_labels();
    rejects_bad_framing_and_invalid_camera_payloads();
    enforces_messagepack_size_limit();
    return 0;
}
