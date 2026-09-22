#pragma once

#include "nativekit_sensor_wire_generated.hpp"
#include <string>

namespace nksensor::wire::detail { class MessagePackReader; class MessagePackWriter; }
namespace nksensor::wire::generated::detail {

bool write(::nksensor::wire::detail::MessagePackWriter &, const PackedFrameMessage &, std::string *error);
bool read(::nksensor::wire::detail::MessagePackReader &, PackedFrameMessage &, std::string *error);
bool write(::nksensor::wire::detail::MessagePackWriter &, const ImuSampleMessage &, std::string *error);
bool read(::nksensor::wire::detail::MessagePackReader &, ImuSampleMessage &, std::string *error);
bool write(::nksensor::wire::detail::MessagePackWriter &, const LidarScanMessage &, std::string *error);
bool read(::nksensor::wire::detail::MessagePackReader &, LidarScanMessage &, std::string *error);
std::array<std::uint8_t, imusampledata_size> pack(const ImuSampleData &);
bool unpack(std::span<const std::uint8_t>, ImuSampleData &);
std::array<std::uint8_t, lidarreturn_size> pack(const LidarReturn &);
bool unpack(std::span<const std::uint8_t>, LidarReturn &);

} // namespace nksensor::wire::generated::detail
