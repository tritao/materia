#include "robotkit_runtime.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

static void roundTrip(rk_recording_compression compression, const char* suffix) {
  const std::string path = std::string("robotkit-recording-") + suffix + ".mcap";
  rk_recording_writer_handle writer{};
  assert(rk_recording_writer_create(path.c_str(), 1024 * 1024, compression, &writer) == RK_OK);
  const char schema[] = "{\"name\":\"TestMsg\",\"fields\":[{\"id\":1,\"name\":\"pixels\",\"type\":\"Bytes\"}]}";
  uint32_t channel = 0;
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "TestMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &channel) == RK_OK);
  assert(channel == 1);
  uint32_t duplicate = 0;
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "TestMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &duplicate) == RK_OK && duplicate == channel);
  assert(rk_recording_writer_register_channel(writer, "robotkit/test", "OtherMsg",
      "robotkit-wire", reinterpret_cast<const uint8_t*>(schema), sizeof(schema) - 1,
      "msgpack", &duplicate) == RK_ERROR_INVALID_ARGUMENT);
  const uint8_t payload[] = {0x81, 0x01, 0xc4, 0x02, 0x00, 0xff};
  assert(rk_recording_writer_enqueue(writer, channel, 9, 123456789, payload, sizeof(payload)) == RK_OK);
  uint8_t oversized[1024 * 1024 + 1]{};
  assert(rk_recording_writer_enqueue(writer, channel, 10, 123456790,
      oversized, sizeof(oversized)) == RK_ERROR_QUEUE_FULL);
  rk_recording_writer_status status{}; status.struct_size = sizeof(status);
  assert(rk_recording_writer_get_status(writer, &status) == RK_OK && status.dropped == 1);
  assert(rk_recording_writer_finish(writer) == RK_OK);
  rk_recording_writer_destroy(writer);

  rk_recording_reader_handle reader{};
  assert(rk_recording_reader_open(path.c_str(), &reader) == RK_OK);
  rk_recording_message message{}; message.struct_size = sizeof(message);
  uint8_t output[256]{}; uint32_t size = sizeof(output);
  assert(rk_recording_reader_next(reader, &message, output, &size) == RK_OK);
  assert(message.schema_version == 6 && message.ordinal == 9 &&
      message.recording_timestamp_ns == 123456789 &&
      std::strcmp(message.topic, "robotkit/test") == 0 &&
      size == sizeof(payload) && std::memcmp(output, payload, size) == 0);
  assert(rk_recording_reader_next(reader, &message, output, &size) == RK_ERROR_STALE_STATE);
  rk_recording_reader_destroy(reader);
  std::remove(path.c_str());
}
int main() {
  roundTrip(RK_RECORDING_COMPRESSION_NONE, "none");
  roundTrip(RK_RECORDING_COMPRESSION_LZ4, "lz4");
}
