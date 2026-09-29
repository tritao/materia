#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include "robotkit_runtime.h"

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
struct ChannelDef {
  std::string topic, schemaName, schemaEncoding, messageEncoding;
  mcap::ByteArray schemaData;
};
struct Item {
  uint32_t channelId;
  uint64_t ordinal, timestamp;
  std::vector<std::byte> data;
};
struct Writer {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<Item> queue;
  std::vector<ChannelDef> channels;
  uint64_t capacityBytes;
  uint64_t queuedBytes = 0, accepted = 0, written = 0, dropped = 0;
  rk_recording_compression compression;
  mcap::McapWriter output;
  uint32_t state = RK_RECORDING_OPEN;
  std::string error, marker;
  bool stopping = false;
  std::thread thread;
  Writer(uint64_t capacity, rk_recording_compression compressionValue, std::string markerPath)
      : capacityBytes(capacity), compression(compressionValue), marker(std::move(markerPath)) {}
};
struct Reader {
  mcap::McapReader mcap;
  bool damaged = false;
  std::optional<mcap::LinearMessageView> messages;
  std::optional<mcap::LinearMessageView::Iterator> iterator;
  std::optional<Item> current;
  std::string topic;
};
std::mutex RegistryMutex;
uint32_t NextHandle = 1;
std::unordered_map<uint32_t, std::shared_ptr<Writer>> Writers;
std::unordered_map<uint32_t, std::shared_ptr<Reader>> Readers;

template<class T> std::shared_ptr<T> lookup(std::unordered_map<uint32_t,std::shared_ptr<T>>& values, uint32_t id) {
  std::lock_guard lock(RegistryMutex);
  auto found = values.find(id);
  return found == values.end() ? nullptr : found->second;
}
bool validText(const char* value) { return value && *value; }
void fail(const std::shared_ptr<Writer>& writer, const std::string& message) {
  std::lock_guard lock(writer->mutex);
  if (writer->error.empty()) writer->error = message;
  writer->state = RK_RECORDING_FAILED;
  writer->stopping = true;
  writer->cv.notify_all();
}
void writeStatus(const std::shared_ptr<Writer>& writer) {
  std::ofstream out(writer->marker + ".status", std::ios::binary | std::ios::trunc);
  if (!out) return;
  out << "state=" << writer->state << "\naccepted=" << writer->accepted
      << "\nwritten=" << writer->written << "\ndropped=" << writer->dropped
      << "\nqueued=" << writer->queue.size() << "\nqueued_bytes=" << writer->queuedBytes
      << "\nerror=" << writer->error << "\n";
}
void writerMain(const std::shared_ptr<Writer>& writer) {
  auto& output = writer->output;
  std::vector<mcap::ChannelId> mcapChannels(1);
  for (;;) {
    Item item;
    {
      std::unique_lock lock(writer->mutex);
      writer->cv.wait(lock, [&] { return writer->stopping || !writer->queue.empty(); });
      if (writer->queue.empty()) { if (writer->stopping) break; continue; }
      item = std::move(writer->queue.front());
      writer->queuedBytes -= item.data.size();
      writer->queue.pop_front();
    }
    while (mcapChannels.size() <= item.channelId) {
      ChannelDef def;
      { std::lock_guard lock(writer->mutex); def = writer->channels[mcapChannels.size() - 1]; }
      mcap::Schema schema(def.schemaName, def.schemaEncoding, def.schemaData);
      output.addSchema(schema);
      mcap::Channel channel(def.topic, def.messageEncoding, schema.id);
      channel.metadata["robotkit.schema_version"] = "6";
      output.addChannel(channel);
      mcapChannels.push_back(channel.id);
    }
    mcap::Message message;
    message.channelId = mcapChannels[item.channelId];
    message.sequence = static_cast<uint32_t>(item.ordinal);
    message.logTime = item.timestamp;
    message.publishTime = item.ordinal;
    message.data = item.data.data();
    message.dataSize = item.data.size();
    auto result = output.write(message);
    if (!result.ok()) { fail(writer, result.message); break; }
    std::lock_guard lock(writer->mutex);
    ++writer->written;
  }
  output.close();
  std::lock_guard lock(writer->mutex);
  if (writer->state != RK_RECORDING_FAILED) writer->state = RK_RECORDING_CLOSED;
  writeStatus(writer);
}
rk_result validateAndCache(const std::shared_ptr<Reader>& reader) {
  if (reader->damaged) return RK_ERROR_BACKEND;
  if (!reader->iterator || *reader->iterator == reader->messages->end()) return RK_ERROR_STALE_STATE;
  const auto& view = **reader->iterator;
  auto version = view.channel->metadata.find("robotkit.schema_version");
  if (!view.schema || view.schema->encoding != "robotkit-wire" ||
      view.channel->messageEncoding != "msgpack" ||
      version == view.channel->metadata.end() || version->second != "6" ||
      view.channel->topic.size() >= 128) return RK_ERROR_UNSUPPORTED;
  Item item;
  item.channelId = 0;
  item.ordinal = view.message.publishTime;
  item.timestamp = view.message.logTime;
  item.data.assign(view.message.data, view.message.data + view.message.dataSize);
  reader->topic = view.channel->topic;
  reader->current = std::move(item);
  ++(*reader->iterator);
  return reader->damaged ? RK_ERROR_BACKEND : RK_OK;
}
}

extern "C" {
rk_result rk_recording_writer_create(const char* path, uint64_t capacity,
    rk_recording_compression compression, rk_recording_writer_handle* out) {
  if (!path || !*path || !capacity || !out || compression > RK_RECORDING_COMPRESSION_LZ4)
    return RK_ERROR_INVALID_ARGUMENT;
  try {
    std::string marker = std::string(path) + ".incomplete";
    { std::ofstream pending(marker, std::ios::trunc); if (!pending) return RK_ERROR_BACKEND; pending << "incomplete\n"; }
    auto writer = std::make_shared<Writer>(capacity, compression, marker);
    auto options = mcap::McapWriterOptions("robotkit");
    options.compression = compression == RK_RECORDING_COMPRESSION_LZ4
        ? mcap::Compression::Lz4 : mcap::Compression::None;
    auto result = writer->output.open(path, options);
    if (!result.ok()) { std::filesystem::remove(marker); return RK_ERROR_BACKEND; }
    mcap::Metadata metadata;
    metadata.name = "robotkit";
    metadata.metadata["robotkit.schema_version"] = "6";
    result = writer->output.write(metadata);
    if (!result.ok()) { writer->output.close(); std::filesystem::remove(marker); return RK_ERROR_BACKEND; }
    { std::lock_guard lock(RegistryMutex); out->id = NextHandle++; Writers[out->id] = writer; }
    writer->thread = std::thread(writerMain, writer);
    return RK_OK;
  } catch (...) { return RK_ERROR_OUT_OF_MEMORY; }
}
rk_result rk_recording_writer_register_channel(rk_recording_writer_handle handle,
    const char* topic, const char* schema_name, const char* schema_encoding,
    const uint8_t* schema_data, uint32_t schema_data_len, const char* message_encoding,
    uint32_t* out_channel_id) {
  auto writer = lookup(Writers, handle.id);
  if (!writer) return RK_ERROR_INVALID_HANDLE;
  if (!validText(topic) || std::strlen(topic) >= 128 || !validText(schema_name) ||
      !validText(schema_encoding) || !schema_data || !schema_data_len ||
      !validText(message_encoding) || !out_channel_id) return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(writer->mutex);
  if (writer->state != RK_RECORDING_OPEN) return RK_ERROR_INVALID_STATE;
  for (uint32_t index = 0; index < writer->channels.size(); ++index) {
    const auto& existing = writer->channels[index];
    if (existing.topic == topic) {
      if (existing.schemaName != schema_name || existing.schemaEncoding != schema_encoding ||
          existing.messageEncoding != message_encoding || existing.schemaData.size() != schema_data_len ||
          std::memcmp(existing.schemaData.data(), schema_data, schema_data_len) != 0)
        return RK_ERROR_INVALID_ARGUMENT;
      *out_channel_id = index + 1;
      return RK_OK;
    }
  }
  if (writer->channels.size() >= UINT16_MAX) return RK_ERROR_LIMIT;
  ChannelDef def{topic, schema_name, schema_encoding, message_encoding, {}};
  def.schemaData.assign(reinterpret_cast<const std::byte*>(schema_data),
      reinterpret_cast<const std::byte*>(schema_data) + schema_data_len);
  writer->channels.push_back(std::move(def));
  *out_channel_id = writer->channels.size();
  return RK_OK;
}
rk_result rk_recording_writer_enqueue(rk_recording_writer_handle handle, uint32_t channel_id,
    uint64_t ordinal, uint64_t timestamp, const uint8_t* payload, uint32_t size) {
  auto writer = lookup(Writers, handle.id);
  if (!writer) return RK_ERROR_INVALID_HANDLE;
  if (!channel_id || (!payload && size)) return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(writer->mutex);
  if (writer->state != RK_RECORDING_OPEN) return RK_ERROR_INVALID_STATE;
  if (channel_id > writer->channels.size()) return RK_ERROR_INVALID_ARGUMENT;
  if (size > writer->capacityBytes || writer->queuedBytes > writer->capacityBytes - size) {
    ++writer->dropped; return RK_ERROR_QUEUE_FULL;
  }
  Item item{channel_id, ordinal, timestamp, {}};
  item.data.resize(size);
  if (size) std::memcpy(item.data.data(), payload, size);
  writer->queuedBytes += size;
  writer->queue.push_back(std::move(item));
  ++writer->accepted;
  writer->cv.notify_one();
  return RK_OK;
}
rk_result rk_recording_writer_get_status(rk_recording_writer_handle handle, rk_recording_writer_status* status) {
  auto writer=lookup(Writers,handle.id);if(!writer)return RK_ERROR_INVALID_HANDLE;
  if(!status||status->struct_size<sizeof(*status))return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(writer->mutex);
  status->state=writer->state;status->accepted=writer->accepted;status->written=writer->written;
  status->dropped=writer->dropped;status->queued=writer->queue.size();status->queued_bytes=writer->queuedBytes;
  std::memset(status->error,0,sizeof(status->error));
  std::memcpy(status->error,writer->error.data(),std::min(writer->error.size(),sizeof(status->error)-1));
  return RK_OK;
}
rk_result rk_recording_writer_finish(rk_recording_writer_handle handle) {
  auto writer=lookup(Writers,handle.id);if(!writer)return RK_ERROR_INVALID_HANDLE;
  {std::lock_guard lock(writer->mutex);writer->stopping=true;if(writer->state==RK_RECORDING_OPEN)writer->state=RK_RECORDING_CLOSING;}
  writer->cv.notify_all();if(writer->thread.joinable())writer->thread.join();
  std::lock_guard lock(writer->mutex);writeStatus(writer);
  if(writer->state!=RK_RECORDING_CLOSED)return RK_ERROR_BACKEND;
  std::error_code error;std::filesystem::remove(writer->marker,error);return error?RK_ERROR_BACKEND:RK_OK;
}
void rk_recording_writer_destroy(rk_recording_writer_handle handle) {
  std::shared_ptr<Writer> writer;
  {std::lock_guard lock(RegistryMutex);auto found=Writers.find(handle.id);if(found==Writers.end())return;writer=found->second;Writers.erase(found);}
  {std::lock_guard lock(writer->mutex);if(writer->state==RK_RECORDING_OPEN){writer->error="writer destroyed without finish";writer->state=RK_RECORDING_FAILED;}writer->stopping=true;writeStatus(writer);}
  writer->cv.notify_all();if(writer->thread.joinable())writer->thread.join();
}
rk_result rk_recording_reader_open(const char* path,rk_recording_reader_handle* out) {
  if(!path||!*path||!out)return RK_ERROR_INVALID_ARGUMENT;
  try {
    if(std::filesystem::exists(std::string(path)+".incomplete"))return RK_ERROR_INVALID_STATE;
    auto reader=std::make_shared<Reader>();auto status=reader->mcap.open(path);if(!status.ok())return RK_ERROR_BACKEND;
    status=reader->mcap.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);if(!status.ok())return RK_ERROR_BACKEND;
    for (const auto& [_, channel] : reader->mcap.channels()) {
      auto version = channel->metadata.find("robotkit.schema_version");
      if (version == channel->metadata.end() || version->second != "6") return RK_ERROR_UNSUPPORTED;
    }
    reader->messages.emplace(reader->mcap.readMessages([raw=reader.get()](const mcap::Status&){raw->damaged=true;}));
    reader->iterator.emplace(reader->messages->begin());
    {std::lock_guard lock(RegistryMutex);out->id=NextHandle++;Readers[out->id]=reader;}return RK_OK;
  } catch(...) {return RK_ERROR_BACKEND;}
}
rk_result rk_recording_reader_next(rk_recording_reader_handle handle,rk_recording_message* message,uint8_t* payload,uint32_t* size) {
  auto reader=lookup(Readers,handle.id);if(!reader)return RK_ERROR_INVALID_HANDLE;
  if(!message||message->struct_size<sizeof(*message)||!size)return RK_ERROR_INVALID_ARGUMENT;
  if(!reader->current){auto status=validateAndCache(reader);if(status!=RK_OK)return status;}
  auto& item=*reader->current;message->schema_version=6;
  message->ordinal=item.ordinal;message->recording_timestamp_ns=item.timestamp;message->payload_size=item.data.size();
  std::memset(message->topic,0,sizeof(message->topic));
  std::memcpy(message->topic,reader->topic.data(),reader->topic.size());
  if(!payload||*size<item.data.size()){*size=item.data.size();return RK_ERROR_LIMIT;}
  std::memcpy(payload,item.data.data(),item.data.size());*size=item.data.size();reader->current.reset();return RK_OK;
}
void rk_recording_reader_destroy(rk_recording_reader_handle handle){std::lock_guard lock(RegistryMutex);Readers.erase(handle.id);}
}
