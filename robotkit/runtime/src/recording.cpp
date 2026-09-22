#define MCAP_COMPRESSION_NO_LZ4
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
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
constexpr const char* kNames[] = {"", "command", "snapshot", "sensor", "fault", "world", "world_event"};
constexpr const char* kSchema = R"({"$schema":"https://json-schema.org/draft/2020-12/schema","title":"RobotKit recording payload","type":"object","required":["version","ordinal","type"],"properties":{"version":{"const":1},"ordinal":{"type":"string","pattern":"^[0-9]+$"},"type":{"type":"string"}}})";
struct Item { uint32_t kind; uint32_t version; uint64_t ordinal; std::vector<std::byte> data; };
struct Writer {
  std::mutex mutex; std::condition_variable cv; std::deque<Item> queue; size_t capacity;
  uint64_t accepted=0, written=0, dropped=0; uint32_t state=RK_RECORDING_OPEN;
  std::string error; std::string incompleteMarker; bool stopping=false; std::thread thread;
  Writer(size_t cap,std::string marker): capacity(cap),incompleteMarker(std::move(marker)) {}
};
struct Reader {
  mcap::McapReader mcap; std::vector<Item> items; size_t index=0;
};
std::mutex g_mutex; uint32_t g_next=1;
std::unordered_map<uint32_t,std::shared_ptr<Writer>> g_writers;
std::unordered_map<uint32_t,std::shared_ptr<Reader>> g_readers;

template<class T> std::shared_ptr<T> lookup(std::unordered_map<uint32_t,std::shared_ptr<T>>& map,uint32_t id){
  std::lock_guard lock(g_mutex); auto it=map.find(id); return it==map.end()?nullptr:it->second;
}
bool valid_kind(uint32_t kind){ return kind>=RK_RECORDING_COMMAND && kind<=RK_RECORDING_WORLD_EVENT; }
void set_failure(const std::shared_ptr<Writer>& w,const std::string& error){
  std::lock_guard lock(w->mutex); if(w->error.empty()) w->error=error; w->state=RK_RECORDING_FAILED; w->stopping=true; w->cv.notify_all();
}
void run_writer(const std::shared_ptr<Writer>& w,std::string path){
  mcap::McapWriter out; auto options=mcap::McapWriterOptions("robotkit"); options.compression=mcap::Compression::None;
  auto status=out.open(path,options); if(!status.ok()){set_failure(w,status.message);return;}
  std::array<mcap::ChannelId,7> channels{};
  for(uint32_t kind=1;kind<=6;++kind){
    mcap::Schema schema(std::string("robotkit.")+kNames[kind]+".v1","jsonschema",kSchema); out.addSchema(schema);
    mcap::Channel channel(std::string("robotkit/")+kNames[kind],"json",schema.id); channel.metadata["robotkit.schema_version"]="1"; out.addChannel(channel); channels[kind]=channel.id;
  }
  for(;;){
    Item item; { std::unique_lock lock(w->mutex); w->cv.wait(lock,[&]{return w->stopping||!w->queue.empty();});
      if(w->queue.empty()){if(w->stopping) break; continue;} item=std::move(w->queue.front()); w->queue.pop_front(); }
    mcap::Message msg; msg.channelId=channels[item.kind]; msg.sequence=static_cast<uint32_t>(item.ordinal); msg.logTime=item.ordinal; msg.publishTime=item.ordinal; msg.data=item.data.data(); msg.dataSize=item.data.size();
    status=out.write(msg); if(!status.ok()){set_failure(w,status.message);break;}
    std::lock_guard lock(w->mutex); ++w->written;
  }
  out.close(); std::lock_guard lock(w->mutex); if(w->state!=RK_RECORDING_FAILED) w->state=RK_RECORDING_CLOSED;
}
}

extern "C" {
rk_result rk_recording_writer_create(const char* path,uint32_t capacity,rk_recording_writer_handle* out){
  if(!path||!*path||capacity==0||!out) return RK_ERROR_INVALID_ARGUMENT;
  try { std::string marker=std::string(path)+".incomplete";{std::ofstream pending(marker,std::ios::binary|std::ios::trunc);if(!pending)return RK_ERROR_BACKEND;pending<<"RobotKit recording was not closed cleanly\n";}auto w=std::make_shared<Writer>(capacity,marker); {std::lock_guard lock(g_mutex);out->id=g_next++;g_writers[out->id]=w;} w->thread=std::thread(run_writer,w,std::string(path)); return RK_OK; } catch(...) {return RK_ERROR_OUT_OF_MEMORY;}
}
rk_result rk_recording_writer_enqueue(rk_recording_writer_handle h,rk_recording_event_kind kind,uint32_t version,uint64_t ordinal,const uint8_t* payload,uint32_t size){
  auto w=lookup(g_writers,h.id); if(!w) return RK_ERROR_INVALID_HANDLE; if(!valid_kind(kind)||version!=1||(!payload&&size)) return RK_ERROR_INVALID_ARGUMENT;
  std::lock_guard lock(w->mutex); if(w->state!=RK_RECORDING_OPEN) return RK_ERROR_INVALID_STATE; if(w->queue.size()>=w->capacity){++w->dropped;return RK_ERROR_QUEUE_FULL;}
  Item item{kind,version,ordinal,{}}; item.data.resize(size); std::memcpy(item.data.data(),payload,size); w->queue.push_back(std::move(item));++w->accepted;w->cv.notify_one();return RK_OK;
}
rk_result rk_recording_writer_get_status(rk_recording_writer_handle h,rk_recording_writer_status* s){
  auto w=lookup(g_writers,h.id); if(!w) return RK_ERROR_INVALID_HANDLE; if(!s||s->struct_size<sizeof(*s)) return RK_ERROR_INVALID_ARGUMENT; std::lock_guard lock(w->mutex);
  s->state=w->state;s->accepted=w->accepted;s->written=w->written;s->dropped=w->dropped;s->queued=w->queue.size();std::memset(s->error,0,sizeof(s->error));std::memcpy(s->error,w->error.data(),std::min(w->error.size(),sizeof(s->error)-1));return RK_OK;
}
rk_result rk_recording_writer_finish(rk_recording_writer_handle h){auto w=lookup(g_writers,h.id);if(!w)return RK_ERROR_INVALID_HANDLE;{{std::lock_guard lock(w->mutex);w->stopping=true;if(w->state==RK_RECORDING_OPEN)w->state=RK_RECORDING_CLOSING;}w->cv.notify_all();}if(w->thread.joinable())w->thread.join();std::lock_guard lock(w->mutex);if(w->state!=RK_RECORDING_CLOSED)return RK_ERROR_BACKEND;std::error_code error;std::filesystem::remove(w->incompleteMarker,error);return error?RK_ERROR_BACKEND:RK_OK;}
void rk_recording_writer_destroy(rk_recording_writer_handle h){
  std::shared_ptr<Writer>w;
  {std::lock_guard lock(g_mutex);auto it=g_writers.find(h.id);if(it==g_writers.end())return;w=it->second;g_writers.erase(it);}
  {std::lock_guard lock(w->mutex);if(w->state==RK_RECORDING_OPEN){w->error="writer destroyed without finish";w->state=RK_RECORDING_FAILED;}w->stopping=true;}
  w->cv.notify_all();if(w->thread.joinable())w->thread.join();
}
rk_result rk_recording_reader_open(const char* path,rk_recording_reader_handle* out){if(!path||!*path||!out)return RK_ERROR_INVALID_ARGUMENT;try{if(std::filesystem::exists(std::string(path)+".incomplete"))return RK_ERROR_INVALID_STATE;auto r=std::make_shared<Reader>();auto s=r->mcap.open(path);if(!s.ok())return RK_ERROR_BACKEND;s=r->mcap.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);if(!s.ok())return RK_ERROR_BACKEND;bool damaged=false;auto problem=[&](const mcap::Status&){damaged=true;};for(const auto& view:r->mcap.readMessages(problem)){auto topic=view.channel->topic;if(topic.rfind("robotkit/",0)!=0||!view.schema||view.schema->encoding!="jsonschema")return RK_ERROR_UNSUPPORTED;uint32_t kind=0;for(uint32_t i=1;i<=6;++i)if(topic==std::string("robotkit/")+kNames[i])kind=i;if(!kind||view.schema->name!=std::string("robotkit.")+kNames[kind]+".v1")return RK_ERROR_UNSUPPORTED;auto version=view.channel->metadata.find("robotkit.schema_version");if(version==view.channel->metadata.end()||version->second!="1")return RK_ERROR_UNSUPPORTED;Item item{kind,1,view.message.logTime,{}};item.data.assign(view.message.data,view.message.data+view.message.dataSize);r->items.push_back(std::move(item));}if(damaged)return RK_ERROR_BACKEND;{std::lock_guard lock(g_mutex);out->id=g_next++;g_readers[out->id]=r;}return RK_OK;}catch(...){return RK_ERROR_BACKEND;}}
rk_result rk_recording_reader_next(rk_recording_reader_handle h,rk_recording_message* m,uint8_t* payload,uint32_t* size){auto r=lookup(g_readers,h.id);if(!r)return RK_ERROR_INVALID_HANDLE;if(!m||m->struct_size<sizeof(*m)||!size)return RK_ERROR_INVALID_ARGUMENT;if(r->index>=r->items.size())return RK_ERROR_STALE_STATE;auto& item=r->items[r->index];m->kind=item.kind;m->schema_version=item.version;m->ordinal=item.ordinal;m->payload_size=item.data.size();if(!payload||*size<item.data.size()){*size=item.data.size();return RK_ERROR_LIMIT;}std::memcpy(payload,item.data.data(),item.data.size());*size=item.data.size();++r->index;return RK_OK;}
void rk_recording_reader_destroy(rk_recording_reader_handle h){std::lock_guard lock(g_mutex);g_readers.erase(h.id);}
}
