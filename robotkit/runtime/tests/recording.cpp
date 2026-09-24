#include "robotkit_runtime.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
int main(){
  const std::string path="robotkit-recording-test.mcap"; rk_recording_writer_handle w{};
  assert(rk_recording_writer_create(path.c_str(),1024,&w)==RK_OK);
  const char data[]="{\"version\":2,\"ordinal\":\"18446744073709551614\",\"type\":\"sensor\"}";
  assert(rk_recording_writer_enqueue(w,RK_RECORDING_SENSOR,2,UINT64_MAX-1,123456789,
    reinterpret_cast<const uint8_t*>(data),sizeof(data)-1)==RK_OK);
  assert(rk_recording_writer_finish(w)==RK_OK); rk_recording_writer_destroy(w);
  rk_recording_reader_handle r{}; assert(rk_recording_reader_open(path.c_str(),&r)==RK_OK);
  rk_recording_message m{};m.struct_size=sizeof(m);uint8_t output[256];uint32_t size=sizeof(output);
  assert(rk_recording_reader_next(r,&m,output,&size)==RK_OK);assert(m.ordinal==UINT64_MAX-1);
  assert(m.recording_timestamp_ns==123456789);assert(m.kind==RK_RECORDING_SENSOR);
  assert(size==sizeof(data)-1);assert(std::memcmp(output,data,size)==0);
  assert(rk_recording_reader_next(r,&m,output,&size)==RK_ERROR_STALE_STATE);rk_recording_reader_destroy(r);std::remove(path.c_str());
  const std::string truncated="robotkit-recording-truncated.mcap";
  {std::ofstream file(truncated,std::ios::binary);file.write("\x89MCAP0\r\n",8);}
  assert(rk_recording_reader_open(truncated.c_str(),&r)==RK_ERROR_BACKEND);std::remove(truncated.c_str());
  const std::string incomplete="robotkit-recording-incomplete.mcap";
  assert(rk_recording_writer_create(incomplete.c_str(),1024,&w)==RK_OK);rk_recording_writer_destroy(w);
  assert(rk_recording_reader_open(incomplete.c_str(),&r)==RK_ERROR_INVALID_STATE);
  std::remove(incomplete.c_str());std::remove((incomplete+".incomplete").c_str());
}
