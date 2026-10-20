// Sample the prebuilt MotionKit TOPPRA/Ruckig planners, SI units. No CAD/native rebuild.
#include "motionkit.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
static void check(mk_result r) { if(r!=MK_OK)throw std::runtime_error("MotionKit result "+std::to_string(r)); }
int main(int argc,char**argv) {
 try {
  if(argc!=7)throw std::runtime_error("backend distance vmax initialV finalV dt");
  const std::string backend=argv[1];double distance=std::atof(argv[2]),vmax=std::atof(argv[3]),v0=std::atof(argv[4]),v1=std::atof(argv[5]),dt=std::atof(argv[6]);
  mk_trajectory_handle trajectory{};mk_time_law_handle law{};mk_path_handle path{};
  if(backend=="ruckig") {
   mk_state_to_state_request request{};request.struct_size=sizeof(request);request.joint_count=1;
   request.current_velocity[0]=v0;request.target_position[0]=distance;request.target_velocity[0]=v1;
   request.max_velocity[0]=vmax;request.max_acceleration[0]=3;request.max_jerk[0]=20;
   int32_t result=0;check(mk_generate_state_to_state(&request,&trajectory,&result));
  } else if(backend=="toppra") {
   mk_path_sample samples[3]{};
   for(int i=0;i<3;i++){samples[i].struct_size=sizeof(samples[i]);samples[i].joint_count=1;samples[i].s=distance*i/2.;samples[i].position[0]=samples[i].s;samples[i].first[0]=1;}
   check(mk_path_create(samples,3,&path));double acceleration=3;
   mk_timing_report report{};report.struct_size=sizeof(report);
   if(v0>0) {
    mk_path_timing_session_handle session{};
    check(mk_path_timing_session_create(path,&vmax,&acceleration,1,nullptr,0,1e-7,0,&session));
    double lower=0,upper=0;check(mk_path_timing_session_entry_interval(session,v1,v1,1,&lower,&upper));
    const double entry=std::fmin(v0,upper*(1-1e-8));
    check(mk_path_timing_session_time(session,entry,v1,&report,&law,&trajectory));mk_path_timing_session_destroy(session);
   } else check(mk_time_path(path,&vmax,&acceleration,1,nullptr,0,v0,v1,1e-7,0,&report,&law,&trajectory));
  } else throw std::runtime_error("Unknown backend");
  int64_t duration=0;check(mk_trajectory_duration_ns(trajectory,&duration));
  std::printf("# duration %.12g\nt,p,v,a,j\n",duration*1e-9);
  const int count=static_cast<int>(std::ceil(duration*1e-9/dt));
  for(int i=0;i<=count;i++) {
   int64_t t=i==count?duration:static_cast<int64_t>(std::llround(i*dt*1e9));
   mk_trajectory_state state{};state.struct_size=sizeof(state);check(mk_trajectory_evaluate(trajectory,t,&state));
   std::printf("%.12g,%.12g,%.12g,%.12g,%.12g\n",t*1e-9,state.position[0],state.velocity[0],state.acceleration[0],state.jerk[0]);
  }
  mk_trajectory_destroy(trajectory);if(law.id)mk_time_law_destroy(law);if(path.id)mk_path_destroy(path);return 0;
 } catch(const std::exception&e){std::fprintf(stderr,"%s\n",e.what());return 1;}
}
