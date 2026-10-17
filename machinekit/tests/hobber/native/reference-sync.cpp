// HB7 ElectronicGear implementation with the same explicitly assumed servo plant.
// This exports simulated observations, not measurements of a built machine.
#include "electronic_gear.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <memory>
#include <numbers>
#include <string>

static rk_event_value event(const char *command, double argument=0) {
    rk_event_value v{}; v.kind=RK_EVENT_PROCESS;
    std::strncpy(v.command,command,sizeof(v.command)-1); v.argument=argument; return v;
}
int main(int argc,char **argv) {
    if(argc!=6) return 2; // output, rpm, duration, master counts, work teeth
    const double rpm=std::stod(argv[2]), duration=std::stod(argv[3]), counts=std::stod(argv[4]);
    const double ratio=1/std::stod(argv[5]), dt=.00025, omega=rpm*2*std::numbers::pi/60;
    if(!(rpm>0&&duration>0&&duration<1000&&counts>0&&ratio>0))return 2;
    robotkit::ElectronicGear gears[2];
    for(unsigned k=0;k<2;++k) {
        for(auto v:{event("gear.begin"),event("gear.loop",4000),
                    event(k==0?"gear.term/0/1000000/1":"gear.term/0/1000000/0",ratio),
                    event("gear.quantum",2*std::numbers::pi/counts),event("gear.engage")})
            if(!gears[k].event(1,v,2))return 3;
    }
    const double bandwidth=2*std::numbers::pi*200;
    double p[2]={-ratio*omega,-ratio*omega},v[2]={ratio*omega,ratio*omega};
    auto observed=std::make_unique<rk_robot_state>(); observed->joint_count=2;
    std::ofstream out(argv[1]);if(!out)return 4;
    out<<std::setprecision(17)<<"time_s,predicted_error_rad,no_prediction_error_rad\n";
    // One second warm-up before the reference feed. Label the integrated plant
    // state at the END of each tick, avoiding a one-tick phase bookkeeping error.
    for(long tick=-4000;tick<=static_cast<long>(std::ceil(duration/dt));++tick) {
        const double t=tick*dt;
        if(t>=0)out<<t<<','<<p[0]-ratio*omega*t<<','<<p[1]-ratio*omega*t<<'\n';
        observed->position[0]=omega*(t-.001);observed->velocity[0]=omega;
        for(unsigned k=0;k<2;++k) {
            const double target=gears[k].offset(1,*observed,0);
            const double acceleration=bandwidth*bandwidth*(target-p[k])+2*bandwidth*(gears[k].velocity(1,*observed)-v[k]);
            v[k]+=acceleration*dt;p[k]+=v[k]*dt;
        }
    }
    return out?0:5;
}
