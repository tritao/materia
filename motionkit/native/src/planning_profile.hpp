#ifndef MOTIONKIT_PLANNING_PROFILE_HPP
#define MOTIONKIT_PLANNING_PROFILE_HPP

#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace motionkit {
// Opt-in wall and CPU stage measurements; disabled runs read neither clock.
class PathProfilePhase {
    using Clock=std::chrono::steady_clock;
    const bool enabled_;
    const char* stage_;
    Clock::time_point began_{};
    std::clock_t cpu_{};
public:
    explicit PathProfilePhase(const char* stage):enabled_([] {
        const char* value=std::getenv("PROCESS_PATH_PROFILE");
        return value && std::strcmp(value,"1")==0;
    }()),stage_(stage) { if (enabled_) { began_=Clock::now();cpu_=std::clock(); } }
    void mark(const char* next) {
        if (enabled_) {
            const auto now=Clock::now();
            const auto cpu=std::clock();
            std::fprintf(stderr,"PROCESS_PATH_STAGE {\"stage\":\"%s\",\"seconds\":%.9g,\"cpuSeconds\":%.9g}\n",
                stage_,std::chrono::duration<double>(now-began_).count(),
                static_cast<double>(cpu-cpu_)/CLOCKS_PER_SEC);
            began_=now;cpu_=cpu;
        }
        stage_=next;
    }
    bool enabled() const { return enabled_; }
    ~PathProfilePhase() { if (enabled_) mark(stage_); }
};

}
#endif
