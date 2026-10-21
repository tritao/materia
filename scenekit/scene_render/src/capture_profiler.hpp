#pragma once

#include "nativekit_gpu.h"
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <array>

namespace nkscene::render_internal {

/** Opt-in diagnostics, sampling every 16 frames by default. Query results are
 * polled on later sampled frames without waiting for GPU completion. */
class CaptureProfiler {
    using Clock = std::chrono::steady_clock;
public:
    struct Interval {
        nkgpu_timestamp query{};
        std::string pass;
        bool ended = false;
        double ms = 0;
    };
    struct Sample {
        nkgpu_renderer renderer{};
        std::vector<Interval> intervals;
        std::uint64_t frame = 0;
        std::array<float,16> view_projection{};
        std::size_t clip_plane_count=0;
        Clock::time_point started{}, prepared{};
        double wall_seconds = 0.0;
        double prepare_ms = 0.0;
        double synchronize_ms = 0.0;
        double geometry_ms = 0.0;
        double packing_ms = 0.0;
        double submit_ms = 0.0;
        bool active = false;
        bool gpu_failed = false;
    };

    explicit CaptureProfiler(const char *path) : output_(path, std::ios::app) {
        if (const auto *value = std::getenv("NK_SCENE_GPU_TIMING_INTERVAL"); value && *value) {
            char *end = nullptr;
            const auto parsed = std::strtoul(value, &end, 10);
            if (end != value && !*end && parsed > 0)
                sample_interval_ = parsed;
        }
    }
    bool enabled() const noexcept { return output_.is_open(); }
    bool active() const noexcept { return active_frame_; }

    Sample start(nkgpu_renderer renderer) {
        ++frame_count_;
        active_frame_ = (frame_count_ - 1) % sample_interval_ == 0;
        if (!active_frame_) return {};
        collect();
        geometry_ms_ = 0.0;
        packing_ms_ = 0.0;
        Sample sample;
        sample.renderer = renderer;
        sample.frame = frame_count_;
        sample.active = true;
        sample.wall_seconds = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sample.started = Clock::now();
        return sample;
    }

    void packingElapsed(std::chrono::steady_clock::duration elapsed) {
        if (active_frame_) packing_ms_ += milliseconds(elapsed);
    }

    void geometryElapsed(std::chrono::steady_clock::duration elapsed) {
        if (active_frame_) geometry_ms_ += milliseconds(elapsed);
    }

    void synchronized(Sample &sample) {
        if (!sample.active) return;
        sample.geometry_ms = geometry_ms_;
        sample.packing_ms = packing_ms_;
        sample.synchronize_ms = milliseconds(Clock::now() - sample.started);
    }

    void prepared(Sample &sample) {
        if (!sample.active) return;
        sample.prepared = Clock::now();
        sample.prepare_ms = milliseconds(sample.prepared - sample.started);
    }

    // Intervals never overlap: GL_TIME_ELAPSED cannot nest. A pass can have
    // several intervals (per-part parity/quads); aggregate them on collection.
    class Binding {
    public:
        Binding(CaptureProfiler *profiler, Sample &sample) : profiler_(profiler), sample_(sample) {
            if(profiler_)profiler_->current_=sample.active?&sample:nullptr;
        }
        ~Binding() { if(profiler_)profiler_->abort(sample_); }
        Binding(const Binding &)=delete;
        Binding &operator=(const Binding &)=delete;
    private:
        CaptureProfiler *profiler_;
        Sample &sample_;
    };
    void pass(const char *name) { if(current_) begin(*current_,name); }
    void pause() { if(current_) end(*current_); }
    void begin(Sample &sample, const char *name="surfaces") {
        if (!sample.active) return;
        end(sample);
        if (!queries_supported_ || pending_.size() >= 8) return;
        Interval interval;interval.pass=name;
        if(nkgpu_timestamp_begin(sample.renderer,&interval.query)!=NKGPU_OK) {
            sample.gpu_failed=true;queries_supported_=false;return;
        }
        sample.intervals.push_back(interval);
    }
    void end(Sample &sample) {
        if(!sample.active || sample.intervals.empty())return;
        auto &interval=sample.intervals.back();
        if(interval.ended)return;
        interval.ended=nkgpu_timestamp_end(sample.renderer,interval.query)==NKGPU_OK;
        if(!interval.ended){sample.gpu_failed=true;queries_supported_=false;}
    }
    void finish(Sample &sample) {
        if(!sample.active)return;
        end(sample);current_=nullptr;
        sample.submit_ms=milliseconds(Clock::now()-sample.prepared);
        if(!sample.intervals.empty())pending_.push_back(std::move(sample));
        else write(sample,false);
        sample.active=false;
    }
    void abort(Sample &sample) {
        if(!sample.active)return;
        end(sample);current_=nullptr;destroy(sample);sample.active=false;
    }
    void release() {
        collect();
        for(auto &sample:pending_){write(sample,false);destroy(sample);}
        pending_.clear();
    }

private:
    Sample *current_=nullptr;
    std::ofstream output_;
    std::vector<Sample> pending_;
    std::uint64_t frame_count_ = 0;
    std::uint64_t sample_interval_ = 16;
    bool active_frame_ = false;
    bool queries_supported_ = true;
    double geometry_ms_ = 0.0;
    double packing_ms_ = 0.0;

    static double milliseconds(Clock::duration value) {
        return std::chrono::duration<double, std::milli>(value).count();
    }

    void destroy(Sample &sample) {
        for(auto &interval:sample.intervals)(void)nkgpu_timestamp_destroy(sample.renderer,interval.query);
        sample.intervals.clear();
    }
    void collect() {
        for(auto it=pending_.begin();it!=pending_.end();) {
            bool pending=false,valid=!it->gpu_failed;
            for(auto &interval:it->intervals) {
                nkgpu_timestamp_info info{};info.struct_size=sizeof(info);
                const auto result=nkgpu_timestamp_query(it->renderer,interval.query,&info);
                if(result==NKGPU_OK && info.state==NKGPU_TIMESTAMP_PENDING)pending=true;
                else if(result!=NKGPU_OK || info.state!=NKGPU_TIMESTAMP_READY)valid=false;
                else interval.ms=double(info.nanoseconds)/1000000.0;
            }
            if(pending){++it;continue;}
            write(*it,valid);destroy(*it);it=pending_.erase(it);
        }
    }
    void write(const Sample &sample, bool valid) {
        std::ostringstream line;
        line << std::setprecision(17) << "{\"frame\":" << sample.frame
             << ",\"startedAtSeconds\":" << sample.wall_seconds
             << ",\"prepareMilliseconds\":" << sample.prepare_ms
             << ",\"synchronizeMilliseconds\":" << sample.synchronize_ms
             << ",\"geometryMilliseconds\":" << sample.geometry_ms
             << ",\"packingMilliseconds\":" << sample.packing_ms
             << ",\"submitMilliseconds\":" << sample.submit_ms << ",\"gpuMilliseconds\":";
        std::map<std::string,double> passes;double total=0;
        for(const auto &interval:sample.intervals){passes[interval.pass]+=interval.ms;total+=interval.ms;}
        if(valid)line<<total;else line<<"null";
        line<<",\"gpuPassMilliseconds\":{";
        bool first=true;
        for(const auto &[name,ms]:passes){if(!first)line<<",";first=false;line<<"\""<<name<<"\":";if(valid)line<<ms;else line<<"null";}
        line << "},\"clipPlaneCount\":"<<sample.clip_plane_count<<",\"viewProjection\":[";
        for(std::size_t i=0;i<sample.view_projection.size();++i){if(i)line<<",";line<<sample.view_projection[i];}
        line << "]}\n";
        output_ << line.str();
        output_.flush();
    }
};
} // namespace nkscene::render_internal
