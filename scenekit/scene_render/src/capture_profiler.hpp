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
        nkgpu_renderer_stats before{};
        std::uint64_t draw_calls = 0, triangles = 0, upload_bytes = 0;
        std::uint64_t lod_draw_calls = 0, lod_triangles_saved = 0;
        std::map<std::string, std::uint64_t> pass_triangles, pass_draws;
        std::string current_pass;
        bool active = false;
        bool gpu_failed = false;
    };

    explicit CaptureProfiler(const char *path) : output_(path, std::ios::app), inventory_path_(std::string(path) + ".inventory.tsv") {
        if (const auto *value = std::getenv("NK_SCENE_GPU_TIMING_INTERVAL"); value && *value) {
            char *end = nullptr;
            const auto parsed = std::strtoul(value, &end, 10);
            if (end != value && !*end && parsed > 0)
                sample_interval_ = parsed;
        }
    }
    bool enabled() const noexcept { return output_.is_open(); }
    bool active() const noexcept { return active_frame_; }

    // Camera motion leaves the inventory intact. Keep the latest resource-level
    // census beside the timeline, outside the measured submission interval.
    void inventory(const RenderPlan &plan, const SceneSnapshot &snapshot) {
        if (inventory_revision_ == snapshot.revision()) return;
        inventory_revision_ = snapshot.revision();
        std::ofstream out(inventory_path_);
        out << "node\tgeometry\tmaterial\ttriangles\tstrokes\thidden\tculled\tname\n";
        for (const auto &item : plan.items()) {
            const auto *geometry = snapshot.find_geometry(item.geometry);
            const auto *node = snapshot.find_node(item.node);
            if (!geometry || !node) continue;
            auto name = node->name;
            for (auto &c : name) if (c == '\t' || c == '\n' || c == '\r') c = ' ';
            out << item.node.value << '\t' << item.geometry.value << '\t' << item.material.value
                << '\t' << geometry->payload->element_count()/3
                << '\t' << geometry->payload->stroke_segments.size()
                << '\t' << has_render_flag(item.flags, RenderFlags::Hidden)
                << '\t' << has_render_flag(item.flags, RenderFlags::Culled)
                << '\t' << name << '\n';
        }
    }

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
        sample.before.struct_size = sizeof(sample.before);
        nkgpu_renderer_get_stats(renderer, &sample.before);
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
    void triangles(std::uint64_t elements, std::uint64_t instances = 1) {
        if (current_) {
            const auto count = (elements / 3) * instances;
            current_->triangles += count;
            current_->pass_triangles[current_->current_pass] += count;
            ++current_->pass_draws[current_->current_pass];
        }
    }
    void pause() { if(current_) end(*current_); }
    void surfaceLod(std::uint64_t original, std::uint64_t selected, std::uint64_t instances) {
        if (current_ && original > selected) {
            ++current_->lod_draw_calls;
            current_->lod_triangles_saved += (original-selected)/3*instances;
        }
    }
    void begin(Sample &sample, const char *name="surfaces") {
        if (!sample.active) return;
        end(sample);
        sample.current_pass = name;
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
        nkgpu_renderer_stats after{}; after.struct_size = sizeof(after);
        nkgpu_renderer_get_stats(sample.renderer, &after);
        sample.draw_calls = after.draw_calls - sample.before.draw_calls;
        sample.upload_bytes = after.upload_bytes - sample.before.upload_bytes;
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
    std::string inventory_path_;
    std::uint64_t inventory_revision_ = ~std::uint64_t{0};
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
             << ",\"drawCalls\":" << sample.draw_calls
             << ",\"triangles\":" << sample.triangles
             << ",\"lodDrawCalls\":" << sample.lod_draw_calls
             << ",\"lodTrianglesSaved\":" << sample.lod_triangles_saved
             << ",\"uploadBytes\":" << sample.upload_bytes
             << ",\"submitMilliseconds\":" << sample.submit_ms << ",\"gpuMilliseconds\":";
        std::map<std::string,double> passes;double total=0;
        for(const auto &interval:sample.intervals){passes[interval.pass]+=interval.ms;total+=interval.ms;}
        if(valid)line<<total;else line<<"null";
        line<<",\"gpuPassMilliseconds\":{";
        bool first=true;
        for(const auto &[name,ms]:passes){if(!first)line<<",";first=false;line<<"\""<<name<<"\":";if(valid)line<<ms;else line<<"null";}
        line << "},\"passWork\":{";
        first=true;
        for(const auto &[name,count]:sample.pass_triangles) {
            if(!first)line<<",";first=false;
            line<<"\""<<name<<"\":{\"triangles\":"<<count<<",\"drawCalls\":"<<sample.pass_draws.at(name)<<"}";
        }
        line << "},\"clipPlaneCount\":"<<sample.clip_plane_count<<",\"viewProjection\":[";
        for(std::size_t i=0;i<sample.view_projection.size();++i){if(i)line<<",";line<<sample.view_projection[i];}
        line << "]}\n";
        output_ << line.str();
        output_.flush();
    }
};
} // namespace nkscene::render_internal
