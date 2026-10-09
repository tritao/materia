#pragma once

#include "nativekit_gpu.h"
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace nkscene::render_internal {

/** Opt-in diagnostics, sampling every 16 frames by default. Query results are
 * polled on later sampled frames without waiting for GPU completion. */
class CaptureProfiler {
    using Clock = std::chrono::steady_clock;
public:
    struct Sample {
        nkgpu_renderer renderer{};
        nkgpu_timestamp query{};
        std::uint64_t frame = 0;
        Clock::time_point started{}, prepared{};
        double wall_seconds = 0.0;
        double prepare_ms = 0.0;
        double synchronize_ms = 0.0;
        double geometry_ms = 0.0;
        double packing_ms = 0.0;
        double submit_ms = 0.0;
        bool active = false;
        bool query_ended = false;
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

    void begin(Sample &sample) {
        if (!sample.active) return;
        // Bound retained query resources even if the GPU stops completing work.
        if (queries_supported_ && pending_.size() < 8 &&
            nkgpu_timestamp_begin(sample.renderer, &sample.query) != NKGPU_OK) {
            sample.query = {};
            queries_supported_ = false;
        }
    }

    void end(Sample &sample) {
        if (!sample.active) return;
        if (!sample.query.id || sample.query_ended) return;
        if (nkgpu_timestamp_end(sample.renderer, sample.query) == NKGPU_OK)
            sample.query_ended = true;
        else {
            (void)nkgpu_timestamp_destroy(sample.renderer, sample.query);
            sample.query = {};
            queries_supported_ = false;
        }
    }

    void finish(Sample &sample) {
        if (!sample.active) return;
        sample.submit_ms = milliseconds(Clock::now() - sample.prepared);
        if (sample.query.id && sample.query_ended) pending_.push_back(sample);
        else write(sample, nullptr);
        sample.query = {};
    }

    void abort(Sample &sample) {
        if (!sample.active) return;
        end(sample);
        if (sample.query.id) (void)nkgpu_timestamp_destroy(sample.renderer, sample.query);
        sample.query = {};
    }

    void release() {
        collect();
        for (const auto &sample : pending_) {
            write(sample, nullptr);
            (void)nkgpu_timestamp_destroy(sample.renderer, sample.query);
        }
        pending_.clear();
    }

private:
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

    void collect() {
        for (auto it = pending_.begin(); it != pending_.end();) {
            nkgpu_timestamp_info info{};
            info.struct_size = sizeof(info);
            const auto result = nkgpu_timestamp_query(it->renderer, it->query, &info);
            if (result == NKGPU_OK && info.state == NKGPU_TIMESTAMP_PENDING) { ++it; continue; }
            write(*it, result == NKGPU_OK && info.state == NKGPU_TIMESTAMP_READY ? &info : nullptr);
            (void)nkgpu_timestamp_destroy(it->renderer, it->query);
            it = pending_.erase(it);
        }
    }

    void write(const Sample &sample, const nkgpu_timestamp_info *info) {
        std::ostringstream line;
        line << std::setprecision(17) << "{\"frame\":" << sample.frame
             << ",\"startedAtSeconds\":" << sample.wall_seconds
             << ",\"prepareMilliseconds\":" << sample.prepare_ms
             << ",\"synchronizeMilliseconds\":" << sample.synchronize_ms
             << ",\"geometryMilliseconds\":" << sample.geometry_ms
             << ",\"packingMilliseconds\":" << sample.packing_ms
             << ",\"submitMilliseconds\":" << sample.submit_ms << ",\"gpuMilliseconds\":";
        if (info) line << static_cast<double>(info->nanoseconds) / 1000000.0;
        else line << "null";
        line << "}\n";
        output_ << line.str();
        output_.flush();
    }
};
} // namespace nkscene::render_internal
