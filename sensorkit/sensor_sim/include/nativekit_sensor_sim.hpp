#pragma once

#include "nativekit_sensor_core.hpp"
#include "nativekit_scene_render.h"
#include "nativekit_sim.h"

#include <optional>

#if defined(_WIN32)
#if defined(NKSENSOR_SIM_STATIC)
#define NKSENSOR_SIM_API
#elif defined(NKSENSOR_SIM_BUILDING_LIBRARY)
#define NKSENSOR_SIM_API __declspec(dllexport)
#else
#define NKSENSOR_SIM_API __declspec(dllimport)
#endif
#else
#define NKSENSOR_SIM_API __attribute__((visibility("default")))
#endif

namespace nksensor::sim {

/**
 * Reads one body from an immutable SimKit snapshot and turns it into sensor
 * truth. The adapter never accesses a live world and does not retain the
 * snapshot handle passed to read().
 *
 * Linear acceleration is estimated from consecutive snapshot velocities. The
 * first sample and samples after a non-increasing time jump have zero
 * estimated acceleration.
 */
class NKSENSOR_SIM_API ImuTruthAdapter {
public:
    ImuTruthAdapter(nksim_body body, Vec3 gravity) noexcept;

    nksim_body body() const noexcept { return body_; }
    Vec3 gravity() const noexcept { return gravity_; }
    void set_gravity(Vec3 gravity) noexcept { gravity_ = gravity; }

    /** Read the bound body from one immutable snapshot. */
    std::optional<ImuTruth> read(nksim_snapshot snapshot) noexcept;

    /** Last SimKit result produced by read(). */
    nksim_result last_result() const noexcept { return last_result_; }

    /** Discard derivative history after a seek or source restart. */
    void reset() noexcept;

private:
    nksim_body body_ = NKSIM_INVALID_BODY;
    Vec3 gravity_;
    Vec3 previous_linear_velocity_;
    double previous_time_ = 0.0;
    bool has_previous_ = false;
    nksim_result last_result_ = NKSIM_OK;
};

/**
 * Runs LiDAR rays against a read-only SceneKit spatial index. The index owns
 * its copy of the scene snapshot, so callers may destroy the source snapshot
 * after build() returns.
 */
class NKSENSOR_SIM_API SceneLidarAdapter {
public:
    SceneLidarAdapter() = default;
    ~SceneLidarAdapter();

    SceneLidarAdapter(const SceneLidarAdapter &) = delete;
    SceneLidarAdapter &operator=(const SceneLidarAdapter &) = delete;
    SceneLidarAdapter(SceneLidarAdapter &&other) noexcept;
    SceneLidarAdapter &operator=(SceneLidarAdapter &&other) noexcept;

    nkscene_result build(nkscene_snapshot snapshot) noexcept;
    void reset() noexcept;
    bool ready() const noexcept { return index_ != 0; }

    /** Raycast and model one scan in the supplied world pose. */
    nkscene_result scan(LidarSensor &sensor, const SensorTick &tick,
                        const Pose &sensor_pose, std::optional<LidarScan> &out_scan) const;

private:
    nkscene_render_spatial_index index_ = 0;
};

} // namespace nksensor::sim
