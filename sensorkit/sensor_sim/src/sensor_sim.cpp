#include "nativekit_sensor_sim.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace nksensor::sim {
namespace {

bool find_body(nksim_snapshot snapshot, nksim_body wanted, nksim_body_state &out_state) noexcept {
    std::uint64_t body_count = 0;
    if (nksim_snapshot_get_body_count(snapshot, &body_count) != NKSIM_OK)
        return false;

    for (std::uint64_t index = 0; index < body_count; ++index) {
        nksim_body_state state{};
        state.struct_size = sizeof(state);
        if (nksim_snapshot_get_body(snapshot, index, &state) != NKSIM_OK)
            return false;
        if (state.body == wanted) {
            out_state = state;
            return true;
        }
    }
    return false;
}

Vec3 vector_from_array(const double value[3]) noexcept {
    return {value[0], value[1], value[2]};
}

double quaternion_norm(Quaternion value) noexcept {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z +
                     value.w * value.w);
}

Vec3 rotate(Quaternion orientation, Vec3 value) noexcept {
    const auto norm = quaternion_norm(orientation);
    if (!(norm > 0.0) || !std::isfinite(norm))
        return value;
    const auto x = orientation.x / norm;
    const auto y = orientation.y / norm;
    const auto z = orientation.z / norm;
    const auto w = orientation.w / norm;
    const auto tx = 2.0 * (y * value.z - z * value.y);
    const auto ty = 2.0 * (z * value.x - x * value.z);
    const auto tz = 2.0 * (x * value.y - y * value.x);
    return {
        value.x + w * tx + (y * tz - z * ty),
        value.y + w * ty + (z * tx - x * tz),
        value.z + w * tz + (x * ty - y * tx)};
}

Vec3 rotate_inverse(Quaternion orientation, Vec3 value) noexcept {
    return rotate({-orientation.x, -orientation.y, -orientation.z, orientation.w}, value);
}

} // namespace

ImuTruthAdapter::ImuTruthAdapter(nksim_body body, Vec3 gravity) noexcept
    : body_(body), gravity_(gravity) {}

std::optional<ImuTruth> ImuTruthAdapter::read(nksim_snapshot snapshot) noexcept {
    if (snapshot == NKSIM_INVALID_SNAPSHOT) {
        last_result_ = NKSIM_ERROR_INVALID_HANDLE;
        return std::nullopt;
    }

    nksim_clock clock{};
    clock.struct_size = sizeof(clock);
    last_result_ = nksim_snapshot_get_clock(snapshot, &clock);
    if (last_result_ != NKSIM_OK)
        return std::nullopt;

    nksim_body_state state{};
    state.struct_size = sizeof(state);
    if (!find_body(snapshot, body_, state)) {
        last_result_ = NKSIM_ERROR_STALE_ID;
        return std::nullopt;
    }

    ImuTruth truth;
    truth.time = clock.time;
    truth.pose.position = vector_from_array(state.position);
    truth.pose.orientation = {
        state.rotation[0], state.rotation[1], state.rotation[2], state.rotation[3]};
    truth.linear_velocity = vector_from_array(state.linear_velocity);
    truth.angular_velocity = vector_from_array(state.angular_velocity);
    truth.gravity = gravity_;

    const auto delta_time = truth.time - previous_time_;
    if (has_previous_ && delta_time > 0.0 && std::isfinite(delta_time)) {
        truth.linear_acceleration =
            (truth.linear_velocity - previous_linear_velocity_) * (1.0 / delta_time);
    }

    previous_linear_velocity_ = truth.linear_velocity;
    previous_time_ = truth.time;
    has_previous_ = true;
    last_result_ = NKSIM_OK;
    return truth;
}

void ImuTruthAdapter::reset() noexcept {
    previous_linear_velocity_ = {};
    previous_time_ = 0.0;
    has_previous_ = false;
    last_result_ = NKSIM_OK;
}

SceneLidarAdapter::~SceneLidarAdapter() {
    reset();
}

SceneLidarAdapter::SceneLidarAdapter(SceneLidarAdapter &&other) noexcept
    : index_(other.index_) {
    other.index_ = 0;
}

SceneLidarAdapter &SceneLidarAdapter::operator=(SceneLidarAdapter &&other) noexcept {
    if (this == &other)
        return *this;
    reset();
    index_ = other.index_;
    other.index_ = 0;
    return *this;
}

nkscene_result SceneLidarAdapter::build(nkscene_snapshot snapshot) noexcept {
    nkscene_render_spatial_index candidate = 0;
    const auto result = nkscene_render_spatial_index_create(snapshot, &candidate);
    if (result != NKS_OK)
        return result;
    reset();
    index_ = candidate;
    return NKS_OK;
}

void SceneLidarAdapter::reset() noexcept {
    if (index_ != 0) {
        nkscene_render_spatial_index_destroy(index_);
        index_ = 0;
    }
}

nkscene_result SceneLidarAdapter::scan(LidarSensor &sensor, const SensorTick &tick,
                                        const Pose &sensor_pose,
                                        std::optional<LidarScan> &out_scan) const {
    out_scan.reset();
    if (!ready())
        return NKS_ERROR_INVALID_STATE;

    const auto local_rays = sensor.rays();
    std::vector<nkscene_render_ray> world_rays;
    world_rays.reserve(local_rays.size());
    for (const auto &local_ray : local_rays) {
        const auto world_origin = sensor_pose.position +
                                  rotate(sensor_pose.orientation, local_ray.origin);
        const auto world_direction = rotate(sensor_pose.orientation, local_ray.direction);
        world_rays.push_back({
            {static_cast<float>(world_origin.x), static_cast<float>(world_origin.y),
             static_cast<float>(world_origin.z)},
            {static_cast<float>(world_direction.x), static_cast<float>(world_direction.y),
             static_cast<float>(world_direction.z)}});
    }

    std::vector<nkscene_render_pick_result> results(local_rays.size());
    const auto result = nkscene_render_spatial_index_pick_rays(
        index_, world_rays.data(), world_rays.size(), results.data());
    if (result != NKS_OK)
        return result;

    std::vector<LidarHit> hits(local_rays.size());
    for (std::size_t index = 0; index < local_rays.size(); ++index) {
        const auto &pick = results[index];
        if (!pick.occurrence.value)
            continue;
        const Vec3 world_point{
            pick.world_position[0], pick.world_position[1], pick.world_position[2]};
        const auto local_point = rotate_inverse(
            sensor_pose.orientation, world_point - sensor_pose.position);
        hits[index].hit = true;
        hits[index].range = pick.depth;
        hits[index].point = local_point;
    }

    out_scan = sensor.sample(tick, hits);
    return NKS_OK;
}

} // namespace nksensor::sim
