#include "nativekit_sensor_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nksensor {
namespace {

constexpr double schedule_epsilon = 1e-12;

double clamp(double value, double minimum, double maximum) noexcept {
    return std::clamp(value, minimum, maximum);
}

double quantize(double value, double step) noexcept {
    if (!(step > 0.0) || !std::isfinite(step))
        return value;
    return std::round(value / step) * step;
}

double gaussian_scalar(std::mt19937_64 &random, double standard_deviation) {
    if (!(standard_deviation > 0.0))
        return 0.0;
    return std::normal_distribution<double>(0.0, standard_deviation)(random);
}

Vec3 componentwise_clamp(Vec3 value, Vec3 minimum, Vec3 maximum) noexcept {
    return {
        clamp(value.x, minimum.x, maximum.x),
        clamp(value.y, minimum.y, maximum.y),
        clamp(value.z, minimum.z, maximum.z)};
}

Vec3 componentwise_quantize(Vec3 value, Vec3 step) noexcept {
    return {quantize(value.x, step.x), quantize(value.y, step.y), quantize(value.z, step.z)};
}

Vec3 gaussian_vec3(std::mt19937_64 &random, Vec3 standard_deviation) {
    std::normal_distribution<double> normal(0.0, 1.0);
    return {
        normal(random) * standard_deviation.x,
        normal(random) * standard_deviation.y,
        normal(random) * standard_deviation.z};
}

double quaternion_norm(Quaternion value) noexcept {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z +
                     value.w * value.w);
}

Vec3 rotate_by_inverse(Quaternion orientation, Vec3 value) noexcept {
    const auto norm = quaternion_norm(orientation);
    if (!(norm > 0.0) || !std::isfinite(norm))
        return value;

    const auto x = orientation.x / norm;
    const auto y = orientation.y / norm;
    const auto z = orientation.z / norm;
    const auto w = orientation.w / norm;

    /* q^-1 * (v, 0) * q, expanded to avoid another public math dependency. */
    const auto tx = 2.0 * (y * value.z - z * value.y);
    const auto ty = 2.0 * (z * value.x - x * value.z);
    const auto tz = 2.0 * (x * value.y - y * value.x);
    return {
        value.x - w * tx + (y * tz - z * ty),
        value.y - w * ty + (z * tx - x * tz),
        value.z - w * tz + (x * ty - y * tx)};
}

} // namespace

GaussianNoise::GaussianNoise(double standard_deviation, std::uint64_t seed)
    : standard_deviation_(std::max(0.0, standard_deviation)),
      random_(seed),
      normal_(0.0, 1.0) {}

double GaussianNoise::sample() {
    return normal_(random_) * standard_deviation_;
}

void GaussianNoise::reseed(std::uint64_t seed) {
    random_.seed(seed);
    normal_.reset();
}

Sensor::Sensor(SensorConfig config)
    : config_(std::move(config)),
      period_(config_.timing.update_rate_hz > 0.0
                  ? 1.0 / config_.timing.update_rate_hz
                  : 0.0),
      next_capture_time_(config_.timing.phase_offset),
      random_(config_.seed) {
    config_.timing.capture_delay = std::max(0.0, config_.timing.capture_delay);
    config_.timing.integration_period = std::max(0.0, config_.timing.integration_period);
    config_.timing.latency = std::max(0.0, config_.timing.latency);
    config_.timing.latency_jitter = std::max(0.0, config_.timing.latency_jitter);
    config_.timing.dropout_probability =
        clamp(config_.timing.dropout_probability, 0.0, 1.0);
}

double Sensor::uniform_zero_to_one() {
    return std::generate_canonical<double, 53>(random_);
}

SensorTick Sensor::make_tick(double capture_time) {
    const auto jitter = config_.timing.latency_jitter > 0.0
                            ? std::normal_distribution<double>(0.0,
                                                               config_.timing.latency_jitter)(random_)
                            : 0.0;
    const auto delivery_delay = std::max(0.0, config_.timing.latency + jitter);
    const auto dropout = uniform_zero_to_one() < config_.timing.dropout_probability;

    SensorTick tick;
    tick.header.sensor = config_.id;
    tick.header.sequence = sequence_++;
    tick.header.capture_time = capture_time;
    tick.header.delivery_time = capture_time + config_.timing.capture_delay + delivery_delay;
    tick.header.frame = config_.frame;
    tick.integration_start_time = capture_time - config_.timing.integration_period;
    tick.dropped = dropout;
    return tick;
}

std::vector<SensorTick> Sensor::poll(double simulation_time) {
    std::vector<SensorTick> ticks;
    if (!config_.enabled || !periodic() || !std::isfinite(simulation_time))
        return ticks;

    while (next_capture_time_ <= simulation_time + schedule_epsilon) {
        ticks.emplace_back(make_tick(next_capture_time_));
        next_capture_time_ += period_;
        if (!std::isfinite(next_capture_time_))
            break;
    }
    return ticks;
}

std::optional<SensorTick> Sensor::trigger(double capture_time) {
    if (!config_.enabled || !std::isfinite(capture_time))
        return std::nullopt;
    return make_tick(capture_time);
}

void Sensor::reset(double next_capture_time) noexcept {
    next_capture_time_ = next_capture_time;
    sequence_ = 0;
}

bool SensorManager::add(const std::shared_ptr<Sensor> &sensor) {
    if (!sensor || sensor->id() == invalid_sensor || find(sensor->id()))
        return false;
    sensors_.push_back(sensor);
    return true;
}

bool SensorManager::remove(SensorId id) {
    const auto found = std::remove_if(sensors_.begin(), sensors_.end(),
                                      [id](const auto &sensor) {
                                          return sensor && sensor->id() == id;
                                      });
    if (found == sensors_.end())
        return false;
    sensors_.erase(found, sensors_.end());
    return true;
}

std::shared_ptr<Sensor> SensorManager::find(SensorId id) const {
    const auto found = std::find_if(sensors_.begin(), sensors_.end(),
                                    [id](const auto &sensor) {
                                        return sensor && sensor->id() == id;
                                    });
    return found == sensors_.end() ? nullptr : *found;
}

std::vector<SensorTick> SensorManager::poll(double simulation_time) {
    std::vector<SensorTick> ticks;
    for (const auto &sensor : sensors_) {
        if (!sensor)
            continue;
        auto sensor_ticks = sensor->poll(simulation_time);
        ticks.insert(ticks.end(), sensor_ticks.begin(), sensor_ticks.end());
    }
    return ticks;
}

ImuSensor::ImuSensor(SensorConfig config, ImuNoiseConfig noise)
    : Sensor(std::move(config)), noise_(noise) {}

void ImuSensor::reset_model() noexcept {
    angular_bias_ = {};
    linear_acceleration_bias_ = {};
    last_truth_time_ = 0.0;
    has_last_truth_time_ = false;
}

std::optional<ImuSample> ImuSensor::sample(const ImuTruth &truth, const SensorTick &tick) {
    if (tick.dropped)
        return std::nullopt;

    double delta_time = 0.0;
    if (has_last_truth_time_ && truth.time > last_truth_time_)
        delta_time = truth.time - last_truth_time_;
    last_truth_time_ = truth.time;
    has_last_truth_time_ = true;

    if (delta_time > 0.0) {
        angular_bias_ += gaussian_vec3(random(), noise_.angular_velocity_bias_random_walk) *
                         std::sqrt(delta_time);
        linear_acceleration_bias_ +=
            gaussian_vec3(random(), noise_.linear_acceleration_bias_random_walk) *
            std::sqrt(delta_time);
    }

    const auto true_angular_velocity = rotate_by_inverse(truth.pose.orientation,
                                                          truth.angular_velocity);
    const auto specific_force = rotate_by_inverse(
        truth.pose.orientation, truth.linear_acceleration - truth.gravity);

    auto angular_velocity = true_angular_velocity + angular_bias_ +
                            gaussian_vec3(random(), noise_.angular_velocity_stddev);
    auto linear_acceleration = specific_force + linear_acceleration_bias_ +
                               gaussian_vec3(random(), noise_.linear_acceleration_stddev);

    angular_velocity = componentwise_quantize(angular_velocity,
                                               noise_.angular_velocity_quantization);
    linear_acceleration = componentwise_quantize(
        linear_acceleration, noise_.linear_acceleration_quantization);
    angular_velocity = componentwise_clamp(angular_velocity, noise_.angular_velocity_min,
                                            noise_.angular_velocity_max);
    linear_acceleration = componentwise_clamp(
        linear_acceleration, noise_.linear_acceleration_min,
        noise_.linear_acceleration_max);

    ImuSample sample;
    sample.header = tick.header;
    sample.angular_velocity = angular_velocity;
    sample.linear_acceleration = linear_acceleration;
    sample.angular_velocity_covariance = Matrix3::diagonal({
        noise_.angular_velocity_stddev.x * noise_.angular_velocity_stddev.x,
        noise_.angular_velocity_stddev.y * noise_.angular_velocity_stddev.y,
        noise_.angular_velocity_stddev.z * noise_.angular_velocity_stddev.z});
    sample.linear_acceleration_covariance = Matrix3::diagonal({
        noise_.linear_acceleration_stddev.x * noise_.linear_acceleration_stddev.x,
        noise_.linear_acceleration_stddev.y * noise_.linear_acceleration_stddev.y,
        noise_.linear_acceleration_stddev.z * noise_.linear_acceleration_stddev.z});
    return sample;
}

LidarSensor::LidarSensor(SensorConfig config, LidarConfig lidar, LidarNoiseConfig noise)
    : Sensor(std::move(config)), lidar_(lidar), noise_(noise) {
    lidar_.range_min = std::max(0.0, lidar_.range_min);
    lidar_.range_max = std::max(lidar_.range_min, lidar_.range_max);
    noise_.range_stddev = std::max(0.0, noise_.range_stddev);
    noise_.range_quantization = std::max(0.0, noise_.range_quantization);
    noise_.ray_dropout_probability = clamp(noise_.ray_dropout_probability, 0.0, 1.0);

    const auto horizontal_step = lidar_.horizontal_count > 1
                                     ? (lidar_.horizontal_angle_max -
                                        lidar_.horizontal_angle_min) /
                                           static_cast<double>(lidar_.horizontal_count - 1)
                                     : 0.0;
    const auto vertical_step = lidar_.vertical_count > 1
                                   ? (lidar_.vertical_angle_max - lidar_.vertical_angle_min) /
                                         static_cast<double>(lidar_.vertical_count - 1)
                                   : 0.0;
    rays_.reserve(static_cast<std::size_t>(lidar_.horizontal_count) *
                  static_cast<std::size_t>(lidar_.vertical_count));
    for (std::uint32_t vertical = 0; vertical < lidar_.vertical_count; ++vertical) {
        const auto elevation = lidar_.vertical_angle_min + vertical * vertical_step;
        for (std::uint32_t horizontal = 0; horizontal < lidar_.horizontal_count;
             ++horizontal) {
            const auto azimuth = lidar_.horizontal_angle_min + horizontal * horizontal_step;
            rays_.push_back({
                {0.0, 0.0, 0.0},
                {std::cos(elevation) * std::cos(azimuth),
                 std::cos(elevation) * std::sin(azimuth), std::sin(elevation)},
                horizontal,
                vertical});
        }
    }
}

std::optional<LidarScan> LidarSensor::sample(const SensorTick &tick,
                                             std::span<const LidarHit> hits) {
    if (tick.dropped || hits.size() != rays_.size())
        return std::nullopt;

    LidarScan scan;
    scan.header = tick.header;
    scan.horizontal_count = lidar_.horizontal_count;
    scan.vertical_count = lidar_.vertical_count;
    scan.returns.resize(hits.size());

    for (std::size_t index = 0; index < hits.size(); ++index) {
        const auto &hit = hits[index];
        auto &value = scan.returns[index];
        if (!hit.hit || !std::isfinite(hit.range) ||
            uniform_zero_to_one() < noise_.ray_dropout_probability) {
            value.range = lidar_.range_max;
            continue;
        }

        auto range = hit.range + gaussian_scalar(random(), noise_.range_stddev);
        range = clamp(range, lidar_.range_min, lidar_.range_max);
        range = quantize(range, noise_.range_quantization);
        range = clamp(range, lidar_.range_min, lidar_.range_max);

        value.hit = true;
        value.range = range;
        value.point = rays_[index].origin + rays_[index].direction * range;
        value.normal = hit.normal;
        value.intensity = hit.intensity;
    }
    return scan;
}

CameraSensor::CameraSensor(SensorConfig config, CameraConfig camera)
    : Sensor(std::move(config)), camera_(camera) {
    camera_.width = std::max<std::uint32_t>(1, camera_.width);
    camera_.height = std::max<std::uint32_t>(1, camera_.height);
    if (!std::isfinite(camera_.fov_y))
        camera_.fov_y = 1.04719755f;
    camera_.fov_y = std::clamp(camera_.fov_y, 1.0e-4f, 3.1415925f - 1.0e-4f);
    if (!std::isfinite(camera_.near_plane))
        camera_.near_plane = 0.01f;
    camera_.near_plane = std::max(1.0e-5f, camera_.near_plane);
    if (!std::isfinite(camera_.far_plane))
        camera_.far_plane = 1000.0f;
    camera_.far_plane = std::max(camera_.near_plane + 1.0e-5f, camera_.far_plane);
    for (auto &component : camera_.clear_color) {
        if (!std::isfinite(component))
            component = 0.0f;
        component = std::clamp(component, 0.0f, 1.0f);
    }
}

std::optional<CameraFrame> CameraSensor::sample(const SensorTick &tick,
                                                std::span<const std::uint8_t> rgba8) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count = static_cast<std::size_t>(camera_.width) * camera_.height;
    if ((camera_.width != 0 &&
         pixel_count > std::numeric_limits<std::size_t>::max() / 4) ||
        rgba8.size() != pixel_count * 4)
        return std::nullopt;

    CameraFrame frame;
    frame.header = tick.header;
    frame.width = camera_.width;
    frame.height = camera_.height;
    frame.rgba8.assign(rgba8.begin(), rgba8.end());
    return frame;
}

DepthSensor::DepthSensor(SensorConfig config, DepthConfig depth)
    : Sensor(std::move(config)), depth_(depth) {
    depth_.width = std::max<std::uint32_t>(1, depth_.width);
    depth_.height = std::max<std::uint32_t>(1, depth_.height);
    if (!std::isfinite(depth_.fov_y))
        depth_.fov_y = 1.04719755f;
    depth_.fov_y = std::clamp(depth_.fov_y, 1.0e-4f, 3.1415925f - 1.0e-4f);
    if (!std::isfinite(depth_.near_plane))
        depth_.near_plane = 0.01f;
    depth_.near_plane = std::max(1.0e-5f, depth_.near_plane);
    if (!std::isfinite(depth_.far_plane))
        depth_.far_plane = 1000.0f;
    depth_.far_plane = std::max(depth_.near_plane + 1.0e-5f, depth_.far_plane);
}

std::optional<DepthFrame> DepthSensor::sample(const SensorTick &tick,
                                              std::span<const float> meters) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count = static_cast<std::size_t>(depth_.width) * depth_.height;
    if (meters.size() != pixel_count)
        return std::nullopt;
    DepthFrame frame;
    frame.header = tick.header;
    frame.width = depth_.width;
    frame.height = depth_.height;
    frame.meters.assign(meters.begin(), meters.end());
    return frame;
}

SegmentationSensor::SegmentationSensor(SensorConfig config, SegmentationConfig segmentation)
    : Sensor(std::move(config)), segmentation_(segmentation) {
    segmentation_.width = std::max<std::uint32_t>(1, segmentation_.width);
    segmentation_.height = std::max<std::uint32_t>(1, segmentation_.height);
    if (!std::isfinite(segmentation_.fov_y))
        segmentation_.fov_y = 1.04719755f;
    segmentation_.fov_y =
        std::clamp(segmentation_.fov_y, 1.0e-4f, 3.1415925f - 1.0e-4f);
    if (!std::isfinite(segmentation_.near_plane))
        segmentation_.near_plane = 0.01f;
    segmentation_.near_plane = std::max(1.0e-5f, segmentation_.near_plane);
    if (!std::isfinite(segmentation_.far_plane))
        segmentation_.far_plane = 1000.0f;
    segmentation_.far_plane =
        std::max(segmentation_.near_plane + 1.0e-5f, segmentation_.far_plane);
}

std::optional<SegmentationFrame> SegmentationSensor::sample(
    const SensorTick &tick, std::span<const std::uint64_t> labels) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count =
        static_cast<std::size_t>(segmentation_.width) * segmentation_.height;
    if (labels.size() != pixel_count)
        return std::nullopt;
    SegmentationFrame frame;
    frame.header = tick.header;
    frame.width = segmentation_.width;
    frame.height = segmentation_.height;
    frame.labels.assign(labels.begin(), labels.end());
    return frame;
}

} // namespace nksensor
