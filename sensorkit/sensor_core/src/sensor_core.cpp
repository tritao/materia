#include "nativekit_sensor_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

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

void sanitize(PinholeConfig &config) noexcept {
    config.width = std::max<std::uint32_t>(1, config.width);
    config.height = std::max<std::uint32_t>(1, config.height);
    if (!std::isfinite(config.fov_y))
        config.fov_y = 1.04719755f;
    config.fov_y = std::clamp(config.fov_y, 1.0e-4f, 3.1415925f - 1.0e-4f);
    if (!std::isfinite(config.near_plane))
        config.near_plane = 0.01f;
    config.near_plane = std::max(1.0e-5f, config.near_plane);
    if (!std::isfinite(config.far_plane))
        config.far_plane = 1000.0f;
    config.far_plane = std::max(config.near_plane + 1.0e-5f, config.far_plane);
}

template <typename Frame, typename Value>
Frame make_frame(const SensorTick &tick, const PinholeConfig &projection,
                 std::span<const Value> values, std::vector<Value> Frame::*payload) {
    Frame frame;
    frame.header = tick.header;
    frame.width = projection.width;
    frame.height = projection.height;
    (frame.*payload).assign(values.begin(), values.end());
    return frame;
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

std::uint32_t hash32(std::uint32_t value) noexcept {
    /* A small integer mixer gives every pixel/channel event an independent,
     * reproducible random value without keeping mutable RNG state per pixel. */
    value ^= value >> 16;
    value *= 0x7feb352dU;
    value ^= value >> 15;
    value *= 0x846ca68bU;
    return value ^ (value >> 16);
}

float pixel_random(std::uint64_t seed, std::uint64_t sequence, std::uint32_t x,
                   std::uint32_t y, std::uint32_t salt) noexcept {
    /* Fold the sensor seed, frame sequence, pixel coordinate, and operation
     * salt into one stream. The salt keeps dropout and noise independent. */
    auto value = static_cast<std::uint32_t>(seed) ^ static_cast<std::uint32_t>(seed >> 32);
    value ^= static_cast<std::uint32_t>(sequence);
    value ^= static_cast<std::uint32_t>(sequence >> 32);
    value ^= x * 0x9e3779b9U;
    value ^= y * 0x85ebca6bU;
    value ^= salt * 0xc2b2ae35U;
    return static_cast<float>(hash32(value) & 0x00ffffffU) / 16777216.0f;
}

float pixel_gaussian(std::uint64_t seed, std::uint64_t sequence, std::uint32_t x,
                     std::uint32_t y) noexcept {
    /* Box-Muller converts two uniform values into one normal deviate. */
    const auto first = std::max(1.0e-7f, pixel_random(seed, sequence, x, y, 0));
    const auto second = pixel_random(seed, sequence, x, y, 1);
    return std::sqrt(-2.0f * std::log(first)) * std::cos(6.283185307179586f * second);
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

bool SensorRuntime::add(const std::shared_ptr<Sensor> &sensor, Producer producer) {
    if (!sensor || sensor->id() == invalid_sensor || !producer || find(sensor->id()))
        return false;
    bindings_.push_back({sensor, std::move(producer)});
    return true;
}

bool SensorRuntime::remove(SensorId id) {
    const auto found = std::remove_if(bindings_.begin(), bindings_.end(),
                                      [id](const Binding &binding) {
                                          return binding.sensor && binding.sensor->id() == id;
                                      });
    if (found == bindings_.end())
        return false;
    bindings_.erase(found, bindings_.end());
    return true;
}

std::shared_ptr<Sensor> SensorRuntime::find(SensorId id) const {
    const auto found = std::find_if(bindings_.begin(), bindings_.end(),
                                    [id](const Binding &binding) {
                                        return binding.sensor && binding.sensor->id() == id;
                                    });
    return found == bindings_.end() ? nullptr : found->sensor;
}

std::vector<SensorMeasurement> SensorRuntime::poll(double simulation_time) {
    struct Due {
        SensorTick tick;
        Binding *binding = nullptr;
    };

    std::vector<Due> due;
    for (auto &binding : bindings_) {
        if (!binding.sensor)
            continue;
        for (auto &tick : binding.sensor->poll(simulation_time))
            due.push_back({std::move(tick), &binding});
    }
    std::stable_sort(due.begin(), due.end(), [](const Due &lhs, const Due &rhs) {
        if (lhs.tick.header.capture_time != rhs.tick.header.capture_time)
            return lhs.tick.header.capture_time < rhs.tick.header.capture_time;
        return lhs.tick.header.sensor < rhs.tick.header.sensor;
    });

    std::vector<SensorMeasurement> measurements;
    for (auto &item : due) {
        if (item.tick.dropped)
            continue;
        if (auto measurement = item.binding->producer(item.tick))
            measurements.push_back(std::move(*measurement));
    }
    return measurements;
}

void SensorRuntime::reset(double next_capture_time) noexcept {
    for (const auto &binding : bindings_)
        if (binding.sensor)
            binding.sensor->reset(next_capture_time);
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
    sanitize(camera_.projection);
    for (auto &component : camera_.clear_color) {
        if (!std::isfinite(component))
            component = 0.0f;
        component = std::clamp(component, 0.0f, 1.0f);
    }
    if (!std::isfinite(camera_.post_process.exposure_stops))
        camera_.post_process.exposure_stops = 0.0f;
    if (!std::isfinite(camera_.post_process.gain))
        camera_.post_process.gain = 1.0f;
    camera_.post_process.gain = std::max(0.0f, camera_.post_process.gain);
    if (!std::isfinite(camera_.post_process.noise_stddev))
        camera_.post_process.noise_stddev = 0.0f;
    camera_.post_process.noise_stddev = std::max(0.0f, camera_.post_process.noise_stddev);
    if (!std::isfinite(camera_.post_process.quantization))
        camera_.post_process.quantization = 0.0f;
    camera_.post_process.quantization = std::max(0.0f, camera_.post_process.quantization);
    if (!std::isfinite(camera_.post_process.distortion_k1))
        camera_.post_process.distortion_k1 = 0.0f;
    if (!std::isfinite(camera_.post_process.distortion_k2))
        camera_.post_process.distortion_k2 = 0.0f;
    if (!std::isfinite(camera_.post_process.dropout_probability))
        camera_.post_process.dropout_probability = 0.0f;
    camera_.post_process.dropout_probability =
        std::clamp(camera_.post_process.dropout_probability, 0.0f, 1.0f);
}

std::optional<CameraFrame> CameraSensor::sample(const SensorTick &tick,
                                                std::span<const std::uint8_t> rgba8) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count = static_cast<std::size_t>(camera_.projection.width) *
                             camera_.projection.height;
    if ((camera_.projection.width != 0 &&
         pixel_count > std::numeric_limits<std::size_t>::max() / 4) ||
        rgba8.size() != pixel_count * 4)
        return std::nullopt;

    return make_frame(tick, camera_.projection, rgba8, &CameraFrame::rgba8);
}

void CameraSensor::apply_post_process_cpu(std::span<std::uint8_t> rgba8,
                                           std::uint64_t sequence) const {
    const auto pixel_count = static_cast<std::size_t>(camera_.projection.width) *
                             camera_.projection.height;
    if (rgba8.size() != pixel_count * 4)
        return;

    const auto &post = camera_.post_process;
    if (!post.enabled())
        return;
    /* Read from an immutable copy so distortion can remap pixels without
     * feeding already-processed output back into later pixels. */
    const auto source = std::vector<std::uint8_t>(rgba8.begin(), rgba8.end());
    const auto scale = std::exp2(post.exposure_stops) * post.gain;
    for (std::uint32_t y = 0; y < camera_.projection.height; ++y) {
        for (std::uint32_t x = 0; x < camera_.projection.width; ++x) {
            auto source_x = x;
            auto source_y = y;
            /* Inverse-map each output pixel into the source image. This is a
             * deliberately simple nearest-neighbour reference implementation;
             * the GPU path uses the same mapping with filtered sampling. */
            if (post.distortion_k1 != 0.0f || post.distortion_k2 != 0.0f) {
                const auto u = (static_cast<float>(x) + 0.5f) / camera_.projection.width;
                const auto v = (static_cast<float>(y) + 0.5f) / camera_.projection.height;
                const auto centered_x = u * 2.0f - 1.0f;
                const auto centered_y = v * 2.0f - 1.0f;
                const auto radius_squared = centered_x * centered_x + centered_y * centered_y;
                const auto factor = 1.0f + post.distortion_k1 * radius_squared +
                                    post.distortion_k2 * radius_squared * radius_squared;
                const auto distorted_u = centered_x * factor * 0.5f + 0.5f;
                const auto distorted_v = centered_y * factor * 0.5f + 0.5f;
                if (distorted_u < 0.0f || distorted_u >= 1.0f || distorted_v < 0.0f ||
                    distorted_v >= 1.0f) {
                    source_x = camera_.projection.width;
                    source_y = camera_.projection.height;
                } else {
                    source_x = static_cast<std::uint32_t>(distorted_u * camera_.projection.width);
                    source_y = static_cast<std::uint32_t>(distorted_v * camera_.projection.height);
                }
            }

            const auto output_index =
                (static_cast<std::size_t>(y) * camera_.projection.width + x) *
                static_cast<std::size_t>(4);
            if (source_x >= camera_.projection.width || source_y >= camera_.projection.height ||
                pixel_random(config().seed, sequence, x, y, 2) < post.dropout_probability) {
                rgba8[output_index + 0] = 0;
                rgba8[output_index + 1] = 0;
                rgba8[output_index + 2] = 0;
                rgba8[output_index + 3] = 255;
                continue;
            }

            const auto source_index =
                (static_cast<std::size_t>(source_y) * camera_.projection.width + source_x) *
                static_cast<std::size_t>(4);
            for (std::size_t channel = 0; channel < 3; ++channel) {
                /* Work in normalized linear values, apply exposure/gain,
                 * additive Gaussian noise, optional quantization, then encode
                 * back to the camera's 8-bit representation. */
                auto value = static_cast<float>(source[source_index + channel]) / 255.0f;
                value = value * scale + pixel_gaussian(config().seed, sequence, x, y) *
                                              post.noise_stddev;
                if (post.quantization > 0.0f)
                    value = std::round(value / post.quantization) * post.quantization;
                rgba8[output_index + channel] = static_cast<std::uint8_t>(std::lround(
                    std::clamp(value, 0.0f, 1.0f) * 255.0f));
            }
            rgba8[output_index + 3] = source[source_index + 3];
        }
    }
}

DepthSensor::DepthSensor(SensorConfig config, DepthConfig depth)
    : Sensor(std::move(config)), depth_(depth) {
    sanitize(depth_.projection);
}

std::optional<DepthFrame> DepthSensor::sample(const SensorTick &tick,
                                              std::span<const float> meters) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count = static_cast<std::size_t>(depth_.projection.width) *
                             depth_.projection.height;
    if (meters.size() != pixel_count)
        return std::nullopt;
    return make_frame(tick, depth_.projection, meters, &DepthFrame::meters);
}

SegmentationSensor::SegmentationSensor(SensorConfig config, SegmentationConfig segmentation)
    : Sensor(std::move(config)), segmentation_(segmentation) {
    sanitize(segmentation_.projection);
}

std::optional<SegmentationFrame> SegmentationSensor::sample(
    const SensorTick &tick, std::span<const std::uint64_t> labels) {
    if (tick.dropped)
        return std::nullopt;

    const auto pixel_count =
        static_cast<std::size_t>(segmentation_.projection.width) *
        segmentation_.projection.height;
    if (labels.size() != pixel_count)
        return std::nullopt;
    return make_frame(tick, segmentation_.projection, labels, &SegmentationFrame::labels);
}

} // namespace nksensor
