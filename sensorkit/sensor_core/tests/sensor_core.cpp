#include "nativekit_sensor_core.hpp"

#include <cassert>
#include <cmath>
#include <memory>

namespace {

using namespace nksensor;

bool close(double lhs, double rhs, double epsilon = 1e-12) {
    return std::abs(lhs - rhs) <= epsilon;
}

SensorConfig periodic_config(SensorId id) {
    SensorConfig config;
    config.id = id;
    config.frame = 42;
    config.timing.update_rate_hz = 10.0;
    config.timing.phase_offset = 0.02;
    config.timing.capture_delay = 0.001;
    config.timing.latency = 0.01;
    config.seed = 1234;
    return config;
}

void periodic_scheduling_keeps_capture_and_delivery_times() {
    Sensor sensor(periodic_config(1));
    const auto ticks = sensor.poll(0.25);
    assert(ticks.size() == 3);
    assert(ticks[0].header.sequence == 0);
    assert(close(ticks[0].header.capture_time, 0.02));
    assert(close(ticks[0].header.delivery_time, 0.031));
    assert(close(ticks[0].integration_start_time, 0.02));
    assert(ticks[2].header.sequence == 2);
    assert(close(sensor.next_update_time(), 0.32));

    const auto later = sensor.poll(0.32);
    assert(later.size() == 1);
    assert(later[0].header.sequence == 3);
}

void dropout_and_manual_trigger_are_deterministic() {
    auto config = periodic_config(2);
    config.timing.dropout_probability = 1.0;
    config.timing.update_rate_hz = 0.0;
    Sensor sensor(config);
    const auto tick = sensor.trigger(4.0);
    assert(tick.has_value());
    assert(tick->dropped);
    assert(tick->header.sequence == 0);
    assert(!sensor.trigger(std::numeric_limits<double>::quiet_NaN()).has_value());
}

void imu_converts_specific_force_and_applies_postprocessing() {
    SensorConfig config;
    config.id = 3;
    config.frame = 7;
    config.timing.update_rate_hz = 100.0;
    config.seed = 77;

    ImuNoiseConfig noise;
    noise.linear_acceleration_quantization = {0.5, 0.5, 0.5};
    noise.linear_acceleration_max = {10.0, 10.0, 10.0};

    ImuSensor imu(config, noise);
    const auto ticks = imu.poll(0.0);
    assert(ticks.size() == 1);

    ImuTruth truth;
    truth.time = 0.0;
    truth.gravity = {0.0, 0.0, -9.81};
    truth.linear_acceleration = {0.0, 0.0, 0.0};
    const auto sample = imu.sample(truth, ticks.front());
    assert(sample.has_value());
    assert(close(sample->linear_acceleration.x, 0.0));
    assert(close(sample->linear_acceleration.y, 0.0));
    assert(close(sample->linear_acceleration.z, 10.0));
    assert(close(sample->linear_acceleration_covariance(2, 2), 0.0));
}

void identical_seeds_replay_identical_measurements() {
    SensorConfig first_config = periodic_config(10);
    SensorConfig second_config = periodic_config(10);
    ImuNoiseConfig noise;
    noise.angular_velocity_stddev = {0.1, 0.2, 0.3};
    noise.linear_acceleration_stddev = {0.4, 0.5, 0.6};

    ImuSensor first(first_config, noise);
    ImuSensor second(second_config, noise);
    const auto first_tick = first.poll(0.02).front();
    const auto second_tick = second.poll(0.02).front();
    ImuTruth truth;
    truth.time = 0.02;
    truth.gravity = {0.0, 0.0, -9.81};
    truth.linear_acceleration = {1.0, 2.0, 3.0};
    truth.angular_velocity = {0.1, 0.2, 0.3};

    const auto first_sample = first.sample(truth, first_tick);
    const auto second_sample = second.sample(truth, second_tick);
    assert(first_sample.has_value() && second_sample.has_value());
    assert(first_sample->angular_velocity == second_sample->angular_velocity);
    assert(first_sample->linear_acceleration == second_sample->linear_acceleration);
}

void runtime_dispatches_measurements_in_capture_order() {
    SensorConfig late_config;
    late_config.id = 23;
    late_config.timing.update_rate_hz = 10.0;
    late_config.timing.phase_offset = 0.05;
    auto late = std::make_shared<Sensor>(late_config);

    SensorConfig early_config;
    early_config.id = 22;
    early_config.timing.update_rate_hz = 10.0;
    early_config.timing.phase_offset = 0.0;
    auto early = std::make_shared<Sensor>(early_config);

    const auto producer = [](const SensorTick &tick) -> std::optional<SensorMeasurement> {
        ImuSample sample;
        sample.header = tick.header;
        return SensorMeasurement{sample};
    };

    SensorRuntime runtime;
    assert(runtime.add(late, producer));
    assert(runtime.add(early, producer));
    assert(!runtime.add(early, producer));
    assert(runtime.find(late->id()) == late);
    assert(runtime.size() == 2);

    const auto measurements = runtime.poll(0.1);
    assert(measurements.size() == 3);
    assert(std::get<ImuSample>(measurements[0]).header.sensor == 22);
    assert(std::get<ImuSample>(measurements[1]).header.sensor == 23);
    assert(std::get<ImuSample>(measurements[2]).header.sensor == 22);
    assert(close(std::get<ImuSample>(measurements[0]).header.capture_time, 0.0));
    assert(close(std::get<ImuSample>(measurements[1]).header.capture_time, 0.05));
    assert(close(std::get<ImuSample>(measurements[2]).header.capture_time, 0.1));

    runtime.reset(2.0);
    const auto replay = runtime.poll(2.0);
    assert(replay.size() == 2);
    assert(close(std::get<ImuSample>(replay[0]).header.capture_time, 2.0));
    assert(close(std::get<ImuSample>(replay[1]).header.capture_time, 2.0));
    assert(runtime.remove(22));
    assert(runtime.size() == 1);
    assert(!runtime.remove(22));
    assert(!runtime.find(22));
}

void lidar_generates_rays_and_models_returns() {
    SensorConfig sensor_config;
    sensor_config.id = 30;
    sensor_config.timing.update_rate_hz = 0.0;

    LidarConfig lidar_config;
    lidar_config.horizontal_count = 3;
    lidar_config.vertical_count = 1;
    lidar_config.horizontal_angle_min = 0.0;
    lidar_config.horizontal_angle_max = 1.5707963267948966;
    lidar_config.range_max = 20.0;

    LidarNoiseConfig noise;
    noise.range_quantization = 0.5;
    LidarSensor lidar(sensor_config, lidar_config, noise);
    assert(lidar.rays().size() == 3);
    assert(close(lidar.rays()[0].direction.x, 1.0));
    assert(close(lidar.rays()[1].direction.x, std::sqrt(0.5)));
    assert(close(lidar.rays()[1].direction.y, std::sqrt(0.5)));
    assert(close(lidar.rays()[2].direction.y, 1.0));

    const auto tick = lidar.trigger(2.0);
    assert(tick.has_value());
    std::vector<LidarHit> hits(3);
    hits[1].hit = true;
    hits[1].range = 4.2;
    hits[1].intensity = 0.75;
    std::optional<LidarScan> scan = lidar.sample(*tick, hits);
    assert(scan.has_value());
    assert(scan->returns.size() == 3);
    assert(!scan->returns[0].hit);
    assert(close(scan->returns[0].range, 20.0));
    assert(scan->returns[1].hit);
    assert(close(scan->returns[1].range, 4.0));
    /* Range is distance along the unit ray, not the local-X projection. */
    assert(close(scan->returns[1].point.x, 4.0 * std::sqrt(0.5)));
    assert(close(scan->returns[1].point.y, 4.0 * std::sqrt(0.5)));
    assert(close(scan->returns[1].intensity, 0.75));
}

void camera_packages_backend_pixels() {
    SensorConfig sensor_config;
    sensor_config.id = 40;
    sensor_config.frame = 9;

    CameraConfig camera_config;
    camera_config.projection.width = 2;
    camera_config.projection.height = 1;
    CameraSensor camera(sensor_config, camera_config);
    const auto tick = camera.trigger(3.0);
    assert(tick.has_value());

    const std::uint8_t pixels[] = {255, 0, 0, 255, 0, 255, 0, 255};
    const auto frame = camera.sample(*tick, pixels);
    assert(frame.has_value());
    assert(frame->header.capture_time == 3.0);
    assert(frame->header.frame == 9);
    assert(frame->width == 2 && frame->height == 1);
    assert(frame->rgba8.size() == sizeof(pixels));
    assert(frame->pixel(1, 0)[0] == 0);
    assert(frame->pixel(1, 0)[1] == 255);
    assert(frame->pixel(2, 0) == nullptr);
}

void camera_cpu_post_process_is_deterministic() {
    SensorConfig sensor_config;
    sensor_config.id = 43;
    sensor_config.seed = 123;

    CameraConfig camera_config;
    camera_config.projection.width = 2;
    camera_config.projection.height = 1;
    camera_config.post_process.gain = 0.5f;
    CameraSensor camera(sensor_config, camera_config);
    const auto tick = camera.trigger(3.0);
    assert(tick.has_value());

    const std::vector<std::uint8_t> source{255, 100, 0, 255, 0, 255, 64, 255};
    auto pixels = source;
    camera.apply_post_process_cpu(pixels, tick->header.sequence);
    assert(pixels[0] == 128);
    assert(pixels[1] == 50);
    assert(pixels[2] == 0);
    assert(pixels[4] == 0);
    assert(pixels[5] == 128);
    assert(pixels[6] == 32);
    assert(pixels[3] == 255 && pixels[7] == 255);

    auto replay = source;
    camera.apply_post_process_cpu(replay, tick->header.sequence);
    assert(pixels == replay);

    CameraConfig dropout_config = camera_config;
    dropout_config.post_process.gain = 1.0f;
    dropout_config.post_process.dropout_probability = 1.0f;
    CameraSensor dropout_camera(sensor_config, dropout_config);
    auto dropout_pixels = source;
    dropout_camera.apply_post_process_cpu(dropout_pixels, tick->header.sequence);
    for (std::size_t index = 0; index < dropout_pixels.size(); index += 4) {
        assert(dropout_pixels[index + 0] == 0);
        assert(dropout_pixels[index + 1] == 0);
        assert(dropout_pixels[index + 2] == 0);
        assert(dropout_pixels[index + 3] == 255);
    }

    CameraConfig distortion_config = camera_config;
    distortion_config.post_process.gain = 1.0f;
    distortion_config.post_process.distortion_k1 = 100.0f;
    distortion_config.projection.width = 2;
    distortion_config.projection.height = 2;
    CameraSensor distortion_camera(sensor_config, distortion_config);
    auto distortion_pixels = std::vector<std::uint8_t>(16, 127);
    distortion_camera.apply_post_process_cpu(distortion_pixels, tick->header.sequence);
    for (std::size_t index = 0; index < distortion_pixels.size(); index += 4) {
        assert(distortion_pixels[index + 0] == 0);
        assert(distortion_pixels[index + 1] == 0);
        assert(distortion_pixels[index + 2] == 0);
        assert(distortion_pixels[index + 3] == 255);
    }
}

void depth_packages_metric_pixels() {
    SensorConfig sensor_config;
    sensor_config.id = 41;
    DepthConfig depth_config;
    depth_config.projection.width = 2;
    depth_config.projection.height = 1;
    DepthSensor depth(sensor_config, depth_config);
    const auto tick = depth.trigger(4.0);
    assert(tick.has_value());

    const float meters[] = {2.0f, 10.0f};
    const auto frame = depth.sample(*tick, meters);
    assert(frame.has_value());
    assert(frame->header.capture_time == 4.0);
    assert(frame->meters.size() == 2);
    assert(*frame->pixel(0, 0) == 2.0f);
    assert(frame->pixel(2, 0) == nullptr);
}

void segmentation_packages_labels() {
    SensorConfig sensor_config;
    sensor_config.id = 42;
    SegmentationConfig segmentation_config;
    segmentation_config.projection.width = 2;
    segmentation_config.projection.height = 1;
    segmentation_config.background_label = 99;
    SegmentationSensor segmentation(sensor_config, segmentation_config);
    const auto tick = segmentation.trigger(5.0);
    assert(tick.has_value());

    const std::uint64_t labels[] = {99, 1234};
    const auto frame = segmentation.sample(*tick, labels);
    assert(frame.has_value());
    assert(frame->header.capture_time == 5.0);
    assert(*frame->pixel(0, 0) == 99);
    assert(*frame->pixel(1, 0) == 1234);
    assert(frame->pixel(2, 0) == nullptr);
}

} // namespace

int main() {
    periodic_scheduling_keeps_capture_and_delivery_times();
    dropout_and_manual_trigger_are_deterministic();
    imu_converts_specific_force_and_applies_postprocessing();
    identical_seeds_replay_identical_measurements();
    runtime_dispatches_measurements_in_capture_order();
    lidar_generates_rays_and_models_returns();
    camera_packages_backend_pixels();
    camera_cpu_post_process_is_deterministic();
    depth_packages_metric_pixels();
    segmentation_packages_labels();
    return 0;
}
