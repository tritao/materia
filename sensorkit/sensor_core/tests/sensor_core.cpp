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

void manager_preserves_sensor_insertion_order() {
    auto first = std::make_shared<Sensor>(periodic_config(20));
    auto second = std::make_shared<Sensor>(periodic_config(21));
    SensorManager manager;
    assert(manager.add(first));
    assert(manager.add(second));
    assert(!manager.add(first));
    const auto ticks = manager.poll(0.02);
    assert(ticks.size() == 2);
    assert(ticks[0].header.sensor == 20);
    assert(ticks[1].header.sensor == 21);
    assert(manager.remove(20));
    assert(!manager.remove(20));
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
    camera_config.width = 2;
    camera_config.height = 1;
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

} // namespace

int main() {
    periodic_scheduling_keeps_capture_and_delivery_times();
    dropout_and_manual_trigger_are_deterministic();
    imu_converts_specific_force_and_applies_postprocessing();
    identical_seeds_replay_identical_measurements();
    manager_preserves_sensor_insertion_order();
    lidar_generates_rays_and_models_returns();
    camera_packages_backend_pixels();
    return 0;
}
