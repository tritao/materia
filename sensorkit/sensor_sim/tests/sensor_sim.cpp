#include "nativekit_sensor_sim.hpp"

#include "nativekit_scene.h"

#include <cassert>
#include <cmath>

namespace {

using namespace nksensor;
using namespace nksensor::sim;

nkscene_occurrence_id make_occurrence(nkscene_scene scene, double z) {
    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);

    nkscene_occurrence_id occurrence{};
    assert(nkscene_tx_create_occurrence(transaction, &occurrence) == NKS_OK);
    nkscene_transform transform{};
    transform.matrix[0] = 1.0f;
    transform.matrix[5] = 1.0f;
    transform.matrix[10] = 1.0f;
    transform.matrix[15] = 1.0f;
    transform.matrix[14] = static_cast<float>(z);
    assert(nkscene_tx_set_transform(transaction, occurrence, &transform) == NKS_OK);

    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);
    return occurrence;
}

struct SimFixture {
    nkscene_scene scene = 0;
    nksim_world world = 0;
    nksim_body body = 0;

    SimFixture() = default;
    SimFixture(const SimFixture &) = delete;
    SimFixture &operator=(const SimFixture &) = delete;
    SimFixture(SimFixture &&other) noexcept
        : scene(other.scene), world(other.world), body(other.body) {
        other.scene = 0;
        other.world = 0;
        other.body = 0;
    }
    SimFixture &operator=(SimFixture &&other) noexcept {
        if (this == &other)
            return *this;
        if (body)
            nksim_body_destroy(world, body);
        if (world)
            nksim_world_destroy(world);
        if (scene)
            nkscene_scene_destroy(scene);
        scene = other.scene;
        world = other.world;
        body = other.body;
        other.scene = 0;
        other.world = 0;
        other.body = 0;
        return *this;
    }

    ~SimFixture() {
        if (body)
            nksim_body_destroy(world, body);
        if (world)
            nksim_world_destroy(world);
        if (scene)
            nkscene_scene_destroy(scene);
    }
};

SimFixture make_fixture() {
    SimFixture fixture;
    assert(nkscene_scene_create(&fixture.scene) == NKS_OK);
    const auto occurrence = make_occurrence(fixture.scene, 10.0);

    nksim_world_desc world_desc{};
    world_desc.struct_size = sizeof(world_desc);
    world_desc.scene = fixture.scene;
    world_desc.fixed_timestep = 0.01;
    world_desc.physics_substeps = 1;
    world_desc.gravity[2] = -9.81;
    assert(nksim_world_create(&world_desc, &fixture.world) == NKSIM_OK);

    nksim_body_desc body_desc{};
    body_desc.struct_size = sizeof(body_desc);
    body_desc.occurrence = occurrence;
    body_desc.motion_type = NKSIM_MOTION_DYNAMIC;
    body_desc.mass = 1.0;
    assert(nksim_body_create(fixture.world, &body_desc, &fixture.body) == NKSIM_OK);
    return fixture;
}

nksim_snapshot snapshot(nksim_world world) {
    nksim_snapshot result = 0;
    assert(nksim_world_snapshot(world, &result) == NKSIM_OK);
    return result;
}

void reads_immutable_snapshot_and_estimates_acceleration() {
    auto fixture = make_fixture();
    ImuTruthAdapter adapter(fixture.body, {0.0, 0.0, -9.81});

    const auto initial_snapshot = snapshot(fixture.world);
    const auto initial = adapter.read(initial_snapshot);
    assert(initial.has_value());
    assert(initial->time == 0.0);
    assert(initial->linear_acceleration == Vec3{});
    nksim_snapshot_destroy(initial_snapshot);

    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(fixture.world, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);

    const auto next_snapshot = snapshot(fixture.world);
    const auto next = adapter.read(next_snapshot);
    assert(next.has_value());
    assert(std::abs(next->time - 0.01) < 1e-12);
    assert(std::abs(next->linear_acceleration.z + 9.81) < 1e-10);
    assert((next->gravity == Vec3{0.0, 0.0, -9.81}));
    nksim_snapshot_destroy(next_snapshot);
}

void invalid_and_nonmatching_snapshots_do_not_mutate_history() {
    auto fixture = make_fixture();
    ImuTruthAdapter adapter(fixture.body + 1, {0.0, 0.0, -9.81});
    assert(!adapter.read(NKSIM_INVALID_SNAPSHOT).has_value());
    assert(adapter.last_result() == NKSIM_ERROR_INVALID_HANDLE);

    const auto current = snapshot(fixture.world);
    assert(!adapter.read(current).has_value());
    assert(adapter.last_result() == NKSIM_ERROR_STALE_ID);
    nksim_snapshot_destroy(current);

    adapter.reset();
    const auto reset_snapshot = snapshot(fixture.world);
    assert(!adapter.read(reset_snapshot).has_value());
    assert(adapter.last_result() == NKSIM_ERROR_STALE_ID);
    nksim_snapshot_destroy(reset_snapshot);
}

void reset_discards_derivative_history() {
    auto fixture = make_fixture();
    ImuTruthAdapter adapter(fixture.body, {0.0, 0.0, -9.81});
    const auto first_snapshot = snapshot(fixture.world);
    assert(adapter.read(first_snapshot).has_value());
    nksim_snapshot_destroy(first_snapshot);

    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(fixture.world, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);
    adapter.reset();

    const auto reset_snapshot = snapshot(fixture.world);
    const auto truth = adapter.read(reset_snapshot);
    assert(truth.has_value());
    assert(truth->linear_acceleration == Vec3{});
    nksim_snapshot_destroy(reset_snapshot);
}

void scene_lidar_adapter_batch_raycast_returns_local_scan() {
    nkscene_scene scene = 0;
    assert(nkscene_scene_create(&scene) == NKS_OK);

    nkscene_geometry_id geometry = {0};
    nkscene_material_id material = {0};
    assert(nkscene_geometry_create(scene, &geometry) == NKS_OK);
    assert(nkscene_material_create(scene, &material) == NKS_OK);

    const nkscene_geometry_vertex vertices[] = {
        {{5.0f, -10.0f, -10.0f}},
        {{5.0f, 10.0f, -10.0f}},
        {{5.0f, 10.0f, 10.0f}},
        {{5.0f, -10.0f, -10.0f}},
        {{5.0f, 10.0f, 10.0f}},
        {{5.0f, -10.0f, 10.0f}},
    };
    nkscene_geometry_data geometry_data{};
    geometry_data.struct_size = sizeof(geometry_data);
    geometry_data.vertices = vertices;
    geometry_data.vertex_count = 6;
    geometry_data.bounds.valid = 1;
    geometry_data.bounds.minimum[0] = 5.0f;
    geometry_data.bounds.minimum[1] = -10.0f;
    geometry_data.bounds.minimum[2] = -10.0f;
    geometry_data.bounds.maximum[0] = 5.0f;
    geometry_data.bounds.maximum[1] = 10.0f;
    geometry_data.bounds.maximum[2] = 10.0f;
    assert(nkscene_geometry_set_data(scene, geometry, &geometry_data) == NKS_OK);

    nkscene_material_data material_data{};
    material_data.struct_size = sizeof(material_data);
    material_data.base_color[3] = 1.0f;
    material_data.opacity = 1.0f;
    material_data.flags = NKS_MATERIAL_OPAQUE;
    assert(nkscene_material_set_data(scene, material, &material_data) == NKS_OK);

    nkscene_transaction transaction = 0;
    assert(nkscene_transaction_begin(scene, &transaction) == NKS_OK);
    nkscene_occurrence_id occurrence{};
    assert(nkscene_tx_create_occurrence(transaction, &occurrence) == NKS_OK);
    assert(nkscene_tx_set_geometry(transaction, occurrence, geometry) == NKS_OK);
    assert(nkscene_tx_set_material(transaction, occurrence, material) == NKS_OK);
    nkscene_change_set changes = 0;
    assert(nkscene_transaction_commit_with_changes(transaction, &changes) == NKS_OK);
    nkscene_change_set_destroy(changes);

    nkscene_snapshot snapshot_handle = 0;
    assert(nkscene_scene_snapshot(scene, &snapshot_handle) == NKS_OK);
    SceneLidarAdapter adapter;
    assert(adapter.build(snapshot_handle) == NKS_OK);
    nkscene_snapshot_destroy(snapshot_handle);

    SensorConfig sensor_config;
    sensor_config.id = 31;
    LidarConfig lidar_config;
    lidar_config.horizontal_count = 3;
    lidar_config.vertical_count = 1;
    lidar_config.horizontal_angle_min = 0.0;
    lidar_config.horizontal_angle_max = 1.5707963267948966;
    lidar_config.vertical_angle_min = 0.0;
    lidar_config.vertical_angle_max = 0.0;
    lidar_config.range_max = 20.0;
    LidarSensor lidar(sensor_config, lidar_config);
    const auto tick = lidar.trigger(0.0);
    assert(tick.has_value());

    std::optional<LidarScan> scan;
    assert(adapter.scan(lidar, *tick, {}, scan) == NKS_OK);
    assert(scan.has_value());
    assert(scan->returns.size() == 3);
    assert(scan->returns[0].hit);
    assert(std::abs(scan->returns[0].range - 5.0) < 1e-6);
    assert(std::abs(scan->returns[0].point.x - 5.0) < 1e-6);
    assert(scan->returns[1].hit);
    /* The 45-degree return reaches x=5 after travelling sqrt(50), not 5.
       The reported point is nevertheless (5, 5, 0) in the sensor frame. */
    assert(std::abs(scan->returns[1].range - 5.0 * std::sqrt(2.0)) < 1e-6);
    assert(std::abs(scan->returns[1].point.x - 5.0) < 1e-6);
    assert(std::abs(scan->returns[1].point.y - 5.0) < 1e-6);
    assert(!scan->returns[2].hit);
    assert(std::abs(scan->returns[2].range - 20.0) < 1e-6);

    Pose translated_pose;
    translated_pose.position = {1.0, 0.0, 0.0};
    std::optional<LidarScan> translated_scan;
    assert(adapter.scan(lidar, *tick, translated_pose, translated_scan) == NKS_OK);
    assert(translated_scan.has_value());
    assert(translated_scan->returns[0].hit);
    assert(std::abs(translated_scan->returns[0].range - 4.0) < 1e-6);
    assert(std::abs(translated_scan->returns[0].point.x - 4.0) < 1e-6);

    /* A -90 degree sensor yaw maps local +Y to world +X. The same scene
       should therefore be hit by the second ray, with the point reported
       back in the sensor frame. */
    const auto half_sqrt_two = std::sqrt(0.5);
    Pose rotated_pose;
    rotated_pose.orientation = {0.0, 0.0, -half_sqrt_two, half_sqrt_two};
    std::optional<LidarScan> rotated_scan;
    assert(adapter.scan(lidar, *tick, rotated_pose, rotated_scan) == NKS_OK);
    assert(rotated_scan.has_value());
    assert(!rotated_scan->returns[0].hit);
    assert(rotated_scan->returns[1].hit);
    assert(std::abs(rotated_scan->returns[1].range - 5.0 * std::sqrt(2.0)) < 1e-6);
    assert(std::abs(rotated_scan->returns[1].point.x - 5.0) < 1e-6);
    assert(std::abs(rotated_scan->returns[1].point.y - 5.0) < 1e-6);
    assert(rotated_scan->returns[2].hit);
    assert(std::abs(rotated_scan->returns[2].range - 5.0) < 1e-6);
    assert(std::abs(rotated_scan->returns[2].point.x) < 1e-6);
    assert(std::abs(rotated_scan->returns[2].point.y - 5.0) < 1e-6);

    SensorConfig noisy_sensor_config = sensor_config;
    noisy_sensor_config.seed = 91;
    LidarNoiseConfig noise;
    noise.range_stddev = 0.1;
    LidarSensor first_noisy(noisy_sensor_config, lidar_config, noise);
    LidarSensor second_noisy(noisy_sensor_config, lidar_config, noise);
    const auto first_noisy_tick = first_noisy.trigger(0.0);
    const auto second_noisy_tick = second_noisy.trigger(0.0);
    assert(first_noisy_tick.has_value() && second_noisy_tick.has_value());
    std::optional<LidarScan> first_noisy_scan;
    std::optional<LidarScan> second_noisy_scan;
    assert(adapter.scan(first_noisy, *first_noisy_tick, {}, first_noisy_scan) == NKS_OK);
    assert(adapter.scan(second_noisy, *second_noisy_tick, {}, second_noisy_scan) == NKS_OK);
    assert(first_noisy_scan.has_value() && second_noisy_scan.has_value());
    for (std::size_t index = 0; index < first_noisy_scan->returns.size(); ++index)
        assert(first_noisy_scan->returns[index].range == second_noisy_scan->returns[index].range);

    SensorConfig dropped_sensor_config = sensor_config;
    dropped_sensor_config.timing.dropout_probability = 1.0;
    LidarSensor dropped_lidar(dropped_sensor_config, lidar_config);
    const auto dropped_tick = dropped_lidar.trigger(0.0);
    assert(dropped_tick.has_value() && dropped_tick->dropped);
    std::optional<LidarScan> dropped_scan;
    assert(adapter.scan(dropped_lidar, *dropped_tick, {}, dropped_scan) == NKS_OK);
    assert(!dropped_scan.has_value());

    adapter.reset();
    nkscene_geometry_destroy(scene, geometry);
    nkscene_material_destroy(scene, material);
    nkscene_scene_destroy(scene);
}

void runtime_dispatches_real_snapshot_adapters() {
    auto fixture = make_fixture();

    nkscene_snapshot scene_snapshot_handle = 0;
    assert(nkscene_scene_snapshot(fixture.scene, &scene_snapshot_handle) == NKS_OK);
    SceneLidarAdapter lidar_adapter;
    assert(lidar_adapter.build(scene_snapshot_handle) == NKS_OK);
    nkscene_snapshot_destroy(scene_snapshot_handle);

    ImuTruthAdapter imu_truth(fixture.body, {0.0, 0.0, -9.81});

    SensorConfig imu_config;
    imu_config.id = 60;
    imu_config.timing.update_rate_hz = 100.0;
    auto imu = std::make_shared<ImuSensor>(imu_config);

    SensorConfig lidar_config_sensor;
    lidar_config_sensor.id = 61;
    lidar_config_sensor.timing.update_rate_hz = 100.0;
    LidarConfig lidar_config;
    lidar_config.horizontal_count = 1;
    lidar_config.vertical_count = 1;
    lidar_config.horizontal_angle_min = 0.0;
    lidar_config.horizontal_angle_max = 0.0;
    lidar_config.range_max = 25.0;
    auto lidar = std::make_shared<LidarSensor>(lidar_config_sensor, lidar_config);

    nksim_snapshot current_snapshot = snapshot(fixture.world);
    SensorRuntime runtime;
    assert(runtime.add(imu, [&](const SensorTick &tick) -> std::optional<SensorMeasurement> {
        const auto truth = imu_truth.read(current_snapshot);
        if (!truth)
            return std::nullopt;
        const auto sample = imu->sample(*truth, tick);
        if (!sample)
            return std::nullopt;
        return SensorMeasurement{*sample};
    }));
    assert(runtime.add(lidar, [&](const SensorTick &tick) -> std::optional<SensorMeasurement> {
        std::optional<LidarScan> scan;
        if (lidar_adapter.scan(*lidar, tick, {}, scan) != NKS_OK || !scan)
            return std::nullopt;
        return SensorMeasurement{*scan};
    }));

    const auto first_batch = runtime.poll(0.0);
    assert(first_batch.size() == 2);
    assert(std::get<ImuSample>(first_batch[0]).header.sensor == 60);
    assert(std::get<LidarScan>(first_batch[1]).header.sensor == 61);
    assert(std::get<LidarScan>(first_batch[1]).returns.size() == 1);
    assert(!std::get<LidarScan>(first_batch[1]).returns[0].hit);

    nksim_step_result step{};
    step.struct_size = sizeof(step);
    assert(nksim_world_step(fixture.world, &step) == NKSIM_OK);
    nkscene_change_set_destroy(step.scene_changes);
    const auto next_snapshot = snapshot(fixture.world);
    nksim_snapshot_destroy(current_snapshot);
    current_snapshot = next_snapshot;

    const auto second_batch = runtime.poll(0.01);
    assert(second_batch.size() == 2);
    const auto &second_imu = std::get<ImuSample>(second_batch[0]);
    assert(second_imu.header.sequence == 1);
    /* The free-falling body follows gravity, so the IMU measures near-zero
       specific force after the truth adapter and sensor model are composed. */
    assert(std::abs(second_imu.linear_acceleration.z) < 1e-10);
    assert(std::get<LidarScan>(second_batch[1]).header.sequence == 1);

    nksim_snapshot_destroy(current_snapshot);
}

} // namespace

int main() {
    reads_immutable_snapshot_and_estimates_acceleration();
    invalid_and_nonmatching_snapshots_do_not_mutate_history();
    reset_discards_derivative_history();
    scene_lidar_adapter_batch_raycast_returns_local_scan();
    runtime_dispatches_real_snapshot_adapters();
    return 0;
}
