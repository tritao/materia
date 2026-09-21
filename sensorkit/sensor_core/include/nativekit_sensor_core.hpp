#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <vector>

#if defined(_WIN32)
#if defined(NKSENSOR_STATIC)
#define NKSENSOR_API
#elif defined(NKSENSOR_BUILDING_LIBRARY)
#define NKSENSOR_API __declspec(dllexport)
#else
#define NKSENSOR_API __declspec(dllimport)
#endif
#else
#define NKSENSOR_API __attribute__((visibility("default")))
#endif

namespace nksensor {

using SensorId = std::uint64_t;
using FrameId = std::uint64_t;

constexpr SensorId invalid_sensor = 0;
constexpr FrameId invalid_frame = 0;

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vec3 &operator+=(Vec3 other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Vec3 &operator-=(Vec3 other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vec3 &operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
};

constexpr Vec3 operator+(Vec3 lhs, Vec3 rhs) noexcept {
    lhs += rhs;
    return lhs;
}

constexpr Vec3 operator-(Vec3 lhs, Vec3 rhs) noexcept {
    lhs -= rhs;
    return lhs;
}

constexpr Vec3 operator-(Vec3 value) noexcept {
    return {-value.x, -value.y, -value.z};
}

constexpr Vec3 operator*(Vec3 value, double scalar) noexcept {
    value *= scalar;
    return value;
}

constexpr Vec3 operator*(double scalar, Vec3 value) noexcept {
    return value * scalar;
}

constexpr bool operator==(Vec3 lhs, Vec3 rhs) noexcept {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

struct Quaternion {
    /* Quaternion components use x, y, z, w order. */
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Pose {
    Vec3 position;
    Quaternion orientation;
};

struct Matrix3 {
    /* Row-major storage. */
    std::array<double, 9> values{
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0};

    static constexpr Matrix3 diagonal(Vec3 diagonal_values) noexcept {
        Matrix3 result;
        result.values[0] = diagonal_values.x;
        result.values[4] = diagonal_values.y;
        result.values[8] = diagonal_values.z;
        return result;
    }

    constexpr double operator()(std::size_t row, std::size_t column) const noexcept {
        return values[row * 3 + column];
    }
};

struct SensorSampleHeader {
    SensorId sensor = invalid_sensor;
    std::uint64_t sequence = 0;
    double capture_time = 0.0;
    double delivery_time = 0.0;
    FrameId frame = invalid_frame;
};

/** Truth for one sensor frame at one capture time.
 *
 * Linear acceleration is the kinematic acceleration in world coordinates;
 * gravity is supplied separately so an IMU can produce specific force.
 */
struct ImuTruth {
    double time = 0.0;
    Pose pose;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    Vec3 linear_acceleration;
    Vec3 gravity;
};

struct ImuSample {
    SensorSampleHeader header;
    Vec3 angular_velocity;
    Vec3 linear_acceleration;
    Matrix3 angular_velocity_covariance;
    Matrix3 linear_acceleration_covariance;
};

struct LidarRay {
    Vec3 origin;
    Vec3 direction;
    std::uint32_t horizontal_index = 0;
    std::uint32_t vertical_index = 0;
};

/** A geometric hit in the sensor frame, before LiDAR post-processing. */
struct LidarHit {
    bool hit = false;
    double range = 0.0;
    Vec3 point;
    Vec3 normal;
    double intensity = 0.0;
};

struct LidarReturn {
    bool hit = false;
    double range = 0.0;
    Vec3 point;
    Vec3 normal;
    double intensity = 0.0;
};

struct LidarScan {
    SensorSampleHeader header;
    std::uint32_t horizontal_count = 0;
    std::uint32_t vertical_count = 0;
    std::vector<LidarReturn> returns;

    std::size_t index(std::uint32_t horizontal, std::uint32_t vertical) const noexcept {
        return static_cast<std::size_t>(vertical) * horizontal_count + horizontal;
    }
};

struct SensorTiming {
    /** Zero or a negative rate selects manual triggering. */
    double update_rate_hz = 0.0;
    double phase_offset = 0.0;
    double capture_delay = 0.0;
    double integration_period = 0.0;
    double latency = 0.0;
    double latency_jitter = 0.0;
    double dropout_probability = 0.0;
};

struct SensorConfig {
    SensorId id = invalid_sensor;
    FrameId frame = invalid_frame;
    bool enabled = true;
    SensorTiming timing;
    std::uint64_t seed = 0;
};

struct SensorTick {
    SensorSampleHeader header;
    double integration_start_time = 0.0;
    bool dropped = false;
};

/** A deterministic Gaussian scalar source useful for custom sensor models. */
class NKSENSOR_API GaussianNoise {
public:
    explicit GaussianNoise(double standard_deviation = 0.0, std::uint64_t seed = 0);

    double sample();
    void reseed(std::uint64_t seed);
    double standard_deviation() const noexcept { return standard_deviation_; }
    void set_standard_deviation(double value) noexcept { standard_deviation_ = value; }

private:
    double standard_deviation_ = 0.0;
    std::mt19937_64 random_;
    std::normal_distribution<double> normal_;
};

/** Backend-independent scheduling and delivery timing for one sensor. */
class NKSENSOR_API Sensor {
public:
    explicit Sensor(SensorConfig config);
    virtual ~Sensor() = default;

    Sensor(const Sensor &) = delete;
    Sensor &operator=(const Sensor &) = delete;
    Sensor(Sensor &&) = delete;
    Sensor &operator=(Sensor &&) = delete;

    SensorId id() const noexcept { return config_.id; }
    FrameId frame() const noexcept { return config_.frame; }
    bool enabled() const noexcept { return config_.enabled; }
    void set_enabled(bool value) noexcept { config_.enabled = value; }

    const SensorConfig &config() const noexcept { return config_; }
    const SensorTiming &timing() const noexcept { return config_.timing; }
    bool periodic() const noexcept { return period_ > 0.0; }
    double update_period() const noexcept { return period_; }
    double next_update_time() const noexcept { return next_capture_time_; }

    /** Return every periodic tick due through simulation_time, in order. */
    std::vector<SensorTick> poll(double simulation_time);

    /** Produce one tick regardless of the periodic schedule. */
    std::optional<SensorTick> trigger(double capture_time);

    /** Restart periodic scheduling at a known simulation time. */
    void reset(double next_capture_time = 0.0) noexcept;

protected:
    std::mt19937_64 &random() noexcept { return random_; }
    double uniform_zero_to_one();
    SensorTick make_tick(double capture_time);

private:
    SensorConfig config_;
    double period_ = 0.0;
    double next_capture_time_ = 0.0;
    std::uint64_t sequence_ = 0;
    std::mt19937_64 random_;
};

/** A stable insertion-ordered collection of sensors. */
class NKSENSOR_API SensorManager {
public:
    bool add(const std::shared_ptr<Sensor> &sensor);
    bool remove(SensorId id);
    std::shared_ptr<Sensor> find(SensorId id) const;
    std::vector<SensorTick> poll(double simulation_time);
    std::size_t size() const noexcept { return sensors_.size(); }

private:
    std::vector<std::shared_ptr<Sensor>> sensors_;
};

struct ImuNoiseConfig {
    Vec3 angular_velocity_stddev;
    Vec3 linear_acceleration_stddev;
    Vec3 angular_velocity_bias_random_walk;
    Vec3 linear_acceleration_bias_random_walk;
    Vec3 angular_velocity_quantization;
    Vec3 linear_acceleration_quantization;
    Vec3 angular_velocity_min{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    Vec3 angular_velocity_max{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    Vec3 linear_acceleration_min{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    Vec3 linear_acceleration_max{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
};

/** IMU measurement model consuming only truth and a scheduled sensor tick. */
class NKSENSOR_API ImuSensor final : public Sensor {
public:
    explicit ImuSensor(SensorConfig config, ImuNoiseConfig noise = {});

    const ImuNoiseConfig &noise_config() const noexcept { return noise_; }
    void reset_model() noexcept;

    /** Return no sample for a dropped tick. */
    std::optional<ImuSample> sample(const ImuTruth &truth, const SensorTick &tick);

private:
    ImuNoiseConfig noise_;
    Vec3 angular_bias_;
    Vec3 linear_acceleration_bias_;
    double last_truth_time_ = 0.0;
    bool has_last_truth_time_ = false;
};

struct LidarConfig {
    std::uint32_t horizontal_count = 1;
    std::uint32_t vertical_count = 1;
    double horizontal_angle_min = -1.5707963267948966;
    double horizontal_angle_max = 1.5707963267948966;
    double vertical_angle_min = 0.0;
    double vertical_angle_max = 0.0;
    double range_min = 0.0;
    double range_max = 100.0;
};

struct LidarNoiseConfig {
    double range_stddev = 0.0;
    double range_quantization = 0.0;
    double ray_dropout_probability = 0.0;
};

/** CPU LiDAR measurement model independent of any scene or raycast backend. */
class NKSENSOR_API LidarSensor final : public Sensor {
public:
    explicit LidarSensor(SensorConfig config, LidarConfig lidar = {},
                         LidarNoiseConfig noise = {});

    const LidarConfig &lidar_config() const noexcept { return lidar_; }
    const LidarNoiseConfig &noise_config() const noexcept { return noise_; }
    std::span<const LidarRay> rays() const noexcept { return rays_; }

    /** Convert one backend raycast result per configured ray into a scan. */
    std::optional<LidarScan> sample(const SensorTick &tick,
                                    std::span<const LidarHit> hits);

private:
    LidarConfig lidar_;
    LidarNoiseConfig noise_;
    std::vector<LidarRay> rays_;
};

struct CameraConfig {
    std::uint32_t width = 640;
    std::uint32_t height = 480;
    float fov_y = 1.04719755f;
    float near_plane = 0.01f;
    float far_plane = 1000.0f;
    std::array<float, 4> clear_color{0.0f, 0.0f, 0.0f, 1.0f};
};

struct CameraFrame {
    SensorSampleHeader header;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    /** Tightly packed top-to-bottom RGBA8 pixels. */
    std::vector<std::uint8_t> rgba8;

    const std::uint8_t *pixel(std::uint32_t x, std::uint32_t y) const noexcept {
        if (x >= width || y >= height)
            return nullptr;
        return rgba8.data() +
               (static_cast<std::size_t>(y) * width + x) * static_cast<std::size_t>(4);
    }
};

/** RGB camera measurement model independent of any rendering backend. */
class NKSENSOR_API CameraSensor final : public Sensor {
public:
    explicit CameraSensor(SensorConfig config, CameraConfig camera = {});

    const CameraConfig &camera_config() const noexcept { return camera_; }

    /** Package one backend-rendered RGBA8 image as a camera sample. */
    std::optional<CameraFrame> sample(const SensorTick &tick,
                                      std::span<const std::uint8_t> rgba8);

private:
    CameraConfig camera_;
};

} // namespace nksensor
