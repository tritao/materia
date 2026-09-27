#include "motionkit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <unordered_map>
#include <vector>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/solver/seidel.hpp>

namespace {

using Poly = std::vector<double>; // ascending powers on the unit interval

std::mutex mutex;
std::unordered_map<uint32_t, std::vector<mk_path_sample>> paths;
struct Law {
    std::vector<mk_time_stage> stages;
    std::vector<mk_timing_binding> bindings;
};
std::unordered_map<uint32_t, Law> laws;
uint32_t next_path = 1;
uint32_t next_law = 1;

double end_s(const mk_time_stage &stage) {
    const double duration = static_cast<double>(stage.duration_ns) * 1e-9;
    return stage.start_s + stage.speed * duration +
        0.5 * stage.acceleration * duration * duration;
}

double stage_s(const mk_time_stage &stage, int64_t time_ns) {
    const double tau = static_cast<double>(time_ns - stage.start_ns) * 1e-9;
    return stage.start_s + stage.speed * tau + 0.5 * stage.acceleration * tau * tau;
}

double stage_speed(const mk_time_stage &stage, int64_t time_ns) {
    return stage.speed + stage.acceleration *
        static_cast<double>(time_ns - stage.start_ns) * 1e-9;
}

bool valid_path(const mk_path_sample *samples, uint32_t count) {
    if (!samples || count < 2 || samples[0].joint_count == 0 ||
        samples[0].joint_count > MK_MAX_JOINTS) return false;
    const auto joints = samples[0].joint_count;
    for (uint32_t i = 0; i < count; ++i) {
        const auto &point = samples[i];
        if (point.struct_size < sizeof(point) || point.joint_count != joints ||
            !std::isfinite(point.s) ||
            (i && point.s <= samples[i - 1].s)) return false;
        for (uint32_t j = 0; j < joints; ++j)
            if (!std::isfinite(point.position[j]) || !std::isfinite(point.first[j]) ||
                !std::isfinite(point.second[j])) return false;
    }
    return true;
}

bool valid_law(const mk_time_stage *stages, uint32_t count) {
    if (!stages || !count || stages[0].start_ns != 0) return false;
    for (uint32_t i = 0; i < count; ++i) {
        const auto &stage = stages[i];
        if (stage.struct_size < sizeof(stage) || stage.duration_ns <= 0 ||
            stage.start_ns < 0 || stage.start_ns > INT64_MAX - stage.duration_ns ||
            !std::isfinite(stage.start_s) || !std::isfinite(stage.speed) ||
            !std::isfinite(stage.acceleration) || stage.speed < 0.0 ||
            stage_speed(stage, stage.start_ns + stage.duration_ns) < -1e-8 ||
            end_s(stage) <= stage.start_s) return false;
        if (i && (stage.start_ns != stages[i - 1].start_ns + stages[i - 1].duration_ns ||
            std::abs(stage.start_s - end_s(stages[i - 1])) >
                1e-10 * std::max(1.0, std::abs(stage.start_s)) ||
            std::abs(stage.speed - stage_speed(stages[i - 1], stage.start_ns)) >
                1e-9 * std::max(1.0, std::abs(stage.speed)))) return false;
    }
    return true;
}

bool inverse_time(const std::vector<mk_time_stage> &stages, double s, double &seconds) {
    const double epsilon = 1e-12 * std::max(1.0, std::abs(s));
    if (!std::isfinite(s) || s < stages.front().start_s - epsilon ||
        s > end_s(stages.back()) + epsilon)
        return false;
    for (const auto &stage : stages) {
        if (std::abs(s - stage.start_s) <= epsilon) {
            seconds = static_cast<double>(stage.start_ns) * 1e-9;
            return true;
        }
        if (s > end_s(stage) + epsilon) continue;
        if (std::abs(s - end_s(stage)) <= epsilon) {
            seconds = static_cast<double>(stage.start_ns + stage.duration_ns) * 1e-9;
            return true;
        }
        const double distance = s - stage.start_s;
        double tau;
        if (std::abs(stage.acceleration) < 1e-15) tau = distance / stage.speed;
        else {
            const double discriminant = stage.speed * stage.speed +
                2.0 * stage.acceleration * distance;
            if (discriminant < 0.0) return false;
            tau = 2.0 * distance / (stage.speed + std::sqrt(discriminant));
        }
        seconds = static_cast<double>(stage.start_ns) * 1e-9 + tau;
        return std::isfinite(seconds);
    }
    return false;
}

Poly add(const Poly &a, const Poly &b) {
    Poly out(std::max(a.size(), b.size()), 0.0);
    for (size_t i = 0; i < a.size(); ++i) out[i] += a[i];
    for (size_t i = 0; i < b.size(); ++i) out[i] += b[i];
    return out;
}

Poly multiply(const Poly &a, const Poly &b) {
    Poly out(a.size() + b.size() - 1, 0.0);
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < b.size(); ++j) out[i + j] += a[i] * b[j];
    return out;
}

double eval(const Poly &p, double x) {
    double result = 0.0;
    for (auto i = p.rbegin(); i != p.rend(); ++i) result = result * x + *i;
    return result;
}

Poly derivative(const Poly &p) {
    Poly out;
    for (size_t i = 1; i < p.size(); ++i) out.push_back(p[i] * static_cast<double>(i));
    return out;
}

// Quintic Hermite coefficients on x in [0,1].
Poly hermite(double p0, double v0, double a0, double p1, double v1, double a1) {
    const double d = p1 - p0 - v0 - 0.5 * a0;
    const double e = v1 - v0 - a0;
    const double f = a1 - a0;
    return {p0, v0, 0.5 * a0, 10.0 * d - 4.0 * e + 0.5 * f,
        -15.0 * d + 7.0 * e - f, 6.0 * d - 3.0 * e + 0.5 * f};
}

Poly compose(const Poly &outer, const Poly &inner) {
    Poly result{0.0};
    for (auto i = outer.rbegin(); i != outer.rend(); ++i) {
        result = multiply(result, inner);
        result[0] += *i;
    }
    return result;
}

// Isolate every stationary point by recursively partitioning at the derivative's
// roots. A polynomial is monotone between consecutive roots of its derivative.
std::vector<double> roots(const Poly &p) {
    if (p.size() < 2) return {};
    if (p.size() == 2) {
        if (p[1] == 0.0) return {};
        const double root = -p[0] / p[1];
        return root > 0.0 && root < 1.0 ? std::vector<double>{root} : std::vector<double>{};
    }
    auto cuts = roots(derivative(p));
    cuts.insert(cuts.begin(), 0.0);
    cuts.push_back(1.0);
    std::vector<double> result;
    for (size_t i = 1; i < cuts.size(); ++i) {
        double lo = cuts[i - 1], hi = cuts[i];
        double ylo = eval(p, lo), yhi = eval(p, hi);
        if (std::abs(ylo) < 1e-14 && lo > 0.0) result.push_back(lo);
        if ((ylo < 0.0 && yhi < 0.0) || (ylo > 0.0 && yhi > 0.0) ||
            ylo == 0.0 || yhi == 0.0) continue;
        for (int step = 0; step < 60; ++step) {
            const double mid = 0.5 * (lo + hi);
            if (mid == lo || mid == hi) break;
            if ((eval(p, mid) < 0.0) == (ylo < 0.0)) lo = mid;
            else hi = mid;
        }
        const double root = 0.5 * (lo + hi);
        if (root > 0.0 && root < 1.0) result.push_back(root);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end(), [](double a, double b) {
        return std::abs(a - b) < 1e-12;
    }), result.end());
    return result;
}

struct PathState { double position, first, second; };

Poly path_poly(const mk_path_sample &a, const mk_path_sample &b, uint32_t joint) {
    const double ds = b.s - a.s;
    return hermite(a.position[joint], a.first[joint] * ds,
        a.second[joint] * ds * ds, b.position[joint],
        b.first[joint] * ds, b.second[joint] * ds * ds);
}

PathState path_state(const mk_path_sample &a, const mk_path_sample &b,
                     uint32_t joint, double s) {
    const Poly q = path_poly(a, b, joint);
    const double ds = b.s - a.s;
    const double x = std::clamp((s - a.s) / ds, 0.0, 1.0);
    return {eval(q, x), eval(derivative(q), x) / ds,
        eval(derivative(derivative(q)), x) / (ds * ds)};
}

bool append_interval(const std::vector<mk_path_sample> &path,
                     const mk_time_stage &stage, int64_t t0, int64_t t1,
                     double tolerance, mk_trajectory_handle trajectory,
                     uint32_t depth) {
    if (t1 <= t0) return true;
    const double s0 = stage_s(stage, t0), s1 = stage_s(stage, t1);
    auto found = std::upper_bound(path.begin(), path.end(), 0.5 * (s0 + s1),
        [](double value, const mk_path_sample &sample) { return value < sample.s; });
    const size_t index = static_cast<size_t>(found - path.begin() - 1);
    if (index + 1 >= path.size()) return false;
    const auto &a = path[index], &b = path[index + 1];
    const double duration = static_cast<double>(t1 - t0) * 1e-9;
    const double v0 = stage_speed(stage, t0), v1 = stage_speed(stage, t1);
    const double ds = b.s - a.s;
    const Poly normalized_s{(s0 - a.s) / ds, v0 * duration / ds,
        0.5 * stage.acceleration * duration * duration / ds};

    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = t0;
    segment.duration_ns = t1 - t0;
    segment.degree = 5;
    segment.joint_count = a.joint_count;
    double worst = 0.0;
    for (uint32_t joint = 0; joint < a.joint_count; ++joint) {
        const auto left = path_state(a, b, joint, s0);
        const auto right = path_state(a, b, joint, s1);
        Poly fit = hermite(left.position, left.first * v0 * duration,
            (left.second * v0 * v0 + left.first * stage.acceleration) * duration * duration,
            right.position, right.first * v1 * duration,
            (right.second * v1 * v1 + right.first * stage.acceleration) * duration * duration);
        const Poly authored = path_poly(a, b, joint);
        if (std::abs(authored[2]) < 1e-12 && std::abs(authored[3]) < 1e-12 &&
            std::abs(authored[4]) < 1e-12 && std::abs(authored[5]) < 1e-12)
            fit = compose(authored, normalized_s);
        for (size_t i = 0; i < fit.size(); ++i)
            segment.coefficients[joint].value[i] = fit[i] / std::pow(duration, static_cast<int>(i));
        Poly residual = compose(path_poly(a, b, joint), normalized_s);
        for (size_t i = 0; i < fit.size(); ++i) residual[i] -= fit[i];
        worst = std::max(worst, std::abs(eval(residual, 0.5)));
        for (double root : roots(derivative(residual)))
            worst = std::max(worst, std::abs(eval(residual, root)));
    }
    if (worst > tolerance) {
        if (depth >= 40 || t1 - t0 < 2) return false;
        const int64_t middle = t0 + (t1 - t0) / 2;
        return append_interval(path, stage, t0, middle, tolerance, trajectory, depth + 1) &&
            append_interval(path, stage, middle, t1, tolerance, trajectory, depth + 1);
    }
    return mk_trajectory_append_segment(trajectory, &segment) == MK_OK;
}

class VelocityWithCaps final : public toppra::constraint::LinearJointVelocity {
public:
    VelocityWithCaps(const toppra::Vector &lower, const toppra::Vector &upper,
                     std::vector<double> knots, std::vector<double> caps)
        : LinearJointVelocity(lower, upper), knots_(std::move(knots)),
          caps_(std::move(caps)) {}

protected:
    void computeVelocityLimits(double s) override {
        auto next = std::upper_bound(knots_.begin(), knots_.end(), s);
        size_t span = std::min(caps_.size() - 1,
            static_cast<size_t>(std::max<int64_t>(0, next - knots_.begin() - 1)));
        double cap = caps_[span] > 0.0 ? caps_[span] : 1e8;
        if (span > 0 && std::abs(s - knots_[span]) < 1e-12 && caps_[span - 1] > 0.0)
            cap = std::min(cap, caps_[span - 1]);
        m_lower[m_lower.size() - 1] = -cap;
        m_upper[m_upper.size() - 1] = cap;
    }

private:
    std::vector<double> knots_;
    std::vector<double> caps_;
};

bool valid_timing_request(const std::vector<mk_path_sample> &path,
                          const double *velocity, const double *acceleration,
                          uint32_t joints, const double *caps, uint32_t cap_count,
                          double start_speed, double end_speed) {
    if (!velocity || !acceleration || joints != path.front().joint_count ||
        (cap_count && (!caps || cap_count != path.size() - 1)) ||
        (!cap_count && caps) || !std::isfinite(start_speed) || start_speed < 0.0 ||
        !std::isfinite(end_speed) || end_speed < 0.0) return false;
    for (uint32_t j = 0; j < joints; ++j)
        if (!std::isfinite(velocity[j]) || velocity[j] <= 0.0 ||
            !std::isfinite(acceleration[j]) || acceleration[j] <= 0.0) return false;
    for (uint32_t i = 0; i < cap_count; ++i)
        if (!std::isfinite(caps[i]) || caps[i] < 0.0) return false;
    return true;
}

mk_timing_binding binding_for_stage(const std::vector<mk_path_sample> &path,
                                    uint32_t path_span, uint32_t stage_index,
                                    double s, double speed_squared,
                                    double path_acceleration,
                                    const double *velocity,
                                    const double *acceleration,
                                    double feed_cap) {
    mk_timing_binding binding{};
    binding.struct_size = sizeof(binding);
    binding.stage_index = stage_index;
    binding.kind = MK_TIMING_BINDING_FEED_CAP;
    binding.joint = UINT32_MAX;
    binding.limit = feed_cap;
    double ratio = feed_cap > 0.0 ? std::sqrt(speed_squared) / feed_cap : -1.0;
    for (uint32_t j = 0; j < path.front().joint_count; ++j) {
        const auto q = path_state(path[path_span], path[path_span + 1], j, s);
        const double velocity_ratio = std::abs(q.first) *
            std::sqrt(speed_squared) / velocity[j];
        if (velocity_ratio > ratio) {
            ratio = velocity_ratio;
            binding.kind = MK_TIMING_BINDING_JOINT_VELOCITY;
            binding.joint = j;
            binding.limit = velocity[j];
        }
        const double acceleration_ratio = std::abs(q.first * path_acceleration +
            q.second * speed_squared) / acceleration[j];
        if (acceleration_ratio > ratio) {
            ratio = acceleration_ratio;
            binding.kind = MK_TIMING_BINDING_JOINT_ACCELERATION;
            binding.joint = j;
            binding.limit = acceleration[j];
        }
    }
    return binding;
}

bool stretch_stages(std::vector<mk_time_stage> &stages, double factor) {
    int64_t start_ns = 0;
    double speed = stages.front().speed / factor;
    for (auto &stage : stages) {
        const double distance = end_s(stage) - stage.start_s;
        const double scaled = std::floor(static_cast<double>(stage.duration_ns) * factor);
        if (!std::isfinite(scaled) || scaled < 1.0 ||
            scaled > static_cast<double>(INT64_MAX - start_ns)) return false;
        const int64_t duration_ns = static_cast<int64_t>(scaled);
        const double duration = static_cast<double>(duration_ns) * 1e-9;
        stage.start_ns = start_ns;
        stage.duration_ns = duration_ns;
        stage.acceleration = 2.0 * (distance - speed * duration) /
            (duration * duration);
        stage.speed = speed;
        speed += stage.acceleration * duration;
        start_ns += duration_ns;
    }
    return true;
}

// A collocation peak can exceed a joint bound after quintic lowering. Reduce
// path speed around that peak, preserving all other stages' time scale.
bool soften_stages(std::vector<mk_time_stage> &stages, double peak_time,
                   double factor) {
    if (stages.empty() || !std::isfinite(peak_time) || factor <= 1.0) return false;
    const int64_t peak_ns = static_cast<int64_t>(std::llround(peak_time * 1e9));
    double center_s = stages.back().start_s;
    for (const auto &stage : stages) {
        if (peak_ns <= stage.start_ns + stage.duration_ns) {
            center_s = stage_s(stage, std::clamp(peak_ns, stage.start_ns,
                stage.start_ns + stage.duration_ns));
            break;
        }
    }
    std::vector<double> positions, speeds;
    positions.reserve(stages.size() + 1);
    speeds.reserve(stages.size() + 1);
    for (const auto &stage : stages) {
        positions.push_back(stage.start_s);
        speeds.push_back(std::max(0.0, stage.speed));
    }
    positions.push_back(end_s(stages.back()));
    speeds.push_back(std::max(0.0, stage_speed(stages.back(),
        stages.back().start_ns + stages.back().duration_ns)));
    const double radius = std::min(0.015, (positions.back() - positions.front()) / 10.0);
    bool changed = false;
    for (size_t i = 1; i + 1 < speeds.size(); ++i) {
        const double offset = std::abs(positions[i] - center_s) / radius;
        const double weight = offset < 1.0 ?
            0.5 * (1.0 + std::cos(std::acos(-1.0) * offset)) : 0.0;
        if (weight > 0.0) {
            speeds[i] *= 1.0 - weight * (1.0 - 0.99 / factor);
            changed = true;
        }
    }
    if (!changed) return false;
    int64_t start_ns = 0;
    double speed = speeds.front();
    for (size_t i = 0; i < stages.size(); ++i) {
        auto &stage = stages[i];
        const double distance = positions[i + 1] - positions[i];
        const double seconds = 2.0 * distance / (speed + speeds[i + 1]);
        if (!std::isfinite(seconds) || seconds <= 0.0 ||
            seconds > static_cast<double>(INT64_MAX - start_ns) * 1e-9) return false;
        const int64_t duration_ns = std::max<int64_t>(1,
            static_cast<int64_t>(std::floor(seconds * 1e9)));
        const double rounded = static_cast<double>(duration_ns) * 1e-9;
        stage.start_ns = start_ns;
        stage.duration_ns = duration_ns;
        stage.speed = speed;
        stage.acceleration = 2.0 * (distance - speed * rounded) / (rounded * rounded);
        speed += stage.acceleration * rounded;
        start_ns += duration_ns;
    }
    return true;
}

} // namespace

extern "C" {

mk_result MK_CALL mk_path_create(const mk_path_sample *samples, uint32_t sample_count,
                                  mk_path_handle *out_path) {
    if (!out_path || !valid_path(samples, sample_count)) return MK_ERROR_INVALID_ARGUMENT;
    out_path->id = 0;
    try {
        std::lock_guard lock(mutex);
        while (!next_path || paths.count(next_path)) ++next_path;
        const uint32_t id = next_path++;
        paths.emplace(id, std::vector<mk_path_sample>(samples, samples + sample_count));
        out_path->id = id;
        return MK_OK;
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
}

void MK_CALL mk_path_destroy(mk_path_handle path) {
    std::lock_guard lock(mutex);
    paths.erase(path.id);
}

mk_result MK_CALL mk_time_law_create(const mk_time_stage *stages, uint32_t stage_count,
                                      mk_time_law_handle *out_law) {
    if (!out_law || !valid_law(stages, stage_count)) return MK_ERROR_INVALID_ARGUMENT;
    out_law->id = 0;
    try {
        std::lock_guard lock(mutex);
        while (!next_law || laws.count(next_law)) ++next_law;
        const uint32_t id = next_law++;
        laws.emplace(id, Law{std::vector<mk_time_stage>(stages, stages + stage_count), {}});
        out_law->id = id;
        return MK_OK;
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
}

void MK_CALL mk_time_law_destroy(mk_time_law_handle law) {
    std::lock_guard lock(mutex);
    laws.erase(law.id);
}

mk_result MK_CALL mk_path_distance_to_time(mk_time_law_handle law, double s,
                                            double *out_seconds) {
    if (!out_seconds) return MK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(mutex);
    const auto found = laws.find(law.id);
    if (found == laws.end()) return MK_ERROR_INVALID_HANDLE;
    return inverse_time(found->second.stages, s, *out_seconds) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
}

mk_result MK_CALL mk_path_lower(mk_path_handle path, mk_time_law_handle law,
                                 double tolerance, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory || !std::isfinite(tolerance) || tolerance <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    try {
        std::lock_guard lock(mutex);
        const auto p = paths.find(path.id);
        const auto l = laws.find(law.id);
        if (p == paths.end() || l == laws.end()) return MK_ERROR_INVALID_HANDLE;
        if (std::abs(p->second.front().s - l->second.stages.front().start_s) > 1e-10 ||
            std::abs(p->second.back().s - end_s(l->second.stages.back())) > 1e-10)
            return MK_ERROR_INVALID_ARGUMENT;
        mk_trajectory_handle trajectory{};
        auto result = mk_trajectory_create(p->second.front().joint_count, &trajectory);
        if (result != MK_OK) return result;
        for (const auto &stage : l->second.stages) {
            std::vector<int64_t> knots{stage.start_ns,
                stage.start_ns + stage.duration_ns};
            for (const auto &sample : p->second) {
                if (sample.s <= stage.start_s || sample.s >= end_s(stage)) continue;
                double seconds = 0.0;
                if (!inverse_time(l->second.stages, sample.s, seconds)) {
                    mk_trajectory_destroy(trajectory);
                    return MK_ERROR_INVALID_ARGUMENT;
                }
                knots.push_back(static_cast<int64_t>(std::llround(seconds * 1e9)));
            }
            std::sort(knots.begin(), knots.end());
            knots.erase(std::unique(knots.begin(), knots.end()), knots.end());
            for (size_t i = 1; i < knots.size(); ++i) {
                if (!append_interval(p->second, stage, knots[i - 1], knots[i],
                    tolerance, trajectory, 0)) {
                    mk_trajectory_destroy(trajectory);
                    return MK_ERROR_GENERATION;
                }
            }
        }
        *out_trajectory = trajectory;
        return MK_OK;
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
}

mk_result MK_CALL mk_time_path(mk_path_handle path, const double *max_velocity,
                               const double *max_acceleration, uint32_t joint_count,
                               const double *speed_caps, uint32_t speed_cap_count,
                               double start_speed, double end_speed,
                               double lowering_tolerance,
                               mk_time_law_handle *out_law,
                               mk_trajectory_handle *out_trajectory) {
    if (!out_law || !out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_law->id = 0;
    out_trajectory->id = 0;
    if (!std::isfinite(lowering_tolerance) || lowering_tolerance <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    std::vector<mk_path_sample> samples;
    {
        std::lock_guard lock(mutex);
        const auto found = paths.find(path.id);
        if (found == paths.end()) return MK_ERROR_INVALID_HANDLE;
        samples = found->second;
    }
    if (!valid_timing_request(samples, max_velocity, max_acceleration, joint_count,
        speed_caps, speed_cap_count, start_speed, end_speed))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const auto dof = static_cast<Eigen::Index>(joint_count + 1);
        std::vector<double> knots, caps;
        toppra::Matrices coefficients;
        knots.reserve(samples.size());
        caps.reserve(samples.size() - 1);
        coefficients.reserve(samples.size() - 1);
        for (const auto &sample : samples) knots.push_back(sample.s);
        for (size_t i = 0; i + 1 < samples.size(); ++i) {
            const double ds = samples[i + 1].s - samples[i].s;
            toppra::Matrix matrix = toppra::Matrix::Zero(6, dof);
            for (uint32_t j = 0; j < joint_count; ++j) {
                const Poly p = path_poly(samples[i], samples[i + 1], j);
                for (size_t k = 0; k < p.size(); ++k)
                    matrix(5 - static_cast<Eigen::Index>(k), j) =
                        p[k] / std::pow(ds, static_cast<int>(k));
            }
            matrix(4, joint_count) = 1.0;
            matrix(5, joint_count) = samples[i].s;
            coefficients.push_back(std::move(matrix));
            caps.push_back(speed_cap_count ? speed_caps[i] : 0.0);
        }
        auto geometric = std::make_shared<toppra::PiecewisePolyPath>(coefficients, knots);
        bool nonlinear = false;
        for (size_t span = 0; span + 1 < samples.size() && !nonlinear; ++span)
            for (uint32_t joint = 0; joint < joint_count; ++joint)
                if (std::abs(samples[span].second[joint]) > 1e-10 ||
                    std::abs(samples[span + 1].second[joint]) > 1e-10 ||
                    std::abs(samples[span].first[joint] - samples[span + 1].first[joint]) > 1e-10) {
                    nonlinear = true;
                    break;
                }
        // Collocation can miss acceleration peaks between samples on a curved
        // path. Reserve room for those peaks; final validation still uses the
        // caller's exact acceleration limit.
        const double interpolation_margin = nonlinear ? 0.8 : 1.0;
        toppra::Vector lower_velocity(dof), upper_velocity(dof);
        toppra::Vector lower_acceleration(dof), upper_acceleration(dof);
        const double moving_boundary_margin = (start_speed > 0.0 || end_speed > 0.0)
            ? 0.98 : 1.0;
        for (uint32_t j = 0; j < joint_count; ++j) {
            lower_velocity[j] = -max_velocity[j] * moving_boundary_margin;
            upper_velocity[j] = max_velocity[j] * moving_boundary_margin;
            lower_acceleration[j] = -max_acceleration[j] * moving_boundary_margin * interpolation_margin;
            upper_acceleration[j] = max_acceleration[j] * moving_boundary_margin * interpolation_margin;
        }
        lower_velocity[joint_count] = -1e8;
        upper_velocity[joint_count] = 1e8;
        lower_acceleration[joint_count] = -1e12;
        upper_acceleration[joint_count] = 1e12;
        toppra::LinearConstraintPtrs constraints{
            std::make_shared<VelocityWithCaps>(lower_velocity, upper_velocity, knots, caps),
            std::make_shared<toppra::constraint::LinearJointAcceleration>(
                lower_acceleration, upper_acceleration)};
        for (auto &constraint : constraints)
            constraint->discretizationType(toppra::DiscretizationType::Collocation);
        toppra::algorithm::TOPPRA algorithm(constraints, geometric);
        algorithm.solver(std::make_shared<toppra::solver::Seidel>());
        toppra::Vector grid(static_cast<Eigen::Index>(1 + 32 * (samples.size() - 1)));
        for (size_t span = 0; span + 1 < samples.size(); ++span)
            for (int step = 0; step < 32; ++step)
                grid[static_cast<Eigen::Index>(span * 32 + step)] = samples[span].s +
                    (samples[span + 1].s - samples[span].s) * step / 32.0;
        grid[grid.size() - 1] = samples.back().s;
        algorithm.setGridpoints(grid);
        algorithm.setInitialXBounds(toppra::Bound{0.0, 1e16});
        // TOPP-RA's forward pass stores its first parameter as squared path
        // speed, while the public endpoint contract uses path speed.
        if (algorithm.computePathParametrization(start_speed * start_speed, end_speed) !=
            toppra::ReturnCode::OK) return MK_ERROR_GENERATION;
        const auto &data = algorithm.getParameterizationData();
        std::vector<mk_time_stage> stages;
        std::vector<mk_timing_binding> bindings;
        stages.reserve(static_cast<size_t>(grid.size() - 1));
        bindings.reserve(stages.capacity());
        int64_t start_ns = 0;
        double speed = start_speed;
        for (Eigen::Index i = 0; i + 1 < grid.size(); ++i) {
            const double ds = grid[i + 1] - grid[i];
            const double target_speed = std::sqrt(std::max(0.0, data.parametrization[i + 1]));
            const double duration = 2.0 * ds / (speed + target_speed);
            if (!std::isfinite(duration) || duration <= 0.0 ||
                duration > static_cast<double>(INT64_MAX - start_ns) * 1e-9)
                return MK_ERROR_GENERATION;
            const int64_t duration_ns = std::max<int64_t>(1,
                static_cast<int64_t>(std::floor(duration * 1e9)));
            const double rounded = static_cast<double>(duration_ns) * 1e-9;
            const double path_acceleration = 2.0 * (ds - speed * rounded) /
                (rounded * rounded);
            mk_time_stage stage{};
            stage.struct_size = sizeof(stage);
            stage.start_ns = start_ns;
            stage.duration_ns = duration_ns;
            stage.start_s = grid[i];
            stage.speed = speed;
            stage.acceleration = path_acceleration;
            stages.push_back(stage);
            const auto span = static_cast<size_t>(i) / 32;
            const double mid_s = 0.5 * (grid[i] + grid[i + 1]);
            const double mid_speed_squared = speed * speed + path_acceleration * ds;
            bindings.push_back(binding_for_stage(samples, static_cast<uint32_t>(span),
                static_cast<uint32_t>(i), mid_s, std::max(0.0, mid_speed_squared),
                path_acceleration, max_velocity, max_acceleration, caps[span]));
            speed += path_acceleration * rounded;
            start_ns += duration_ns;
        }
        mk_time_law_handle created{};
        bool accepted = false;
        for (int attempt = 0; attempt < 24; ++attempt) {
            const auto result = mk_time_law_create(stages.data(),
                static_cast<uint32_t>(stages.size()), &created);
            if (result != MK_OK) return result;
            mk_trajectory_handle lowered{};
            const auto lowered_result = mk_path_lower(path, created,
                lowering_tolerance, &lowered);
            if (lowered_result != MK_OK) {
                mk_time_law_destroy(created);
                return lowered_result;
            }
            mk_limits limits{};
            limits.struct_size = sizeof(limits);
            limits.joint_count = joint_count;
            for (uint32_t j = 0; j < joint_count; ++j) {
                limits.max_velocity[j] = max_velocity[j];
                limits.max_acceleration[j] = max_acceleration[j];
            }
            mk_validation_report report{};
            report.struct_size = sizeof(report);
            const auto validation = mk_validate(lowered, &limits, &report);
            if (validation != MK_OK) {
                mk_trajectory_destroy(lowered);
                mk_time_law_destroy(created);
                return MK_ERROR_GENERATION;
            }
            double factor = 1.0;
            const auto &velocity_check = report.checks[MK_CHECK_VELOCITY];
            const auto &acceleration_check = report.checks[MK_CHECK_ACCELERATION];
            if (velocity_check.status == MK_CHECK_FAILED)
                factor = std::max(factor, velocity_check.value / velocity_check.limit);
            if (acceleration_check.status == MK_CHECK_FAILED)
                factor = std::max(factor, std::sqrt(acceleration_check.value /
                    acceleration_check.limit));
            if (factor <= 1.0) {
                accepted = true;
                *out_trajectory = lowered;
                break;
            }
            mk_trajectory_destroy(lowered);
            mk_time_law_destroy(created);
            // Uniform stretching would change an authored moving endpoint
            // speed, so report infeasibility instead of returning a law with
            // a different boundary contract.
            if (start_speed > 0.0 || end_speed > 0.0) return MK_ERROR_GENERATION;
            if (stages.size() > 1000 &&
                acceleration_check.status == MK_CHECK_FAILED && attempt < 20 &&
                soften_stages(stages, acceleration_check.time_seconds, factor * 1.002))
                continue;
            if (!stretch_stages(stages, factor * 1.002)) return MK_ERROR_GENERATION;
        }
        if (!accepted) return MK_ERROR_GENERATION;
        {
            std::lock_guard lock(mutex);
            laws.at(created.id).bindings = std::move(bindings);
        }
        *out_law = created;
        return MK_OK;
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}

mk_result MK_CALL mk_time_law_binding_count(mk_time_law_handle law,
                                             uint32_t *out_count) {
    if (!out_count) return MK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(mutex);
    const auto found = laws.find(law.id);
    if (found == laws.end()) return MK_ERROR_INVALID_HANDLE;
    *out_count = static_cast<uint32_t>(found->second.bindings.size());
    return MK_OK;
}

mk_result MK_CALL mk_time_law_get_binding(mk_time_law_handle law,
                                          uint32_t index, mk_timing_binding *out_binding) {
    if (!out_binding || out_binding->struct_size < sizeof(*out_binding))
        return MK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(mutex);
    const auto found = laws.find(law.id);
    if (found == laws.end()) return MK_ERROR_INVALID_HANDLE;
    if (index >= found->second.bindings.size()) return MK_ERROR_INVALID_ARGUMENT;
    *out_binding = found->second.bindings[index];
    return MK_OK;
}

} // extern C
