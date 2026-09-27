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

namespace {

using Poly = std::vector<double>; // ascending powers on the unit interval

std::mutex mutex;
std::unordered_map<uint32_t, std::vector<mk_path_sample>> paths;
std::unordered_map<uint32_t, std::vector<mk_time_stage>> laws;
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
            stage_speed(stage, stage.start_ns + stage.duration_ns) < 0.0 ||
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
    if (!std::isfinite(s) || s < stages.front().start_s || s > end_s(stages.back()))
        return false;
    for (const auto &stage : stages) {
        if (s == stage.start_s) {
            seconds = static_cast<double>(stage.start_ns) * 1e-9;
            return true;
        }
        if (s > end_s(stage)) continue;
        if (s == end_s(stage)) {
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
        const Poly fit = hermite(left.position, left.first * v0 * duration,
            (left.second * v0 * v0 + left.first * stage.acceleration) * duration * duration,
            right.position, right.first * v1 * duration,
            (right.second * v1 * v1 + right.first * stage.acceleration) * duration * duration);
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
        laws.emplace(id, std::vector<mk_time_stage>(stages, stages + stage_count));
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
    return inverse_time(found->second, s, *out_seconds) ? MK_OK : MK_ERROR_INVALID_ARGUMENT;
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
        if (std::abs(p->second.front().s - l->second.front().start_s) > 1e-10 ||
            std::abs(p->second.back().s - end_s(l->second.back())) > 1e-10)
            return MK_ERROR_INVALID_ARGUMENT;
        mk_trajectory_handle trajectory{};
        auto result = mk_trajectory_create(p->second.front().joint_count, &trajectory);
        if (result != MK_OK) return result;
        for (const auto &stage : l->second) {
            std::vector<int64_t> knots{stage.start_ns,
                stage.start_ns + stage.duration_ns};
            for (const auto &sample : p->second) {
                if (sample.s <= stage.start_s || sample.s >= end_s(stage)) continue;
                double seconds = 0.0;
                if (!inverse_time(l->second, sample.s, seconds)) {
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

} // extern C
