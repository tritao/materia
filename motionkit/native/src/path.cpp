#include "motionkit.h"
#include "trajectory_merge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
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
    // Which limit bound each of TOPP-RA's stages is worked out when it is asked for (a plan rarely is).
    std::function<std::vector<mk_timing_binding>()> make_bindings;
};
std::unordered_map<uint32_t, Law> laws;
uint32_t next_path = 1;
uint32_t next_law = 1;

// Called with the mutex held.
void ensure_bindings(Law &law) {
    if (!law.make_bindings) return;
    law.bindings = law.make_bindings();
    law.make_bindings = nullptr;
}

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
                !std::isfinite(point.second[j]) || !std::isfinite(point.second_before[j]))
                return false;
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
    // Only the final rest boundary receives the larger tolerance caused by
    // nanosecond rounding. Interior path-event positions keep full precision.
    const auto &last = stages.back();
    const double final_epsilon = 1e-8 * std::max(1.0, std::abs(s));
    if (!std::isfinite(s) || s < stages.front().start_s - epsilon ||
        s > end_s(last) + final_epsilon)
        return false;
    if (std::abs(s - end_s(last)) <= final_epsilon) {
        seconds = static_cast<double>(last.start_ns + last.duration_ns) * 1e-9;
        return true;
    }
    // Stages are contiguous and ordered, so the ones that end before `s` are skipped by bisection.
    const auto first = std::partition_point(stages.begin(), stages.end(),
        [s, epsilon](const mk_time_stage &stage) { return s > end_s(stage) + epsilon; });
    for (auto at = first; at != stages.end(); ++at) {
        const auto &stage = *at;
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

/** Distance reached `seconds` after the epoch; clamped to the law's first and last distance. */
double distance_at(const std::vector<mk_time_stage> &stages, double seconds) {
    const auto &first = stages.front();
    const auto &last = stages.back();
    if (seconds <= static_cast<double>(first.start_ns) * 1e-9) return first.start_s;
    if (seconds >= static_cast<double>(last.start_ns + last.duration_ns) * 1e-9)
        return end_s(last);
    // The first stage that ends after `seconds`; stages are contiguous and ordered.
    auto stage = std::partition_point(stages.begin(), stages.end(),
        [seconds](const mk_time_stage &candidate) {
            return static_cast<double>(candidate.start_ns + candidate.duration_ns) * 1e-9 <= seconds;
        });
    if (stage == stages.end()) stage = std::prev(stages.end());
    const double tau = seconds - static_cast<double>(stage->start_ns) * 1e-9;
    return stage->start_s + stage->speed * tau + 0.5 * stage->acceleration * tau * tau;
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
        b.first[joint] * ds, b.second_before[joint] * ds * ds);
}

PathState path_state(const mk_path_sample &a, const mk_path_sample &b,
                     uint32_t joint, double s) {
    const Poly q = path_poly(a, b, joint);
    const double ds = b.s - a.s;
    const double x = std::clamp((s - a.s) / ds, 0.0, 1.0);
    return {eval(q, x), eval(derivative(q), x) / ds,
        eval(derivative(derivative(q)), x) / (ds * ds)};
}

// Polynomials of the lowering: composing a quintic path with a quadratic time law gives degree ten, and this runs for
// every joint of every interval of a print (a million), so they live on the stack.
struct Small {
    double c[13];
    int n;
};

Small small_hermite(const mk_path_sample &a, const mk_path_sample &b, uint32_t joint) {
    const double ds = b.s - a.s;
    const double p0 = a.position[joint], v0 = a.first[joint] * ds, a0 = a.second[joint] * ds * ds;
    const double p1 = b.position[joint], v1 = b.first[joint] * ds, a1 = b.second_before[joint] * ds * ds;
    const double d = p1 - p0 - v0 - 0.5 * a0;
    const double e = v1 - v0 - a0;
    const double f = a1 - a0;
    Small q{};
    q.n = 6;
    q.c[0] = p0; q.c[1] = v0; q.c[2] = 0.5 * a0; q.c[3] = 10.0 * d - 4.0 * e + 0.5 * f;
    q.c[4] = -15.0 * d + 7.0 * e - f; q.c[5] = 6.0 * d - 3.0 * e + 0.5 * f;
    return q;
}

// `outer(inner(x))` by Horner's scheme, the same sums as `compose`, keeping the first `keep` coefficients.
Small small_compose(const Small &outer, const double inner[3], int keep) {
    Small result{};
    result.n = 1;
    for (int k = outer.n - 1; k >= 0; --k) {
        double product[13] = {};
        const int size = std::min(result.n + 2, keep);
        for (int i = 0; i < result.n; ++i)
            for (int j = 0; j < 3; ++j)
                if (i + j < size) product[i + j] += result.c[i] * inner[j];
        for (int i = 0; i < size; ++i) result.c[i] = product[i];
        result.n = size;
        result.c[0] += outer.c[k];
    }
    return result;
}

// The path's position, first and second derivative at `x` on [0,1] of a span `ds` long.
PathState small_state(const Small &q, double ds, double x) {
    double d1[12], d2[11];
    for (int i = 1; i < q.n; ++i) d1[i - 1] = q.c[i] * static_cast<double>(i);
    for (int i = 1; i < q.n - 1; ++i) d2[i - 1] = d1[i] * static_cast<double>(i);
    auto horner = [x](const double *c, int count) {
        double value = 0.0;
        for (int i = count - 1; i >= 0; --i) value = value * x + c[i];
        return value;
    };
    return {horner(q.c, q.n), horner(d1, q.n - 1) / ds, horner(d2, q.n - 2) / (ds * ds)};
}

// The span's polynomials, kept between the intervals of one stage and the stages that share a span.
struct SpanCache {
    size_t index = SIZE_MAX;
    Small poly[MK_MAX_JOINTS];
    bool exact[MK_MAX_JOINTS];

    void load(const std::vector<mk_path_sample> &path, size_t span) {
        index = span;
        const auto &a = path[span], &b = path[span + 1];
        for (uint32_t joint = 0; joint < a.joint_count; ++joint) {
            poly[joint] = small_hermite(a, b, joint);
            const auto &c = poly[joint].c;
            // A joint that is affine in s over the span (a straight move of a Cartesian machine) is composed with the
            // time law exactly: nothing is approximated, so there is no residual to bound.
            exact[joint] = std::abs(c[2]) < 1e-12 && std::abs(c[3]) < 1e-12 &&
                std::abs(c[4]) < 1e-12 && std::abs(c[5]) < 1e-12;
        }
    }
};

bool append_interval(const std::vector<mk_path_sample> &path, SpanCache &cache,
                     const mk_time_stage &stage, int64_t t0, int64_t t1,
                     double tolerance, mk_trajectory_handle trajectory,
                     uint32_t depth) {
    if (t1 <= t0) return true;
    const double s0 = stage_s(stage, t0), s1 = stage_s(stage, t1);
    auto found = std::upper_bound(path.begin(), path.end(), 0.5 * (s0 + s1),
        [](double value, const mk_path_sample &sample) { return value < sample.s; });
    const size_t index = static_cast<size_t>(found - path.begin() - 1);
    if (index + 1 >= path.size()) return false;
    if (cache.index != index) cache.load(path, index);
    const auto &a = path[index], &b = path[index + 1];
    const double duration = static_cast<double>(t1 - t0) * 1e-9;
    const double v0 = stage_speed(stage, t0), v1 = stage_speed(stage, t1);
    const double ds = b.s - a.s;
    const double normalized_s[3] = {(s0 - a.s) / ds, v0 * duration / ds,
        0.5 * stage.acceleration * duration * duration / ds};
    double duration_power[6];
    for (int i = 0; i < 6; ++i) duration_power[i] = std::pow(duration, i);

    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = t0;
    segment.duration_ns = t1 - t0;
    segment.degree = 5;
    segment.joint_count = a.joint_count;
    double worst = 0.0;
    for (uint32_t joint = 0; joint < a.joint_count; ++joint) {
        const Small &authored = cache.poly[joint];
        if (cache.exact[joint]) {
            const Small fit = small_compose(authored, normalized_s, 6);
            for (int i = 0; i < 6; ++i) segment.coefficients[joint].value[i] = fit.c[i] / duration_power[i];
            continue;
        }
        const auto left = small_state(authored, ds, std::clamp((s0 - a.s) / ds, 0.0, 1.0));
        const auto right = small_state(authored, ds, std::clamp((s1 - a.s) / ds, 0.0, 1.0));
        const Poly fit = hermite(left.position, left.first * v0 * duration,
            (left.second * v0 * v0 + left.first * stage.acceleration) * duration * duration,
            right.position, right.first * v1 * duration,
            (right.second * v1 * v1 + right.first * stage.acceleration) * duration * duration);
        for (size_t i = 0; i < fit.size(); ++i)
            segment.coefficients[joint].value[i] = fit[i] / duration_power[i];
        const Small full = small_compose(authored, normalized_s, 13);
        double residual[13] = {};
        double bound = 0.0;
        for (int i = 0; i < full.n; ++i) {
            residual[i] = full.c[i] - (i < 6 ? fit[static_cast<size_t>(i)] : 0.0);
            bound += std::abs(residual[i]);
        }
        // On [0,1] a polynomial stays within the sum of its coefficients' magnitudes: when that is within the
        // tolerance there is no need to isolate its extrema (nearly every stage of a fine path).
        if (bound <= tolerance) { worst = std::max(worst, bound); continue; }
        // The fit matches the path's position, velocity and acceleration at both ends of the interval, so the residual
        // is x^3 (1 - x)^3 g(x) (at most g / 64) plus rounding: bounded by its quotient, it needs no extrema either
        // unless the quotient is large (a stage that cuts a tight curve in one piece).
        {
            const double divisor[7] = {0.0, 0.0, 0.0, 1.0, -3.0, 3.0, -1.0};
            double rest[13];
            std::copy(residual, residual + 13, rest);
            double quotient_sum = 0.0;
            for (int k = 12; k >= 6; --k) {
                const double q = rest[k] / divisor[6];
                quotient_sum += std::abs(q);
                for (int j = 0; j <= 6; ++j) rest[k - 6 + j] -= q * divisor[j];
            }
            double remainder_sum = 0.0;
            for (int k = 0; k < 6; ++k) remainder_sum += std::abs(rest[k]);
            const double hull = quotient_sum / 64.0 + remainder_sum;
            if (hull <= tolerance) { worst = std::max(worst, hull); continue; }
        }
        const Poly extremum(residual, residual + full.n);
        worst = std::max(worst, std::abs(eval(extremum, 0.5)));
        for (double root : roots(derivative(extremum)))
            worst = std::max(worst, std::abs(eval(extremum, root)));
    }
    if (worst > tolerance) {
        if (depth >= 40 || t1 - t0 < 2) return false;
        const int64_t middle = t0 + (t1 - t0) / 2;
        return append_interval(path, cache, stage, t0, middle, tolerance, trajectory, depth + 1) &&
            append_interval(path, cache, stage, middle, t1, tolerance, trajectory, depth + 1);
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

// TOPP-RA's stages in a span of the path: enough that the profile and the acceleration peaks between its grid points
// are resolved, never more than `MAX_STAGES_PER_SPAN` (the grid every span used to have, which the 0.8 interpolation
// margin is calibrated with).
//
// Two things need stages. The speed profile: a stage assumes one path acceleration, so it should change the speed by
// no more than `PROFILE_STEP` of itself, `a ds <= PROFILE_STEP v^2`, with `a` the path acceleration the joints allow
// and `v` the fastest the span can be taken (a joint's velocity limit, the centripetal bound of its curvature, the
// span's cap). And the collocation: the acceleration constraint is imposed at the grid points, and between them the
// joint-space direction turns; a stage turns it by at most `MAX_STAGE_TURN`, which bounds the missed peak well inside
// the margin (about turn^2 / 8 of it, 4.5 % at 0.6 rad). A straight span of a few micrometres needs two stages, a
// quarter circle of fifty micrometres' radius four.
//
// Measured on a 135 degree arc of 50 micrometres (tests/path.cpp), the acceleration the lowered law reaches over what
// TOPP-RA planned (0.8 of the limit) is about 2.4 `PROFILE_STEP`: 2.3 % with the old 32 stages a span, 12 % at 0.05
// (the value here, 0.9 of the limit), 24 % at 0.1 and, at 0.2, the whole of the margin. The planned time of the printing
// loads is unchanged at 0.05 (the 8 mm gear 2356.9 s, the cube 870.3 s, the gate 1047.2 s) and 0.015 % longer at 0.1.
constexpr size_t MAX_STAGES_PER_SPAN = 32;
constexpr size_t MIN_STAGES_PER_SPAN = 2;
constexpr double PROFILE_STEP = 0.05;
constexpr double MAX_STAGE_TURN = 0.6;

// A straight span (every joint affine in s: a line of a Cartesian machine, however long) has the same limits
// everywhere, so its speed profile changes only in the stretch next to each end where it speeds up to its fastest or
// slows from it, `Z = v^2 / 2a` long (`v` the fastest it can go, `a` the path acceleration the joints allow): beyond
// that it cruises, and one stage covers the middle. The stretches get `ZONE_STAGES` stages each, so the stages of a line
// do not grow with its length, where a stage per few micrometres of a hundred millimetres was a million for a cube.
constexpr size_t ZONE_STAGES = 10;

// Keep each solver's constraint tables bounded. Window boundaries propagate complete squared-speed
// controllable intervals backwards and the actually reachable squared speed forwards.
constexpr size_t WINDOW_NODES = 8192;
constexpr double ZONE_REACH = 1.5;

// Stage positions within a span, chosen from its local velocity, curvature and acceleration bounds.
struct SpanGrid {
    std::vector<double> nodes; // the fractions of the span at which its stages start, the first being zero
};

SpanGrid nodes_for_span(const std::vector<mk_path_sample> &samples, size_t span,
                                   const double *velocity, const double *acceleration, uint32_t joints,
                                   double cap, double margin, bool reference) {
    const double length = samples[span + 1].s - samples[span].s;
    double fastest = cap > 0.0 ? cap : std::numeric_limits<double>::infinity();
    double accelerating = std::numeric_limits<double>::infinity();
    double turn = 0.0;
    std::vector<double> before(joints, 0.0), after(joints, 0.0);
    bool moving = false, straight = true;
    for (int point = 0; point <= 4; ++point) {
        const double s = samples[span].s + length * point / 4.0;
        double load = 0.0;
        for (uint32_t j = 0; j < joints; ++j) {
            const auto q = path_state(samples[span], samples[span + 1], j, s);
            after[j] = q.first / velocity[j];
            load = std::max(load, std::abs(after[j]));
            if (q.first != 0.0) accelerating = std::min(accelerating, acceleration[j] / std::abs(q.first));
            if (q.second != 0.0)
                fastest = std::min(fastest, std::sqrt(margin * acceleration[j] / std::abs(q.second)));
        }
        if (load > 0.0) {
            moving = true;
            fastest = std::min(fastest, 1.0 / load);
            if (point > 0) {
                double dot = 0.0, a = 0.0, b = 0.0;
                for (uint32_t j = 0; j < joints; ++j) {
                    dot += before[j] * after[j]; a += before[j] * before[j]; b += after[j] * after[j];
                }
                if (a > 0.0 && b > 0.0) turn += std::acos(std::clamp(dot / std::sqrt(a * b), -1.0, 1.0));
            }
        }
        before = after;
    }
    for (uint32_t j = 0; j < joints && straight; ++j) {
        const Small q = small_hermite(samples[span], samples[span + 1], j);
        straight = std::abs(q.c[2]) < 1e-12 && std::abs(q.c[3]) < 1e-12 &&
            std::abs(q.c[4]) < 1e-12 && std::abs(q.c[5]) < 1e-12;
    }
    SpanGrid grid;
    auto uniform = [&grid](size_t count) {
        grid.nodes.resize(count);
        for (size_t step = 0; step < count; ++step) grid.nodes[step] = static_cast<double>(step) / static_cast<double>(count);
        return grid;
    };
    if (reference) return uniform(MAX_STAGES_PER_SPAN);
    if (!moving) return uniform(MIN_STAGES_PER_SPAN);
    if (straight && std::isfinite(accelerating) && std::isfinite(fastest) && fastest > 0.0) {
        // The stretch where the speed changes, with room to spare: the profile may start slowing a little before the
        // distance the limits give (their margins are not the whole story), and a node must not come after it.
        const double exact = fastest * fastest / (2.0 * margin * accelerating);
        const double step = exact / static_cast<double>(ZONE_STAGES);
        const size_t zone_stages = static_cast<size_t>(std::lround(ZONE_REACH * static_cast<double>(ZONE_STAGES)));
        const double zone = step * static_cast<double>(zone_stages);
        if (length <= 2.0 * zone + step)
            return uniform(std::clamp(static_cast<size_t>(std::ceil(length / step)), MIN_STAGES_PER_SPAN, 2 * zone_stages + 1));
        for (size_t k = 0; k <= zone_stages; ++k) grid.nodes.push_back(step * static_cast<double>(k) / length);
        for (size_t k = 0; k < zone_stages; ++k) grid.nodes.push_back((length - zone + step * static_cast<double>(k)) / length);
        return grid;
    }
    const double profile = accelerating * length / (PROFILE_STEP * fastest * fastest);
    const double stages = std::max(profile, turn / MAX_STAGE_TURN);
    if (!std::isfinite(stages)) return uniform(MAX_STAGES_PER_SPAN);
    return uniform(static_cast<size_t>(std::clamp(std::ceil(stages),
        static_cast<double>(MIN_STAGES_PER_SPAN), static_cast<double>(MAX_STAGES_PER_SPAN))));
}

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

// Builds the integer-nanosecond law for a reference profile: path positions
// `s` with path speeds `v` at them, each reference stage at constant
// acceleration, so stage j lasts exactly 2*(s[j+1]-s[j])/(v[j]+v[j+1]).
//
// Only knot times are rounded, each from the exact time since its span's
// start, so rounding never compounds; every knot keeps its reference speed, so a law
// stage's acceleration is its reference stage's, off by at most a nanosecond
// over the stage's duration (relative). Deriving acceleration from a rounded
// duration instead turns 1 ns on a 1 us stage of a micrometre corner span
// into tens of m/s^2. A reference stage shorter than the rounding merges into
// the next.
//
// Anchors (the path samples, which lowering needs at knots) are reached
// exactly at their knots. Rounding the span's duration up leaves its stages
// up to a nanosecond of travel too far; that is taken off by scaling the
// interior knot speeds by 1 - c*t*(L-t), a parabola in time that vanishes at
// both anchors (least acceleration for a nearly constant speed: at most about
// 6*v*(1 ns)/L^2, about 1% of a printer's limit on a 1.5 um span) and, being
// relative, never reverses a knot at or near rest.
bool stages_from_profile(const std::vector<double> &s, const std::vector<double> &v,
                         const std::vector<char> &anchor,
                         std::vector<mk_time_stage> &stages) {
    if (s.size() < 2 || v.size() != s.size() || anchor.size() != s.size()) return false;
    const size_t n = s.size() - 1;
    std::vector<double> times(n + 1, 0.0);
    for (size_t j = 0; j < n; ++j) {
        const double ds = s[j + 1] - s[j];
        const double duration = 2.0 * ds / (std::max(0.0, v[j]) + std::max(0.0, v[j + 1]));
        if (!(ds > 0.0) || !std::isfinite(duration) || duration <= 0.0 || v[j] < 0.0)
            return false;
        times[j + 1] = times[j] + duration;
    }
    if (!(times[n] < static_cast<double>(INT64_MAX / 2) * 1e-9) || v[n] < 0.0) return false;
    stages.clear();
    stages.reserve(n);
    int64_t start_ns = 0;
    double speed = v.front();
    std::vector<int64_t> knots;
    std::vector<double> speeds;
    for (size_t a = 0; a < n;) {
        size_t b = a + 1;
        while (b < n && !anchor[b]) ++b;
        // The span's knots: its anchors and each interior reference knot
        // that rounds strictly between them.
        // The span's duration rounds up, so making up its travel only ever
        // slows the reference (never past a bound it holds); the law's clock
        // falls behind the reference's by under a nanosecond a span.
        const int64_t end_ns = start_ns +
            std::max<int64_t>(static_cast<int64_t>(std::ceil((times[b] - times[a]) * 1e9)), 1);
        knots.assign(1, start_ns);
        speeds.assign(1, speed);
        for (size_t j = a + 1; j < b; ++j) {
            const int64_t knot = start_ns + std::llround((times[j] - times[a]) * 1e9);
            if (knot > knots.back() && knot < end_ns) {
                knots.push_back(knot);
                speeds.push_back(v[j]);
            }
        }
        knots.push_back(end_ns);
        speeds.push_back(v[b]);
        const size_t m = knots.size() - 1;
        if (m > 1) {
            const double length = static_cast<double>(end_ns - start_ns) * 1e-9;
            double travel = 0.0, weight = 0.0;
            std::vector<double> bump(m + 1);
            for (size_t k = 0; k <= m; ++k) {
                const double x = static_cast<double>(knots[k] - start_ns) * 1e-9;
                bump[k] = k == 0 || k == m ? 0.0 : x * (length - x) * speeds[k];
            }
            for (size_t k = 0; k < m; ++k) {
                const double duration = static_cast<double>(knots[k + 1] - knots[k]) * 1e-9;
                travel += 0.5 * (speeds[k] + speeds[k + 1]) * duration;
                weight += 0.5 * (bump[k] + bump[k + 1]) * duration;
            }
            // With every interior knot at rest there is nothing to adjust;
            // the last stage then takes up the difference.
            const double scale = weight > 0.0 ? (s[b] - s[a] - travel) / weight : 0.0;
            for (size_t k = 1; k < m; ++k) {
                speeds[k] += scale * bump[k];
                if (!(speeds[k] >= 0.0)) return false;
            }
        }
        double position = s[a];
        for (size_t k = 0; k < m; ++k) {
            const int64_t duration_ns = knots[k + 1] - knots[k];
            const double duration = static_cast<double>(duration_ns) * 1e-9;
            mk_time_stage stage{};
            stage.struct_size = sizeof(stage);
            stage.start_ns = knots[k];
            stage.duration_ns = duration_ns;
            stage.start_s = position;
            stage.speed = speed;
            // The span's last stage lands on its anchor exactly; its end speed
            // is the anchor's to rounding (or, for a one-stage span, whatever
            // reaching the anchor takes).
            stage.acceleration = k + 1 < m ? (speeds[k + 1] - speed) / duration
                : 2.0 * (s[b] - position - speed * duration) / (duration * duration);
            stages.push_back(stage);
            position = k + 1 < m ? end_s(stage) : s[b];
            speed = stage_speed(stage, knots[k + 1]);
            if (speed < 0.0) {
                if (speed < -1e-9 * std::max(1.0, v[b])) return false;
                speed = 0.0;
            }
        }
        start_ns = end_ns;
        a = b;
    }
    return true;
}

// Anchors the law's stage boundaries that are path samples.
std::vector<char> sample_anchors(const std::vector<mk_path_sample> &samples,
                                 const std::vector<double> &positions) {
    std::vector<double> sample_s;
    sample_s.reserve(samples.size());
    for (const auto &sample : samples) sample_s.push_back(sample.s);
    std::vector<char> anchor(positions.size(), 0);
    for (size_t i = 0; i < positions.size(); ++i)
        anchor[i] = std::binary_search(sample_s.begin(), sample_s.end(), positions[i]);
    if (!anchor.empty()) anchor.front() = anchor.back() = 1;
    return anchor;
}

// Rebuilds the stages over their own distances, from `start_speed`, each
// stage reaching its target end speed.
bool retime_stages(std::vector<mk_time_stage> &stages,
                   const std::vector<mk_path_sample> &samples, double start_speed,
                   const std::vector<double> &end_speeds) {
    std::vector<double> positions, speeds{start_speed};
    positions.reserve(stages.size() + 1);
    for (const auto &stage : stages) positions.push_back(stage.start_s);
    positions.push_back(end_s(stages.back()));
    // A law's speeds can dip below zero by rounding where it comes to rest.
    for (double speed : end_speeds) speeds.push_back(std::max(0.0, speed));
    speeds.front() = std::max(0.0, speeds.front());
    return stages_from_profile(positions, speeds, sample_anchors(samples, positions), stages);
}

// Slows the whole law down uniformly by `factor`: every speed divided by it.
bool stretch_stages(std::vector<mk_time_stage> &stages,
                    const std::vector<mk_path_sample> &samples, double factor) {
    if (stages.empty() || !(factor > 0.0)) return false;
    std::vector<double> ends;
    ends.reserve(stages.size());
    for (const auto &stage : stages)
        ends.push_back(stage_speed(stage, stage.start_ns + stage.duration_ns) / factor);
    return retime_stages(stages, samples, stages.front().speed / factor, ends);
}

// A collocation peak can exceed a joint bound after quintic lowering. Reduce
// path speed around that peak, preserving all other stages' time scale.
bool soften_stages(std::vector<mk_time_stage> &stages,
                   const std::vector<mk_path_sample> &samples, double peak_time,
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
    return retime_stages(stages, samples, speeds.front(),
        std::vector<double>(speeds.begin() + 1, speeds.end()));
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

mk_result MK_CALL mk_path_times_to_distances(mk_time_law_handle law, const double *seconds,
                                              uint32_t count, double *out_distances) {
    if ((count && (!seconds || !out_distances))) return MK_ERROR_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < count; ++i)
        if (!std::isfinite(seconds[i])) return MK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(mutex);
    const auto found = laws.find(law.id);
    if (found == laws.end()) return MK_ERROR_INVALID_HANDLE;
    for (uint32_t i = 0; i < count; ++i)
        out_distances[i] = distance_at(found->second.stages, seconds[i]);
    return MK_OK;
}

mk_result MK_CALL mk_path_lower(mk_path_handle path, mk_time_law_handle law,
                                 double tolerance, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory || !std::isfinite(tolerance) || tolerance <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    try {
        // Lowering runs on copies, so the lock covers only the lookups.
        std::vector<mk_path_sample> samples;
        std::vector<mk_time_stage> stages;
        {
            std::lock_guard lock(mutex);
            const auto p = paths.find(path.id);
            const auto l = laws.find(law.id);
            if (p == paths.end() || l == laws.end()) return MK_ERROR_INVALID_HANDLE;
            samples = p->second;
            stages = l->second.stages;
        }
        if (std::abs(samples.front().s - stages.front().start_s) > 1e-10 ||
            std::abs(samples.back().s - end_s(stages.back())) > 1e-8)
            return MK_ERROR_INVALID_ARGUMENT;
        mk_trajectory_handle trajectory{};
        auto cache = std::make_unique<SpanCache>();
        auto result = mk_trajectory_create(samples.front().joint_count, &trajectory);
        if (result != MK_OK) return result;
        for (const auto &stage : stages) {
            std::vector<int64_t> knots{stage.start_ns,
                stage.start_ns + stage.duration_ns};
            const double stage_end = end_s(stage);
            for (auto at = std::upper_bound(samples.begin(), samples.end(), stage.start_s,
                     [](double value, const mk_path_sample &sample) { return value < sample.s; });
                 at != samples.end() && at->s < stage_end; ++at) {
                const auto &sample = *at;
                double seconds = 0.0;
                if (!inverse_time(stages, sample.s, seconds)) {
                    mk_trajectory_destroy(trajectory);
                    return MK_ERROR_INVALID_ARGUMENT;
                }
                knots.push_back(static_cast<int64_t>(std::llround(seconds * 1e9)));
            }
            std::sort(knots.begin(), knots.end());
            knots.erase(std::unique(knots.begin(), knots.end()), knots.end());
            for (size_t i = 1; i < knots.size(); ++i) {
                if (!append_interval(samples, *cache, stage, knots[i - 1], knots[i],
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

namespace {
mk_result time_path(mk_path_handle path, const double *max_velocity,
                    const double *max_acceleration, uint32_t joint_count,
                    const double *speed_caps, uint32_t speed_cap_count,
                    double start_speed, double end_speed,
                    double lowering_tolerance, uint32_t require_feasible_path,
                    mk_timing_report *out_report,
                    mk_time_law_handle *out_law,
                    mk_trajectory_handle *out_trajectory, bool reference) {
    if (!out_law || !out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_law->id = 0;
    out_trajectory->id = 0;
    if(out_report){*out_report={};out_report->struct_size=sizeof(*out_report);out_report->joint=UINT32_MAX;out_report->sustainable_scale=1;}
    if(require_feasible_path>1)return MK_ERROR_INVALID_ARGUMENT;
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
        // Scale each joint by its velocity limit so rotary shafts and linear
        // carriages have comparable LP coefficients. The time law remains in
        // the original path coordinate and the final validation uses SI units.
        std::vector<double> coordinate_scale(joint_count);
        for (uint32_t j = 0; j < joint_count; ++j)
            coordinate_scale[j] = 1.0 / max_velocity[j];
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
                        p[k] * coordinate_scale[j] /
                        std::pow(ds, static_cast<int>(k));
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
                    std::abs(samples[span + 1].second_before[joint]) > 1e-10 ||
                    std::abs(samples[span].first[joint] - samples[span + 1].first[joint]) > 1e-10) {
                    nonlinear = true;
                    break;
                }
        // Collocation can miss acceleration peaks between samples on a curved
        // path. Reserve room for those peaks; final validation still uses the
        // caller's exact acceleration limit.
        const double interpolation_margin = nonlinear ? 0.8 : 1.0;
        // Stages per span, and the grid they make: each span's first point is a path sample (an anchor).
        std::vector<size_t> span_first(samples.size(), 0);
        std::vector<uint32_t> stage_span;
        std::vector<double> grid_values;
        for (size_t span = 0; span + 1 < samples.size(); ++span) {
            span_first[span] = stage_span.size();
            const auto span_grid = nodes_for_span(samples, span, max_velocity, max_acceleration, joint_count,
                speed_cap_count ? speed_caps[span] : 0.0, interpolation_margin, reference);
            const double begin = samples[span].s, length = samples[span + 1].s - begin;
            for (const double node : span_grid.nodes) {
                stage_span.push_back(static_cast<uint32_t>(span));
                grid_values.push_back(begin + length * node);
            }
        }
        span_first.back() = stage_span.size();
        grid_values.push_back(samples.back().s);
        toppra::Vector grid = Eigen::Map<const toppra::Vector>(grid_values.data(), static_cast<Eigen::Index>(grid_values.size()));
        const double moving_boundary_margin = (start_speed > 0.0 || end_speed > 0.0)
            ? 0.98 : 1.0;
        toppra::Vector lower_velocity(dof), upper_velocity(dof);
        toppra::Vector lower_acceleration(dof), upper_acceleration(dof);
        for (uint32_t j = 0; j < joint_count; ++j) {
            lower_velocity[j] = -max_velocity[j] * coordinate_scale[j] * moving_boundary_margin;
            upper_velocity[j] = max_velocity[j] * coordinate_scale[j] * moving_boundary_margin;
            lower_acceleration[j] = -max_acceleration[j] * coordinate_scale[j] *
                moving_boundary_margin * interpolation_margin;
            upper_acceleration[j] = max_acceleration[j] * coordinate_scale[j] *
                moving_boundary_margin * interpolation_margin;
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
        // Full controllable intervals matter: a following window may require a minimum entry speed to reach
        // a moving end, and a maximum computed for one fixed exit is not a valid bound for every slower exit.
        std::vector<size_t> window_first{0};
        const size_t span_count = samples.size() - 1;
        for (size_t span = 1; span < span_count && !reference; ++span)
            if (span_first[span] - span_first[window_first.back()] >= WINDOW_NODES &&
                span_first[span_count] - span_first[span] >= WINDOW_NODES / 4) window_first.push_back(span);
        auto make_window = [&](size_t window) {
            const size_t first_node = span_first[window_first[window]];
            const size_t last_node = window + 1 < window_first.size() ? span_first[window_first[window + 1]]
                : grid_values.size() - 1;
            toppra::Vector window_grid = Eigen::Map<const toppra::Vector>(grid_values.data() + first_node,
                static_cast<Eigen::Index>(last_node - first_node + 1));
            std::shared_ptr<toppra::GeometricPath> window_path = geometric;
            if (window_first.size() > 1) {
                const size_t first_span = window_first[window];
                const size_t after_span = window + 1 < window_first.size() ? window_first[window + 1] : span_count;
                toppra::Matrices window_coefficients(coefficients.begin() + static_cast<std::ptrdiff_t>(first_span),
                    coefficients.begin() + static_cast<std::ptrdiff_t>(after_span));
                std::vector<double> window_knots(knots.begin() + static_cast<std::ptrdiff_t>(first_span),
                    knots.begin() + static_cast<std::ptrdiff_t>(after_span) + 1);
                window_path = std::make_shared<toppra::PiecewisePolyPath>(window_coefficients, window_knots);
            }
            auto algorithm = std::make_unique<toppra::algorithm::TOPPRA>(constraints, window_path);
            algorithm->solver(std::make_shared<toppra::solver::Seidel>());
            algorithm->setGridpoints(window_grid);
            algorithm->setInitialXBounds(toppra::Bound{0.0, 1e16});
            return algorithm;
        };
        if (out_report) {
            out_report->windows = static_cast<uint32_t>(window_first.size());
            out_report->boundary_lowerings = 0;
        }
        std::vector<std::unique_ptr<toppra::algorithm::TOPPRA>> retained(window_first.size());
        std::vector<toppra::Bound> controllable(window_first.size() + 1);
        controllable.back().setConstant(end_speed * end_speed);
        // The first window computes its backward pass in the forward call below. Propagate only later
        // windows here, retaining compact backward results but releasing constraint tables.
        // Solver workspace stays bounded by one window in both sweeps.
        if (window_first.size() > 1) for (size_t window = window_first.size() - 1; window > 0; --window) {
            auto algorithm = make_window(window);
            const auto result = algorithm->computeControllableSetsFromSquaredSpeed(controllable[window + 1]);
            if (result != toppra::ReturnCode::OK) {
                if (std::getenv("PROCESS_PATH_PROFILE"))
                    std::fprintf(stderr, "PROCESS_PATH_WINDOW window=%zu result=%d %s\n", window,
                        static_cast<int>(result), algorithm->getErrorMessage().c_str());
                return MK_ERROR_GENERATION;
            }
            controllable[window] = algorithm->getParameterizationData().controllable_sets.row(0);
            algorithm->releaseSolverWorkspace();
            retained[window] = std::move(algorithm);
        }
        std::vector<double> speeds(grid_values.size(), 0.0);
        double reached_squared = start_speed * start_speed;
        for (size_t window = 0; window < window_first.size(); ++window) {
            auto algorithm = window == 0 ? make_window(window) : std::move(retained[window]);
            const auto result = window == 0
                ? algorithm->computePathParametrizationFromSquaredSpeed(reached_squared, controllable[window + 1])
                : algorithm->computeForwardFromControllableSets(reached_squared, std::make_shared<toppra::solver::Seidel>());
            if (result != toppra::ReturnCode::OK) {
                if (std::getenv("PROCESS_PATH_PROFILE"))
                    std::fprintf(stderr, "PROCESS_PATH_WINDOW window=%zu result=%d %s\n", window,
                        static_cast<int>(result), algorithm->getErrorMessage().c_str());
                return MK_ERROR_GENERATION;
            }
            const auto &data = algorithm->getParameterizationData();
            const size_t first_node = span_first[window_first[window]];
            const size_t last_node = window + 1 < window_first.size() ? span_first[window_first[window + 1]]
                : grid_values.size() - 1;
            for (size_t node = first_node; node <= last_node; ++node)
                speeds[node] = std::sqrt(std::max(0.0, data.parametrization[static_cast<Eigen::Index>(node - first_node)]));
            reached_squared = data.parametrization[data.parametrization.size() - 1];
        }
        speeds.front() = start_speed;
        speeds.back() = end_speed;
        std::vector<mk_time_stage> stages;
        std::vector<double> positions(grid.data(), grid.data() + grid.size());
        std::vector<char> anchors(positions.size(), 0);
        for (size_t span = 0; span + 1 < samples.size(); ++span) anchors[span_first[span]] = 1;
        anchors.back() = 1;
        if (!stages_from_profile(positions, speeds, anchors, stages)) return MK_ERROR_GENERATION;
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
                limits.derivative_claimed[j] = 3u;
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
            if(out_report){
                out_report->adjustments=static_cast<uint32_t>(attempt);
                if(factor>1){
                    const bool acceleration=acceleration_check.status==MK_CHECK_FAILED &&
                        (velocity_check.status!=MK_CHECK_FAILED || std::sqrt(acceleration_check.value/acceleration_check.limit)>=velocity_check.value/velocity_check.limit);
                    const auto &binding=acceleration?acceleration_check:velocity_check;
                    out_report->joint=binding.joint;out_report->derivative=acceleration?2u:1u;
                    out_report->value=binding.value;out_report->limit=binding.limit;out_report->sustainable_scale=1/factor;
                }else {out_report->derivative=0;out_report->joint=UINT32_MAX;out_report->sustainable_scale=1;}
            }
            const char *profile = std::getenv("PROCESS_PATH_PROFILE");
            if (profile && std::strcmp(profile, "1") == 0) {
                const auto &last = stages.back();
                std::fprintf(stderr, "PROCESS_PATH_TIMING_CHECK {\"attempt\":%d,\"stages\":%zu,\"durationSeconds\":%.12g,\"factor\":%.12g,\"velocityJoint\":%u,\"velocityValue\":%.12g,\"velocityLimit\":%.12g,\"accelerationJoint\":%u,\"accelerationValue\":%.12g,\"accelerationLimit\":%.12g,\"accelerationTime\":%.12g}\n",
                    attempt, stages.size(), (last.start_ns + last.duration_ns) * 1e-9,
                    factor, velocity_check.joint, velocity_check.value, velocity_check.limit,
                    acceleration_check.joint, acceleration_check.value,
                    acceleration_check.limit, acceleration_check.time_seconds);
            }
            if (factor <= 1.0) {
                accepted = true;
                *out_trajectory = lowered;
                break;
            }
            mk_trajectory_destroy(lowered);
            mk_time_law_destroy(created);
            if(require_feasible_path)return MK_ERROR_GENERATION;
            // Uniform stretching would change an authored moving endpoint
            // speed, so report infeasibility instead of returning a law with
            // a different boundary contract.
            if (start_speed > 0.0 || end_speed > 0.0) return MK_ERROR_GENERATION;
            // Both boundaries are at rest here (checked above).
            if (stages.size() > 1000 &&
                acceleration_check.status == MK_CHECK_FAILED && attempt < 20 &&
                soften_stages(stages, samples, acceleration_check.time_seconds, factor * 1.002))
                continue;
            if (!stretch_stages(stages, samples, factor * 1.002)) return MK_ERROR_GENERATION;
        }
        if (!accepted) return MK_ERROR_GENERATION;
        {
            // Bindings name TOPP-RA's own stages, with its own control.
            auto shared_samples = std::make_shared<const std::vector<mk_path_sample>>(std::move(samples));
            std::vector<double> velocity_limits(max_velocity, max_velocity + joint_count);
            std::vector<double> acceleration_limits(max_acceleration, max_acceleration + joint_count);
            auto make = [shared_samples, positions = std::move(positions), speeds = std::move(speeds),
                         stage_span = std::move(stage_span), caps = std::move(caps),
                         velocity_limits = std::move(velocity_limits),
                         acceleration_limits = std::move(acceleration_limits)]() {
                std::vector<mk_timing_binding> bindings;
                bindings.reserve(positions.size() - 1);
                for (size_t i = 0; i + 1 < positions.size(); ++i) {
                    const double ds = positions[i + 1] - positions[i];
                    const double path_acceleration =
                        (speeds[i + 1] * speeds[i + 1] - speeds[i] * speeds[i]) / (2.0 * ds);
                    const auto span = stage_span[i];
                    bindings.push_back(binding_for_stage(*shared_samples, static_cast<uint32_t>(span),
                        static_cast<uint32_t>(i), 0.5 * (positions[i] + positions[i + 1]),
                        std::max(0.0, speeds[i] * speeds[i] + path_acceleration * ds),
                        path_acceleration, velocity_limits.data(), acceleration_limits.data(), caps[span]));
                }
                return bindings;
            };
            std::lock_guard lock(mutex);
            laws.at(created.id).make_bindings = std::move(make);
        }
        *out_law = created;
        return MK_OK;
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}
} // namespace

mk_result MK_CALL mk_time_path(mk_path_handle path, const double *max_velocity,
                               const double *max_acceleration, uint32_t joint_count,
                               const double *speed_caps, uint32_t speed_cap_count,
                               double start_speed, double end_speed,
                               double lowering_tolerance, uint32_t require_feasible_path,
                               mk_timing_report *out_report,
                               mk_time_law_handle *out_law,
                               mk_trajectory_handle *out_trajectory) {
    return time_path(path, max_velocity, max_acceleration, joint_count, speed_caps, speed_cap_count, start_speed,
        end_speed, lowering_tolerance, require_feasible_path, out_report, out_law, out_trajectory, false);
}

mk_result MK_CALL mk_time_path_reference(mk_path_handle path, const double *max_velocity,
                                         const double *max_acceleration, uint32_t joint_count,
                                         const double *speed_caps, uint32_t speed_cap_count,
                                         double start_speed, double end_speed,
                                         double lowering_tolerance, uint32_t require_feasible_path,
                                         mk_timing_report *out_report,
                                         mk_time_law_handle *out_law,
                                         mk_trajectory_handle *out_trajectory) {
    return time_path(path, max_velocity, max_acceleration, joint_count, speed_caps, speed_cap_count, start_speed,
        end_speed, lowering_tolerance, require_feasible_path, out_report, out_law, out_trajectory, true);
}

mk_result MK_CALL mk_project_couplings(mk_trajectory_handle trajectory,
                                       const uint32_t *followers, const uint32_t *leaders,
                                       const double *ratios, const double *offsets,
                                       uint32_t term_count, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    if (term_count && (!followers || !leaders || !ratios || !offsets)) return MK_ERROR_INVALID_ARGUMENT;
    uint32_t joints = 0, segments = 0;
    auto status = mk_trajectory_joint_count(trajectory, &joints);
    if (status != MK_OK) return status;
    status = mk_trajectory_segment_count(trajectory, &segments);
    if (status != MK_OK) return status;
    for (uint32_t term = 0; term < term_count; ++term)
        if (followers[term] >= joints || leaders[term] >= joints || !std::isfinite(ratios[term]) ||
            !std::isfinite(offsets[term])) return MK_ERROR_INVALID_ARGUMENT;
    mk_trajectory_handle projected{};
    status = mk_trajectory_create(joints, &projected);
    if (status != MK_OK) return status;
    try {
        auto segment = std::make_unique<mk_segment>();
        for (uint32_t index = 0; index < segments; ++index) {
            *segment = mk_segment{};
            segment->struct_size = sizeof(mk_segment);
            status = mk_trajectory_get_segment(trajectory, index, segment.get());
            if (status != MK_OK) { mk_trajectory_destroy(projected); return status; }
            // Followers start from nothing; terms are in dependency order, so a leader that is itself a
            // follower is complete before it is used.
            for (uint32_t term = 0; term < term_count; ++term) {
                auto &target = segment->coefficients[followers[term]];
                if (term == 0 || followers[term] != followers[term - 1])
                    for (int degree = 0; degree <= MK_MAX_DEGREE; ++degree) target.value[degree] = 0.0;
            }
            for (uint32_t term = 0; term < term_count; ++term) {
                auto &target = segment->coefficients[followers[term]];
                const auto &source = segment->coefficients[leaders[term]];
                for (int degree = 0; degree <= static_cast<int>(segment->degree); ++degree)
                    target.value[degree] += ratios[term] * source.value[degree] + (degree == 0 ? offsets[term] : 0.0);
            }
            status = mk_trajectory_append_segment(projected, segment.get());
            if (status != MK_OK) { mk_trajectory_destroy(projected); return status; }
        }
    } catch (const std::bad_alloc &) { mk_trajectory_destroy(projected); return MK_ERROR_OUT_OF_MEMORY; }
    *out_trajectory = projected;
    return MK_OK;
}

mk_result MK_CALL mk_advance_joint(mk_trajectory_handle trajectory, uint32_t joint, double lead_seconds,
                                   double window_seconds, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    if (!std::isfinite(lead_seconds) || lead_seconds < 0.0 || !std::isfinite(window_seconds) || window_seconds <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    uint32_t joints = 0, segments = 0;
    int64_t duration_ns = 0;
    auto status = mk_trajectory_joint_count(trajectory, &joints);
    if (status != MK_OK) return status;
    if (joint >= joints) return MK_ERROR_INVALID_ARGUMENT;
    status = mk_trajectory_segment_count(trajectory, &segments);
    if (status != MK_OK) return status;
    status = mk_trajectory_duration_ns(trajectory, &duration_ns);
    if (status != MK_OK) return status;
    const int64_t half_ns = std::max<int64_t>(1, static_cast<int64_t>(std::llround(window_seconds * 0.5e9)));
    const double window = static_cast<double>(2 * half_ns) * 1e-9;
    // The joint's position, velocity and acceleration at `t`, the trajectory held at rest beyond either end.
    auto state_at = [&](int64_t t, double &position, double &velocity, double &acceleration) -> mk_result {
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        const int64_t clamped = std::clamp<int64_t>(t, 0, duration_ns);
        const auto result = mk_trajectory_evaluate(trajectory, clamped, &state);
        if (result != MK_OK) return result;
        const bool outside = t < 0 || t > duration_ns;
        position = state.position[joint];
        velocity = outside ? 0.0 : state.velocity[joint];
        acceleration = outside ? 0.0 : state.acceleration[joint];
        return MK_OK;
    };
    // The taper and its first two derivatives at `t`: the product of a rise from the start and a fall to the end, each
    // zero with both derivatives at its end and one from a window in, so a plan shorter than two windows is smooth too.
    auto smoother = [&](double x, double &value, double &first, double &second) {
        if (x >= 1.0) { value = 1.0; first = 0.0; second = 0.0; return; }
        x = std::max(x, 0.0);
        value = x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
        first = 30.0 * x * x * (1.0 - x) * (1.0 - x) / window;
        second = 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x) / (window * window);
    };
    auto taper = [&](int64_t t, double &value, double &first, double &second) {
        double u = 0, du = 0, ddu = 0, v = 0, dv = 0, ddv = 0;
        smoother(static_cast<double>(t) * 1e-9 / window, u, du, ddu);
        smoother(static_cast<double>(duration_ns - t) * 1e-9 / window, v, dv, ddv);
        dv = -dv;  // the fall runs backwards in time
        value = u * v;
        first = du * v + u * dv;
        second = ddu * v + 2.0 * du * dv + u * ddv;
    };
    // The corrected position, velocity and acceleration at `t`: p + lead * mean velocity * taper.
    auto corrected = [&](int64_t t, double &position, double &velocity, double &acceleration) -> mk_result {
        double p = 0, v = 0, a = 0, pa = 0, va = 0, aa = 0, pb = 0, vb = 0, ab = 0;
        auto result = state_at(t, p, v, a);
        if (result == MK_OK) result = state_at(t + half_ns, pa, va, aa);
        if (result == MK_OK) result = state_at(t - half_ns, pb, vb, ab);
        if (result != MK_OK) return result;
        const double mean = (pa - pb) / window, mean_rate = (va - vb) / window, mean_slope = (aa - ab) / window;
        double w = 0, dw = 0, ddw = 0;
        taper(t, w, dw, ddw);
        position = p + lead_seconds * mean * w;
        velocity = v + lead_seconds * (mean_rate * w + mean * dw);
        acceleration = a + lead_seconds * (mean_slope * w + 2.0 * mean_rate * dw + mean * ddw);
        return MK_OK;
    };
    // The advance changes form wherever the window's ends cross a boundary of the plan's segments, and at the taper's
    // ends; between those instants it is a polynomial (of degree five, or nine under the taper). So the trajectory is cut
    // at every segment boundary, each boundary half a window either side, and the taper's ends (the tapers cut eight
    // times more), and each piece of the joint is the quintic through the advance's position, velocity and acceleration
    // at its ends: exact where the advance is a quintic, and within a piece's rounding under the taper. The other joints
    // keep their polynomials, re-expressed from the start of each piece.
    std::vector<mk_segment> source;
    try {
        source.resize(segments);
        for (uint32_t index = 0; index < segments; ++index) {
            source[index] = mk_segment{};
            source[index].struct_size = sizeof(mk_segment);
            status = mk_trajectory_get_segment(trajectory, index, &source[index]);
            if (status != MK_OK) return status;
        }
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
    std::vector<int64_t> cuts;
    const int64_t window_ns = 2 * half_ns;
    auto cut = [&](int64_t at) { if (at > 0 && at < duration_ns) cuts.push_back(at); };
    for (const auto &piece : source) {
        for (const int64_t boundary : {piece.t0_ns, piece.t0_ns + piece.duration_ns}) {
            cut(boundary); cut(boundary - half_ns); cut(boundary + half_ns);
        }
    }
    for (int k = 1; k <= 8; ++k) {
        cut(window_ns * k / 8);
        cut(duration_ns - window_ns * k / 8);
    }
    cuts.push_back(0);
    cuts.push_back(duration_ns);
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    mk_trajectory_handle advanced{};
    status = mk_trajectory_create(joints, &advanced);
    if (status != MK_OK) return status;
    try {
        auto piece = std::make_unique<mk_segment>();
        size_t from = 0;
        double position_a = 0, velocity_a = 0, acceleration_a = 0;
        status = corrected(0, position_a, velocity_a, acceleration_a);
        for (size_t index = 0; index + 1 < cuts.size() && status == MK_OK; ++index) {
            const int64_t begin = cuts[index], finish = cuts[index + 1];
            // The plan's segment the piece lies in.
            while (from + 1 < source.size() && source[from].t0_ns + source[from].duration_ns <= begin) ++from;
            const auto &original = source[from];
            *piece = mk_segment{};
            piece->struct_size = sizeof(mk_segment);
            piece->t0_ns = begin;
            piece->duration_ns = finish - begin;
            piece->joint_count = original.joint_count;
            piece->degree = 5;
            // p(shift + tau) = sum_k c_k (shift + tau)^k, re-expanded in tau.
            const double shift = static_cast<double>(begin - original.t0_ns) * 1e-9;
            for (uint32_t joint_index = 0; joint_index < original.joint_count; ++joint_index) {
                const auto &c = original.coefficients[joint_index].value;
                auto &out = piece->coefficients[joint_index].value;
                for (int m = 0; m <= MK_MAX_DEGREE; ++m) {
                    double sum = 0.0, binomial = 1.0, power = 1.0;
                    for (int k = m; k <= static_cast<int>(original.degree); ++k) {
                        sum += c[k] * binomial * power;
                        binomial = binomial * static_cast<double>(k + 1) / static_cast<double>(k + 1 - m);
                        power *= shift;
                    }
                    out[m] = sum;
                }
            }
            double position_b = 0, velocity_b = 0, acceleration_b = 0;
            status = corrected(finish, position_b, velocity_b, acceleration_b);
            if (status != MK_OK) break;
            // The quintic through both ends' position, velocity and acceleration.
            const double h = static_cast<double>(piece->duration_ns) * 1e-9;
            const double d = position_b - position_a - velocity_a * h - 0.5 * acceleration_a * h * h;
            const double e = velocity_b - velocity_a - acceleration_a * h;
            const double f = acceleration_b - acceleration_a;
            auto &c = piece->coefficients[joint].value;
            for (auto &value : c) value = 0.0;
            c[0] = position_a;
            c[1] = velocity_a;
            c[2] = 0.5 * acceleration_a;
            c[3] = (10.0 * d - 4.0 * e * h + 0.5 * f * h * h) / (h * h * h);
            c[4] = (-15.0 * d + 7.0 * e * h - f * h * h) / (h * h * h * h);
            c[5] = (6.0 * d - 3.0 * e * h + 0.5 * f * h * h) / (h * h * h * h * h);
            status = mk_trajectory_append_segment(advanced, piece.get());
            position_a = position_b; velocity_a = velocity_b; acceleration_a = acceleration_b;
        }
        if (status != MK_OK) { mk_trajectory_destroy(advanced); return status; }
    } catch (const std::bad_alloc &) { mk_trajectory_destroy(advanced); return MK_ERROR_OUT_OF_MEMORY; }
    *out_trajectory = advanced;
    return MK_OK;
}

mk_result MK_CALL mk_merge_segments(mk_trajectory_handle trajectory, const double *position_tolerance,
                                    const double *velocity_tolerance, const double *acceleration_tolerance,
                                    const double *velocity_limit, const double *acceleration_limit,
                                    const double *position_lower, const double *position_upper,
                                    uint32_t joint_count, double max_duration_seconds,
                                    mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    uint32_t joints = 0;
    const auto status = mk_trajectory_joint_count(trajectory, &joints);
    if (status != MK_OK) return status;
    if (joints != joint_count) return MK_ERROR_INVALID_ARGUMENT;
    return motionkit::merge_trajectory(trajectory, {position_tolerance, velocity_tolerance, acceleration_tolerance,
        velocity_limit, acceleration_limit, position_lower, position_upper, max_duration_seconds}, out_trajectory);
}

mk_result MK_CALL mk_shape_trajectory(mk_trajectory_handle trajectory, const uint32_t *impulse_counts,
                                       const double *delays_seconds, const double *amplitudes, uint32_t joint_count,
                                       uint32_t table_size, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    if (!impulse_counts || !delays_seconds || !amplitudes || joint_count == 0 ||
        table_size != joint_count * static_cast<uint32_t>(MK_MAX_SHAPER_IMPULSES))
        return MK_ERROR_INVALID_ARGUMENT;
    constexpr uint32_t kSlots = MK_MAX_SHAPER_IMPULSES;
    std::vector<int64_t> delay_ns(table_size, 0);
    for (uint32_t j = 0; j < joint_count; ++j) {
        if (impulse_counts[j] == 0 || impulse_counts[j] > kSlots) return MK_ERROR_INVALID_ARGUMENT;
        double sum = 0.0;
        for (uint32_t i = 0; i < impulse_counts[j]; ++i) {
            const double delay = delays_seconds[j * kSlots + i], amplitude = amplitudes[j * kSlots + i];
            if (!std::isfinite(delay) || delay < 0.0 || delay > 60.0 || !std::isfinite(amplitude) || !(amplitude > 0.0))
                return MK_ERROR_INVALID_ARGUMENT;
            delay_ns[j * kSlots + i] = static_cast<int64_t>(std::llround(delay * 1e9));
            sum += amplitude;
        }
        if (std::abs(sum - 1.0) > 1e-6) return MK_ERROR_INVALID_ARGUMENT;
    }
    uint32_t joints = 0, count = 0;
    auto status = mk_trajectory_joint_count(trajectory, &joints);
    if (status != MK_OK) return status;
    if (joints != joint_count) return MK_ERROR_INVALID_ARGUMENT;
    status = mk_trajectory_segment_count(trajectory, &count);
    if (status != MK_OK) return status;
    if (count == 0) return MK_ERROR_INVALID_ARGUMENT;
    constexpr int kTerms = MK_MAX_DEGREE + 1;
    mk_trajectory_handle shaped{};
    status = mk_trajectory_create(joints, &shaped);
    if (status != MK_OK) return status;
    try {
        // The input in compact form: where each segment starts, how long it is, its degree and its coefficients.
        std::vector<int64_t> start(count), length(count);
        std::vector<uint8_t> degree(count);
        std::vector<double> coefficient(static_cast<size_t>(count) * joints * kTerms, 0.0);
        {
            auto segment = std::make_unique<mk_segment>();
            for (uint32_t s = 0; s < count; ++s) {
                *segment = mk_segment{};
                segment->struct_size = sizeof(mk_segment);
                status = mk_trajectory_get_segment(trajectory, s, segment.get());
                if (status != MK_OK) { mk_trajectory_destroy(shaped); return status; }
                start[s] = segment->t0_ns;
                length[s] = segment->duration_ns;
                degree[s] = static_cast<uint8_t>(segment->degree);
                for (uint32_t j = 0; j < joints; ++j)
                    for (int k = 0; k < kTerms; ++k)
                        coefficient[(static_cast<size_t>(s) * joints + j) * kTerms + k] = segment->coefficients[j].value[k];
            }
        }
        const int64_t origin = start[0], finish = start[count - 1] + length[count - 1];
        auto coefficients_of = [&](uint32_t s, uint32_t j) { return &coefficient[(static_cast<size_t>(s) * joints + j) * kTerms]; };
        // A joint's position at the end of the last segment: where it is held afterwards.
        std::vector<double> first_position(joints), last_position(joints);
        for (uint32_t j = 0; j < joints; ++j) {
            first_position[j] = coefficients_of(0, j)[0];
            const double tau = static_cast<double>(length[count - 1]) * 1e-9;
            double p = 0.0;
            for (int k = kTerms - 1; k >= 0; --k) p = p * tau + coefficients_of(count - 1, j)[k];
            last_position[j] = p;
        }
        // Every instant at which some delayed copy of some joint changes segment, the start and the end of the result.
        std::vector<int64_t> distinct;
        for (uint32_t j = 0; j < joints; ++j)
            for (uint32_t i = 0; i < impulse_counts[j]; ++i) {
                const auto offset = delay_ns[j * kSlots + i];
                if (std::find(distinct.begin(), distinct.end(), offset) == distinct.end()) distinct.push_back(offset);
            }
        int64_t longest = 0;
        for (const auto offset : distinct) longest = std::max(longest, offset);
        std::vector<int64_t> cuts;
        cuts.reserve((static_cast<size_t>(count) + 1) * distinct.size() + 2);
        cuts.push_back(origin);
        cuts.push_back(finish + longest);
        for (const auto offset : distinct) {
            for (uint32_t s = 0; s < count; ++s) cuts.push_back(start[s] + offset);
            cuts.push_back(finish + offset);
        }
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
        // Index of the segment holding instant `t` (start[s] <= t < start[s] + length[s]), found from a moving guess.
        auto segment_at = [&](int64_t t) {
            const auto found = std::upper_bound(start.begin(), start.end(), t);
            return static_cast<uint32_t>(found - start.begin()) - 1;
        };
        double binomial[kTerms][kTerms] = {};
        for (int m = 0; m < kTerms; ++m) {
            binomial[m][0] = 1.0;
            for (int k = 1; k <= m; ++k) binomial[m][k] = binomial[m - 1][k - 1] + (k <= m - 1 ? binomial[m - 1][k] : 0.0);
        }
        auto output = std::make_unique<mk_segment>();
        for (size_t c = 0; c + 1 < cuts.size(); ++c) {
            const int64_t u0 = cuts[c], u1 = cuts[c + 1];
            *output = mk_segment{};
            output->struct_size = sizeof(mk_segment);
            output->t0_ns = u0;
            output->duration_ns = u1 - u0;
            uint32_t used_degree = 0;
            for (uint32_t j = 0; j < joints; ++j) {
                double sum[kTerms] = {};
                for (uint32_t i = 0; i < impulse_counts[j]; ++i) {
                    const double amplitude = amplitudes[j * kSlots + i];
                    const int64_t begin = u0 - delay_ns[j * kSlots + i];
                    // The copy is in the segment it spends the interval in (its middle says which, for a cut that was dropped
                    // as too close leaves a change of segment a hair inside the interval).
                    const int64_t middle = begin + (u1 - u0) / 2;
                    if (middle < origin) { sum[0] += amplitude * first_position[j]; continue; }
                    if (middle >= finish) { sum[0] += amplitude * last_position[j]; continue; }
                    const uint32_t s = segment_at(middle);
                    used_degree = std::max<uint32_t>(used_degree, degree[s]);
                    // The segment's polynomial in the time since `u0`: its coefficients moved by `shift` seconds.
                    const double shift = static_cast<double>(begin - start[s]) * 1e-9;
                    const double *source = coefficients_of(s, j);
                    for (int k = 0; k <= static_cast<int>(degree[s]); ++k) {
                        double moved = 0.0, power = 1.0;
                        for (int m = k; m <= static_cast<int>(degree[s]); ++m) {
                            moved += binomial[m][k] * source[m] * power;
                            power *= shift;
                        }
                        sum[k] += amplitude * moved;
                    }
                }
                for (int k = 0; k < kTerms; ++k) output->coefficients[j].value[k] = sum[k];
            }
            output->joint_count = joints;
            output->degree = used_degree;
            status = mk_trajectory_append_segment(shaped, output.get());
            if (status != MK_OK) { mk_trajectory_destroy(shaped); return status; }
        }
    } catch (const std::bad_alloc &) { mk_trajectory_destroy(shaped); return MK_ERROR_OUT_OF_MEMORY; }
    *out_trajectory = shaped;
    return MK_OK;
}

mk_result MK_CALL mk_time_law_binding_count(mk_time_law_handle law,
                                             uint32_t *out_count) {
    if (!out_count) return MK_ERROR_INVALID_ARGUMENT;
    std::lock_guard lock(mutex);
    const auto found = laws.find(law.id);
    if (found == laws.end()) return MK_ERROR_INVALID_HANDLE;
    ensure_bindings(found->second);
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
    ensure_bindings(found->second);
    if (index >= found->second.bindings.size()) return MK_ERROR_INVALID_ARGUMENT;
    *out_binding = found->second.bindings[index];
    return MK_OK;
}

} // extern C
