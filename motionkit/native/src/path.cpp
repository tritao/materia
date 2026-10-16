#include "motionkit.h"
#include "trajectory_merge.hpp"
#include "retained_window_timing.hpp"
#include "window_timing_stream.hpp"
#include "clock_profile_cursor.hpp"
#include "planning_profile.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <system_error>
#include <unordered_map>
#include <vector>

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/solver/seidel.hpp>

namespace {
template<class Handle, auto Destroy>
struct NativeResultOwner {
    Handle value{};
    NativeResultOwner() = default;
    NativeResultOwner(const NativeResultOwner&) = delete;
    NativeResultOwner& operator=(const NativeResultOwner&) = delete;
    ~NativeResultOwner() { reset(); }
    void reset() { if (value.id) Destroy(value); value = {}; }
    Handle release() { const Handle result = value; value = {}; return result; }
};

using motionkit::PathProfilePhase;

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
    // Exact zero coefficients carry no geometry. Preserve every nonzero term,
    // including tiny process/rounding terms, while avoiding fictitious degree five work.
    while (q.n > 1 && q.c[q.n-1] == 0.0) --q.n;
    return q;
}

// `outer(inner(x))` by Horner's scheme, the same sums as `compose`, keeping the first `keep` coefficients.
Small small_compose(const Small &outer, const double inner[3], int keep) {
    Small result{};
    result.n = 1;
    if (outer.n <= 2) {
        result.c[0] = outer.c[0];
        if (outer.n == 2) {
            result.n = std::min(3,keep);
            result.c[0] = outer.c[1]*inner[0]+outer.c[0];
            for (int k=1;k<result.n;++k) result.c[k] = outer.c[1]*inner[k];
        }
        return result;
    }
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

// An authored sample needs its own integer-clock anchor when geometry or the
// feed limit changes. Consecutive spans representing exactly the same affine
// function share a clock region: forcing every redundant sample onto a separate
// nanosecond duration creates unnecessary acceleration corrections on tiny spans.
// All authored samples remain in the path and in lowering/binding lookup tables.
bool timing_anchor_required(const std::vector<mk_path_sample>& samples,
                            const std::vector<double>& caps, std::size_t index) {
    if (index == 0 || index + 1 == samples.size()) return true;
    if (caps[index-1] != caps[index]) return true;
    const double before = samples[index].s - samples[index-1].s;
    const double after = samples[index+1].s - samples[index].s;
    for (uint32_t joint = 0; joint < samples[index].joint_count; ++joint) {
        const Small left = small_hermite(samples[index-1], samples[index], joint);
        const Small right = small_hermite(samples[index], samples[index+1], joint);
        for (int power = 2; power < 6; ++power)
            if (left.c[power] != 0.0 || right.c[power] != 0.0) return true;
        if (left.c[1] / before != right.c[1] / after) return true;
    }
    return false;
}

bool append_interval(const std::vector<mk_path_sample> &path, SpanCache &cache,
                     const mk_time_stage &stage, int64_t t0, int64_t t1,
                     double tolerance, mk_trajectory_handle trajectory,
                     uint32_t depth, const mk_limits* physical_limits = nullptr) {
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
    bool approximated = false;
    for (uint32_t joint = 0; joint < a.joint_count; ++joint) {
        const Small &authored = cache.poly[joint];
        if (cache.exact[joint]) {
            const Small fit = small_compose(authored, normalized_s, 6);
            for (int i = 0; i < 6; ++i) segment.coefficients[joint].value[i] = fit.c[i] / duration_power[i];
            continue;
        }
        const Small composed = small_compose(authored, normalized_s, 13);
        double position_tail=0, velocity_tail=0, acceleration_tail=0;
        for (int i=6;i<composed.n;++i) {
            position_tail+=std::abs(composed.c[i]);
            velocity_tail+=i*std::abs(composed.c[i])/duration;
            acceleration_tail+=i*(i-1)*std::abs(composed.c[i])/(duration*duration);
        }
        // On a sub-tick geometric crossing, subtracting two absolute endpoint
        // positions to refit a quintic can amplify one ULP into a large false
        // acceleration. Use the directly composed polynomial when its omitted
        // tail is below representation roundoff in position AND both derivatives.
        const double roundoff=64*std::numeric_limits<double>::epsilon();
        if (position_tail<=roundoff*std::max(1.0,std::abs(composed.c[0])) &&
                velocity_tail<=roundoff*std::max(1.0,std::abs(composed.c[1]/duration)) &&
                acceleration_tail<=roundoff*std::max(1.0,std::abs(2*composed.c[2]/(duration*duration)))) {
            for (int i=0;i<6;++i) segment.coefficients[joint].value[i]=composed.c[i]/duration_power[i];
            worst=std::max(worst,position_tail);
            continue;
        }
        approximated = true;
        const auto left = small_state(authored, ds, (s0 - a.s) / ds);
        const auto right = small_state(authored, ds, (s1 - a.s) / ds);
        const Poly fit = hermite(left.position, left.first * v0 * duration,
            (left.second * v0 * v0 + left.first * stage.acceleration) * duration * duration,
            right.position, right.first * v1 * duration,
            (right.second * v1 * v1 + right.first * stage.acceleration) * duration * duration);
        for (size_t i = 0; i < fit.size(); ++i)
            segment.coefficients[joint].value[i] = fit[i] / duration_power[i];
        // The full composition above is independent of the endpoint fit.
        const Small &full = composed;
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
    // A geometric sample usually falls between integer ticks. Clamping a
    // fitted endpoint to that sample creates a position correction over a tiny
    // interval and an artificial acceleration spike. Instead extrapolate the
    // selected polynomial and certify against every actual span it overlaps.
    auto overlap=std::upper_bound(path.begin(),path.end(),s0,
        [](double value,const mk_path_sample& sample){return value<sample.s;});
    size_t other=overlap==path.begin() ? 0 : static_cast<size_t>(overlap-path.begin()-1);
    for (;other+1<path.size() && path[other].s<s1;++other) {
        if (other==index) continue;
        const double length=path[other+1].s-path[other].s;
        const double normalized[3]={(s0-path[other].s)/length,v0*duration/length,
            0.5*stage.acceleration*duration*duration/length};
        for (uint32_t joint=0;joint<a.joint_count;++joint) {
            const auto actual=small_compose(small_hermite(path[other],path[other+1],joint),normalized,13);
            double bound=0;
            for (int k=0;k<actual.n;++k)
                bound+=std::abs(actual.c[k]-(k<6 ? segment.coefficients[joint].value[k]*duration_power[k] : 0));
            worst=std::max(worst,bound);
        }
    }
    bool derivative_failure = false;
    if (physical_limits && approximated) {
        // A position-accurate Hermite fit can overshoot a saturated derivative.
        // Refine only these representations, using the same exact continuous
        // validator as the final trajectory, without changing any clock stage.
        NativeResultOwner<mk_trajectory_handle,mk_trajectory_destroy> candidate;
        auto local = segment; local.t0_ns = 0;
        if (mk_trajectory_create(segment.joint_count,&candidate.value)!=MK_OK ||
            mk_trajectory_append_segment(candidate.value,&local)!=MK_OK) return false;
        mk_validation_report report{}; report.struct_size = sizeof(report);
        if (mk_validate(candidate.value,physical_limits,&report)!=MK_OK) return false;
        derivative_failure = report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_FAILED ||
            report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_FAILED;
    }
    if (worst > tolerance || derivative_failure) {
        if (depth >= 40 || t1 - t0 < 2) return false;
        const int64_t middle = t0 + (t1 - t0) / 2;
        return append_interval(path, cache, stage, t0, middle, tolerance, trajectory, depth + 1, physical_limits) &&
            append_interval(path, cache, stage, middle, t1, tolerance, trajectory, depth + 1, physical_limits);
    }
    return mk_trajectory_append_segment(trajectory, &segment) == MK_OK;
}

struct PathSpeedCaps {
    std::vector<double> knots;
    std::vector<double> caps;
    std::vector<double> grid, continuous_caps;
};

class VelocityWithCaps final : public toppra::constraint::LinearJointVelocity {
public:
    VelocityWithCaps(const toppra::Vector &lower, const toppra::Vector &upper,
                     std::shared_ptr<const PathSpeedCaps> limits)
        : LinearJointVelocity(lower, upper), limits_(std::move(limits)) {}

protected:
    void computeVelocityLimits(double s) override {
        const auto& knots = limits_->knots;
        const auto& caps = limits_->caps;
        auto next = std::upper_bound(knots.begin(), knots.end(), s);
        size_t span = std::min(caps.size() - 1,
            static_cast<size_t>(std::max<int64_t>(0, next - knots.begin() - 1)));
        double cap = caps[span] > 0.0 ? caps[span] : 1e8;
        if (span > 0 && std::abs(s - knots[span]) < 1e-12 && caps[span - 1] > 0.0)
            cap = std::min(cap, caps[span - 1]);
        if (!limits_->continuous_caps.empty()) {
            const auto node=std::lower_bound(limits_->grid.begin(),limits_->grid.end(),s);
            if (node==limits_->grid.end() || *node!=s) throw std::logic_error("Velocity certificate grid mismatch");
            cap=std::min(cap,limits_->continuous_caps[static_cast<size_t>(node-limits_->grid.begin())]);
        }
        m_lower[m_lower.size() - 1] = -cap;
        m_upper[m_upper.size() - 1] = cap;
    }

private:
    std::shared_ptr<const PathSpeedCaps> limits_;
};

// For a constant-control stage, x(s)=x0+2*u*(s-s0). Joint acceleration
// q''(s)*x(s)+q'(s)*u is a quartic polynomial, linear in (u,x0).
// Bounding every Bernstein control bounds the complete interval, including
// peaks that collocation misses. Final execution validation remains mandatory.
// Keep the repeated head/successor proofs, with a fixed memory budget even
// when a longer pending suffix needs recomputation. Each worker owns its cache.
class IntervalAccelerationProofCache {
public:
    struct Key {};
private:
    struct Entry {
        std::shared_ptr<const Key> key;
        toppra::Vector grid;
        toppra::Matrices F;
        toppra::Vectors g;
        toppra::Bounds controls;
    };
    std::array<Entry,2> entries_;
public:
    bool load(const std::shared_ptr<const Key>& key, const toppra::Vector& grid,
            toppra::Matrices& F, toppra::Vectors& g, toppra::Bounds& controls) {
        for (size_t i=0;i<entries_.size();++i) {
            auto& entry=entries_[i];
            if (entry.key!=key || entry.grid.size()!=grid.size() ||
                    !(entry.grid.array()==grid.array()).all()) continue;
            F=entry.F;g=entry.g;controls=entry.controls;
            if (i) std::swap(entries_[0],entries_[i]);
            return true;
        }
        return false;
    }
    void retain(const std::shared_ptr<const Key>& key, const toppra::Vector& grid,
            const toppra::Matrices& F, const toppra::Vectors& g, const toppra::Bounds& controls) {
        Entry entry{key,grid,F,g,controls};
        entries_[1]=std::move(entries_[0]);entries_[0]=std::move(entry);
    }
};

class IntervalJointAcceleration final : public toppra::LinearConstraint {
    toppra::Vector lower_, upper_, velocity_;
    toppra::Matrices coefficients_;
    std::vector<double> knots_, caps_;
    std::shared_ptr<IntervalAccelerationProofCache> cache_;
    const std::shared_ptr<const IntervalAccelerationProofCache::Key> key_=
        std::make_shared<const IntervalAccelerationProofCache::Key>();
public:
    IntervalJointAcceleration(const toppra::Vector& lower, const toppra::Vector& upper,
            const toppra::Vector& velocity, const toppra::Matrices& coefficients, const std::vector<double>& knots,
            const std::vector<double>& caps, size_t first, size_t after,
            std::shared_ptr<IntervalAccelerationProofCache> cache)
        : LinearConstraint(14*lower.size(), 2, false, true, false), lower_(lower), upper_(upper), velocity_(velocity),
          coefficients_(coefficients.begin()+(first ? first-1 : 0),
              coefficients.begin()+std::min(after+1,coefficients.size())),
          knots_(knots.begin()+(first ? first-1 : 0),
              knots.begin()+std::min(after+2,knots.size())),
          caps_(caps.begin()+(first ? first-1 : 0),
              caps.begin()+std::min(after+1,caps.size())),cache_(std::move(cache)) {}
protected:
    void computeParams_impl(const toppra::GeometricPath& path, const toppra::Vector& grid,
            toppra::Vectors& a, toppra::Vectors& b, toppra::Vectors& c,
            toppra::Matrices& F, toppra::Vectors& g, toppra::Bounds& control_limits, toppra::Bounds&) override {
        PathProfilePhase profile("interval_acceleration_constraints");
        (void)path;
        if (cache_ && cache_->load(key_,grid,F,g,control_limits)) {
            for (Eigen::Index i=0;i<grid.size();++i) {
                a[i]<<1.0,0.0;b[i]<<0.0,1.0;c[i].setZero();
            }
            return;
        }
        toppra::Matrix derivatives(5,lower_.size());
        for (Eigen::Index i=0;i<grid.size();++i) {
            const double h=i+1<grid.size() ? grid[i+1]-grid[i] : 0.0;
            const auto next=std::upper_bound(knots_.begin(),knots_.end(),grid[i]);
            const size_t span=std::min(coefficients_.size()-1,
                static_cast<size_t>(std::max<ptrdiff_t>(0,next-knots_.begin()-1)));
            const double offset=grid[i]-knots_[span];
            // GeometricPath exposes derivatives only through order two. Use
            // our owned polynomial snapshot for the complete Taylor law.
            for (Eigen::Index joint=0;joint<lower_.size();++joint) for (int order=1;order<=5;++order) {
                double value=0;
                for (int power=5;power>=order;--power) {
                    double factor=1;
                    for (int n=0;n<order;++n) factor*=power-n;
                    value=value*offset+coefficients_[span](5-power,joint)*factor;
                }
                derivatives(order-1,joint)=value;
            }
            a[i]<<1.0,0.0; b[i]<<0.0,1.0; c[i].setZero();
            for (Eigen::Index joint=0;joint<lower_.size();++joint) {
                double prime[5]{},second[5]{},control_u[5]{},control_x[5]{};
                double factor=1.0;
                for (int k=0;k<5;++k) {
                    prime[k]=derivatives(k,joint)*factor;
                    if (k<4) second[k]=derivatives(k+1,joint)*factor;
                    factor*=h/(k+1);
                }
                for (int k=0;k<5;++k) {
                    const double power_u=prime[k]+(k ? 2*h*second[k-1] : 0.0);
                    for (int j=k;j<5;++j) {
                        double ratio=1.0;
                        for (int n=0;n<k;++n) ratio*=static_cast<double>(j-n)/(4-n);
                        control_u[j]+=ratio*power_u;
                        control_x[j]+=ratio*second[k];
                    }
                }
                for (int k=0;k<5;++k) {
                    const Eigen::Index row=14*joint+2*k;
                    F[i](row,0)=control_u[k]; F[i](row,1)=control_x[k];
                    F[i](row+1,0)=-control_u[k]; F[i](row+1,1)=-control_x[k];
                    g[i][row]=upper_[joint]; g[i][row+1]=-lower_[joint];
                }
                // The integer clock can switch control on either side of a
                // geometric curvature join. Enforce its incoming and outgoing
                // accelerations as well as the nominal interval polynomial.
                for (int side=0;side<2;++side) {
                    const double point=side && i+1<grid.size() ? grid[i+1] : grid[i];
                    auto after=std::upper_bound(knots_.begin(),knots_.end(),point);
                    size_t other=std::min(coefficients_.size()-1,
                        static_cast<size_t>(std::max<ptrdiff_t>(0,after-knots_.begin()-1)));
                    if (!side && other>0 && point==knots_[other]) --other;
                    double qprime=0,qsecond=0;
                    const double offset=point-knots_[other];
                    for (int power=5;power>=1;--power)
                        qprime=qprime*offset+power*coefficients_[other](5-power,joint);
                    for (int power=5;power>=2;--power)
                        qsecond=qsecond*offset+power*(power-1)*coefficients_[other](5-power,joint);
                    const Eigen::Index row=14*joint+10+2*side;
                    const double u=qprime+2*(point-grid[i])*qsecond;
                    F[i](row,0)=u; F[i](row,1)=qsecond;
                    F[i](row+1,0)=-u; F[i](row+1,1)=-qsecond;
                    g[i][row]=upper_[joint]; g[i][row+1]=-lower_[joint];
                }
            }
            double cap=caps_[span]>0 ? caps_[span] : 1e8;
            if (span>0 && std::abs(grid[i]-knots_[span])<1e-12 && caps_[span-1]>0)
                cap=std::min(cap,caps_[span-1]);
            // The collocated velocity constraints already bound this node's
            // squared speed. Use that finite domain even without an authored
            // feed cap, rather than the generic 1e16 path-speed ceiling.
            for (Eigen::Index joint=0;joint<velocity_.size();++joint)
                if (derivatives(0,joint)!=0) cap=std::min(cap,std::nextafter(
                    std::nextafter(velocity_[joint]/std::abs(derivatives(0,joint)),std::numeric_limits<double>::infinity()),
                    std::numeric_limits<double>::infinity()));
            const long double xmax=std::nextafter(cap*cap,std::numeric_limits<double>::infinity());
            const long double infinity=std::numeric_limits<long double>::infinity();
            auto upward=[&](long double value){return std::nextafter(value,infinity);};
            long double control_bound=infinity;
            for (Eigen::Index row=0;row<F[i].rows();row+=2) {
                // Repeated Bernstein controls on an affine span contribute the
                // same candidate bound. Keep every LP row and its order, but
                // evaluate that outward-rounded proof only once per run.
                if (row>0 && F[i](row,0)==F[i](row-2,0) && F[i](row,1)==F[i](row-2,1) &&
                        g[i][row]==g[i][row-2] && g[i][row+1]==g[i][row-1]) continue;
                const long double slope=std::abs(F[i](row,0));
                if (slope>0) {
                    const long double term=upward(std::abs(static_cast<long double>(F[i](row,1)))*xmax);
                    const long double numerator=upward(std::max(g[i][row],g[i][row+1])+term);
                    control_bound=std::min(control_bound,upward(numerator/slope));
                }
            }
            // Retain the derived finite domain explicitly after conditioning.
            const double bounded_control=std::nextafter(static_cast<double>(control_bound),
                std::numeric_limits<double>::infinity());
            control_limits[i]<<-bounded_control,bounded_control;
            control_bound=bounded_control;
            // At a tangent, a nonzero trigonometric remnant can turn the
            // backward pass's rounding into an arbitrary control bound. Never
            // drop it: replace its contribution by its proved worst case over
            // the finite control domain, tightening both inequalities. Only do
            // this when that contribution fits the arithmetic reserve.
            double previous_slope=0;
            long double contribution=0;
            for (Eigen::Index row=0;row<F[i].rows();row+=2) {
                const double slope=std::abs(F[i](row,0));
                if (slope==0.0) continue;
                // The domain is fixed for this node. Reusing the exact same
                // rounded product leaves each inequality's tightening intact.
                if (slope!=previous_slope) {
                    contribution=upward(static_cast<long double>(slope)*control_bound);
                    previous_slope=slope;
                }
                // Tightening may consume only roundoff of the physical
                // acceleration budget, never a large cancellation against Ax.
                // In particular the feasible stationary state must survive.
                const long double scale=std::max(g[i][row],g[i][row+1]);
                if (std::isfinite(contribution) && contribution>0 && contribution<=
                        512*std::numeric_limits<double>::epsilon()*scale) {
                    F[i](row,0)=F[i](row+1,0)=0;
                    for (int side=0;side<2;++side) g[i][row+side]=std::nextafter(
                        static_cast<double>(static_cast<long double>(g[i][row+side])-contribution),
                        -std::numeric_limits<double>::infinity());
                }
            }
        }
        if (cache_) cache_->retain(key_,grid,F,g,control_limits);
    }
};

// TOPP-RA's stages in a span of the path: enough that the profile and the acceleration peaks between its grid points
// are resolved, never more than `MAX_STAGES_PER_SPAN` (the grid every span used to have, which the 0.8 interpolation
// margin is calibrated with).
//
// Two things need stages. The speed profile: a stage assumes one path acceleration, so it should change the speed by
// no more than `PROFILE_STEP` of itself, `a ds <= PROFILE_STEP v^2`, with `a` the path acceleration the joints allow
// and `v` the fastest the span can be taken (a joint's velocity limit, the centripetal bound of its curvature, the
// span's cap). And the collocation: the acceleration constraint is imposed at the grid points, and between them the
// velocity-normalized joint-space direction turns. `MAX_STAGE_TURN` targets the mean turn per stage; uniform s spacing
// does not bound each stage's turn when that direction changes nonuniformly. For uniform turning the missed peak is
// about turn^2 / 8 (4.5 % at 0.6 rad); continuous validation, not this estimate, enforces the limits. A straight span of a few micrometres needs two stages, a
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
// that it cruises, and one stage covers the middle. Grid size stays independent of a long line's length.
//
// For fixed boundary speeds, the exact squared-speed envelope is min(v0^2+2As,
// v1^2+2A(L-s), V^2). The discrete optimum interpolates that envelope at nodes.
// Only intervals containing a switch lose time. With spacing h, their combined
// integral squared-speed deficit is <= A*h^2 (one triangular switch, or two
// ramp/cruise switches). The derivative of x^(-1/2) bounds the time loss by
// A*h^2/(2*x_min^(3/2)). Use N stages per end zone, or 2N uniformly when L<=2Z.
// In either case, normalizing by the exact time gives a relative loss bounded by
// 1/(8*N^2*(1-1/N)^(3/2)) <= 1/(8*(N-1)^2), N>=2. On short spans, peak^2>=A*L;
// on long spans, T>=V/A and every switch lies in an end zone. In switch intervals
// x_min >= peak^2*(1-1/N). This bounds discretization, not later polynomial lowering;
// the latter retains its independent validation. Two-stage short asymmetric
// triangles can lose over 4%, even though symmetric rest-to-rest triangles are exact.
// Make the existing straight-line regression's 0.4% accuracy target explicit.
constexpr double STRAIGHT_GRID_TIME_ERROR = 0.004;
constexpr size_t straight_zone_stages() {
    size_t count = 2;
    while (8.0 * static_cast<double>((count - 1) * (count - 1)) * STRAIGHT_GRID_TIME_ERROR < 1.0) ++count;
    return count;
}
constexpr size_t STRAIGHT_ZONE_STAGES = straight_zone_stages();

// Keep each solver's constraint tables bounded. Window boundaries propagate complete squared-speed
// controllable intervals backwards and the actually reachable squared speed forwards.
constexpr size_t WINDOW_NODES = 8192;
constexpr double MAX_PATH_SQUARED_SPEED = 1e16;
constexpr double MOVING_BOUNDARY_MARGIN = 0.98;

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
        // At constant path acceleration A, reaching any speed <= V from any
        // nonnegative entry speed takes at most V^2/(2A) distance; braking is
        // symmetric. `margin` includes the exact moving-boundary acceleration
        // reserve used by the solver, so no empirical reach multiplier is needed.
        const double exact = fastest * fastest / (2.0 * margin * accelerating);
        const double step = exact / static_cast<double>(STRAIGHT_ZONE_STAGES);
        const size_t zone_stages = STRAIGHT_ZONE_STAGES;
        const double zone = step * static_cast<double>(zone_stages);
        if (length <= 2.0 * zone)
            return uniform(2 * zone_stages);
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
// Anchors (path ends and changes in geometry or feed limits) are reached
// exactly at their knots. Redundant affine samples share a clock region.
// Rounding the region's duration up leaves its stages
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

// Preserve required geometry/feed anchors when rebuilding an integer-clock law.
std::vector<char> sample_anchors(const std::vector<mk_path_sample> &samples,
                                 const std::vector<double>& caps,
                                 const std::vector<double> &positions) {
    std::vector<double> sample_s;
    sample_s.reserve(samples.size());
    for (const auto &sample : samples) sample_s.push_back(sample.s);
    std::vector<char> anchor(positions.size(), 0);
    for (size_t i = 0; i < positions.size(); ++i) {
        const auto found = std::lower_bound(sample_s.begin(), sample_s.end(), positions[i]);
        if (found != sample_s.end() && *found == positions[i])
            anchor[i] = timing_anchor_required(samples, caps, static_cast<std::size_t>(found - sample_s.begin()));
    }
    if (!anchor.empty()) anchor.front() = anchor.back() = 1;
    return anchor;
}

// Rebuilds the stages over their own distances, from `start_speed`, each
// stage reaching its target end speed.
bool retime_stages(std::vector<mk_time_stage> &stages,
                   const std::vector<mk_path_sample> &samples, const std::vector<double>& caps, double start_speed,
                   const std::vector<double> &end_speeds) {
    std::vector<double> positions, speeds{start_speed};
    positions.reserve(stages.size() + 1);
    for (const auto &stage : stages) positions.push_back(stage.start_s);
    positions.push_back(end_s(stages.back()));
    // A law's speeds can dip below zero by rounding where it comes to rest.
    for (double speed : end_speeds) speeds.push_back(std::max(0.0, speed));
    speeds.front() = std::max(0.0, speeds.front());
    return stages_from_profile(positions, speeds, sample_anchors(samples, caps, positions), stages);
}

// Slows the whole law down uniformly by `factor`: every speed divided by it.
bool stretch_stages(std::vector<mk_time_stage> &stages,
                    const std::vector<mk_path_sample> &samples, const std::vector<double>& caps, double factor) {
    if (stages.empty() || !(factor > 0.0)) return false;
    std::vector<double> ends;
    ends.reserve(stages.size());
    for (const auto &stage : stages)
        ends.push_back(stage_speed(stage, stage.start_ns + stage.duration_ns) / factor);
    return retime_stages(stages, samples, caps, stages.front().speed / factor, ends);
}

// A collocation peak can exceed a joint bound after quintic lowering. Reduce
// path speed around that peak, preserving all other stages' time scale.
bool soften_stages(std::vector<mk_time_stage> &stages,
                   const std::vector<mk_path_sample> &samples, const std::vector<double>& caps, double peak_time,
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
    return retime_stages(stages, samples, caps, speeds.front(),
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

namespace {


mk_result lower_path_samples(const std::vector<mk_path_sample>& samples,
        const std::vector<mk_time_stage>& stages, double tolerance,
        mk_trajectory_handle* out_trajectory, bool subrange = false, const mk_limits* physical_limits = nullptr) {
    if (!out_trajectory || !std::isfinite(tolerance) || tolerance <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    try {
        if (subrange ? (stages.front().start_s < samples.front().s-1e-10 ||
                end_s(stages.back()) > samples.back().s+1e-8) :
                (std::abs(samples.front().s - stages.front().start_s) > 1e-10 ||
                 std::abs(samples.back().s - end_s(stages.back())) > 1e-8))
            return MK_ERROR_INVALID_ARGUMENT;
        NativeResultOwner<mk_trajectory_handle, mk_trajectory_destroy> owner;
        auto& trajectory = owner.value;
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
                    return MK_ERROR_INVALID_ARGUMENT;
                }
                knots.push_back(static_cast<int64_t>(std::llround(seconds * 1e9)));
            }
            std::sort(knots.begin(), knots.end());
            knots.erase(std::unique(knots.begin(), knots.end()), knots.end());
            for (size_t i = 1; i < knots.size(); ++i) {
                if (!append_interval(samples, *cache, stage, knots[i - 1], knots[i],
                    tolerance, trajectory, 0, physical_limits)) {
                    return MK_ERROR_GENERATION;
                }
            }
        }
        *out_trajectory = owner.release();
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
}
} // namespace

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
        return lower_path_samples(samples, stages, tolerance, out_trajectory);
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
}

namespace {
// Prepared geometry, adaptive grid and constraints have one owner independent of any
// endpoint solve/result. A session can retain this object without capturing a caller's stack.
class PreparedTimingPath {
    const std::shared_ptr<IntervalAccelerationProofCache> acceleration_proofs=
        std::make_shared<IntervalAccelerationProofCache>();
public:
    std::vector<double> knots, caps, grid_values;
    toppra::Matrices coefficients;
    std::shared_ptr<toppra::PiecewisePolyPath> geometric;
    bool nonlinear = false;
    bool interval_acceleration = false;
    std::vector<size_t> span_first, window_first;
    std::shared_ptr<const std::vector<uint32_t>> shared_stage_span;
    toppra::Vector grid, lower_velocity, upper_velocity, lower_acceleration, upper_acceleration;
    std::shared_ptr<const PathSpeedCaps> window_limits;
    size_t span_count = 0;

    PreparedTimingPath(const std::vector<mk_path_sample>& samples, const double* max_velocity,
            const double* max_acceleration, uint32_t joint_count, const double* speed_caps,
            uint32_t speed_cap_count, bool moving_boundary, bool reference, bool bound_intervals = false) {
        interval_acceleration = bound_intervals;
        PathProfilePhase stage_profile("native_geometry");
        const auto dof = static_cast<Eigen::Index>(joint_count + 1);
        // Scale each joint by its velocity limit so rotary shafts and linear
        // carriages have comparable LP coefficients. The time law remains in
        // the original path coordinate and the final validation uses SI units.
        std::vector<double> coordinate_scale(joint_count);
        for (uint32_t j = 0; j < joint_count; ++j)
            coordinate_scale[j] = 1.0 / max_velocity[j];
        size_t affine_spans=0, zero_curvature_spans=0;
        knots.reserve(samples.size());
        caps.reserve(samples.size() - 1);
        coefficients.reserve(samples.size() - 1);
        for (const auto &sample : samples) knots.push_back(sample.s);
        for (size_t i = 0; i + 1 < samples.size(); ++i) {
            const double ds = samples[i + 1].s - samples[i].s;
            bool affine_span=true, zero_curvature_span=true;
            toppra::Matrix matrix = toppra::Matrix::Zero(6, dof);
            for (uint32_t j = 0; j < joint_count; ++j) {
                const Poly p = path_poly(samples[i], samples[i + 1], j);
                if (stage_profile.enabled()) {
                    if (samples[i].second[j]!=0 || samples[i+1].second_before[j]!=0 ||
                        samples[i].first[j]!=samples[i+1].first[j]) zero_curvature_span=false;
                    for (size_t k=2;k<p.size();++k) if (std::abs(p[k])>=1e-12) affine_span=false;
                }
                for (size_t k = 0; k < p.size(); ++k)
                    matrix(5 - static_cast<Eigen::Index>(k), j) =
                        p[k] * coordinate_scale[j] /
                        std::pow(ds, static_cast<int>(k));
            }
            if (stage_profile.enabled()) {
                if (affine_span) ++affine_spans;
                if (zero_curvature_span) ++zero_curvature_spans;
            }
            matrix(4, joint_count) = 1.0;
            matrix(5, joint_count) = samples[i].s;
            coefficients.push_back(std::move(matrix));
            caps.push_back(speed_cap_count ? speed_caps[i] : 0.0);
        }
        geometric = std::make_shared<toppra::PiecewisePolyPath>(coefficients, knots);
        nonlinear = false;
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
        const double moving_boundary_margin = moving_boundary ? MOVING_BOUNDARY_MARGIN : 1.0;
        std::vector<double> grid_velocity(joint_count);
        for (uint32_t j = 0; j < joint_count; ++j)
            grid_velocity[j] = max_velocity[j] * moving_boundary_margin;
        stage_profile.mark("grid_construction");
        // Stages per span, and the grid they make: each span's first point is a path sample (an anchor).
        span_first.resize(samples.size(), 0);
        std::vector<uint32_t> stage_span;
        for (size_t span = 0; span + 1 < samples.size(); ++span) {
            span_first[span] = stage_span.size();
            const auto span_grid = nodes_for_span(samples, span, grid_velocity.data(), max_acceleration, joint_count,
                speed_cap_count ? speed_caps[span] : 0.0, interpolation_margin * moving_boundary_margin, reference);
            const double begin = samples[span].s, length = samples[span + 1].s - begin;
            for (const double node : span_grid.nodes) {
                stage_span.push_back(static_cast<uint32_t>(span));
                grid_values.push_back(begin + length * node);
            }
        }
        span_first.back() = stage_span.size();
        grid_values.push_back(samples.back().s);
        if (stage_profile.enabled())
            std::fprintf(stderr,"PROCESS_PATH_KIND {\"affine_spans\":%zu,\"zero_curvature_spans\":%zu,\"spans\":%zu,\"grid_nodes\":%zu,\"joints\":%u}\n",
                affine_spans,zero_curvature_spans,samples.size()-1,grid_values.size(),joint_count);
        shared_stage_span = std::make_shared<const std::vector<uint32_t>>(std::move(stage_span));
        grid = Eigen::Map<const toppra::Vector>(grid_values.data(), static_cast<Eigen::Index>(grid_values.size()));
        lower_velocity.resize(dof); upper_velocity.resize(dof);
        lower_acceleration.resize(dof); upper_acceleration.resize(dof);
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
        // Cap lookup data is immutable across windows. Constraint objects retain
        // mutable evaluation scratch, so every window owns separate instances.
        std::vector<double> continuous_caps;
        if (bound_intervals) {
            continuous_caps.assign(grid_values.size(),1e8);
            for (size_t span=0;span+1<samples.size();++span) {
                const double ds=knots[span+1]-knots[span];
                for (uint32_t joint=0;joint<joint_count;++joint) {
                    const auto q=small_hermite(samples[span],samples[span+1],joint);
                    Small prime{};prime.n=std::max(1,q.n-1);
                    bool varying=false;
                    for (int k=1;k<q.n;++k) {
                        prime.c[k-1]=k*q.c[k]*coordinate_scale[joint]/ds;
                        if (k>1 && prime.c[k-1]!=0) varying=true;
                    }
                    if (!varying) continue; // Existing point bounds are exact for constant tangents.
                    for (size_t node=span_first[span];node<span_first[span+1];++node) {
                        const double inner[3]={(grid_values[node]-knots[span])/ds,
                            (grid_values[node+1]-grid_values[node])/ds,0};
                        const auto local=small_compose(prime,inner,5);
                        long double magnitude=0,bound=0;
                        for (int k=0;k<local.n;++k) magnitude+=std::abs(static_cast<long double>(local.c[k]));
                        for (int control=0;control<5;++control) {
                            long double value=0;
                            for (int k=0;k<=control;++k) {
                                long double ratio=1;
                                for (int n=0;n<k;++n) ratio*=static_cast<long double>(control-n)/(4-n);
                                value+=ratio*local.c[k];
                            }
                            bound=std::max(bound,std::abs(value));
                        }
                        // Bernstein's convex hull bounds the full quartic
                        // tangent. Both endpoint squared-speed bounds enforce
                        // it throughout the affine x(s), without extra LP rows.
                        bound+=512*std::numeric_limits<double>::epsilon()*magnitude;
                        if (bound==0) continue;
                        const double cap=std::nextafter(static_cast<double>(moving_boundary_margin/bound),0.0);
                        continuous_caps[node]=std::min(continuous_caps[node],cap);
                        continuous_caps[node+1]=std::min(continuous_caps[node+1],cap);
                    }
                }
            }
        }
        window_limits = std::make_shared<const PathSpeedCaps>(PathSpeedCaps{knots,caps,grid_values,std::move(continuous_caps)});
        // Full controllable intervals matter: a following window may require a minimum entry speed to reach
        // a moving end, and a maximum computed for one fixed exit is not a valid bound for every slower exit.
        window_first.push_back(0);
        span_count = samples.size() - 1;
        for (size_t span = 1; span < span_count && !reference; ++span)
            if (span_first[span] - span_first[window_first.back()] >= WINDOW_NODES &&
                span_first[span_count] - span_first[span] >= WINDOW_NODES / 4) window_first.push_back(span);
    }

    std::unique_ptr<toppra::algorithm::TOPPRA> make_window(size_t window,
            const std::shared_ptr<toppra::solver::Seidel>& workspace) const {
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
        toppra::LinearConstraintPtrs constraints{
            std::make_shared<VelocityWithCaps>(lower_velocity, upper_velocity, window_limits)};
        if (interval_acceleration) {
            const size_t first_span=window_first[window];
            const size_t after_span=window+1<window_first.size() ? window_first[window+1] : span_count;
            constraints.push_back(std::make_shared<IntervalJointAcceleration>(lower_acceleration,upper_acceleration,upper_velocity,
                coefficients,knots,caps,first_span,after_span,window_first.size()>1 ? acceleration_proofs : nullptr));
        }
        else constraints.push_back(std::make_shared<toppra::constraint::LinearJointAcceleration>(
            lower_acceleration, upper_acceleration));
        for (auto& constraint : constraints)
            constraint->discretizationType(toppra::DiscretizationType::Collocation);
        auto algorithm = std::make_unique<toppra::algorithm::TOPPRA>(std::move(constraints), window_path);
        algorithm->solver(workspace);
        algorithm->setGridpoints(window_grid);
        algorithm->setInitialXBounds(toppra::Bound{0.0, MAX_PATH_SQUARED_SPEED});
        return algorithm;
    }

    std::unique_ptr<motionkit::RetainedWindowTiming> make_serial_timing() const {
        auto workspace = std::make_shared<toppra::solver::Seidel>();
        std::vector<std::unique_ptr<motionkit::RetainedWindowTiming::Algorithm>> problems;
        problems.reserve(window_first.size());
        for (size_t window = 0; window < window_first.size(); ++window)
            problems.push_back(make_window(window, workspace));
        return std::make_unique<motionkit::RetainedWindowTiming>(std::move(problems));
    }
};

struct PathTimingSession {
    std::mutex access;
    std::shared_ptr<const std::vector<mk_path_sample>> samples;
    std::vector<double> velocity, acceleration, caps;
    double lowering_tolerance;
    uint32_t require_feasible_path;
    bool moving_boundary = false;
    std::unique_ptr<PreparedTimingPath> prepared;
    std::unique_ptr<motionkit::RetainedWindowTiming> retained;

    const PreparedTimingPath& prepare(bool moving) {
        if (!prepared || moving_boundary != moving) {
            retained.reset();
            prepared = std::make_unique<PreparedTimingPath>(*samples, velocity.data(), acceleration.data(),
                static_cast<uint32_t>(velocity.size()), caps.empty() ? nullptr : caps.data(),
                static_cast<uint32_t>(caps.size()), moving, false);
            moving_boundary = moving;
        }
        return *prepared;
    }
    motionkit::RetainedWindowTiming& timing() {
        if (!prepared) throw std::logic_error("Timing session has no prepared path");
        if (!retained) retained = prepared->make_serial_timing();
        return *retained;
    }
};
std::unordered_map<uint32_t, std::shared_ptr<PathTimingSession>> timing_sessions;
uint32_t next_timing_session = 1;

// Any failed attempt invalidates incremental state, including validation/lowering
// failures after a successful backward pass. Prepared immutable geometry survives.
struct TimingSessionAttempt {
    PathTimingSession* session;
    bool accepted = false;
    ~TimingSessionAttempt() { if (session && !accepted) session->retained.reset(); }
};

// Borrowed only for this synchronous lowering call. Returned diagnostics copy
// shared ownership of their samples/caps/span indices, never references to this view.
struct PathProfileView {
    const std::shared_ptr<const std::vector<mk_path_sample>>& shared_samples;
    const std::vector<size_t>& span_first;
    const toppra::Vector& grid;
    const std::shared_ptr<const std::vector<uint32_t>>& shared_stage_span;
    const std::shared_ptr<const PathSpeedCaps>& window_limits;
    bool nonlinear;
};
struct PathProfileLimits {
    const double* velocity;
    const double* acceleration;
    uint32_t joint_count;
};
struct PathProfileBoundary {
    double start_speed;
    double end_speed;
    double lowering_tolerance;
    uint32_t require_feasible_path;
};

// Lower and validate a supplied profile without rebuilding its geometry, grid
// or constraint policy. The same path will serve full and streamed profiles.
mk_result lower_timed_profile(const PathProfileView& view, const PathProfileLimits& joint_limits,
        const PathProfileBoundary& boundary, std::vector<double> speeds,
        mk_timing_report* out_report, mk_time_law_handle* out_law, mk_trajectory_handle* out_trajectory,
        const std::vector<mk_time_stage>* clock_stages = nullptr,
        const std::shared_ptr<const std::vector<mk_path_sample>>* clock_geometry = nullptr) {
    if (!out_law || !out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_law->id = 0; out_trajectory->id = 0;
    if (speeds.size() != static_cast<size_t>(view.grid.size()) || speeds.size() < 2)
        return MK_ERROR_INVALID_ARGUMENT;
    const auto& shared_samples = view.shared_samples;
    const auto& samples = *shared_samples;
    const auto& caps = view.window_limits->caps;
    const auto& span_first = view.span_first;
    const auto& grid = view.grid;
    const bool nonlinear = view.nonlinear;
    const auto* max_velocity = joint_limits.velocity;
    const auto* max_acceleration = joint_limits.acceleration;
    const uint32_t joint_count = joint_limits.joint_count;
    const double start_speed = boundary.start_speed, end_speed = boundary.end_speed;
    const double lowering_tolerance = boundary.lowering_tolerance;
    const uint32_t require_feasible_path = boundary.require_feasible_path;
    try {
        std::vector<mk_time_stage> stages;
        std::vector<double> positions(grid.data(), grid.data() + grid.size());
        std::vector<char> anchors(positions.size(), 0);
        for (size_t span = 0; span + 1 < samples.size(); ++span)
            anchors[span_first[span]] = timing_anchor_required(samples, caps, span);
        anchors.back() = 1;
        if (clock_stages) stages=*clock_stages;
        else if (!stages_from_profile(positions, speeds, anchors, stages)) return MK_ERROR_GENERATION;
        NativeResultOwner<mk_time_law_handle, mk_time_law_destroy> law_owner;
        NativeResultOwner<mk_trajectory_handle, mk_trajectory_destroy> trajectory_owner;
        auto& created = law_owner.value;
        bool accepted = false;
        for (int attempt = 0; attempt < 24; ++attempt) {
            PathProfilePhase stage_profile("law_registration");
            const auto result = mk_time_law_create(stages.data(),
                static_cast<uint32_t>(stages.size()), &created);
            if (result != MK_OK) return result;
            auto& lowered = trajectory_owner.value;
            stage_profile.mark("polynomial_lowering");
            const auto lowered_result = lower_path_samples(clock_geometry ? **clock_geometry : samples, stages,
                lowering_tolerance, &lowered, clock_geometry != nullptr);
            if (lowered_result != MK_OK) {
                law_owner.reset();
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
            stage_profile.mark("continuous_validation");
            const auto validation = mk_validate(lowered, &limits, &report);
            stage_profile.mark("validation_policy");
            if (validation != MK_OK) {
                trajectory_owner.reset();
                law_owner.reset();
                return MK_ERROR_GENERATION;
            }
            if (clock_stages && (report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_FAILED ||
                    report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_FAILED)) {
                // Try a derivative-certified representation of the unchanged
                // law before rejecting it. Normal successful paths pay no
                // per-piece validation cost.
                trajectory_owner.reset();
                stage_profile.mark("derivative_certified_lowering");
                const auto refined = lower_path_samples(clock_geometry ? **clock_geometry : samples,
                    stages,lowering_tolerance,&lowered,clock_geometry!=nullptr,&limits);
                if (refined!=MK_OK) return refined;
                stage_profile.mark("continuous_validation");
                if (mk_validate(lowered,&limits,&report)!=MK_OK) return MK_ERROR_GENERATION;
                stage_profile.mark("validation_policy");
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
                if (velocity_check.status==MK_CHECK_FAILED && velocity_check.joint<joint_count) {
                    const auto& geometry=clock_geometry ? **clock_geometry : samples;
                    const int64_t tick=std::llround(velocity_check.time_seconds*1e9);
                    auto at=std::upper_bound(stages.begin(),stages.end(),tick,
                        [](int64_t t,const mk_time_stage& stage){return t<stage.start_ns;});
                    if (at!=stages.begin()) --at;
                    const double distance=stage_s(*at,tick),speed=stage_speed(*at,tick);
                    auto point=std::upper_bound(geometry.begin(),geometry.end(),distance,
                        [](double value,const mk_path_sample& sample){return value<sample.s;});
                    const size_t span=std::min(geometry.size()-2,point==geometry.begin() ? size_t{0} : static_cast<size_t>(point-geometry.begin()-1));
                    const auto q=path_state(geometry[span],geometry[span+1],velocity_check.joint,distance);
                    std::fprintf(stderr,"PROCESS_PATH_VELOCITY_FAILURE time=%.17g span=%zu distance=%.17g speed=%.17g reference=%.17g fitted=%.17g\n",
                        velocity_check.time_seconds,span,distance,speed,q.first*speed,velocity_check.value);
                }
                if (acceleration_check.status==MK_CHECK_FAILED && acceleration_check.joint<joint_count) {
                    const auto& geometry=clock_geometry ? **clock_geometry : samples;
                    const double time=acceleration_check.time_seconds;
                    const int64_t tick=std::llround(time*1e9);
                    auto at=std::upper_bound(stages.begin(),stages.end(),tick,
                        [](int64_t t,const mk_time_stage& stage){return t<stage.start_ns;});
                    if (at!=stages.begin()) --at;
                    const double s=stage_s(*at,tick),v=stage_speed(*at,tick);
                    auto point=std::upper_bound(geometry.begin(),geometry.end(),s,
                        [](double distance,const mk_path_sample& point){return distance<point.s;});
                    size_t span=point==geometry.begin() ? 0 : static_cast<size_t>(point-geometry.begin()-1);
                    span=std::min(span,geometry.size()-2);
                    const auto q=path_state(geometry[span],geometry[span+1],acceleration_check.joint,s);
                    std::fprintf(stderr,"PROCESS_PATH_ACCELERATION_FAILURE {\"span\":%zu,\"distance\":%.17g,\"speed\":%.17g,\"control\":%.17g,\"reference_acceleration\":%.17g,\"fitted_acceleration\":%.17g,\"stage_ns\":%lld,\"span_length\":%.17g}\n",
                        span,s,v,at->acceleration,q.first*at->acceleration+q.second*v*v,
                        acceleration_check.value,static_cast<long long>(at->duration_ns),geometry[span+1].s-geometry[span].s);
                }
            }
            if (factor <= 1.0) {
                accepted = true;
                break;
            }
            trajectory_owner.reset();
            law_owner.reset();
            // A supplied clock preview has an owned continuation cursor. A
            // local retry must not silently retime it behind that cursor.
            if(require_feasible_path || clock_stages)return MK_ERROR_GENERATION;
            // Uniform stretching would change an authored moving endpoint
            // speed, so report infeasibility instead of returning a law with
            // a different boundary contract.
            if (start_speed > 0.0 || end_speed > 0.0) return MK_ERROR_GENERATION;
            // Both boundaries are at rest here (checked above).
            if (nonlinear && stages.size() > 1000 &&
                acceleration_check.status == MK_CHECK_FAILED && attempt < 20 &&
                soften_stages(stages, samples, caps, acceleration_check.time_seconds, factor * 1.002))
                continue;
            if (!stretch_stages(stages, samples, caps, factor * 1.002)) return MK_ERROR_GENERATION;
        }
        if (!accepted) return MK_ERROR_GENERATION;
        {
            // Bindings name TOPP-RA's own stages, with its own control.
            std::vector<double> velocity_limits(max_velocity, max_velocity + joint_count);
            std::vector<double> acceleration_limits(max_acceleration, max_acceleration + joint_count);
            auto make = [shared_samples, positions = std::move(positions), speeds = std::move(speeds),
                         stage_span = view.shared_stage_span, limits = view.window_limits,
                         velocity_limits = std::move(velocity_limits),
                         acceleration_limits = std::move(acceleration_limits)]() {
                std::vector<mk_timing_binding> bindings;
                bindings.reserve(positions.size() - 1);
                for (size_t i = 0; i + 1 < positions.size(); ++i) {
                    const double ds = positions[i + 1] - positions[i];
                    const double path_acceleration =
                        (speeds[i + 1] * speeds[i + 1] - speeds[i] * speeds[i]) / (2.0 * ds);
                    const auto span = (*stage_span)[i];
                    bindings.push_back(binding_for_stage(*shared_samples, static_cast<uint32_t>(span),
                        static_cast<uint32_t>(i), 0.5 * (positions[i] + positions[i + 1]),
                        std::max(0.0, speeds[i] * speeds[i] + path_acceleration * ds),
                        path_acceleration, velocity_limits.data(), acceleration_limits.data(), limits->caps[span]));
                }
                return bindings;
            };
            std::lock_guard lock(mutex);
            laws.at(created.id).make_bindings = std::move(make);
        }
        *out_law = law_owner.release();
        *out_trajectory = trajectory_owner.release();
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}

// A window view copies only its own sample/grid slice. It preserves the
// prepared whole-path policy and never recomputes adaptive nodes or reserves.
struct PathProfileSlice {
    std::shared_ptr<const std::vector<mk_path_sample>> samples;
    std::vector<size_t> span_first;
    toppra::Vector grid;
    std::shared_ptr<const std::vector<uint32_t>> stage_span;
    std::shared_ptr<const PathSpeedCaps> caps;
    bool nonlinear;
    size_t first_span, after_span, first_node;
    PathProfileSlice(const std::shared_ptr<const std::vector<mk_path_sample>>& all_samples,
            const PreparedTimingPath& prepared, size_t window) : nonlinear(prepared.nonlinear) {
        first_span = prepared.window_first.at(window);
        after_span = window + 1 < prepared.window_first.size() ? prepared.window_first[window + 1] : prepared.span_count;
        first_node = prepared.span_first[first_span];
        const size_t last_node = prepared.span_first[after_span];
        samples = std::make_shared<const std::vector<mk_path_sample>>(
            all_samples->begin() + first_span, all_samples->begin() + after_span + 1);
        for (size_t span = first_span; span <= after_span; ++span)
            span_first.push_back(prepared.span_first[span] - first_node);
        grid = prepared.grid.segment(static_cast<Eigen::Index>(first_node),
            static_cast<Eigen::Index>(last_node - first_node + 1));
        std::vector<uint32_t> spans;
        spans.reserve(last_node - first_node);
        for (size_t node = first_node; node < last_node; ++node)
            spans.push_back((*prepared.shared_stage_span)[node] - static_cast<uint32_t>(first_span));
        stage_span = std::make_shared<const std::vector<uint32_t>>(std::move(spans));
        caps = std::make_shared<const PathSpeedCaps>(PathSpeedCaps{
            std::vector<double>(prepared.knots.begin() + first_span, prepared.knots.begin() + after_span + 1),
            std::vector<double>(prepared.caps.begin() + first_span, prepared.caps.begin() + after_span)});
    }
    PathProfileView view() const { return {samples, span_first, grid, stage_span, caps, nonlinear}; }
};
struct AffineTiming {
    std::vector<mk_time_stage> stages;
    std::vector<mk_timing_binding> bindings;
    NativeResultOwner<mk_trajectory_handle, mk_trajectory_destroy> trajectory;
};

// The declared derivatives identify a straight joint path. Endpoint storage can
// still add tiny Hermite terms: certify the linear replacement against that
// polynomial instead of amplifying those terms on a short time interval.
std::unique_ptr<AffineTiming> prepare_stopped_affine(const std::vector<mk_path_sample>& samples,
        const double* velocity, const double* acceleration, const double* caps,
        uint32_t cap_count, double start_speed, double end_speed, double tolerance) {
    if (samples.size()!=2 || start_speed!=0 || end_speed!=0) return {};
    const auto &a=samples[0], &b=samples[1];
    const long double length=static_cast<long double>(b.s)-a.s;
    long double V=cap_count && caps[0]>0 ? caps[0] : std::numeric_limits<long double>::infinity();
    long double A=std::numeric_limits<long double>::infinity();
    for (uint32_t joint=0;joint<a.joint_count;++joint) {
        if (a.second[joint]!=0 || b.second_before[joint]!=0 || a.first[joint]!=b.first[joint]) return {};
        const long double delta=static_cast<long double>(b.position[joint])-a.position[joint];
        if (delta!=0) {
            V=std::min(V,velocity[joint]*length/std::abs(delta));
            A=std::min(A,acceleration[joint]*length/std::abs(delta));
        }
    }
    if (!(V>0) || !(A>0) || !std::isfinite(V) || !std::isfinite(A)) return {};
    PathProfilePhase profile("closed_form_timing");
    const long double peak=std::min(V,std::sqrt(A*length));
    const long double acceleration_time=peak/A;
    const long double cruise_time=std::max(0.0L,length/peak-acceleration_time);
    const long double acceleration_ticks=std::ceil(acceleration_time*1e9L);
    const long double cruise_ticks=std::ceil(cruise_time*1e9L);
    if (!(acceleration_ticks>=1) || 2*acceleration_ticks+cruise_ticks>=INT64_MAX/2) return {};
    const auto ta=static_cast<int64_t>(acceleration_ticks), tc=static_cast<int64_t>(cruise_ticks);
    const long double T=ta*1e-9L, C=tc*1e-9L;
    // Upward phase rounding makes both A=L/(T*(T+C)) and V=L/(T+C)
    // no larger than their continuous bounds. There is no empirical reserve.
    const double actual_a=static_cast<double>(length/(T*(T+C)));
    const double actual_v=static_cast<double>(length/(T+C));
    auto result=std::make_unique<AffineTiming>();
    auto phase=[&](int64_t start,int64_t duration,double position,double speed,double control) {
        mk_time_stage stage{};stage.struct_size=sizeof(stage);stage.start_ns=start;stage.duration_ns=duration;
        stage.start_s=position;stage.speed=speed;stage.acceleration=control;result->stages.push_back(stage);
    };
    const double half_distance=static_cast<double>(length*T/(2*(T+C)));
    phase(0,ta,a.s,0,actual_a);
    if (tc) phase(ta,tc,a.s+half_distance,actual_v,0);
    phase(ta+tc,ta,b.s-half_distance,actual_v,-actual_a);
    if (mk_trajectory_create(a.joint_count,&result->trajectory.value)!=MK_OK) return {};
    for (size_t index=0;index<result->stages.size();++index) {
        const auto& stage=result->stages[index];
        mk_segment segment{};segment.struct_size=sizeof(segment);segment.joint_count=a.joint_count;
        segment.degree=2;segment.t0_ns=stage.start_ns;segment.duration_ns=stage.duration_ns;
        const double seconds=stage.duration_ns*1e-9;
        const double normalized[]={static_cast<double>((stage.start_s-a.s)/length),
            static_cast<double>(stage.speed*seconds/length),
            static_cast<double>(0.5*stage.acceleration*seconds*seconds/length)};
        mk_timing_binding binding{};binding.struct_size=sizeof(binding);binding.stage_index=index;
        binding.kind=MK_TIMING_BINDING_FEED_CAP;binding.joint=UINT32_MAX;
        binding.limit=cap_count ? caps[0] : 0;
        const double middle_speed=stage.speed+0.5*stage.acceleration*seconds;
        double ratio=binding.limit>0 ? middle_speed/binding.limit : -1;
        for (uint32_t joint=0;joint<a.joint_count;++joint) {
            const long double delta=static_cast<long double>(b.position[joint])-a.position[joint];
            const long double rate=delta/length;
            auto& c=segment.coefficients[joint].value;
            c[0]=static_cast<double>(a.position[joint]+rate*(stage.start_s-a.s));
            c[1]=static_cast<double>(rate*stage.speed);c[2]=static_cast<double>(0.5L*rate*stage.acceleration);
            const auto original=small_compose(small_hermite(a,b,joint),normalized,13);
            double bound=0,scale=0,power=1;
            for (int k=0;k<original.n;++k) {
                const double candidate=k<=2 ? c[k]*power : 0;
                bound+=std::abs(original.c[k]-candidate);scale+=std::abs(original.c[k])+std::abs(candidate);
                power*=seconds;
            }
            // Horner composition, normalization and the coefficient sum use
            // fewer than 512 operations at degree <=5; retain that arithmetic reserve.
            const double rounding=512*std::numeric_limits<double>::epsilon()*scale+
                512*std::numeric_limits<double>::denorm_min();
            if (!std::isfinite(bound) || bound+rounding>tolerance) return {};
            const double v=std::abs(static_cast<double>(rate)*middle_speed)/velocity[joint];
            const double acc=std::abs(static_cast<double>(rate)*stage.acceleration)/acceleration[joint];
            if (v>ratio) {ratio=v;binding.kind=MK_TIMING_BINDING_JOINT_VELOCITY;binding.joint=joint;binding.limit=velocity[joint];}
            if (acc>ratio) {ratio=acc;binding.kind=MK_TIMING_BINDING_JOINT_ACCELERATION;binding.joint=joint;binding.limit=acceleration[joint];}
        }
        if (mk_trajectory_append_segment(result->trajectory.value,&segment)!=MK_OK) return {};
        result->bindings.push_back(binding);
    }
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=a.joint_count;
    for (uint32_t j=0;j<a.joint_count;++j) {
        limits.derivative_claimed[j]=3;limits.max_velocity[j]=velocity[j];limits.max_acceleration[j]=acceleration[j];
    }
    mk_validation_report report{};report.struct_size=sizeof(report);
    if (mk_validate(result->trajectory.value,&limits,&report)!=MK_OK ||
        report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_FAILED ||
        report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_FAILED) return {};
    return result;
}

struct FastTimingRevision {};
struct PathTimingStream {
    std::mutex access;
    std::shared_ptr<const std::vector<mk_path_sample>> samples;
    std::vector<double> velocity, acceleration;
    std::unique_ptr<PreparedTimingPath> prepared;
    std::unique_ptr<AffineTiming> affine;
    std::shared_ptr<const FastTimingRevision> fast_revision;
    std::unique_ptr<motionkit::WindowTimingStream> queue;
    std::unique_ptr<PathProfileSlice> head_slice;
    std::shared_ptr<const motionkit::ClockProfileCursor> clock;
    double start_speed, end_speed, lowering_tolerance;
    uint32_t require_feasible_path;
    size_t capacity, head = 0, next = 0;
};
struct PathTimingPreview {
    std::optional<motionkit::WindowTimingStream::Preview> token;
    size_t window;
    std::shared_ptr<const motionkit::ClockProfileCursor> clock;
    std::shared_ptr<const FastTimingRevision> fast_revision;
};
std::unordered_map<uint32_t, std::shared_ptr<PathTimingStream>> timing_streams;
std::unordered_map<uint32_t, std::shared_ptr<PathTimingPreview>> timing_previews;
uint32_t next_timing_stream = 1, next_timing_preview = 1;

mk_result time_path(mk_path_handle path, const double *max_velocity,
                    const double *max_acceleration, uint32_t joint_count,
                    const double *speed_caps, uint32_t speed_cap_count,
                    double start_speed, double end_speed,
                    double lowering_tolerance, uint32_t require_feasible_path,
                    mk_timing_report *out_report,
                    mk_time_law_handle *out_law,
                    mk_trajectory_handle *out_trajectory, bool reference, PathTimingSession* session = nullptr,
                    const toppra::Bound* terminal_squared = nullptr, double* reached_speed = nullptr) {
    TimingSessionAttempt attempt{session};
    if (reached_speed) *reached_speed = 0;
    if (!out_law || !out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_law->id = 0;
    out_trajectory->id = 0;
    if(out_report){*out_report={};out_report->struct_size=sizeof(*out_report);out_report->joint=UINT32_MAX;out_report->sustainable_scale=1;}
    if (terminal_squared && (!terminal_squared->allFinite() || (*terminal_squared)[0] < 0 ||
            (*terminal_squared)[0] > (*terminal_squared)[1])) return MK_ERROR_INVALID_ARGUMENT;
    if(require_feasible_path>1)return MK_ERROR_INVALID_ARGUMENT;
    if (!std::isfinite(lowering_tolerance) || lowering_tolerance <= 0.0)
        return MK_ERROR_INVALID_ARGUMENT;
    std::shared_ptr<const std::vector<mk_path_sample>> shared_samples;
    try {
        if (session) shared_samples = session->samples;
        else {
            std::lock_guard lock(mutex);
            const auto found = paths.find(path.id);
            if (found == paths.end()) return MK_ERROR_INVALID_HANDLE;
            shared_samples = std::make_shared<const std::vector<mk_path_sample>>(found->second);
        }
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    const auto& samples = *shared_samples;
    if (!valid_timing_request(samples, max_velocity, max_acceleration, joint_count,
        speed_caps, speed_cap_count, start_speed, end_speed))
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        const bool moving_boundary = start_speed > 0.0 || end_speed > 0.0;
        std::unique_ptr<PreparedTimingPath> local_prepared;
        if (!session) local_prepared = std::make_unique<PreparedTimingPath>(samples, max_velocity, max_acceleration,
            joint_count, speed_caps, speed_cap_count, moving_boundary, reference);
        const auto& prepared = session ? session->prepare(moving_boundary) : *local_prepared;
        const auto& grid_values = prepared.grid_values;
        const auto& span_first = prepared.span_first;
        const auto& window_first = prepared.window_first;
        if (out_report) {
            out_report->windows = static_cast<uint32_t>(window_first.size());
            out_report->boundary_lowerings = 0;
        }
        toppra::Bound terminal;
        if (terminal_squared) terminal = *terminal_squared;
        else terminal.setConstant(end_speed * end_speed);
        std::vector<double> speeds(grid_values.size(), 0.0);
        double reached_squared = start_speed * start_speed;
        {
            PathProfilePhase stage_profile("solver_preparation");
            std::unique_ptr<motionkit::RetainedWindowTiming> local_retained;
            if (!session) local_retained = prepared.make_serial_timing();
            auto& timing = session ? session->timing() : *local_retained;
            stage_profile.mark("toppra_backward");
            auto result = timing.backward(terminal);
            std::vector<toppra::Vector> profiles;
            stage_profile.mark("toppra_forward");
            if (result == toppra::ReturnCode::OK) result = timing.forward(reached_squared, profiles);
            stage_profile.mark("profile_transfer");
            if (result != toppra::ReturnCode::OK) {
                if (std::getenv("PROCESS_PATH_PROFILE"))
                    std::fprintf(stderr, "PROCESS_PATH_WINDOW result=%d %s\n",
                        static_cast<int>(result), timing.error().c_str());
                return MK_ERROR_GENERATION;
            }
            for (size_t window = 0; window < profiles.size(); ++window) {
                const size_t first_node = span_first[window_first[window]];
                for (Eigen::Index node = 0; node < profiles[window].size(); ++node)
                    speeds[first_node + static_cast<size_t>(node)] =
                        std::sqrt(std::max(0.0, profiles[window][node]));
            }
            if (std::getenv("PROCESS_PATH_PROFILE"))
                std::fprintf(stderr, "PROCESS_PATH_WINDOW_PREPARATION workers=0 prepared=0 windows=%zu\n",
                    window_first.size());
        }

        speeds.front() = start_speed;
        // A terminal interval is permission, not a promise to attain its upper bound.
        if (terminal_squared) end_speed = speeds.back();
        speeds.back() = end_speed;
        const PathProfileView view{shared_samples, prepared.span_first, prepared.grid,
            prepared.shared_stage_span, prepared.window_limits, prepared.nonlinear};
        const auto result = lower_timed_profile(view, {max_velocity, max_acceleration, joint_count},
            {start_speed, end_speed, lowering_tolerance, require_feasible_path}, std::move(speeds),
            out_report, out_law, out_trajectory);
        if (result != MK_OK) return result;
        if (reached_speed) *reached_speed = end_speed;
        attempt.accepted = true;
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
    PathProfilePhase stage_profile("native_timing_total");
    return time_path(path, max_velocity, max_acceleration, joint_count, speed_caps, speed_cap_count, start_speed,
        end_speed, lowering_tolerance, require_feasible_path, out_report, out_law, out_trajectory, false);
}

mk_result MK_CALL mk_path_timing_session_create(mk_path_handle path,
        const double* max_velocity, const double* max_acceleration, uint32_t joint_count,
        const double* speed_caps, uint32_t speed_cap_count, double lowering_tolerance,
        uint32_t require_feasible_path, mk_path_timing_session_handle* out_session) {
    if (!out_session) return MK_ERROR_INVALID_ARGUMENT;
    out_session->id = 0;
    if (!std::isfinite(lowering_tolerance) || lowering_tolerance <= 0 || require_feasible_path > 1)
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        auto session = std::make_shared<PathTimingSession>();
        {
            std::lock_guard lock(mutex);
            const auto found = paths.find(path.id);
            if (found == paths.end()) return MK_ERROR_INVALID_HANDLE;
            session->samples = std::make_shared<const std::vector<mk_path_sample>>(found->second);
        }
        if (!valid_timing_request(*session->samples, max_velocity, max_acceleration, joint_count,
                speed_caps, speed_cap_count, 0, 0)) return MK_ERROR_INVALID_ARGUMENT;
        session->velocity.assign(max_velocity, max_velocity + joint_count);
        session->acceleration.assign(max_acceleration, max_acceleration + joint_count);
        if (speed_cap_count) session->caps.assign(speed_caps, speed_caps + speed_cap_count);
        session->lowering_tolerance = lowering_tolerance;
        session->require_feasible_path = require_feasible_path;
        std::lock_guard lock(mutex);
        while (!next_timing_session || timing_sessions.count(next_timing_session)) ++next_timing_session;
        const uint32_t id = next_timing_session++;
        timing_sessions.emplace(id, std::move(session));
        out_session->id = id;
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}

void MK_CALL mk_path_timing_session_destroy(mk_path_timing_session_handle handle) {
    std::shared_ptr<PathTimingSession> removed;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_sessions.find(handle.id);
        if (found == timing_sessions.end()) return;
        removed = std::move(found->second);
        timing_sessions.erase(found);
    } // Release large tables outside the global handle lock; in-flight calls own their session.
}

mk_result MK_CALL mk_path_timing_session_time(mk_path_timing_session_handle handle,
        double start_speed, double end_speed, mk_timing_report* out_report,
        mk_time_law_handle* out_law, mk_trajectory_handle* out_trajectory) {
    if (out_law) out_law->id = 0;
    if (out_trajectory) out_trajectory->id = 0;
    if (out_report) {
        *out_report = {};
        out_report->struct_size = sizeof(*out_report);
        out_report->joint = UINT32_MAX;
        out_report->sustainable_scale = 1;
    }
    std::shared_ptr<PathTimingSession> session;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_sessions.find(handle.id);
        if (found == timing_sessions.end()) return MK_ERROR_INVALID_HANDLE;
        session = found->second;
    }
    std::lock_guard owner(session->access);
    return time_path({}, session->velocity.data(), session->acceleration.data(),
        static_cast<uint32_t>(session->velocity.size()), session->caps.empty() ? nullptr : session->caps.data(),
        static_cast<uint32_t>(session->caps.size()), start_speed, end_speed, session->lowering_tolerance,
        session->require_feasible_path, out_report, out_law, out_trajectory, false, session.get());
}

mk_result MK_CALL mk_path_timing_session_time_to_interval(mk_path_timing_session_handle handle,
        double start_speed, double end_lower_speed, double end_upper_speed, double* out_reached_speed,
        mk_timing_report* out_report, mk_time_law_handle* out_law, mk_trajectory_handle* out_trajectory) {
    if (out_reached_speed) *out_reached_speed = 0;
    if (out_law) out_law->id = 0;
    if (out_trajectory) out_trajectory->id = 0;
    if (out_report) {
        *out_report = {};
        out_report->struct_size = sizeof(*out_report);
        out_report->joint = UINT32_MAX;
        out_report->sustainable_scale = 1;
    }
    std::shared_ptr<PathTimingSession> session;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_sessions.find(handle.id);
        if (found == timing_sessions.end()) return MK_ERROR_INVALID_HANDLE;
        session = found->second;
    }
    std::lock_guard owner(session->access);
    if (!out_reached_speed || !std::isfinite(end_lower_speed) || end_lower_speed < 0 ||
            !std::isfinite(end_upper_speed) || end_upper_speed < end_lower_speed) {
        session->retained.reset();
        return MK_ERROR_INVALID_ARGUMENT;
    }
    const toppra::Bound terminal{end_lower_speed * end_lower_speed, end_upper_speed * end_upper_speed};
    return time_path({}, session->velocity.data(), session->acceleration.data(),
        static_cast<uint32_t>(session->velocity.size()), session->caps.empty() ? nullptr : session->caps.data(),
        static_cast<uint32_t>(session->caps.size()), start_speed, end_upper_speed, session->lowering_tolerance,
        session->require_feasible_path, out_report, out_law, out_trajectory, false, session.get(), &terminal,
        out_reached_speed);
}

mk_result MK_CALL mk_path_timing_session_entry_interval(mk_path_timing_session_handle handle,
        double end_lower_speed, double end_upper_speed, uint32_t entry_may_move,
        double* out_lower_speed, double* out_upper_speed) {
    if (out_lower_speed) *out_lower_speed = 0;
    if (out_upper_speed) *out_upper_speed = 0;
    std::shared_ptr<PathTimingSession> session;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_sessions.find(handle.id);
        if (found == timing_sessions.end()) return MK_ERROR_INVALID_HANDLE;
        session = found->second;
    }
    std::lock_guard owner(session->access);
    TimingSessionAttempt attempt{session.get()};
    if (!out_lower_speed || !out_upper_speed || entry_may_move > 1 || !std::isfinite(end_lower_speed) ||
            end_lower_speed < 0 || !std::isfinite(end_upper_speed) || end_lower_speed > end_upper_speed)
        return MK_ERROR_INVALID_ARGUMENT;
    const toppra::Bound terminal{end_lower_speed * end_lower_speed, end_upper_speed * end_upper_speed};
    if (!terminal.allFinite()) return MK_ERROR_INVALID_ARGUMENT;
    try {
        session->prepare(entry_may_move || end_upper_speed > 0);
        auto& timing = session->timing();
        if (timing.backward(terminal) != toppra::ReturnCode::OK) return MK_ERROR_GENERATION;
        const auto entry = timing.entrySquaredBounds();
        if (!entry_may_move) {
            if (entry[0] > 0 || entry[1] < 0) return MK_ERROR_GENERATION;
        } else {
            *out_lower_speed = std::sqrt(std::max(0.0, entry[0]));
            *out_upper_speed = std::sqrt(std::max(0.0, entry[1]));
        }
        attempt.accepted = true;
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}

mk_result MK_CALL mk_path_timing_stream_create(mk_path_handle path,
        const double* velocity, const double* acceleration, uint32_t joint_count,
        const double* caps, uint32_t cap_count, double start_speed, double end_speed,
        double tolerance, uint32_t require_feasible, uint32_t capacity,
        mk_path_timing_stream_handle* out_stream) {
    if (!out_stream) return MK_ERROR_INVALID_ARGUMENT;
    out_stream->id = 0;
    if (!capacity || !std::isfinite(tolerance) || tolerance <= 0 || require_feasible > 1 ||
            !std::isfinite(start_speed * start_speed) || !std::isfinite(end_speed * end_speed) ||
            start_speed * start_speed > MAX_PATH_SQUARED_SPEED || end_speed * end_speed > MAX_PATH_SQUARED_SPEED)
        return MK_ERROR_INVALID_ARGUMENT;
    try {
        auto stream = std::make_shared<PathTimingStream>();
        {
            std::lock_guard lock(mutex);
            const auto found = paths.find(path.id);
            if (found == paths.end()) return MK_ERROR_INVALID_HANDLE;
            stream->samples = std::make_shared<const std::vector<mk_path_sample>>(found->second);
        }
        if (!valid_timing_request(*stream->samples, velocity, acceleration, joint_count,
                caps, cap_count, start_speed, end_speed)) return MK_ERROR_INVALID_ARGUMENT;
        stream->velocity.assign(velocity, velocity + joint_count);
        stream->acceleration.assign(acceleration, acceleration + joint_count);
        stream->affine=prepare_stopped_affine(*stream->samples,velocity,acceleration,caps,cap_count,
            start_speed,end_speed,tolerance);
        if (!stream->affine) {
            stream->prepared = std::make_unique<PreparedTimingPath>(*stream->samples, velocity, acceleration,
                joint_count, caps, cap_count, start_speed > 0 || end_speed > 0, false, true);
            if (end_speed > 0 && stream->prepared->window_first.size() > capacity) return MK_ERROR_LIMIT;
            stream->queue = std::make_unique<motionkit::WindowTimingStream>(start_speed * start_speed,
                capacity, MAX_PATH_SQUARED_SPEED);
        }
        stream->capacity = capacity; stream->start_speed = start_speed; stream->end_speed = end_speed;
        stream->lowering_tolerance = tolerance; stream->require_feasible_path = require_feasible;
        std::lock_guard lock(mutex);
        while (!next_timing_stream || timing_streams.count(next_timing_stream)) ++next_timing_stream;
        const uint32_t id = next_timing_stream++;
        timing_streams.emplace(id, std::move(stream));
        out_stream->id = id;
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (...) { return MK_ERROR_GENERATION; }
}
void MK_CALL mk_path_timing_stream_destroy(mk_path_timing_stream_handle handle) {
    std::shared_ptr<PathTimingStream> removed;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_streams.find(handle.id);
        if (found == timing_streams.end()) return;
        removed = std::move(found->second); timing_streams.erase(found);
    }
}
void MK_CALL mk_path_timing_preview_destroy(mk_path_timing_preview_handle handle) {
    std::shared_ptr<PathTimingPreview> removed;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_previews.find(handle.id);
        if (found == timing_previews.end()) return;
        removed = std::move(found->second); timing_previews.erase(found);
    }
}
mk_result MK_CALL mk_path_timing_stream_step(mk_path_timing_stream_handle handle,
        mk_path_timing_stream_progress* out_progress, mk_path_timing_preview_handle* out_preview,
        mk_timing_report* out_report, mk_time_law_handle* out_law, mk_trajectory_handle* out_trajectory) {
    if (out_progress) { *out_progress = {}; out_progress->struct_size = sizeof(*out_progress); }
    if (out_preview) out_preview->id = 0;
    if (out_law) out_law->id = 0;
    if (out_trajectory) out_trajectory->id = 0;
    if (out_report) { *out_report = {}; out_report->struct_size = sizeof(*out_report); out_report->joint = UINT32_MAX; out_report->sustainable_scale = 1; }
    if (!out_progress || !out_preview || !out_law || !out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    std::shared_ptr<PathTimingStream> stream;
    {
        std::lock_guard lock(mutex);
        const auto found = timing_streams.find(handle.id);
        if (found == timing_streams.end()) return MK_ERROR_INVALID_HANDLE;
        stream = found->second;
    }
    std::lock_guard owner(stream->access);
    try {
        mk_path_timing_stream_progress progress{};
        progress.struct_size = sizeof(progress);
        progress.window_index = static_cast<uint32_t>(stream->head);
        if (stream->affine) {
            progress.total_windows=1;
            if (stream->head) { progress.state=MK_TIMING_STREAM_DONE;*out_progress=progress;return MK_OK; }
            auto& affine=*stream->affine;
            NativeResultOwner<mk_time_law_handle,mk_time_law_destroy> law;
            NativeResultOwner<mk_trajectory_handle,mk_trajectory_destroy> trajectory;
            auto status=mk_time_law_create(affine.stages.data(),affine.stages.size(),&law.value);
            if (status!=MK_OK) return status;
            const auto &last=affine.stages.back();
            status=mk_trajectory_slice(affine.trajectory.value,0,last.start_ns+last.duration_ns,&trajectory.value);
            if (status!=MK_OK) return status;
            auto revision=std::make_shared<const FastTimingRevision>();
            auto token=std::make_shared<PathTimingPreview>(PathTimingPreview{std::nullopt,0,nullptr,revision});
            uint32_t id;
            {
                std::lock_guard lock(mutex);
                laws.at(law.value.id).bindings=affine.bindings;
                while (!next_timing_preview || timing_previews.count(next_timing_preview)) ++next_timing_preview;
                id=next_timing_preview++;timing_previews.emplace(id,std::move(token));
            }
            stream->fast_revision=std::move(revision);
            progress.state=MK_TIMING_STREAM_READY;progress.span_count=1;
            progress.start_s=stream->samples->front().s;progress.end_s=stream->samples->back().s;
            if (out_report) out_report->windows=1;
            *out_progress=progress;out_preview->id=id;*out_law=law.release();*out_trajectory=trajectory.release();
            return MK_OK;
        }
        PathProfilePhase stream_profile("stream_queue_append");
        const auto& prepared = *stream->prepared;
        progress.total_windows = static_cast<uint32_t>(prepared.window_first.size());
        if (stream->queue->finished()) {
            progress.state = MK_TIMING_STREAM_DONE; *out_progress = progress; return MK_OK;
        }
        if (stream->next < prepared.window_first.size() && stream->queue->pendingWindows() < stream->capacity) {
            stream->queue->append(prepared.make_window(stream->next, std::make_shared<toppra::solver::Seidel>()));
            ++stream->next;
            if (stream->next == prepared.window_first.size()) stream->queue->finish(stream->end_speed * stream->end_speed);
        }
        if (stream->end_speed > 0 && stream->next < prepared.window_first.size()) {
            progress.state = MK_TIMING_STREAM_WAITING; *out_progress = progress; return MK_OK;
        }
        stream_profile.mark("stream_queue_preview");
        auto candidate = stream->queue->preview();
        if (!candidate) {
            if (stream->queue->pendingWindows() == stream->capacity && stream->next < prepared.window_first.size())
                return MK_ERROR_LIMIT;
            progress.state = MK_TIMING_STREAM_WAITING; *out_progress = progress; return MK_OK;
        }
        stream_profile.mark("stream_clock_construction");
        if (!stream->clock) stream->clock=std::make_shared<const motionkit::ClockProfileCursor>(
            stream->samples->front().s,stream->start_speed);
        if (!stream->head_slice) stream->head_slice = std::make_unique<PathProfileSlice>(stream->samples, prepared, stream->head);
        const auto& slice = *stream->head_slice;
        const auto& squared = candidate->profiles().front();
        std::vector<double> speeds;
        speeds.reserve(static_cast<size_t>(squared.size()));
        for (Eigen::Index i = 0; i < squared.size(); ++i) speeds.push_back(std::sqrt(std::max(0.0, squared[i])));
        progress.start_speed = stream->head == 0 ? stream->start_speed : std::sqrt(stream->queue->entrySquaredSpeed());
        progress.end_speed = stream->head + 1 == prepared.window_first.size() ? stream->end_speed :
            std::sqrt(std::max(0.0, candidate->endSquaredSpeed()));
        speeds.front() = progress.start_speed; speeds.back() = progress.end_speed;
        progress.first_span = static_cast<uint32_t>(slice.first_span);
        progress.span_count = static_cast<uint32_t>(slice.after_span - slice.first_span);
        progress.first_stage = static_cast<uint32_t>(slice.first_node);
        progress.start_s = slice.samples->front().s; progress.end_s = slice.samples->back().s;
        const auto& original_controls=candidate->controls().front();
        if (const char* profile = std::getenv("PROCESS_PATH_PROFILE"); profile && std::strcmp(profile, "1") == 0) {
            double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
            for (Eigen::Index i = 0; i < original_controls.size(); ++i) {
                const double control = original_controls[i];
                minimum = std::min(minimum, control); maximum = std::max(maximum, control);
            }
            std::fprintf(stderr, "PROCESS_PATH_STREAM_PREVIEW {\"window\":%zu,\"firstSpan\":%zu,\"spanCount\":%zu,\"gridNodes\":%td,\"startDistance\":%.17g,\"endDistance\":%.17g,\"startSpeed\":%.17g,\"endSpeed\":%.17g,\"rawPathAccelerationMin\":%.17g,\"rawPathAccelerationMax\":%.17g}\n",
                stream->head, slice.first_span, slice.after_span - slice.first_span,
                static_cast<std::ptrdiff_t>(slice.grid.size()), progress.start_s, progress.end_s,
                progress.start_speed, progress.end_speed, minimum, maximum);
        }
        const std::vector<double> controls(original_controls.data(),original_controls.data()+original_controls.size());
        auto next_clock=std::make_shared<motionkit::ClockProfileCursor>(*stream->clock);
        double clock_velocity_bound=0, clock_acceleration_bound=0;
        for (size_t i=0;i<speeds.size();++i) {
            clock_velocity_bound=std::max(clock_velocity_bound,speeds[i]);
            if (i) clock_acceleration_bound=std::max(clock_acceleration_bound,
                std::abs((squared[static_cast<Eigen::Index>(i)]-squared[static_cast<Eigen::Index>(i-1)]) /
                    (2*(slice.grid[static_cast<Eigen::Index>(i)]-slice.grid[static_cast<Eigen::Index>(i-1)]))));
        }
        if (clock_acceleration_bound==0) {
            // A constant-speed reference still needs a nonzero search domain
            // to correct a fractional final clock tick. Affine joint limits
            // supply it; curved candidates remain subject to full validation.
            clock_acceleration_bound=std::numeric_limits<double>::infinity();
            for (const auto& point:*slice.samples) for (size_t j=0;j<stream->acceleration.size();++j)
                if (point.first[j]!=0) clock_acceleration_bound=std::min(clock_acceleration_bound,
                    stream->acceleration[j]/std::abs(point.first[j]));
        }
        // Rounded speed endpoints and original LP controls can differ on tiny
        // spans. Cover both in this search domain; only the original controls
        // drive integration, and physical validation still certifies the result.
        for (const double control:controls) clock_acceleration_bound=std::max(clock_acceleration_bound,std::abs(control));
        // Existing moving-boundary slack supplies the final clock candidate's
        // search domain. These scalar search bounds are not joint certificates;
        // the unchanged continuous validator below remains authoritative.
        const auto pieces=next_clock->append(std::vector<double>(slice.grid.data(),slice.grid.data()+slice.grid.size()),
            speeds,stream->head+1==prepared.window_first.size(),
            clock_velocity_bound/MOVING_BOUNDARY_MARGIN,clock_acceleration_bound/MOVING_BOUNDARY_MARGIN,&controls);
        std::vector<mk_time_stage> clock_stages;
        clock_stages.reserve(pieces.size());
        for (const auto& piece:pieces) {
            mk_time_stage stage{}; stage.struct_size=sizeof(stage);
            stage.start_ns=piece.start_tick; stage.duration_ns=piece.duration;
            stage.start_s=piece.position; stage.speed=piece.velocity; stage.acceleration=piece.acceleration;
            clock_stages.push_back(stage);
        }
        progress.start_s=stream->clock->position(); progress.start_speed=stream->clock->velocity();
        progress.end_s=next_clock->position(); progress.end_speed=next_clock->velocity();
        NativeResultOwner<mk_time_law_handle, mk_time_law_destroy> law;
        NativeResultOwner<mk_trajectory_handle, mk_trajectory_destroy> trajectory;
        if (out_report) { out_report->windows = 1; out_report->boundary_lowerings = 0; }
        stream_profile.mark("stream_lowering_total");
        auto status = lower_timed_profile(slice.view(), {stream->velocity.data(), stream->acceleration.data(),
            static_cast<uint32_t>(stream->velocity.size())}, {progress.start_speed, progress.end_speed,
            stream->lowering_tolerance, stream->require_feasible_path}, std::move(speeds), out_report,
            &law.value, &trajectory.value, &clock_stages, &stream->samples);
        if (status != MK_OK) return status;
        stream_profile.mark("stream_preview_registration");
        auto token = std::make_shared<PathTimingPreview>(PathTimingPreview{std::move(*candidate), stream->head,
            std::move(next_clock),nullptr});
        uint32_t id;
        {
            std::lock_guard lock(mutex);
            while (!next_timing_preview || timing_previews.count(next_timing_preview)) ++next_timing_preview;
            id = next_timing_preview++;
            timing_previews.emplace(id, std::move(token));
        }
        progress.state = MK_TIMING_STREAM_READY;
        *out_progress = progress; out_preview->id = id;
        *out_law = law.release(); *out_trajectory = trajectory.release();
        return MK_OK;
    } catch (const std::bad_alloc&) { return MK_ERROR_OUT_OF_MEMORY; }
    catch (const std::length_error&) { return MK_ERROR_LIMIT; }
    catch (const std::exception& error) {
        if (const char* profile=std::getenv("PROCESS_PATH_PROFILE"); profile && std::strcmp(profile,"1")==0)
            std::fprintf(stderr,"PROCESS_PATH_STREAM_FAILURE %s\n",error.what());
        return MK_ERROR_GENERATION;
    }
    catch (...) { return MK_ERROR_GENERATION; }
}
mk_result MK_CALL mk_path_timing_stream_commit(mk_path_timing_stream_handle handle,
        mk_path_timing_preview_handle preview) {
    std::shared_ptr<PathTimingStream> stream;
    std::shared_ptr<PathTimingPreview> token;
    {
        std::lock_guard lock(mutex);
        const auto a = timing_streams.find(handle.id);
        const auto b = timing_previews.find(preview.id);
        if (a == timing_streams.end() || b == timing_previews.end()) return MK_ERROR_INVALID_HANDLE;
        stream = a->second; token = b->second;
    }
    std::lock_guard owner(stream->access);
    try {
        if (token->window != stream->head) return MK_ERROR_INVALID_ARGUMENT;
        if (stream->affine) {
            if (!stream->fast_revision || token->fast_revision!=stream->fast_revision) return MK_ERROR_INVALID_ARGUMENT;
            ++stream->head;stream->fast_revision.reset();return MK_OK;
        }
        if (!token->token) return MK_ERROR_INVALID_ARGUMENT;
        // Solver state advances at its nominal geometry boundary; the owned
        // clock cursor retains the fractional tail until the next publication.
        stream->queue->retireCertified(*token->token);
        stream->clock=token->clock;
        ++stream->head; stream->head_slice.reset();
        return MK_OK;
    } catch (const std::logic_error&) { return MK_ERROR_INVALID_ARGUMENT; }
    catch (...) { return MK_ERROR_GENERATION; }
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

static mk_result advance_joint_context(mk_trajectory_handle trajectory, uint32_t joint, double lead_seconds,
        double window_seconds, int64_t origin_ns, bool final_context, mk_trajectory_handle *out_trajectory) {
    if (!out_trajectory) return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id = 0;
    if (!std::isfinite(lead_seconds) || lead_seconds < 0.0 || !std::isfinite(window_seconds) || window_seconds <= 0.0 ||
            origin_ns<0 || window_seconds*0.5e9>static_cast<double>(INT64_MAX/16))
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
    if (!segments || duration_ns<=0) return MK_ERROR_INVALID_ARGUMENT;
    const int64_t half_ns = std::max<int64_t>(1, static_cast<int64_t>(std::llround(window_seconds * 0.5e9)));
    if (half_ns>INT64_MAX/16 || duration_ns>INT64_MAX-half_ns || origin_ns>INT64_MAX-duration_ns)
        return MK_ERROR_INVALID_ARGUMENT;
    const double window = static_cast<double>(2 * half_ns) * 1e-9;
    // Cut at source boundaries, their half-window shifts, and the true-end taper
    // cuts. Each piece has a known polynomial: degree five outside the taper,
    // up to fifteen when both tapers overlap. Preserve its quintic terms directly
    // and endpoint-fit only the higher-degree remainder. Other joints keep their
    // polynomials, re-expressed from the start of each piece.
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
        cut(window_ns*k/8-origin_ns);
        if (final_context) cut(duration_ns-window_ns*k/8);
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
            // Work in normalized local time. Outside the taper the pressure
            // advance is exactly quintic; refitting absolute positions over a
            // one-tick piece would turn their rounding into false acceleration.
            using Local = std::array<long double,16>;
            const long double h=static_cast<long double>(finish-begin)*1e-9L;
            auto shifted = [&](int64_t advance) {
                Local result{};
                const long double middle=static_cast<long double>(begin)+
                    static_cast<long double>(finish-begin)*0.5L+advance;
                if ((middle<0 && origin_ns==0) || (middle>=duration_ns && final_context)) {
                    const auto& edge=middle<0 ? source.front() : source.back();
                    const long double t=middle<0 ? 0.0L : edge.duration_ns*1e-9L;
                    for (int k=static_cast<int>(edge.degree);k>=0;--k)
                        result[0]=result[0]*t+edge.coefficients[joint].value[k];
                    return result;
                }
                auto found=std::upper_bound(source.begin(),source.end(),middle,
                    [](long double time,const mk_segment& segment){return time<segment.t0_ns;});
                // A retained context edge is not a physical stop. Continue
                // its polynomial outside the borrowed support; publication
                // still crops that speculative part using the filter horizon.
                const auto& segment=found==source.begin() ? source.front() : *(found-1);
                const long double offset=(static_cast<long double>(begin)+advance-segment.t0_ns)*1e-9L;
                long double time_power=1;
                for (unsigned m=0;m<=segment.degree;++m) {
                    long double binomial=1,power=1;
                    for (unsigned k=m;k<=segment.degree;++k) {
                        result[m]+=segment.coefficients[joint].value[k]*binomial*power*time_power;
                        binomial*=static_cast<long double>(k+1)/(k+1-m);power*=offset;
                    }
                    time_power*=h;
                }
                return result;
            };
            auto smooth = [&](long double start,long double slope) {
                Local result{};
                const long double middle=start+slope*0.5L;
                if (middle>=1) { result[0]=1;return result; }
                if (middle<=0) return result;
                const long double coefficients[]={0,0,0,10,-15,6};
                long double slope_power=1;
                for (int m=0;m<=5;++m) {
                    long double binomial=1,power=1;
                    for (int k=m;k<=5;++k) {
                        result[m]+=coefficients[k]*binomial*power*slope_power;
                        binomial*=static_cast<long double>(k+1)/(k+1-m);power*=start;
                    }
                    slope_power*=slope;
                }
                return result;
            };
            const long double window_ticks=2*static_cast<long double>(half_ns);
            const auto rise=smooth((static_cast<long double>(origin_ns)+begin)/window_ticks,
                static_cast<long double>(finish-begin)/window_ticks);
            Local fall{};fall[0]=1;
            if (final_context) fall=smooth(static_cast<long double>(duration_ns-begin)/window_ticks,
                -static_cast<long double>(finish-begin)/window_ticks);
            Local taper{},mean{},full=shifted(0);
            for (int i=0;i<=5;++i) for (int j=0;j<=5;++j) taper[i+j]+=rise[i]*fall[j];
            const auto plus=shifted(half_ns),minus=shifted(-half_ns);
            for (int i=0;i<=5;++i) mean[i]=(plus[i]-minus[i])*lead_seconds/window;
            for (int i=0;i<=5;++i) for (int j=0;j<=10;++j) full[i+j]+=mean[i]*taper[j];
            // Fit only the high-degree remainder. It and its first two
            // derivatives vanish at zero, so endpoint matching never subtracts
            // the large absolute position or velocity from its own prediction.
            long double value=0,first=0,second=0;
            for (int k=6;k<16;++k) {
                value+=full[k];first+=k*full[k];second+=k*(k-1)*full[k];
            }
            full[3]+=10*value-4*first+0.5L*second;
            full[4]+=-15*value+7*first-second;
            full[5]+=6*value-3*first+0.5L*second;
            long double power=1;
            for (int k=0;k<=5;++k) {
                piece->coefficients[joint].value[k]=static_cast<double>(full[k]/power);
                power*=h;
            }
            status = mk_trajectory_append_segment(advanced, piece.get());
        }
        if (status != MK_OK) { mk_trajectory_destroy(advanced); return status; }
    } catch (const std::bad_alloc &) { mk_trajectory_destroy(advanced); return MK_ERROR_OUT_OF_MEMORY; }
    *out_trajectory = advanced;
    return MK_OK;
}

mk_result MK_CALL mk_advance_joint(mk_trajectory_handle trajectory, uint32_t joint, double lead_seconds,
        double window_seconds, mk_trajectory_handle* out_trajectory) {
    return advance_joint_context(trajectory,joint,lead_seconds,window_seconds,0,true,out_trajectory);
}

mk_result MK_CALL mk_advance_joint_context(mk_trajectory_handle trajectory, uint32_t joint, double lead_seconds,
        double window_seconds, int64_t origin_ns, uint32_t final_context, mk_trajectory_handle* out_trajectory) {
    if (out_trajectory) out_trajectory->id=0;
    if (final_context>1) return MK_ERROR_INVALID_ARGUMENT;
    return advance_joint_context(trajectory,joint,lead_seconds,window_seconds,origin_ns,final_context!=0,out_trajectory);
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

mk_result MK_CALL mk_merge_segments_position_budget(mk_trajectory_handle trajectory,const double *position_tolerance,
    const double *velocity_limit,const double *acceleration_limit,const double *position_lower,const double *position_upper,
    uint32_t joint_count,double max_duration_seconds,mk_trajectory_handle *out_trajectory) {
    if(!out_trajectory)return MK_ERROR_INVALID_ARGUMENT;
    out_trajectory->id=0;
    uint32_t joints=0;
    const auto status=mk_trajectory_joint_count(trajectory,&joints);
    if(status!=MK_OK)return status;
    if(joints!=joint_count)return MK_ERROR_INVALID_ARGUMENT;
    return motionkit::merge_trajectory_position_budget(trajectory,{position_tolerance,nullptr,nullptr,
        velocity_limit,acceleration_limit,position_lower,position_upper,max_duration_seconds},out_trajectory);
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
