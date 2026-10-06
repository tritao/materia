#include "trajectory_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace motionkit {
namespace {

using Polynomial = std::array<long double, MK_MAX_DEGREE + 1>;

int degree_of(const Polynomial &coefficients, int degree) {
    while (degree > 0 && coefficients[degree] == 0.0L) --degree;
    return degree;
}

long double evaluate(const Polynomial &coefficients, int degree, long double time) {
    long double value = coefficients[degree];
    for (int index = degree - 1; index >= 0; --index)
        value = value * time + coefficients[index];
    return value;
}

Polynomial derivative(const Polynomial &coefficients, int degree) {
    Polynomial result{};
    for (int index = 1; index <= degree; ++index)
        result[index - 1] = coefficients[index] * index;
    return result;
}

void unique_sorted(std::vector<long double> &roots) {
    std::sort(roots.begin(), roots.end());
    roots.erase(std::unique(roots.begin(), roots.end(), [](long double a, long double b) {
        return std::abs(a - b) <= 1e-13L;
    }), roots.end());
}

/** Closed-form real roots for degree one through three. */
std::vector<long double> closed_roots(const Polynomial &coefficients, int requested_degree) {
    const int degree = degree_of(coefficients, requested_degree);
    std::vector<long double> roots;
    if (degree == 0) return roots;
    if (degree == 1) {
        roots.push_back(-coefficients[0] / coefficients[1]);
    } else if (degree == 2) {
        const long double a = coefficients[2], b = coefficients[1], c = coefficients[0];
        const long double discriminant = b * b - 4.0L * a * c;
        if (discriminant < 0.0L) return roots;
        const long double square_root = std::sqrt(discriminant);
        const long double q = -0.5L * (b + std::copysign(square_root, b));
        if (q == 0.0L) roots.push_back(-b / (2.0L * a));
        else { roots.push_back(q / a); roots.push_back(c / q); }
    } else {
        const long double a = coefficients[3];
        const long double b = coefficients[2] / a;
        const long double c = coefficients[1] / a;
        const long double d = coefficients[0] / a;
        const long double p = c - b * b / 3.0L;
        const long double q = 2.0L * b * b * b / 27.0L - b * c / 3.0L + d;
        const long double discriminant = q * q / 4.0L + p * p * p / 27.0L;
        const long double shift = b / 3.0L;
        if (discriminant > 0.0L) {
            const long double square_root = std::sqrt(discriminant);
            roots.push_back(std::cbrt(-q / 2.0L + square_root) +
                            std::cbrt(-q / 2.0L - square_root) - shift);
        } else if (discriminant == 0.0L) {
            const long double u = std::cbrt(-q / 2.0L);
            roots.push_back(2.0L * u - shift);
            roots.push_back(-u - shift);
        } else {
            const long double radius = 2.0L * std::sqrt(-p / 3.0L);
            const long double cosine = std::clamp((-q / 2.0L) /
                std::sqrt(-(p * p * p) / 27.0L), -1.0L, 1.0L);
            const long double angle = std::acos(cosine) / 3.0L;
            const long double two_pi_over_three = 2.0943951023931954923L;
            for (int index = 0; index < 3; ++index)
                roots.push_back(radius * std::cos(angle - index * two_pi_over_three) - shift);
        }
    }
    unique_sorted(roots);
    return roots;
}

/** Quartic roots: 64 brackets, plus cubic stationary points for tangent roots. */
std::vector<long double> quartic_roots(const Polynomial &coefficients, long double duration) {
    const auto slope = derivative(coefficients, 4);
    std::vector<long double> divisions;
    divisions.reserve(68);
    for (int index = 0; index <= 64; ++index)
        divisions.push_back(duration * index / 64.0L);
    for (long double root : closed_roots(slope, 3))
        if (root > 0.0L && root < duration) divisions.push_back(root);
    unique_sorted(divisions);
    long double scale = 1.0L;
    long double power = 1.0L;
    for (int index = 0; index <= 4; ++index) {
        scale += std::abs(coefficients[index] * power);
        power *= duration;
    }
    const long double residual_tolerance = scale * 1e-15L;
    std::vector<long double> roots;
    for (long double point : divisions)
        if (std::abs(evaluate(coefficients, 4, point)) <= residual_tolerance)
            roots.push_back(point);
    for (std::size_t index = 1; index < divisions.size(); ++index) {
        long double left = divisions[index - 1], right = divisions[index];
        long double left_value = evaluate(coefficients, 4, left);
        long double right_value = evaluate(coefficients, 4, right);
        if ((left_value < 0.0L) == (right_value < 0.0L)) continue;
        for (int iteration = 0; iteration < 96 && right - left > 1e-14L; ++iteration) {
            const long double middle = (left + right) / 2.0L;
            const long double middle_value = evaluate(coefficients, 4, middle);
            if ((left_value < 0.0L) == (middle_value < 0.0L)) {
                left = middle; left_value = middle_value;
            } else {
                right = middle; right_value = middle_value;
            }
        }
        long double root = (left + right) / 2.0L;
        const long double gradient = evaluate(slope, 3, root);
        if (gradient != 0.0L) {
            const long double refined = root - evaluate(coefficients, 4, root) / gradient;
            if (refined >= divisions[index - 1] && refined <= divisions[index]) root = refined;
        }
        roots.push_back(root);
    }
    unique_sorted(roots);
    return roots;
}

/** Bernstein's convex-hull property proves a derivative has no interior
 * zero when all controls have one strict sign. The roundoff reserve covers
 * normalization, binomial conversion and summation on double/long double.
 * Inconclusive signs retain the existing exact extrema solver. */
bool bernstein_bounds(const Polynomial &coefficients, int degree, long double duration,
                      long double &lower, long double &upper, bool rounded_double=false) {
    Polynomial normalized{};
    long double power=1.0L,scale=0.0L;
    for(int i=0;i<=degree;++i){
        normalized[i]=coefficients[i]*power;
        if(!std::isfinite(normalized[i]) || (coefficients[i]!=0.0L && normalized[i]==0.0L))return false;
        scale+=std::abs(normalized[i]);power*=duration;
    }
    if(!std::isfinite(scale))return false;
    const long double epsilon=rounded_double ? std::numeric_limits<double>::epsilon() :
        std::numeric_limits<long double>::epsilon();
    const long double error=512*epsilon*scale+512*std::numeric_limits<double>::denorm_min();
    lower=std::numeric_limits<long double>::infinity();upper=-lower;
    for(int k=0;k<=degree;++k){
        long double control=0.0L,ratio=1.0L;
        for(int i=0;i<=k;++i){
            if(i)ratio*=static_cast<long double>(k-i+1)/(degree-i+1);
            control+=ratio*normalized[i];
        }
        lower=std::min(lower,control-error);upper=std::max(upper,control+error);
    }
    return std::isfinite(lower) && std::isfinite(upper);
}
bool strict_bernstein_sign(const Polynomial &coefficients, int degree, long double duration) {
    long double lower,upper;
    return bernstein_bounds(coefficients,degree,duration,lower,upper) && (lower>0 || upper<0);
}

std::vector<long double> roots_inside(const Polynomial &coefficients, int degree,
                                      long double duration, bool prove_monotonic) {
    if (degree <= 0 || (prove_monotonic && strict_bernstein_sign(coefficients,degree,duration))) return {};
    auto roots = degree <= 3 ? closed_roots(coefficients, degree) :
        quartic_roots(coefficients, duration);
    roots.erase(std::remove_if(roots.begin(), roots.end(), [duration](long double root) {
        return !std::isfinite(root) || root <= 0.0L || root >= duration;
    }), roots.end());
    return roots;
}

void assumption(mk_validation_report &report, const char *kind, uint32_t joint) {
    if (report.assumption_count >= MK_MAX_ASSUMPTIONS) return;
    auto &text = report.assumptions[report.assumption_count++].text;
    if (joint == UINT32_MAX)
        std::snprintf(text, MK_ASSUMPTION_LENGTH, "%s not claimed", kind);
    else
        std::snprintf(text, MK_ASSUMPTION_LENGTH, "%s not claimed for joint %u", kind, joint);
}

bool valid_limits(const Trajectory &trajectory, const mk_limits &limits) {
    if (limits.struct_size < offsetof(mk_limits, executor_time_resolution_ns) ||
        limits.joint_count != trajectory.joint_count() || trajectory.segment_count() == 0)
        return false;
    for (uint32_t joint = 0; joint < limits.joint_count; ++joint) {
        if (limits.position_claimed[joint] > 1 || (limits.derivative_claimed[joint] & ~7u) ||
            !std::isfinite(limits.max_velocity[joint]) || limits.max_velocity[joint] < 0.0 ||
            !std::isfinite(limits.max_acceleration[joint]) || limits.max_acceleration[joint] < 0.0 ||
            !std::isfinite(limits.max_jerk[joint]) || limits.max_jerk[joint] < 0.0)
            return false;
        if (limits.position_claimed[joint] &&
            (!std::isfinite(limits.position_lower[joint]) ||
             !std::isfinite(limits.position_upper[joint]) ||
             limits.position_lower[joint] > limits.position_upper[joint]))
            return false;
    }
    for (double limit : limits.max_continuity_jump)
        if (!std::isfinite(limit) || limit < 0.0) return false;
    return true;
}

void observe(mk_validation_report &report, std::array<long double, MK_CHECK_COUNT> &scores,
             uint32_t kind, uint32_t joint, uint32_t derivative_order,
             double value, double time_seconds, double limit, double margin,
             double tolerance, long double score,
             bool failed) {
    auto &check = report.checks[kind];
    if (score > scores[kind]) {
        scores[kind] = score;
        check.joint = joint;
        check.derivative_order = derivative_order;
        check.value = value;
        check.time_seconds = time_seconds;
        check.limit = limit;
        check.margin = margin;
        check.tolerance = tolerance;
    }
    if (failed) check.status = MK_CHECK_FAILED;
}

using DerivativeMaxima = std::array<std::array<double, 5>, MK_MAX_JOINTS>;

DerivativeMaxima derivative_maxima(const Trajectory &trajectory, bool prove_monotonic) {
    DerivativeMaxima maxima{};
    for (uint32_t index = 0; index < trajectory.segment_count(); ++index) {
        const auto &segment = trajectory.segment(index);
        const long double duration = static_cast<long double>(segment.duration_ns) / 1e9L;
        for (uint32_t joint = 0; joint < trajectory.joint_count(); ++joint) {
            Polynomial polynomial{};
            for (uint32_t coefficient = 0; coefficient <= segment.degree; ++coefficient)
                polynomial[coefficient] = segment.coefficients[joint].value[coefficient];
            int degree = static_cast<int>(segment.degree);
            for (uint32_t order = 1; order <= 4; ++order) {
                polynomial = derivative(polynomial, degree);
                degree = std::max(0, degree - 1);
                long double lower,upper;
                // A continuous hull below the existing exact maximum cannot
                // change the quantization reserve. Keep its exact value.
                if(prove_monotonic && bernstein_bounds(polynomial,degree,duration,lower,upper,true) &&
                    std::max(std::abs(lower),std::abs(upper))<maxima[joint][order])continue;
                std::vector<long double> times{0.0L, duration};
                const auto critical = roots_inside(derivative(polynomial, degree),
                    std::max(0, degree - 1), duration, prove_monotonic);
                times.insert(times.end(), critical.begin(), critical.end());
                for (const long double time : times)
                    maxima[joint][order] = std::max(maxima[joint][order],
                        static_cast<double>(std::abs(evaluate(polynomial, degree, time))));
            }
        }
    }
    return maxima;
}

double comparison_tolerance(double magnitude_scale, double next_derivative_maximum,
                            uint64_t resolution_ns) {
    // 1e-9 relative slack covers floating-point comparisons. The 1e-6
    // relative cap matches RobotKit's existing chord-velocity slack and
    // prevents extreme higher derivatives from widening the limit unchecked.
    constexpr double relative_epsilon = 1e-9;
    constexpr double maximum_relative_tolerance = 1e-6;
    const long double quantization = static_cast<long double>(next_derivative_maximum) *
        static_cast<long double>(resolution_ns) * 0.5e-9L;
    return static_cast<double>(std::min(
        std::max(static_cast<long double>(relative_epsilon * magnitude_scale), quantization),
        static_cast<long double>(maximum_relative_tolerance * magnitude_scale)));
}

double position_comparison_tolerance(double lower, double upper,
                                     double maximum_velocity, uint64_t resolution_ns) {
    // Positions are affine: shifting the joint origin must not change its
    // tolerance. A degenerate range uses a 1e-12 absolute comparison floor
    // (the same floor as a 1e-3 range under the 1e-9 relative rule).
    const long double range = static_cast<long double>(upper) - lower;
    const double scale = range > 0.0L
        ? static_cast<double>(std::min(range,
            static_cast<long double>(std::numeric_limits<double>::max())))
        : 1e-3;
    return comparison_tolerance(scale, maximum_velocity, resolution_ns);
}

} // namespace

mk_result validate(const Trajectory &trajectory, const mk_limits &limits,
                   mk_validation_report &report) {
    if (!valid_limits(trajectory, limits)) return MK_ERROR_INVALID_ARGUMENT;
    report = {};
    report.struct_size = sizeof(report);
    report.model_revision = limits.model_revision;
    report.calibration_revision = limits.calibration_revision;
    report.trajectory_revision = trajectory.revision();
    report.executor_time_resolution_ns =
        limits.struct_size >= sizeof(mk_limits) && limits.executor_time_resolution_ns != 0
            ? limits.executor_time_resolution_ns : 1;
    const char *audit=std::getenv("PROCESS_PATH_VERIFY_EXECUTION");
    const bool prove_monotonic=!(audit && audit[0]=='1' && audit[1]=='\0');
    const auto maxima = derivative_maxima(trajectory,prove_monotonic);
    for (uint32_t joint = 0; joint < trajectory.joint_count(); ++joint)
        for (uint32_t order = 1; order <= 4; ++order)
            if (!std::isfinite(maxima[joint][order])) return MK_ERROR_INVALID_ARGUMENT;
    std::array<long double, MK_CHECK_COUNT> scores;
    scores.fill(-std::numeric_limits<long double>::infinity());
    std::array<bool, MK_CHECK_COUNT> unchecked{};
    for (auto &check : report.checks) {
        check.status = MK_CHECK_PASSED;
        check.joint = UINT32_MAX;
    }
    unchecked[MK_CHECK_TASK_SPACE] = true;
    for (uint32_t joint = 0; joint < limits.joint_count; ++joint) {
        if (!limits.position_claimed[joint]) {
            unchecked[MK_CHECK_POSITION] = true; assumption(report, "position limit", joint);
        }
        if (!(limits.derivative_claimed[joint] & 1)) {
            unchecked[MK_CHECK_VELOCITY] = true; assumption(report, "velocity limit", joint);
        }
        if (!(limits.derivative_claimed[joint] & 2)) {
            unchecked[MK_CHECK_ACCELERATION] = true; assumption(report, "acceleration limit", joint);
        }
        if (!(limits.derivative_claimed[joint] & 4)) {
            unchecked[MK_CHECK_JERK] = true; assumption(report, "jerk limit", joint);
        }
    }
    for (uint32_t order = 0; order < 3; ++order)
        if (limits.max_continuity_jump[order] == 0.0) {
            unchecked[MK_CHECK_CONTINUITY] = true;
            if (report.assumption_count < MK_MAX_ASSUMPTIONS)
                std::snprintf(report.assumptions[report.assumption_count++].text,
                    MK_ASSUMPTION_LENGTH, "C%u continuity limit not claimed", order);
        }

    for (uint32_t index = 0; index < trajectory.segment_count(); ++index) {
        const auto &segment = trajectory.segment(index);
        const long double duration = static_cast<long double>(segment.duration_ns) / 1e9L;
        for (uint32_t joint = 0; joint < trajectory.joint_count(); ++joint) {
            Polynomial polynomial{};
            for (uint32_t coefficient = 0; coefficient <= segment.degree; ++coefficient)
                polynomial[coefficient] = segment.coefficients[joint].value[coefficient];
            int degree = static_cast<int>(segment.degree);
            for (uint32_t order = 0; order <= 3; ++order) {
                const bool claimed = order == 0 ? limits.position_claimed[joint] != 0 :
                    order == 1 ? (limits.derivative_claimed[joint] & 1) :
                    order == 2 ? (limits.derivative_claimed[joint] & 2) :
                                 (limits.derivative_claimed[joint] & 4);
                bool dominated=false;
                if(claimed && order==0 && prove_monotonic){
                    long double lower,upper;
                    if(bernstein_bounds(polynomial,degree,duration,lower,upper,true)){
                        const double tolerance=position_comparison_tolerance(limits.position_lower[joint],
                            limits.position_upper[joint],maxima[joint][1],report.executor_time_resolution_ns);
                        const long double excess=std::max(upper-limits.position_upper[joint],
                            limits.position_lower[joint]-lower);
                        dominated=(excess-tolerance)/std::max(1.0,limits.position_upper[joint]-
                            limits.position_lower[joint])<scores[MK_CHECK_POSITION];
                    }
                }
                // Dominance preserves the original worst value, time and joint.
                if (claimed && !dominated) {
                    std::vector<long double> times{0.0L, duration};
                    const auto next = derivative(polynomial, degree);
                    auto critical = roots_inside(next, std::max(0, degree - 1), duration, prove_monotonic);
                    times.insert(times.end(), critical.begin(), critical.end());
                    for (long double local_time : times) {
                        const double value = static_cast<double>(evaluate(polynomial, degree, local_time));
                        if (!std::isfinite(value)) return MK_ERROR_INVALID_ARGUMENT;
                        const double time_seconds = static_cast<double>(segment.t0_ns) * 1e-9 +
                            static_cast<double>(local_time);
                        if (order == 0) {
                            const double upper = limits.position_upper[joint];
                            const double lower = limits.position_lower[joint];
                            const bool upper_side = value - upper >= lower - value;
                            const double limit = upper_side ? upper : lower;
                            const long double excess = upper_side ? value - upper : lower - value;
                            const double tolerance = position_comparison_tolerance(lower, upper,
                                maxima[joint][1], report.executor_time_resolution_ns);
                            const double margin = -static_cast<double>(excess);
                            const long double score = (excess - tolerance) /
                                std::max(1.0, upper - lower);
                            observe(report, scores, MK_CHECK_POSITION, joint, 0, value,
                                time_seconds, limit, margin, tolerance, score,
                                margin < -tolerance);
                        } else {
                            const double limit = order == 1 ? limits.max_velocity[joint] :
                                order == 2 ? limits.max_acceleration[joint] : limits.max_jerk[joint];
                            const double magnitude = std::abs(value);
                            const double tolerance = comparison_tolerance(std::abs(limit),
                                maxima[joint][order + 1], report.executor_time_resolution_ns);
                            const double margin = limit - magnitude;
                            const double scale = std::max({limit, tolerance, 1e-30});
                            observe(report, scores, order, joint, order, magnitude,
                                time_seconds, limit, margin, tolerance,
                                (magnitude - limit - tolerance) / scale,
                                margin < -tolerance);
                        }
                    }
                }
                polynomial = derivative(polynomial, degree);
                degree = std::max(0, degree - 1);
            }
        }
    }

    for (uint32_t index = 0; index + 1 < trajectory.segment_count(); ++index) {
        const auto &before = trajectory.segment(index);
        const auto &after = trajectory.segment(index + 1);
        mk_trajectory_state left{}, right{};
        evaluate_segment(before, static_cast<double>(before.duration_ns) / 1e9, left);
        evaluate_segment(after, 0.0, right);
        const double time = static_cast<double>(after.t0_ns) * 1e-9;
        for (uint32_t joint = 0; joint < trajectory.joint_count(); ++joint) {
            const double jumps[3] = {
                std::abs(left.position[joint] - right.position[joint]),
                std::abs(left.velocity[joint] - right.velocity[joint]),
                std::abs(left.acceleration[joint] - right.acceleration[joint])
            };
            for (double jump : jumps)
                if (!std::isfinite(jump)) return MK_ERROR_INVALID_ARGUMENT;
            for (uint32_t order = 0; order < 3; ++order) {
                const double limit = limits.max_continuity_jump[order];
                if (limit > 0.0) {
                    // C0 uses position uncertainty, capped by one clock quantum of
                    // observed travel. An unbounded continuous-joint range must not
                    // turn a tiny continuity claim into an unrestricted jump.
                    const double tolerance = order == 0 && limits.position_claimed[joint]
                        ? std::min(position_comparison_tolerance(limits.position_lower[joint],
                            limits.position_upper[joint], maxima[joint][1],
                            report.executor_time_resolution_ns), maxima[joint][1] * 1e-9 *
                                std::max<uint64_t>(1, report.executor_time_resolution_ns))
                        : comparison_tolerance(std::abs(limit), maxima[joint][order + 1],
                            report.executor_time_resolution_ns);
                    const double margin = limit - jumps[order];
                    const double scale = std::max({limit, tolerance, 1e-30});
                    observe(report, scores, MK_CHECK_CONTINUITY, joint, order,
                        jumps[order], time, limit, margin, tolerance,
                        (jumps[order] - limit - tolerance) / scale,
                        margin < -tolerance);
                }
            }
        }
    }
    for (uint32_t kind = 0; kind < MK_CHECK_COUNT; ++kind)
        if (report.checks[kind].status != MK_CHECK_FAILED && unchecked[kind])
            report.checks[kind].status = MK_CHECK_UNCHECKED;
    return MK_OK;
}

} // namespace motionkit
