#include "trajectory_merge.hpp"
#include <memory>

namespace motionkit {
namespace {
using Poly = std::vector<double>;
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


struct Kin { double position, velocity, acceleration; };

Kin kin_of(const mk_joint_coefficients &c, uint32_t degree, double tau) {
    double p = 0.0, v = 0.0, a = 0.0;
    for (int k = static_cast<int>(degree); k >= 0; --k) p = p * tau + c.value[k];
    for (int k = static_cast<int>(degree) - 1; k >= 0; --k) v = v * tau + (k + 1.0) * c.value[k + 1];
    for (int k = static_cast<int>(degree) - 2; k >= 0; --k) a = a * tau + (k + 2.0) * (k + 1.0) * c.value[k + 2];
    return {p, v, a};
}


} // namespace

mk_result merge_polynomials(const PolynomialSource &source, const MergeOptions &options, const PolynomialOutput &output) {
    const auto *position_tolerance = options.position_tolerance, *velocity_tolerance = options.velocity_tolerance,
        *acceleration_tolerance = options.acceleration_tolerance, *velocity_limit = options.velocity_limit,
        *acceleration_limit = options.acceleration_limit, *position_lower = options.position_lower,
        *position_upper = options.position_upper;
    const auto joints = source.joint_count, count = source.count;
    if (!source.segment || !output.append || !position_tolerance || !velocity_tolerance || !acceleration_tolerance ||
        !velocity_limit || !acceleration_limit || !position_lower || !position_upper || joints == 0 || joints > MK_MAX_JOINTS ||
        !std::isfinite(options.max_duration_seconds) || options.max_duration_seconds <= 0 ||
        options.max_duration_seconds >= static_cast<double>(INT64_MAX) * 1e-9 || options.merge_below_ns <= 0)
        return MK_ERROR_INVALID_ARGUMENT;
    for (uint32_t j = 0; j < joints; ++j)
        if (!(position_tolerance[j] > 0) || !(velocity_tolerance[j] > 0) || !(acceleration_tolerance[j] > 0) ||
            !(velocity_limit[j] > 0) || !(acceleration_limit[j] > 0) || std::isnan(position_lower[j]) ||
            std::isnan(position_upper[j]) || position_lower[j] > position_upper[j]) return MK_ERROR_INVALID_ARGUMENT;
    const int64_t longest = static_cast<int64_t>(std::llround(options.max_duration_seconds * 1e9));
    auto load = [&](uint32_t index, mk_segment &out) -> mk_result {
        const auto view = source.segment(source.context, index);
        out = {}; out.struct_size = sizeof(out); out.t0_ns = view.start_ns; out.duration_ns = view.duration_ns;
        out.degree = view.degree; out.joint_count = view.joint_count;
        if (view.joint_count != joints || view.degree > MK_MAX_DEGREE || view.duration_ns <= 0 ||
            view.start_ns > INT64_MAX - view.duration_ns || !view.coefficients ||
            view.coefficient_stride < sizeof(double) * (view.degree + 1)) return MK_ERROR_INVALID_ARGUMENT;
        for (uint32_t j = 0; j < joints; ++j)
            for (uint32_t k = 0; k <= view.degree; ++k) {
                const auto value = view.joint(j)[k];
                if (!std::isfinite(value)) return MK_ERROR_INVALID_ARGUMENT;
                out.coefficients[j].value[k] = value;
            }
        return MK_OK;
    };
    auto evaluate = [&](int64_t time, mk_trajectory_state &state) -> mk_result {
        uint32_t low = 0, high = count;
        while (low + 1 < high) {
            const auto mid = low + (high - low) / 2;
            if (source.segment(source.context, mid).start_ns <= time) low = mid; else high = mid;
        }
        const auto view = source.segment(source.context, low);
        const double tau = static_cast<double>(std::clamp(time - view.start_ns, int64_t{0}, view.duration_ns)) / 1e9;
        state.joint_count = joints;
        for (uint32_t j = 0; j < joints; ++j) {
            const auto *c = view.joint(j);
            double p = 0, v = 0, a = 0;
            for (int k = static_cast<int>(view.degree); k >= 0; --k) {
                a = a * tau + 2 * v; v = v * tau + p; p = p * tau + c[k];
            }
            state.position[j] = p; state.velocity[j] = v; state.acceleration[j] = a;
        }
        return MK_OK;
    };
    mk_result status = MK_OK;
    try {
        std::vector<int64_t> start(count), duration(count);
        std::vector<uint8_t> smooth(count);
        auto first_segment = std::make_unique<mk_segment>(), last_segment = std::make_unique<mk_segment>();
        auto candidate = std::make_unique<mk_segment>(), best_segment = std::make_unique<mk_segment>();
        for (uint32_t i = 0; i < count; ++i) {
            *first_segment = mk_segment{};
            first_segment->struct_size = sizeof(mk_segment);
            status = load(i, *first_segment);
            if (status != MK_OK) { return status; }
            if (i && first_segment->t0_ns != start[i - 1] + duration[i - 1]) return MK_ERROR_INVALID_ARGUMENT;
            start[i] = first_segment->t0_ns;
            duration[i] = first_segment->duration_ns;
            if (static_cast<uint64_t>(start[i] + duration[i]) - static_cast<uint64_t>(start[0]) >
                static_cast<uint64_t>(INT64_MAX)) return MK_ERROR_INVALID_ARGUMENT;
            // A chord (degree 1) knows no acceleration to match: it is never merged.
            smooth[i] = first_segment->degree >= 2 ? 1 : 0;
        }
        // The quintic through the first segment's start state and the last one's end state.
        auto through = [&](uint32_t last) {
            const double T = static_cast<double>(start[last] + duration[last] - first_segment->t0_ns) * 1e-9;
            const double end = static_cast<double>(duration[last]) * 1e-9;
            *candidate = *first_segment;
            candidate->t0_ns = first_segment->t0_ns;
            candidate->duration_ns = start[last] + duration[last] - first_segment->t0_ns;
            candidate->degree = 5;
            for (uint32_t joint = 0; joint < joints; ++joint) {
                const auto from = kin_of(first_segment->coefficients[joint], first_segment->degree, 0.0);
                const auto to = kin_of(last_segment->coefficients[joint], last_segment->degree, end);
                const double V0 = from.velocity * T, A0 = from.acceleration * T * T;
                const double V1 = to.velocity * T, A1 = to.acceleration * T * T;
                const double D = to.position - from.position - V0 - 0.5 * A0, E = V1 - V0 - A0, F = A1 - A0;
                const double normalized[6] = {from.position, V0, 0.5 * A0, 10.0 * D - 4.0 * E + 0.5 * F,
                    -15.0 * D + 7.0 * E - F, 6.0 * D - 3.0 * E + 0.5 * F};
                double scale = 1.0;
                for (int k = 0; k < 6; ++k) { candidate->coefficients[joint].value[k] = normalized[k] / scale; scale *= T; }
            }
        };
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        // Whether the quintic follows the segments it replaces at their joins and at fifteen even steps across.
        auto follows = [&](uint32_t first, uint32_t last) -> mk_result {
            for (uint32_t j = 0; j < joints; ++j)
                for (uint32_t k = 0; k <= candidate->degree; ++k)
                    if (!std::isfinite(candidate->coefficients[j].value[k])) return MK_ERROR_GENERATION;
            std::vector<double> peak_velocity(joints, 0.0), peak_acceleration(joints, 0.0);
            std::vector<int64_t> times;
            const uint32_t stride = std::max<uint32_t>(1, (last - first + 1) / 24);
            for (uint32_t i = first; i <= last; i += stride) times.push_back(start[i]);
            for (int step = 1; step < 16; ++step)
                times.push_back(candidate->t0_ns + (candidate->duration_ns / 16) * step + (candidate->duration_ns % 16) * step / 16);
            for (const auto time : times) {
                const auto result = evaluate(time, state);
                if (result != MK_OK) return result;
                const double tau = static_cast<double>(time - candidate->t0_ns) * 1e-9;
                for (uint32_t joint = 0; joint < joints; ++joint) {
                    const auto mine = kin_of(candidate->coefficients[joint], 5, tau);
                    if (std::abs(mine.position - state.position[joint]) > position_tolerance[joint] ||
                        std::abs(mine.velocity - state.velocity[joint]) > velocity_tolerance[joint] ||
                        std::abs(mine.acceleration - state.acceleration[joint]) > acceleration_tolerance[joint])
                        return MK_ERROR_GENERATION;
                    // A merged run does not take a joint past a limit that the segments it replaces keep to, nor, where they
                    // are past it themselves (a plateau at the limit, as a timed path cruises), past what they are by
                    // more than rounding (1e-9 of the limit, the validator's own floor: a plan cruising at the limit is
                    // checked against it). The same for the position bounds, to a nanometre.
                    if (mine.position < position_lower[joint] && mine.position < state.position[joint] - 1e-9) return MK_ERROR_GENERATION;
                    if (mine.position > position_upper[joint] && mine.position > state.position[joint] + 1e-9) return MK_ERROR_GENERATION;
                    if ((std::abs(mine.velocity) > velocity_limit[joint] &&
                         std::abs(mine.velocity) > std::abs(state.velocity[joint]) + 1e-9 * velocity_limit[joint]) ||
                        (std::abs(mine.acceleration) > acceleration_limit[joint] &&
                         std::abs(mine.acceleration) > std::abs(state.acceleration[joint]) + 1e-9 * acceleration_limit[joint]))
                        return MK_ERROR_GENERATION;
                    peak_velocity[joint] = std::max(peak_velocity[joint], std::abs(state.velocity[joint]));
                    peak_acceleration[joint] = std::max(peak_acceleration[joint],
                        std::abs(state.acceleration[joint]));
                }
            }
            // Samples can miss the quintic's own peaks between them, as on a plateau at a limit: bound its speed and
            // acceleration exactly, at its stationary points.
            const double T = static_cast<double>(candidate->duration_ns) * 1e-9;
            for (uint32_t joint = 0; joint < joints; ++joint) {
                Poly p(6);
                double scale = 1.0;
                for (int k = 0; k < 6; ++k) { p[k] = candidate->coefficients[joint].value[k] * scale; scale *= T; }
                // A joint that holds still, or moves at a constant speed, has nothing to find the peaks of.
                if (p[2] == 0.0 && p[3] == 0.0 && p[4] == 0.0 && p[5] == 0.0) {
                    if (std::abs(p[1]) / T > velocity_limit[joint] * (1.0 + 1e-9) &&
                        std::abs(p[1]) / T > peak_velocity[joint] + 1e-9 * velocity_limit[joint])
                        return MK_ERROR_GENERATION;
                    continue;
                }
                const Poly v = derivative(p), a = derivative(v);
                for (double x : roots(v)) {
                    const double position = eval(p, x);
                    // Position bounds are hard: reject an interior overshoot even if the sampling grid misses it.
                    if (position < position_lower[joint] - 1e-9 || position > position_upper[joint] + 1e-9)
                        return MK_ERROR_GENERATION;
                }
                // The control points of the Bernstein form bound a polynomial on [0,1]: when they keep within the
                // limit, or within what the segments replaced already reach, there are no peaks to find.
                auto hull = [](const Poly &q) {
                    const int n = static_cast<int>(q.size()) - 1;
                    double bound = 0.0;
                    for (int k = 0; k <= n; ++k) {
                        double control = 0.0, ratio = 1.0;
                        for (int i = 0; i <= k; ++i) {
                            if (i) ratio *= static_cast<double>(k - i + 1) / static_cast<double>(n - i + 1);
                            control += ratio * q[static_cast<size_t>(i)];
                        }
                        bound = std::max(bound, std::abs(control));
                    }
                    return bound;
                };
                auto peak = [](const Poly &q) {
                    double worst = std::max(std::abs(eval(q, 0.0)), std::abs(eval(q, 1.0)));
                    for (double x : roots(derivative(q))) worst = std::max(worst, std::abs(eval(q, x)));
                    return worst;
                };
                const double v_hull = hull(v) / T, a_hull = hull(a) / (T * T);
                const bool v_open = v_hull > velocity_limit[joint] * (1.0 + 1e-9) &&
                    v_hull > peak_velocity[joint] + 1e-9 * velocity_limit[joint];
                const bool a_open = a_hull > acceleration_limit[joint] * (1.0 + 1e-9) &&
                    a_hull > peak_acceleration[joint] + 1e-9 * acceleration_limit[joint];
                if (!v_open && !a_open) continue;
                const double v_peak = v_open ? peak(v) / T : 0.0, a_peak = a_open ? peak(a) / (T * T) : 0.0;
                if ((v_open && v_peak > velocity_limit[joint] * (1.0 + 1e-9) &&
                     v_peak > peak_velocity[joint] + 1e-9 * velocity_limit[joint]) ||
                    (a_open && a_peak > acceleration_limit[joint] * (1.0 + 1e-9) &&
                     a_peak > peak_acceleration[joint] + 1e-9 * acceleration_limit[joint]))
                    return MK_ERROR_GENERATION;
            }
            return MK_OK;
        };
        for (uint32_t first = 0; first < count;) {
            *first_segment = mk_segment{};
            first_segment->struct_size = sizeof(mk_segment);
            status = load(first, *first_segment);
            if (status != MK_OK) { return status; }
            *best_segment = *first_segment;
            uint32_t best = first, limit = first;
            if (smooth[first] && duration[first] < options.merge_below_ns)
                while (limit + 1 < count && smooth[limit + 1] &&
                       start[limit + 1] + duration[limit + 1] - start[first] <= longest) ++limit;
            auto accepts = [&](uint32_t last) -> mk_result {
                *last_segment = mk_segment{};
                last_segment->struct_size = sizeof(mk_segment);
                auto result = load(last, *last_segment);
                if (result != MK_OK) return result;
                through(last);
                result = follows(first, last);
                if (result != MK_OK) return result;
                best = last;
                *best_segment = *candidate;
                return MK_OK;
            };
            // The longest run that works, found by doubling and then bisecting.
            uint32_t step = 1, failed = limit + 1;
            while (best + step <= limit) {
                const auto result = accepts(best + step);
                if (result == MK_OK) step *= 2;
                else if (result == MK_ERROR_GENERATION) { failed = best + step; break; }
                else { return result; }
            }
            while (failed > best + 1) {
                const uint32_t middle = best + (failed - best) / 2;
                if (middle == best) break;
                const auto result = accepts(middle);
                if (result == MK_OK) continue;
                if (result != MK_ERROR_GENERATION) { return result; }
                failed = middle;
            }
            status = output.append(output.context, *best_segment, first, best);
            if (status != MK_OK) return status;
            first = best + 1;
        }
    } catch (const std::bad_alloc &) { return MK_ERROR_OUT_OF_MEMORY; }
    return MK_OK;
}
} // namespace motionkit
