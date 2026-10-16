#ifndef MOTIONKIT_CLOCK_VELOCITY_PROFILE_HPP
#define MOTIONKIT_CLOCK_VELOCITY_PROFILE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace motionkit {
// Construct a bounded piecewise-linear velocity profile on an existing integer
// clock. The caller supplies scalar bounds valid throughout one affine geometry/
// feed region. End speed is allowed to move only inside its certified interval.
// This is a candidate constructor; the lowered joint trajectory still requires
// continuous validation. Infeasible clock horizons leave output untouched.
inline bool clockVelocityProfile(const std::vector<int64_t>& ticks,
        const std::vector<double>& reference, double distance, double start,
        double end_lower, double end_upper, double velocity, double acceleration,
        std::vector<double>& output) {
    const std::size_t n = ticks.size();
    if (n < 2 || reference.size() != n || !std::isfinite(distance) || distance <= 0 ||
            !std::isfinite(start) || start < 0 || !std::isfinite(end_lower) || end_lower < 0 ||
            !std::isfinite(end_upper) || end_upper < end_lower || !std::isfinite(velocity) ||
            velocity <= 0 || start > velocity || end_lower > velocity ||
            !std::isfinite(acceleration) || acceleration <= 0) return false;
    std::vector<long double> time(n), lower(n), upper(n), candidate(n);
    for (std::size_t i = 0; i < n; ++i) {
        if ((i && ticks[i] <= ticks[i-1]) || !std::isfinite(reference[i]) || reference[i] < 0)
            return false;
        // Convert before subtraction to avoid signed integer overflow.
        time[i] = (static_cast<long double>(ticks[i]) - ticks.front()) * 1e-9L;
    }
    const long double duration = time.back(), a = acceleration, vmax = velocity;
    for (std::size_t i = 0; i < n; ++i) {
        const long double elapsed = time[i], remaining = duration - elapsed;
        lower[i] = std::max({0.0L, start - a*elapsed, end_lower - a*remaining});
        upper[i] = std::min({vmax, start + a*elapsed, end_upper + a*remaining});
        if (lower[i] > upper[i]) return false;
        candidate[i] = std::min(upper[i], static_cast<long double>(reference[i]));
    }
    // Greatest acceleration-bounded minorant of the capped reference, followed
    // by the feasible lower envelope. Min/max preserve the same Lipschitz bound.
    for (std::size_t i = 1; i < n; ++i)
        candidate[i] = std::min(candidate[i], candidate[i-1] + a*(time[i]-time[i-1]));
    for (std::size_t i = n-1; i > 0; --i)
        candidate[i-1] = std::min(candidate[i-1], candidate[i] + a*(time[i]-time[i-1]));
    for (std::size_t i = 0; i < n; ++i) candidate[i] = std::max(lower[i], candidate[i]);
    auto area = [&](const std::vector<long double>& values) {
        long double result = 0;
        for (std::size_t i = 1; i < n; ++i)
            result += (values[i-1]+values[i]) * (time[i]-time[i-1]) / 2;
        return result;
    };
    const long double minimum = area(lower), maximum = area(upper), target = distance;
    if (target < minimum || target > maximum) return false;
    const long double current = area(candidate);
    const auto& extremum = target < current ? lower : upper;
    const long double extreme_area = target < current ? minimum : maximum;
    if (target != current) {
        const long double weight = (target-current)/(extreme_area-current);
        for (std::size_t i = 0; i < n; ++i)
            candidate[i] += weight*(extremum[i]-candidate[i]);
    }
    std::vector<double> result(n);
    for (std::size_t i = 0; i < n; ++i) result[i] = static_cast<double>(candidate[i]);
    output.swap(result);
    return true;
}
}
#endif
