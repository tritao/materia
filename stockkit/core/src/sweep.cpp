#include "sweep.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stockkit {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2 * kPi;
constexpr double kInf = std::numeric_limits<double>::infinity();

/** Golden-section minimum of f on [a, b], to the resolution of doubles. */
template <typename F>
double golden(F &&f, double a, double b, double &best) {
    const double ratio = 0.6180339887498949;
    double c = b - ratio * (b - a), d = a + ratio * (b - a);
    double fc = f(c), fd = f(d);
    for (int i = 0; i < 100 && b - a > 1e-16 * std::max(1.0, std::fabs(a) + std::fabs(b)); ++i) {
        if (fc <= fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - ratio * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + ratio * (b - a);
            fd = f(d);
        }
    }
    if (fc <= fd) {
        best = fc;
        return c;
    }
    best = fd;
    return d;
}

void add_range(std::vector<double> &cuts, double lo, double hi) {
    if (hi >= lo) cuts.insert(cuts.end(), {lo, hi});
}

} // namespace

void Motion::position(double t, double &x, double &y, double &z) const {
    if (!arc) {
        x = p0[0] + t * (p1[0] - p0[0]);
        y = p0[1] + t * (p1[1] - p0[1]);
        z = p0[2] + t * (p1[2] - p0[2]);
        return;
    }
    double angle = start_angle + t * sweep;
    x = cx + radius * std::cos(angle);
    y = cy + radius * std::sin(angle);
    z = z0 + t * rise;
}

double Motion::height(double t) const {
    return arc ? z0 + t * rise : p0[2] + t * (p1[2] - p0[2]);
}

SweptVolume::SweptVolume(const Profile &profile, const Motion &motion)
    : profile_(profile), motion_(motion) {
    double reach = profile.radius();
    if (!motion.arc) {
        length_ = std::hypot(motion.p1[0] - motion.p0[0], motion.p1[1] - motion.p0[1]);
        level_ = motion.p0[2] == motion.p1[2];
        fixed_distance_ = length_ == 0;
        if (length_ > 0) {
            direction_[0] = (motion.p1[0] - motion.p0[0]) / length_;
            direction_[1] = (motion.p1[1] - motion.p0[1]) / length_;
        }
        for (int k = 0; k < 3; ++k) {
            bounds_.min[k] = std::min(motion.p0[k], motion.p1[k]);
            bounds_.max[k] = std::max(motion.p0[k], motion.p1[k]);
        }
    } else {
        level_ = motion.rise == 0;
        fixed_distance_ = motion.radius == 0 || motion.sweep == 0;
        double x, y, z;
        motion.position(0, x, y, z);
        bounds_.min[0] = bounds_.max[0] = x;
        bounds_.min[1] = bounds_.max[1] = y;
        motion.position(1, x, y, z);
        bounds_.min[0] = std::min(bounds_.min[0], x);
        bounds_.max[0] = std::max(bounds_.max[0], x);
        bounds_.min[1] = std::min(bounds_.min[1], y);
        bounds_.max[1] = std::max(bounds_.max[1], y);
        // Axis-extreme angles inside the sweep.
        double lo = std::min(motion.start_angle, motion.start_angle + motion.sweep);
        double hi = std::max(motion.start_angle, motion.start_angle + motion.sweep);
        for (double k = std::ceil(lo / (kPi / 2)); k * (kPi / 2) <= hi; k += 1) {
            int quadrant = ((static_cast<long long>(k) % 4) + 4) % 4;
            double px = motion.cx + motion.radius * (quadrant == 0 ? 1 : quadrant == 2 ? -1 : 0);
            double py = motion.cy + motion.radius * (quadrant == 1 ? 1 : quadrant == 3 ? -1 : 0);
            bounds_.min[0] = std::min(bounds_.min[0], px);
            bounds_.max[0] = std::max(bounds_.max[0], px);
            bounds_.min[1] = std::min(bounds_.min[1], py);
            bounds_.max[1] = std::max(bounds_.max[1], py);
        }
        bounds_.min[2] = std::min(motion.z0, motion.z0 + motion.rise);
        bounds_.max[2] = std::max(motion.z0, motion.z0 + motion.rise);
    }
    for (int k = 0; k < 2; ++k) {
        bounds_.min[k] -= reach;
        bounds_.max[k] += reach;
    }
    tip_low_ = bounds_.min[2];
    bounds_.min[2] += profile.base();
    bounds_.max[2] += profile.height();
}

double SweptVolume::distance(double t, double x, double y) const {
    double px, py, pz;
    motion_.position(t, px, py, pz);
    double dx = x - px, dy = y - py;
    return std::sqrt(dx * dx + dy * dy);
}

double SweptVolume::floor_bound(double x, double y) const {
    // Smallest xy distance from the ray to the tool axis over the motion, or
    // a lower bound on it.
    double d;
    if (!motion_.arc) {
        if (fixed_distance_) {
            d = distance(0, x, y);
        } else {
            double rx = x - motion_.p0[0], ry = y - motion_.p0[1];
            double u = std::max(0.0, std::min(length_, rx * direction_[0] + ry * direction_[1]));
            double px = rx - u * direction_[0], py = ry - u * direction_[1];
            d = std::sqrt(px * px + py * py);
        }
    } else {
        double ox = x - motion_.cx, oy = y - motion_.cy;
        d = std::fabs(std::sqrt(ox * ox + oy * oy) - motion_.radius);
    }
    double floor = kInf;
    for (const Run &run : profile_.runs()) {
        if (d > run.dmax) continue;
        floor = std::min(floor, run.widening ? run.envelope(d) : run.za);
    }
    return tip_low_ + floor;
}

/** Stretches of t in [0, 1] where the axis is within `reach` of (x, y). */
void SweptVolume::components(double x, double y, double reach, std::vector<Range> &out) const {
    out.clear();
    if (fixed_distance_) {
        if (distance(0, x, y) <= reach) out.push_back({0, 1});
        return;
    }
    if (!motion_.arc) {
        double dx = direction_[0], dy = direction_[1];
        double rx = x - motion_.p0[0], ry = y - motion_.p0[1];
        double along = rx * dx + ry * dy;
        double across = std::fabs(rx * dy - ry * dx);
        if (across > reach) return;
        double half = std::sqrt(reach * reach - across * across);
        double t0 = std::max(0.0, (along - half) / length_);
        double t1 = std::min(1.0, (along + half) / length_);
        if (t0 <= t1) out.push_back({t0, t1});
        return;
    }
    const double rho = motion_.radius;
    double ox = x - motion_.cx, oy = y - motion_.cy;
    double D = std::sqrt(ox * ox + oy * oy);
    if (D == 0) {
        if (rho <= reach) out.push_back({0, 1});
        return;
    }
    double c = (D * D + rho * rho - reach * reach) / (2 * rho * D);
    if (c > 1) return;
    if (c <= -1) {
        out.push_back({0, 1});
        return;
    }
    double phi = std::atan2(y - motion_.cy, x - motion_.cx);
    double alpha = std::acos(c);
    std::vector<double> cuts;
    double lo = std::min(motion_.start_angle, motion_.start_angle + motion_.sweep);
    double hi = std::max(motion_.start_angle, motion_.start_angle + motion_.sweep);
    for (double m = std::floor((lo - phi - alpha) / kTwoPi); phi + m * kTwoPi - alpha <= hi; m += 1) {
        double a = phi + m * kTwoPi - alpha, b = phi + m * kTwoPi + alpha;
        add_range(cuts, std::max(a, lo), std::min(b, hi));
    }
    for (size_t k = 0; k < cuts.size(); k += 2) {
        double t0 = (cuts[k] - motion_.start_angle) / motion_.sweep;
        double t1 = (cuts[k + 1] - motion_.start_angle) / motion_.sweep;
        if (t0 > t1) std::swap(t0, t1);
        out.push_back({std::max(0.0, t0), std::min(1.0, t1)});
    }
    std::sort(out.begin(), out.end(), [](const Range &a, const Range &b) { return a.t0 < b.t0; });
    // Merge stretches that meet (a full turn wrapping around).
    thread_local std::vector<Range> merged;
    merged.clear();
    for (const Range &r : out) {
        if (!merged.empty() && r.t0 <= merged.back().t1) merged.back().t1 = std::max(merged.back().t1, r.t1);
        else merged.push_back(r);
    }
    out.swap(merged);
}

/** Sub-ranges of `within` where the xy distance lies in the piece's [d0, d1]. */
void SweptVolume::piece_ranges(double x, double y, const Piece &piece, const Range &within,
    std::vector<Range> &out) const {
    out.clear();
    thread_local std::vector<Range> outer, inner;
    inner.clear();
    // Where distance <= d1, minus where distance < d0.
    components(x, y, piece.d1, outer);
    if (piece.d0 > 0) components(x, y, piece.d0, inner);
    for (const Range &o : outer) {
        double t0 = std::max(o.t0, within.t0), t1 = std::min(o.t1, within.t1);
        if (t0 > t1) continue;
        double start = t0;
        for (const Range &i : inner) {
            if (i.t1 < start || i.t0 > t1) continue;
            // The inner range's ends have distance exactly d0, which belongs to the piece.
            if (i.t0 >= start) out.push_back({start, i.t0});
            start = std::max(start, i.t1);
        }
        if (start <= t1) out.push_back({start, t1});
    }
}

/** t in `range` with the smallest xy distance to (x, y). */
double SweptVolume::closest(const Range &range, double x, double y) const {
    if (fixed_distance_) return range.t0;
    if (!motion_.arc) {
        double t = ((x - motion_.p0[0]) * direction_[0] + (y - motion_.p0[1]) * direction_[1]) / length_;
        return std::max(range.t0, std::min(range.t1, t));
    }
    if (x == motion_.cx && y == motion_.cy) return range.t0;
    double phi = std::atan2(y - motion_.cy, x - motion_.cx);
    double a0 = motion_.start_angle + range.t0 * motion_.sweep;
    double a1 = motion_.start_angle + range.t1 * motion_.sweep;
    double lo = std::min(a0, a1), hi = std::max(a0, a1);
    double m = std::ceil((lo - phi) / kTwoPi);
    double angle = phi + m * kTwoPi;
    if (angle <= hi) {
        double t = (angle - motion_.start_angle) / motion_.sweep;
        return std::max(range.t0, std::min(range.t1, t));
    }
    return std::cos(a0 - phi) >= std::cos(a1 - phi) ? range.t0 : range.t1;
}

/** Minimum over t in `range` of sigma * tip height + g(xy distance). */
SweptVolume::Minimum SweptVolume::minimise(const Run &run, double sigma, const Range &range,
    double x, double y) const {
    Minimum best{kInf, range.t0, nullptr};
    auto consider = [&](double t) {
        const Piece *piece = nullptr;
        double g = run.envelope(distance(t, x, y), &piece);
        double value = sigma * motion_.height(t) + g;
        if (value < best.value) best = {value, t, piece};
    };
    if (fixed_distance_ || (motion_.arc && x == motion_.cx && y == motion_.cy)) {
        consider(range.t0);
        consider(range.t1);
        return best;
    }
    if (level_) {
        // g is nondecreasing, so the closest point is lowest.
        consider(closest(range, x, y));
        return best;
    }
    thread_local std::vector<Range> ranges;
    for (const Piece &piece : run.pieces) {
        piece_ranges(x, y, piece, range, ranges);
        auto f = [&](double t) {
            double d = std::max(piece.d0, std::min(piece.d1, distance(t, x, y)));
            return sigma * motion_.height(t) + piece.value(d);
        };
        auto take = [&](double t, double value) {
            if (value < best.value) best = {value, t, &piece};
        };
        for (const Range &r : ranges) {
            take(r.t0, f(r.t0));
            take(r.t1, f(r.t1));
            if (r.t1 <= r.t0) continue;
            if (!motion_.arc && piece.convex) {
                // Convex nondecreasing g of a convex distance, plus a linear height.
                double value;
                double t = golden(f, r.t0, r.t1, value);
                take(t, value);
                continue;
            }
            // Scan for local minima and refine each bracket.
            double span = motion_.arc ? std::fabs(motion_.sweep) * (r.t1 - r.t0) : 1.0;
            int samples = std::max(32, static_cast<int>(std::ceil(64 * span / kPi)));
            std::vector<double> values(samples + 1);
            for (int k = 0; k <= samples; ++k) values[k] = f(r.t0 + (r.t1 - r.t0) * k / samples);
            for (int k = 0; k <= samples; ++k) {
                bool left = k == 0 || values[k] <= values[k - 1];
                bool right = k == samples || values[k] <= values[k + 1];
                if (!left || !right) continue;
                double a = r.t0 + (r.t1 - r.t0) * std::max(0, k - 1) / samples;
                double b = r.t0 + (r.t1 - r.t0) * std::min(samples, k + 1) / samples;
                double value;
                double t = golden(f, a, b, value);
                take(t, value);
            }
        }
    }
    return best;
}

void SweptVolume::normal_at(const Minimum &minimum, double sigma, double x, double y, float out[3]) const {
    double nr = 0, nz = -sigma;
    double px, py, pz;
    motion_.position(minimum.t, px, py, pz);
    double ex = x - px, ey = y - py;
    double d = std::sqrt(ex * ex + ey * ey);
    if (minimum.piece) minimum.piece->normal(d, sigma, nr, nz);
    if (d > 0) {
        ex /= d;
        ey /= d;
    } else {
        ex = 1;
        ey = 0;
    }
    // The remaining material's normal points into the tool.
    out[0] = static_cast<float>(-nr * ex);
    out[1] = static_cast<float>(-nr * ey);
    out[2] = static_cast<float>(-nz);
}

void SweptVolume::intersect_z(double x, double y, std::vector<Span> &out) const {
    out.clear();
    thread_local std::vector<Range> stretches;
    for (const Run &run : profile_.runs()) {
        components(x, y, run.dmax, stretches);
        for (const Range &range : stretches) {
            Span span;
            if (run.widening) {
                Minimum low = minimise(run, 1, range, x, y);
                if (!std::isfinite(low.value)) continue;
                span.lo = low.value;
                normal_at(low, 1, x, y, span.lo_normal);
                span.hi = std::max(motion_.height(range.t0), motion_.height(range.t1)) + run.zb;
                span.hi_normal[0] = span.hi_normal[1] = 0;
                span.hi_normal[2] = -1;
            } else {
                Minimum high = minimise(run, -1, range, x, y);
                if (!std::isfinite(high.value)) continue;
                span.hi = -high.value;
                normal_at(high, -1, x, y, span.hi_normal);
                span.lo = std::min(motion_.height(range.t0), motion_.height(range.t1)) + run.za;
                span.lo_normal[0] = span.lo_normal[1] = 0;
                span.lo_normal[2] = 1;
            }
            if (span.hi > span.lo) out.push_back(span);
        }
    }
    if (out.size() < 2) return;
    std::sort(out.begin(), out.end(), [](const Span &a, const Span &b) { return a.lo < b.lo; });
    size_t kept = 0;
    for (size_t k = 1; k < out.size(); ++k) {
        Span &last = out[kept];
        if (out[k].lo <= last.hi) {
            if (out[k].hi > last.hi) {
                last.hi = out[k].hi;
                std::copy(out[k].hi_normal, out[k].hi_normal + 3, last.hi_normal);
            }
        } else {
            out[++kept] = out[k];
        }
    }
    out.resize(kept + 1);
}

} // namespace stockkit
