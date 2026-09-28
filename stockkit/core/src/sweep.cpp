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

/** Sorts spans and merges those that overlap or touch. */
void merge(std::vector<Span> &out) {
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

/** The motion with x and y exchanged, so a Y ray can be treated as an X ray. */
Motion swapped(const Motion &m) {
    Motion s = m;
    std::swap(s.p0[0], s.p0[1]);
    std::swap(s.p1[0], s.p1[1]);
    std::swap(s.cx, s.cy);
    // x = cx + r cos(a) becomes the second coordinate: cos(a) = sin(pi/2 - a).
    s.start_angle = kPi / 2 - m.start_angle;
    s.sweep = -m.sweep;
    return s;
}

/** Whether an arc's sweep passes through `angle`, modulo full turns. */
bool within_sweep(const Motion &m, double angle) {
    if (std::fabs(m.sweep) >= kTwoPi) return true;
    double lo = std::min(m.start_angle, m.start_angle + m.sweep);
    double hi = std::max(m.start_angle, m.start_angle + m.sweep);
    return angle + std::ceil((lo - angle) / kTwoPi) * kTwoPi <= hi;
}

/** Parameter of a point on the path whose xy position is nearest (x, y). */
double nearest(const Motion &m, double x, double y) {
    if (!m.arc) {
        double dx = m.p1[0] - m.p0[0], dy = m.p1[1] - m.p0[1];
        double squared = dx * dx + dy * dy;
        if (!(squared > 0)) return 0;
        return std::max(0.0, std::min(1.0, ((x - m.p0[0]) * dx + (y - m.p0[1]) * dy) / squared));
    }
    if (m.radius == 0 || m.sweep == 0 || (x == m.cx && y == m.cy)) return 0;
    double phi = std::atan2(y - m.cy, x - m.cx);
    if (within_sweep(m, phi)) {
        // The first time the sweep reaches phi.
        double turns = (m.start_angle - phi) / kTwoPi;
        double angle = phi + (m.sweep > 0 ? std::ceil(turns) : std::floor(turns)) * kTwoPi;
        return std::max(0.0, std::min(1.0, (angle - m.start_angle) / m.sweep));
    }
    double x0, y0, z0, x1, y1, z1;
    m.position(0, x0, y0, z0);
    m.position(1, x1, y1, z1);
    return std::hypot(x - x0, y - y0) <= std::hypot(x - x1, y - y1) ? 0 : 1;
}

double path_distance(const Motion &m, double x, double y) {
    double px, py, pz;
    m.position(nearest(m, x, y), px, py, pz);
    return std::hypot(x - px, y - py);
}

/**
 * Intervals in x of the line y = c within `reach` of a level path: the
 * path's 2D offset, a capsule for a line and an annular sector with end
 * discs for an arc. Its boundary crossings are found in closed form and
 * each stretch between them is tested at its middle. Pairs go to `out`.
 */
void offset_spans(const Motion &m, double reach, double c, std::vector<double> &out) {
    if (!m.arc) {
        // A capsule is convex: its end discs and the strip between them meet
        // the line in overlapping intervals, whose union is one.
        double lo = kInf, hi = -kInf;
        double ax, ay, az, bx, by, bz;
        m.position(0, ax, ay, az);
        m.position(1, bx, by, bz);
        for (const double *end : {&ax, &bx}) {
            double x0 = *end, y0 = end == &ax ? ay : by, dy = c - y0;
            if (std::fabs(dy) > reach) continue;
            double w = std::sqrt(reach * reach - dy * dy);
            lo = std::min(lo, x0 - w);
            hi = std::max(hi, x0 + w);
        }
        double dx = bx - ax, dy = by - ay, length = std::hypot(dx, dy);
        if (length > 0) {
            dx /= length;
            dy /= length;
            double a = -kInf, b = kInf;
            // Sides of the strip, where (x - ax) dy - (c - ay) dx = +-reach.
            if (dy != 0) {
                double p = ax + (-reach + (c - ay) * dx) / dy, q = ax + (reach + (c - ay) * dx) / dy;
                a = std::max(a, std::min(p, q));
                b = std::min(b, std::max(p, q));
            } else if (std::fabs((c - ay) * dx) > reach) {
                b = -kInf;
            }
            // Ends of the strip, where (x - ax) dx + (c - ay) dy = 0 or length.
            if (dx != 0) {
                double p = ax + (0.0 - (c - ay) * dy) / dx, q = ax + (length - (c - ay) * dy) / dx;
                a = std::max(a, std::min(p, q));
                b = std::min(b, std::max(p, q));
            } else if ((c - ay) * dy < 0 || (c - ay) * dy > length) {
                b = -kInf;
            }
            if (a <= b) {
                lo = std::min(lo, a);
                hi = std::max(hi, b);
            }
        }
        out.clear();
        if (lo < hi) out.insert(out.end(), {lo, hi});
        return;
    }
    thread_local std::vector<double> cuts;
    cuts.clear();
    auto disc = [&](double x0, double y0, double r) {
        double dy = c - y0;
        if (std::fabs(dy) > r) return;
        double w = std::sqrt(r * r - dy * dy);
        cuts.insert(cuts.end(), {x0 - w, x0 + w});
    };
    double ax, ay, az, bx, by, bz;
    m.position(0, ax, ay, az);
    m.position(1, bx, by, bz);
    disc(ax, ay, reach);
    disc(bx, by, reach);
    if (!m.arc) {
        double dx = bx - ax, dy = by - ay, length = std::hypot(dx, dy);
        if (length > 0) {
            dx /= length;
            dy /= length;
            // Sides of the strip, where (x - ax) dy - (c - ay) dx = +-reach.
            if (dy != 0)
                for (double side : {-reach, reach}) cuts.push_back(ax + (side + (c - ay) * dx) / dy);
            // Ends of the strip, where (x - ax) dx + (c - ay) dy = 0 or length.
            if (dx != 0)
                for (double along : {0.0, length}) cuts.push_back(ax + (along - (c - ay) * dy) / dx);
        }
    } else {
        disc(m.cx, m.cy, m.radius + reach);
        if (m.radius > reach) disc(m.cx, m.cy, m.radius - reach);
        cuts.push_back(m.cx);
        // Where the sector's edges, rays from the centre, cross the line.
        double dy = c - m.cy;
        for (double angle : {m.start_angle, m.start_angle + m.sweep}) {
            double sine = std::sin(angle);
            if (sine != 0 && dy / sine > 0) cuts.push_back(m.cx + std::cos(angle) * dy / sine);
        }
    }
    std::sort(cuts.begin(), cuts.end());
    out.clear();
    for (size_t k = 0; k + 1 < cuts.size(); ++k) {
        double a = cuts[k], b = cuts[k + 1];
        if (!(b > a) || path_distance(m, 0.5 * (a + b), c) > reach) continue;
        if (!out.empty() && out.back() == a) out.back() = b;
        else out.insert(out.end(), {a, b});
    }
}

/** Minimum of f on [a, b]: n samples, then golden section around each sampled local minimum. */
template <typename F>
double scan_minimum(F &&f, double a, double b, int n, double &at) {
    at = a;
    double best = f(a);
    if (!(b > a)) return best;
    thread_local std::vector<double> values;
    values.resize(n + 1);
    for (int k = 0; k <= n; ++k) values[k] = f(a + (b - a) * k / n);
    for (int k = 0; k <= n; ++k) {
        if (k > 0 && values[k] > values[k - 1]) continue;
        if (k < n && values[k] > values[k + 1]) continue;
        double lo = a + (b - a) * std::max(0, k - 1) / n, hi = a + (b - a) * std::min(n, k + 1) / n;
        double value;
        double t = golden(f, lo, hi, value);
        if (values[k] < value) {
            value = values[k];
            t = a + (b - a) * k / n;
        }
        if (value < best) {
            best = value;
            at = t;
        }
    }
    return best;
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
    merge(out);
}

void SweptVolume::intersect(uint32_t axis, double u, double v, std::vector<Span> &out) const {
    if (axis == 2) intersect_z(u, v, out);
    else intersect_horizontal(axis, u, v, out);
}

void SweptVolume::intersect_horizontal(uint32_t axis, double across, double z, std::vector<Span> &out) const {
    out.clear();
    const int side = axis == 0 ? 1 : 0; // world axis of `across`
    if (!(z >= bounds_.min[2] && z <= bounds_.max[2] && across >= bounds_.min[side] && across <= bounds_.max[side]))
        return;
    // In this frame the ray runs along +x at y = across.
    const Motion m = axis == 0 ? motion_ : swapped(motion_);
    auto normal = [&](double s, double t, double h, double outward, float n[3]) {
        double px, py, pz;
        m.position(t, px, py, pz);
        double ex = s - px, ey = across - py, length = std::hypot(ex, ey);
        if (length > 0) {
            ex /= length;
            ey /= length;
        } else {
            ex = outward;
            ey = 0;
        }
        const Section *section = nullptr;
        double nr = 1, nz = 0;
        profile_.section_radius(h, &section);
        if (section) section->normal(h, nr, nz);
        // The remaining material's normal points into the tool.
        n[axis] = static_cast<float>(-nr * ex);
        n[side] = static_cast<float>(-nr * ey);
        n[2] = static_cast<float>(-nz);
    };
    auto add = [&](double lo, double t_lo, double h_lo, double hi, double t_hi, double h_hi) {
        if (!(hi > lo)) return;
        Span span;
        span.lo = lo;
        span.hi = hi;
        normal(lo, t_lo, h_lo, -1, span.lo_normal);
        normal(hi, t_hi, h_hi, 1, span.hi_normal);
        out.push_back(span);
    };

    if (fixed_distance_) {
        // A plunge sweeps its widest section over the heights it passes.
        double px, py, pz;
        m.position(0, px, py, pz);
        double h0 = z - std::max(m.height(0), m.height(1)), h1 = z - std::min(m.height(0), m.height(1));
        double at = 0;
        double r = profile_.largest_radius(h0, h1, at, nullptr);
        double dy = across - py;
        if (r < std::fabs(dy)) return;
        double w = std::sqrt(r * r - dy * dy);
        add(px - w, 0, at, px + w, 0, at);
        return;
    }

    if (level_) {
        double h = z - m.height(0);
        double r = profile_.section_radius(h);
        if (!(r > 0)) return;
        thread_local std::vector<double> ends;
        offset_spans(m, r, across, ends);
        for (size_t k = 0; k < ends.size(); k += 2)
            add(ends[k], nearest(m, ends[k], across), h, ends[k + 1], nearest(m, ends[k + 1], across), h);
        return;
    }

    // Ramps and helices. The pose at t meets the ray where the section at
    // height h(t) reaches it: margin(t) = R(h(t)) - |across - y(t)| >= 0.
    const double z0 = m.height(0), rise = m.height(1) - z0;
    auto height = [&](double t) { return z - (z0 + t * rise); };
    auto margin = [&](double t) {
        double px, py, pz;
        m.position(t, px, py, pz);
        return profile_.section_radius(height(t)) - std::fabs(across - py);
    };
    auto reach = [&](double t, double &px) {
        double py, pz;
        m.position(t, px, py, pz);
        double r = std::max(0.0, profile_.section_radius(height(t))), dy = across - py;
        return std::sqrt(std::max(0.0, r * r - dy * dy));
    };
    auto samples = [&](double a, double b) {
        double turn = m.arc ? std::fabs(m.sweep) * (b - a) : 0;
        return std::max(32, static_cast<int>(std::ceil(64 * turn / kPi)));
    };
    // Split where the section stops being smooth or the axis crosses the
    // ray's vertical plane (where |across - y| has its corner).
    thread_local std::vector<double> breaks;
    breaks.assign({0.0, 1.0});
    auto split = [&](double t) {
        if (t > 0 && t < 1) breaks.push_back(t);
    };
    for (double b : profile_.breaks()) split((z - z0 - b) / rise);
    if (!m.arc) {
        double dy = m.p1[1] - m.p0[1];
        if (dy != 0) split((across - m.p0[1]) / dy);
    } else if (m.radius > 0 && m.sweep != 0) {
        double sine = (across - m.cy) / m.radius;
        if (std::fabs(sine) <= 1) {
            double lo = std::min(m.start_angle, m.start_angle + m.sweep);
            double hi = std::max(m.start_angle, m.start_angle + m.sweep);
            for (double angle : {std::asin(sine), kPi - std::asin(sine)})
                for (double k = std::ceil((lo - angle) / kTwoPi); angle + k * kTwoPi <= hi; k += 1)
                    split((angle + k * kTwoPi - m.start_angle) / m.sweep);
        }
    }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end()), breaks.end());

    // Stretches of t where the pose meets the ray.
    auto bisect = [&](double in, double out) {
        for (int i = 0; i < 200; ++i) {
            double mid = 0.5 * (in + out);
            if (mid == in || mid == out) break;
            if (margin(mid) >= 0) in = mid;
            else out = mid;
        }
        return in;
    };
    thread_local std::vector<Range> parts;
    thread_local std::vector<double> ts, fs;
    parts.clear();
    for (size_t k = 0; k + 1 < breaks.size(); ++k) {
        const double a = breaks[k], b = breaks[k + 1];
        double mid = height(0.5 * (a + b));
        if (mid < profile_.base() || mid > profile_.height()) continue;
        const int n = samples(a, b);
        ts.resize(n + 1);
        fs.resize(n + 1);
        for (int i = 0; i <= n; ++i) {
            ts[i] = i == n ? b : a + (b - a) * i / n;
            fs[i] = margin(ts[i]);
        }
        bool inside = fs[0] >= 0;
        double start = a;
        for (int i = 1; i <= n; ++i) {
            bool now = fs[i] >= 0;
            if (now == inside) continue;
            if (now) start = bisect(ts[i], ts[i - 1]);
            else parts.push_back({start, bisect(ts[i - 1], ts[i])});
            inside = now;
        }
        if (inside) parts.push_back({start, b});
        // Stretches the samples step over, around local maxima that sampled short.
        for (int i = 0; i <= n; ++i) {
            if (fs[i] >= 0 || (i > 0 && fs[i] < fs[i - 1]) || (i < n && fs[i] < fs[i + 1])) continue;
            double lo = ts[std::max(0, i - 1)], hi = ts[std::min(n, i + 1)];
            double value;
            double t = golden([&](double t) { return -margin(t); }, lo, hi, value);
            if (-value >= 0) parts.push_back({bisect(t, lo), bisect(t, hi)});
        }
    }

    // Over each stretch the swept interval runs from the least x - w to the greatest x + w.
    for (const Range &part : parts) {
        const int n = samples(part.t0, part.t1);
        double t_lo, t_hi;
        double lo = scan_minimum([&](double t) {
            double px;
            double w = reach(t, px);
            return px - w;
        }, part.t0, part.t1, n, t_lo);
        double hi = -scan_minimum([&](double t) {
            double px;
            double w = reach(t, px);
            return -(px + w);
        }, part.t0, part.t1, n, t_hi);
        add(lo, t_lo, height(t_lo), hi, t_hi, height(t_hi));
    }
    merge(out);
}

} // namespace stockkit
