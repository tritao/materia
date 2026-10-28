#include "profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace stockkit {

namespace {

constexpr double kPi = 3.14159265358979323846;

/** A profile piece whose radius and height are each monotone along it. */
struct Monotone {
    Segment segment;
    int direction = 0; // sign of r1 - r0
};

double arc_radius(const Segment &s) {
    return std::hypot(s.r0 - s.cr, s.z0 - s.cz);
}

/** Start and signed sweep of a minor arc. */
void arc_angles(const Segment &s, double &start, double &sweep) {
    start = std::atan2(s.z0 - s.cz, s.r0 - s.cr);
    sweep = std::atan2(s.z1 - s.cz, s.r1 - s.cr) - start;
    while (sweep > kPi) sweep -= 2 * kPi;
    while (sweep <= -kPi) sweep += 2 * kPi;
}

bool near(double a, double b, double scale) {
    return std::fabs(a - b) <= 1e-12 * std::max(1.0, scale);
}

/** Splits arcs at their vertical tangents (angles 0 and pi) so r is monotone on each piece. */
void split_monotone(const std::vector<Segment> &segments, double scale, std::vector<Monotone> &out) {
    for (const Segment &s : segments) {
        if (!s.arc) {
            Monotone m{s, near(s.r0, s.r1, scale) ? 0 : (s.r1 > s.r0 ? 1 : -1)};
            if (m.direction == 0) m.segment.r1 = m.segment.r0;
            out.push_back(m);
            continue;
        }
        double start, sweep;
        arc_angles(s, start, sweep);
        const double radius = arc_radius(s);
        std::vector<double> cuts{start};
        for (double tangent : {-2 * kPi, -kPi, 0.0, kPi, 2 * kPi}) {
            double lo = std::min(start, start + sweep), hi = std::max(start, start + sweep);
            if (tangent > lo + 1e-12 && tangent < hi - 1e-12) cuts.push_back(tangent);
        }
        if (sweep < 0) std::sort(cuts.begin() + 1, cuts.end(), std::greater<double>());
        else std::sort(cuts.begin() + 1, cuts.end());
        cuts.push_back(start + sweep);
        for (size_t k = 0; k + 1 < cuts.size(); ++k) {
            Segment piece = s;
            if (k > 0) {
                piece.r0 = s.cr + radius * std::cos(cuts[k]);
                piece.z0 = s.cz + radius * std::sin(cuts[k]);
            }
            if (k + 2 < cuts.size()) {
                piece.r1 = s.cr + radius * std::cos(cuts[k + 1]);
                piece.z1 = s.cz + radius * std::sin(cuts[k + 1]);
            }
            int direction = near(piece.r0, piece.r1, scale) ? 0 : (piece.r1 > piece.r0 ? 1 : -1);
            out.push_back({piece, direction});
        }
    }
}

/** Branch of the circle an r-monotone arc lies on: +1 above its centre, -1 below. */
double branch(const Segment &s) {
    double start, sweep;
    arc_angles(s, start, sweep);
    return std::sin(start + sweep / 2) >= 0 ? 1.0 : -1.0;
}

Piece constant(double d0, double d1, double g) {
    Piece p;
    p.d0 = d0;
    p.d1 = d1;
    p.a = g;
    return p;
}

/**
 * The run's piece for one monotone segment. For a widening run g is the
 * segment's height as a function of r; for a narrowing run it is minus that.
 */
Piece piece_for(const Segment &s, bool widening) {
    const double sign = widening ? 1.0 : -1.0;
    Piece p;
    p.d0 = std::min(s.r0, s.r1);
    p.d1 = std::max(s.r0, s.r1);
    if (!s.arc) {
        double slope = (s.z1 - s.z0) / (s.r1 - s.r0);
        p.b = sign * slope;
        p.a = sign * s.z0 - p.b * s.r0;
        p.b = std::max(0.0, p.b); // rounding on flat steps
        return p;
    }
    p.arc = true;
    p.cr = s.cr;
    p.cz = sign * s.cz;
    p.radius = arc_radius(s);
    p.s = sign * branch(s);
    p.convex = p.s < 0;
    return p;
}

} // namespace

double Piece::value(double d) const {
    if (!arc) return a + b * d;
    double dr = std::min(std::fabs(d - cr), radius);
    return cz + s * std::sqrt(std::max(0.0, radius * radius - dr * dr));
}

void Piece::normal(double d, double sigma, double &nr, double &nz) const {
    if (!arc) {
        double length = std::hypot(b, 1.0);
        nr = b / length;
        nz = -sigma / length;
        return;
    }
    // (g', -sigma) scaled by sqrt(R^2 - (d - cr)^2) > 0.
    double dr = std::max(-radius, std::min(radius, d - cr));
    double root = std::sqrt(std::max(0.0, radius * radius - dr * dr));
    double x = -s * dr, z = -sigma * root;
    double length = std::hypot(x, z);
    if (!(length > 0)) {
        nr = 0;
        nz = -sigma;
        return;
    }
    nr = x / length;
    nz = z / length;
}

double Section::radius_at(double h) const {
    if (!arc) {
        if (z1 == z0) return std::max(r0, r1);
        double f = std::max(0.0, std::min(1.0, (h - z0) / (z1 - z0)));
        return r0 + f * (r1 - r0);
    }
    double dz = std::max(-radius, std::min(radius, h - cz));
    return cr + side * std::sqrt(std::max(0.0, radius * radius - dz * dz));
}

void Section::normal(double h, double &nr, double &nz) const {
    if (!arc) {
        // Right of the direction of travel up the profile.
        double dr = r1 - r0, dz = z1 - z0;
        double length = std::hypot(dr, dz);
        if (!(length > 0)) {
            nr = 1;
            nz = 0;
            return;
        }
        nr = dz / length;
        nz = -dr / length;
        return;
    }
    double r = radius_at(h);
    double dz = std::max(-radius, std::min(radius, h - cz));
    nr = turn * (r - cr) / radius;
    nz = turn * dz / radius;
}

double Run::envelope(double d, const Piece **piece) const {
    double best = std::numeric_limits<double>::infinity();
    for (const Piece &p : pieces) {
        if (d < p.d0 || d > p.d1) continue;
        double g = p.value(d);
        if (g < best) {
            best = g;
            if (piece) *piece = &p;
        }
    }
    return best;
}

bool Profile::build(const std::vector<Segment> &segments, Profile &out, std::string &error, bool from_tip) {
    if (segments.empty()) {
        error = "profile needs at least one segment";
        return false;
    }
    double scale = 0;
    for (const Segment &s : segments)
        for (double v : {s.r0, s.z0, s.r1, s.z1, s.cr, s.cz}) {
            if (!std::isfinite(v)) {
                error = "profile values must be finite";
                return false;
            }
            scale = std::max(scale, std::fabs(v));
        }
    double r = from_tip ? 0 : segments.front().r0, z = from_tip ? 0 : segments.front().z0;
    const double base = z;
    if (r < 0) {
        error = "profile radii must be non-negative";
        return false;
    }
    for (const Segment &s : segments) {
        if (!near(s.r0, r, scale) || !near(s.z0, z, scale)) {
            error = from_tip ? "profile segments must be continuous from the tip at (0, 0)"
                             : "profile segments must be continuous";
            return false;
        }
        if (s.r1 < 0 || s.z1 < s.z0 - 1e-12 * std::max(1.0, scale)) {
            error = "profile radii must be non-negative and heights must not decrease";
            return false;
        }
        if (s.arc) {
            double a = std::hypot(s.r0 - s.cr, s.z0 - s.cz);
            double b = std::hypot(s.r1 - s.cr, s.z1 - s.cz);
            if (!(a > 0) || std::fabs(a - b) > 1e-9 * std::max(1.0, a)) {
                error = "profile arc ends must be equidistant from its centre";
                return false;
            }
            // Heights must not decrease inside the arc either: no horizontal
            // tangent (angle +-pi/2) strictly inside it.
            double start, sweep;
            arc_angles(s, start, sweep);
            for (double tangent : {-2.5 * kPi, -1.5 * kPi, -0.5 * kPi, 0.5 * kPi, 1.5 * kPi, 2.5 * kPi}) {
                double lo = std::min(start, start + sweep), hi = std::max(start, start + sweep);
                if (tangent > lo + 1e-9 && tangent < hi - 1e-9) {
                    error = "profile arc heights must not decrease";
                    return false;
                }
            }
        }
        r = s.r1;
        z = s.z1;
    }
    if (!(z > base) || !(r > 0)) {
        error = "profile must end above where it starts and away from the axis";
        return false;
    }

    Profile profile;
    profile.segments_ = segments;
    profile.base_ = base;
    profile.height_ = z;

    profile.tolerance_ = 1e-12 * std::max(1.0, scale);
    for (const Segment &s : segments) {
        Section section;
        section.arc = s.arc;
        section.z0 = s.z0;
        section.z1 = std::max(s.z0, s.z1);
        section.r0 = s.r0;
        section.r1 = s.r1;
        if (s.arc) {
            double start, sweep;
            arc_angles(s, start, sweep);
            section.cr = s.cr;
            section.cz = s.cz;
            section.radius = arc_radius(s);
            // Heights never turn back inside an arc, so it keeps to one side of its centre.
            section.side = std::cos(start + sweep / 2) >= 0 ? 1.0 : -1.0;
            section.turn = sweep >= 0 ? 1.0 : -1.0;
        }
        profile.sections_.push_back(section);
        profile.breaks_.push_back(section.z0);
        profile.breaks_.push_back(section.z1);
    }
    std::sort(profile.breaks_.begin(), profile.breaks_.end());
    profile.breaks_.erase(std::unique(profile.breaks_.begin(), profile.breaks_.end()), profile.breaks_.end());

    std::vector<Monotone> monotone;
    split_monotone(segments, scale, monotone);
    // Vertical pieces take the direction of the piece before them (or, at
    // the tip, after them); consecutive pieces of one direction form a run.
    int previous = 1;
    for (const Monotone &m : monotone)
        if (m.direction != 0) {
            previous = m.direction;
            break;
        }
    struct Group { bool widening; std::vector<Segment> segments; };
    std::vector<Group> groups;
    for (const Monotone &m : monotone) {
        int direction = m.direction != 0 ? m.direction : previous;
        previous = direction;
        if (groups.empty() || groups.back().widening != (direction > 0))
            groups.push_back({direction > 0, {}});
        groups.back().segments.push_back(m.segment);
    }

    for (const Group &group : groups) {
        Run run;
        run.widening = group.widening;
        run.za = group.segments.front().z0;
        run.zb = group.segments.back().z1;
        for (const Segment &s : group.segments)
            run.dmax = std::max(run.dmax, std::max(s.r0, s.r1));
        if (group.widening) {
            // Below the run's first radius the run is a flat-bottomed disc.
            double first = group.segments.front().r0;
            if (first > 0) run.pieces.push_back(constant(0, first, run.za));
            for (const Segment &s : group.segments)
                if (s.r1 != s.r0) run.pieces.push_back(piece_for(s, true));
        } else {
            double last = group.segments.back().r1;
            if (last > 0) run.pieces.push_back(constant(0, last, -run.zb));
            for (auto it = group.segments.rbegin(); it != group.segments.rend(); ++it)
                if (it->r1 != it->r0) run.pieces.push_back(piece_for(*it, false));
        }
        if (run.dmax > 0) profile.runs_.push_back(run);
        profile.radius_ = std::max(profile.radius_, run.dmax);
    }
    out = std::move(profile);
    return true;
}

double Profile::section_radius(double h, const Section **section) const {
    double best = -1;
    bool sloped = false;
    for (const Section &s : sections_) {
        if (h < s.z0 - tolerance_ || h > s.z1 + tolerance_) continue;
        double r = s.radius_at(h);
        bool flat = s.z1 == s.z0;
        if (r > best + tolerance_ || (r >= best - tolerance_ && !flat && !sloped)) {
            best = std::max(r, best);
            sloped = !flat;
            if (section) *section = &s;
        }
    }
    return best;
}

double Profile::largest_radius(double h0, double h1, double &at, const Section **section) const {
    double best = -1;
    h0 = std::max(h0, base_);
    h1 = std::min(h1, height_);
    for (const Section &s : sections_) {
        double lo = std::max(h0, s.z0 - tolerance_), hi = std::min(h1, s.z1 + tolerance_);
        if (lo > hi) continue;
        auto consider = [&](double h) {
            double r = s.radius_at(h);
            if (r > best) {
                best = r;
                at = h;
                if (section) *section = &s;
            }
        };
        consider(lo);
        consider(hi);
        if (s.arc && s.side > 0 && s.cz > lo && s.cz < hi) consider(s.cz);
    }
    return best;
}

void Profile::slice(double d, std::vector<double> &intervals) const {
    intervals.clear();
    for (const Run &run : runs_) {
        if (d > run.dmax) continue;
        double g = run.envelope(d);
        if (!std::isfinite(g)) continue;
        if (run.widening) intervals.insert(intervals.end(), {g, run.zb});
        else intervals.insert(intervals.end(), {run.za, -g});
    }
}

} // namespace stockkit
