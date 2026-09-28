#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace stockkit {

/** One tool half-profile segment: r from the axis, z above the tip. */
struct Segment {
    bool arc = false;
    double r0 = 0, z0 = 0, r1 = 0, z1 = 0;
    double cr = 0, cz = 0;
};

/**
 * One smooth piece of an envelope function g(d), defined for radial distance
 * d in [d0, d1] and nondecreasing there. A line piece is g = a + b d (b >= 0);
 * an arc piece is g = cz + s * sqrt(R^2 - (d - cr)^2).
 */
struct Piece {
    bool arc = false;
    double d0 = 0, d1 = 0;
    double a = 0, b = 0;
    double cr = 0, cz = 0, radius = 0, s = 0;
    /** g is convex on [d0, d1] and on its constant/linear extension left of d0. */
    bool convex = true;

    double value(double d) const;
    /**
     * Direction of (g'(d), -sigma) scaled to stay finite where g' is not:
     * the tool's outward normal in the (r, z) half-plane at this piece, for
     * sigma = +1 (lower envelope) or -1 (upper envelope, with z = -g).
     */
    void normal(double d, double sigma, double &nr, double &nz) const;
};

/**
 * A stretch of the profile over heights [za, zb] where the radius only grows
 * (widening) or only shrinks (narrowing) going up. Along a vertical line at
 * radial distance d <= dmax, the run's part of the tool is one interval:
 * widening: [za + ... , zb] with lower end g(d) (pieces give g directly);
 * narrowing: [za, -g(d)] (pieces give the negated upper end).
 * Either way g is nondecreasing in d.
 */
struct Run {
    bool widening = true;
    double za = 0, zb = 0;
    double dmax = 0;
    std::vector<Piece> pieces;

    /** g(d), taking the lowest piece value where pieces meet or overlap. */
    double envelope(double d, const Piece **piece = nullptr) const;
};

class Profile {
public:
    /** Validates `segments`; on failure returns false and sets `error`. */
    static bool build(const std::vector<Segment> &segments, Profile &out, std::string &error);

    double radius() const { return radius_; }
    double height() const { return height_; }
    const std::vector<Run> &runs() const { return runs_; }
    const std::vector<Segment> &segments() const { return segments_; }

    /** Tool solid along a vertical line at radial distance d, heights above the tip. */
    void slice(double d, std::vector<double> &intervals) const;

private:
    std::vector<Segment> segments_;
    std::vector<Run> runs_;
    double radius_ = 0;
    double height_ = 0;
};

} // namespace stockkit
