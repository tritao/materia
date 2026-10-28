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

/**
 * One profile segment seen as the tool's radius against height, for
 * horizontal sections. A horizontal segment has z0 == z1 and stands for its
 * larger radius.
 */
struct Section {
    bool arc = false;
    double z0 = 0, z1 = 0;
    double r0 = 0, r1 = 0;
    // Arc: r = cr + side * sqrt(radius^2 - (h - cz)^2); turn is +1 when the
    // profile turns counter-clockwise about the centre (centre inside the tool).
    double cr = 0, cz = 0, radius = 0, side = 1, turn = 1;

    double radius_at(double h) const;
    /** The tool's outward normal in the (r, z) half-plane at height h. */
    void normal(double h, double &nr, double &nz) const;
};

class Profile {
public:
    /**
     * Validates `segments`; on failure returns false and sets `error`. A
     * profile normally starts at the tip (0, 0). With `from_tip` false it is
     * a band of a tool starting at its first point: the solid between that
     * height and the last one, closed by flat discs at both.
     */
    static bool build(const std::vector<Segment> &segments, Profile &out, std::string &error,
        bool from_tip = true);

    double radius() const { return radius_; }
    /** Height of the bottom of the solid above the tip: 0 unless a band. */
    double base() const { return base_; }
    double height() const { return height_; }
    const std::vector<Run> &runs() const { return runs_; }
    const std::vector<Segment> &segments() const { return segments_; }

    /** Tool solid along a vertical line at radial distance d, heights above the tip. */
    void slice(double d, std::vector<double> &intervals) const;

    /**
     * Radius of the tool's horizontal section (a disc) at height h above the
     * tip, the larger one at a step; negative outside the tool. `section`
     * receives the segment giving it, preferring a sloped one on ties.
     */
    double section_radius(double h, const Section **section = nullptr) const;
    /** Largest section radius over heights [h0, h1], negative if the tool misses them. */
    double largest_radius(double h0, double h1, double &at, const Section **section) const;
    /** Heights where the section radius stops being smooth: segment ends, sorted. */
    const std::vector<double> &breaks() const { return breaks_; }

private:
    std::vector<Segment> segments_;
    std::vector<Run> runs_;
    std::vector<Section> sections_;
    std::vector<double> breaks_;
    double tolerance_ = 0;
    double radius_ = 0;
    double base_ = 0;
    double height_ = 0;
};

} // namespace stockkit
