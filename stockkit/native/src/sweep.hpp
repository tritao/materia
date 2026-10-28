#pragma once

#include "profile.hpp"

#include <cstdint>
#include <vector>

namespace stockkit {

/** Tool-tip motion with the tool axis along +Z, parameterised by t in [0, 1]. */
struct Motion {
    bool arc = false;
    // Line from p0 to p1.
    double p0[3] = {0, 0, 0}, p1[3] = {0, 0, 0};
    // Arc or helix about the vertical axis through (cx, cy).
    double cx = 0, cy = 0, z0 = 0, radius = 0, start_angle = 0, sweep = 0, rise = 0;

    void position(double t, double &x, double &y, double &z) const;
    double height(double t) const;
};

struct Bounds {
    double min[3], max[3];
};

/** One interval along a ray with the normals the remaining material gets at its ends. */
struct Span {
    double lo, hi;
    float lo_normal[3], hi_normal[3];
};

/**
 * The solid swept by a surface-of-revolution tool along one motion. Along a
 * vertical line, each run of the profile sweeps one interval per connected
 * stretch of the motion within the run's reach; its lower (or, for narrowing
 * runs, upper) end minimises tip height plus the run's envelope over that
 * stretch, and its other end is set by the run's flat end at the extreme tip
 * height. Horizontal lines and arcs use the closest point in closed form;
 * sloped lines minimise a convex function; helices and non-convex profiles
 * scan and refine.
 */
class SweptVolume {
public:
    SweptVolume(const Profile &profile, const Motion &motion);

    const Bounds &bounds() const { return bounds_; }
    /** Lowest height the swept solid reaches. */
    double lowest() const { return bounds_.min[2]; }

    /**
     * A height at or below every swept point on the +Z ray through (x, y),
     * +infinity when the ray misses. Cheap; exact for level lines.
     */
    double floor_bound(double x, double y) const;

    /** Swept intervals along the +Z ray through (x, y), sorted and disjoint. */
    void intersect_z(double x, double y, std::vector<Span> &out) const;

    /**
     * Swept intervals along the ray through (u, v) on `axis`, sorted and
     * disjoint: Z rays run along +Z through (x, y) = (u, v), X rays along +X
     * through (y, z) = (u, v) and Y rays along +Y through (x, z) = (u, v).
     * At each height the tool is a disc, so a level move sweeps a 2D offset
     * of its path (closed form); a plunge sweeps its widest section; ramps
     * and helices scan and refine.
     */
    void intersect(uint32_t axis, double u, double v, std::vector<Span> &out) const;

private:
    struct Range { double t0, t1; };
    struct Minimum { double value, t; const Piece *piece; };

    double distance(double t, double x, double y) const;
    void components(double x, double y, double reach, std::vector<Range> &out) const;
    void piece_ranges(double x, double y, const Piece &piece, const Range &within,
        std::vector<Range> &out) const;
    Minimum minimise(const Run &run, double sigma, const Range &range, double x, double y) const;
    double closest(const Range &range, double x, double y) const;
    void normal_at(const Minimum &minimum, double sigma, double x, double y, float out[3]) const;

    void intersect_horizontal(uint32_t axis, double across, double z, std::vector<Span> &out) const;

    const Profile &profile_;
    Motion motion_;
    Bounds bounds_;
    double tip_low_ = 0;         // lowest tip height over the motion
    bool level_ = false;         // tip height constant
    bool fixed_distance_ = false; // xy distance to the axis independent of t (plunge, degenerate arc)
    double length_ = 0;          // xy length of a line
    double direction_[2] = {0, 0}; // its xy unit direction
};

} // namespace stockkit
