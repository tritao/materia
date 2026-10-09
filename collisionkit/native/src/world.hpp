#pragma once

#include <coal/collision.h>
#include <coal/collision_object.h>
#include <coal/distance.h>

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace ck {

/** One pair's closest approach, in world coordinates, with the clearance its query required. */
struct PairResult {
    uint32_t a = 0, b = 0;
    int32_t body_a = -1, body_b = -1;
    double distance = 0.0, required = 0.0;
    double point_a[3] = {0, 0, 0}, point_b[3] = {0, 0, 0}, normal[3] = {0, 0, 0};
};

/** Margins per pair of body groups (`collisionkit.h`): a square table, read at row <= column. */
struct Margins {
    uint32_t groups = 0;
    const double *table = nullptr;
    double between(int32_t g, int32_t h) const {
        return g <= h ? table[size_t(g) * groups + size_t(h)] : table[size_t(h) * groups + size_t(g)];
    }
};

/**
 * Objects attached to bodies the caller poses, with every pair's status (see
 * `collisionkit.h`). Bodies are the world's own; it holds no model and no
 * kinematics.
 */
class World {
public:
    uint32_t add_body(int32_t group);
    uint32_t body_count() const { return uint32_t(bodies_.size()); }
    bool has_body(int32_t body) const { return body >= -1 && body < int32_t(bodies_.size()); }
    int32_t group_of(int32_t body) const { return body < 0 ? 0 : bodies_[size_t(body)].group; }
    /** The largest group any body is in (0 without bodies). */
    int32_t max_group() const;
    void set_body_group(uint32_t body, int32_t group) { bodies_[body].group = group; }
    void set_body_static(uint32_t body, bool fixed);
    /** Poses `body` from a normalized seven-double transform, moving its objects. */
    void set_body_pose(uint32_t body, const double *pose);

    /** Adds `geometry` on `body` (-1: the world) at `offset` (a normalized seven-double transform). */
    uint32_t add(int32_t body, const double *offset, std::shared_ptr<coal::CollisionGeometry> geometry,
                 bool height_field);
    bool contains(uint32_t object) const { return objects_.count(object) != 0; }
    bool is_height_field(uint32_t object) const;
    /** True for primitives and convex sets: what coal can inflate. */
    bool inflatable(uint32_t object) const;
    void set_inflation(uint32_t object, double radius);
    /** Replaces a height field's heights (row-major); false when the count differs or a height is below the minimum. */
    bool set_heights(uint32_t object, const double *heights, uint32_t count);
    void attach(uint32_t object, int32_t body, const double *offset);
    void remove(uint32_t object);
    void set_body_rule(int32_t a, int32_t b, int32_t rule, int32_t reason);
    void set_object_rule(uint32_t a, uint32_t b, int32_t rule, int32_t reason);
    int32_t status(uint32_t a, uint32_t b) const;

    /** Marks the pairs among `bodies` colliding now as overlapping at reference; returns how many. */
    uint32_t allow_overlapping(const std::vector<int32_t> &bodies);

    /** False when a checked pair is unsupported. Otherwise the pairs closer than `margin`, in id order. */
    bool check(double margin, std::vector<PairResult> &out);
    /** False when a checked pair is unsupported. Otherwise the pairs closer than `query`, closest first. */
    bool distances(double query, std::vector<PairResult> &out);
    /** False when coal cannot compare the two objects. Otherwise their distance, whatever their status. */
    bool pair_distance(uint32_t a, uint32_t b, PairResult &out);
    /**
     * False when a checked pair is unsupported. Otherwise `found` says whether a checked pair is closer than
     * its margin plus both bodies' `inflation` (null: none), the first in id order going into `out`.
     */
    bool violation(const Margins &margins, const double *inflation, bool &found, PairResult &out);
    /** As `violation`, for the closest checked pair; `found` is false only when none is checked. */
    bool closest(const Margins &margins, bool &found, PairResult &out);

private:
    struct Body {
        double pose[7] = {0, 0, 0, 0, 0, 0, 1};
        int32_t group = 0;
        bool fixed = false;
        std::vector<uint32_t> objects;
    };
    struct Object {
        int32_t body = -1;
        double offset[7] = {0, 0, 0, 0, 0, 0, 1};
        bool height_field = false;
        std::unique_ptr<coal::CollisionObject> object;
    };
    struct Rule {
        int32_t rule = 0, reason = 0;
    };
    struct Pair {
        uint32_t a, b;
        int32_t body_a, body_b;
        coal::CollisionObject *first, *second;
        /** coal's own queries; no collision query for a height-field pair, which is checked by distance. */
        std::unique_ptr<coal::ComputeCollision> collide;
        std::unique_ptr<coal::ComputeDistance> distance;
        /** 1 when `first` is a height field, 2 when `second` is, else 0. */
        int field = 0;
    };
    using Key = std::pair<uint32_t, uint32_t>;
    using BodyKey = std::pair<int32_t, int32_t>;
    static Key key(uint32_t a, uint32_t b) { return a < b ? Key(a, b) : Key(b, a); }
    static BodyKey body_key(int32_t a, int32_t b) { return a < b ? BodyKey(a, b) : BodyKey(b, a); }

    /** The pair's status before support is considered (CK_PAIR_*). */
    int32_t declared_status(const Key &pair) const;
    bool is_static(int32_t body) const { return body < 0 || bodies_[size_t(body)].fixed; }
    void pose(Object &object);
    void rebuild_pairs();
    /** A pair for coal's queries, or false when coal cannot compare the two. */
    bool make_pair(uint32_t a, uint32_t b, Pair &out);
    /** True when the pair is closer than `margin` (touching counts at 0); `request` carries the margin for coal. */
    static bool collides(Pair &pair, double margin, coal::CollisionRequest &request);
    /** The pair's signed distance with closest points into `out` (ids and bodies included). */
    static void measure(Pair &pair, PairResult &out);
    /** False when coal cannot compare the two objects' shapes (directly, or through height-field cells). */
    static bool supported(const Object &first, const Object &second);

    std::vector<Body> bodies_;
    std::map<uint32_t, Object> objects_;
    uint32_t next_id_ = 1;
    std::map<Key, Rule> object_rules_;
    std::map<BodyKey, Rule> body_rules_;
    std::set<Key> reference_;
    std::vector<Pair> pairs_;
    bool pairs_dirty_ = true, unsupported_ = false;
};

/** A convex set from its points alone: no hull topology, so support queries scan the points (coal without qhull). */
class PointConvex : public coal::ConvexBase {
public:
    explicit PointConvex(std::shared_ptr<std::vector<coal::Vec3s>> points);
    PointConvex *clone() const override { return new PointConvex(*this); }
    /** Replaces the points (reusing one object for many queries). */
    void reset(std::shared_ptr<std::vector<coal::Vec3s>> points);

private:
    PointConvex(const PointConvex &other) = default;
};

} // namespace ck
