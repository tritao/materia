#pragma once

#include "model.hpp"

#include <coal/collision.h>
#include <coal/collision_object.h>
#include <coal/distance.h>

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace kk {

/** One checked pair's closest approach, in world coordinates. */
struct PairDistance {
    uint32_t a = 0, b = 0;
    double distance = 0.0;
    double point_a[3] = {0, 0, 0}, point_b[3] = {0, 0, 0}, normal[3] = {0, 0, 0};
};

/**
 * Shapes attached to the bodies of a copy of a model, posed together by its
 * forward kinematics, with every pair's status (see `kinematicskit.h`).
 */
class CollisionWorld {
public:
    explicit CollisionWorld(const Model &model);

    uint32_t body_count() const { return model_.body_count; }
    uint32_t dof_count() const { return model_.dof_count; }

    /** Adds `geometry` on `body` (-1: the world) at `offset` (a normalized seven-double transform). */
    uint32_t add(int32_t body, const double *offset, std::shared_ptr<coal::CollisionGeometry> geometry,
                 bool height_field);
    bool contains(uint32_t object) const { return objects_.count(object) != 0; }
    bool is_height_field(uint32_t object) const;
    /** Replaces a height field's heights (row-major); false when the count differs from its grid's. */
    bool set_heights(uint32_t object, const double *heights, uint32_t count);
    void attach(uint32_t object, int32_t body, const double *offset);
    void remove(uint32_t object);
    void set_rule(uint32_t a, uint32_t b, int32_t rule);
    int32_t status(uint32_t a, uint32_t b) const;

    /** Poses every object; `roots` (7 per body) overrides the model's root poses when non-null. */
    void update(const double *q, const double *roots);
    /** Marks the pairs colliding at that configuration as overlapping at reference; returns how many. */
    uint32_t allow_overlapping(const double *q, const double *roots);

    /** False when a checked pair is unsupported. Otherwise the pairs closer than `margin`, in id order. */
    bool check(double margin, std::vector<std::pair<uint32_t, uint32_t>> &out);
    /** False when a checked pair is unsupported. Otherwise the pairs closer than `query`, closest first. */
    bool distances(double query, std::vector<PairDistance> &out);

private:
    struct Object {
        int32_t body = -1;
        double offset[7] = {0, 0, 0, 0, 0, 0, 1};
        bool height_field = false;
        std::unique_ptr<coal::CollisionObject> object;
    };
    struct Pair {
        uint32_t a, b;
        coal::CollisionObject *first, *second;
        /** coal's own queries; null for a height-field pair, which goes through its cells instead. */
        std::unique_ptr<coal::ComputeCollision> collide;
        std::unique_ptr<coal::ComputeDistance> distance;
        /** 1 when `first` is a height field, 2 when `second` is, else 0. */
        int field = 0;
    };
    using Key = std::pair<uint32_t, uint32_t>;
    static Key key(uint32_t a, uint32_t b) { return a < b ? Key(a, b) : Key(b, a); }

    /** The pair's status before support is considered (KK_PAIR_*). */
    int32_t declared_status(const Key &pair) const;
    bool adjacent(int32_t a, int32_t b) const;
    void pose(Object &object);
    void rebuild_pairs();
    /** True when the pair is closer than `margin` (touching counts at 0); `request` carries the margin for coal. */
    static bool collides(Pair &pair, double margin, const coal::CollisionRequest &request);
    /** False when coal cannot compare the two objects' shapes (directly, or through height-field cells). */
    static bool supported(const Object &first, const Object &second);
    /**
     * Signed distance between a height field and another object, through the
     * field's cells within `query` of it (see `kinematicskit.h`); `out` gets
     * the closest points with the field as `field_first` says. Infinite when no
     * cell is that close. With `stop_at`, stops once a cell is at or below it.
     */
    static double field_distance(const coal::CollisionObject &field, const coal::CollisionObject &other, double query,
                                 bool field_first, PairDistance *out, double stop_at);

    Model model_;
    /** Per body: the top body of the group it is rigidly fixed to. */
    std::vector<int32_t> rigid_root_;
    std::map<uint32_t, Object> objects_;
    uint32_t next_id_ = 1;
    std::map<Key, int32_t> rules_;
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

} // namespace kk
