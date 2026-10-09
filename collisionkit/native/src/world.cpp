#include "world.hpp"

#include "collisionkit.h"

#include <coal/hfield.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ck {

namespace {

coal::Transform3s transform_of(const double *pose) {
    return coal::Transform3s(coal::Quatf(pose[6], pose[3], pose[4], pose[5]),
                             coal::Vec3s(pose[0], pose[1], pose[2]));
}

/** A lower bound on the distance between two objects from their world boxes (zero when they overlap). */
double box_gap(const coal::CollisionObject &a, const coal::CollisionObject &b) {
    const coal::AABB &x = a.getAABB(), &y = b.getAABB();
    return x.overlap(y) ? 0.0 : x.distance(y);
}

} // namespace

PointConvex::PointConvex(std::shared_ptr<std::vector<coal::Vec3s>> points) : coal::ConvexBase() {
    reset(std::move(points));
}

void PointConvex::reset(std::shared_ptr<std::vector<coal::Vec3s>> points) {
    const unsigned int count = static_cast<unsigned int>(points->size());
    set(std::move(points), count);
    computeLocalAABB();
}

uint32_t World::add_body(int32_t group) {
    bodies_.emplace_back();
    bodies_.back().group = group;
    pairs_dirty_ = true;
    return uint32_t(bodies_.size() - 1);
}

int32_t World::max_group() const {
    int32_t most = 0;
    for (const Body &body : bodies_) most = std::max(most, body.group);
    return most;
}

void World::set_body_static(uint32_t body, bool fixed) {
    bodies_[body].fixed = fixed;
    pairs_dirty_ = true;
}

void World::set_body_pose(uint32_t body, const double *pose) {
    Body &target = bodies_[body];
    std::copy(pose, pose + 7, target.pose);
    for (uint32_t id : target.objects) this->pose(objects_.at(id));
}

uint32_t World::add(int32_t body, const double *offset, std::shared_ptr<coal::CollisionGeometry> geometry,
                    bool height_field) {
    const uint32_t id = next_id_++;
    Object &object = objects_[id];
    object.body = body;
    std::copy(offset, offset + 7, object.offset);
    object.height_field = height_field;
    object.object = std::make_unique<coal::CollisionObject>(std::move(geometry));
    if (body >= 0) bodies_[size_t(body)].objects.push_back(id);
    pose(object);
    pairs_dirty_ = true;
    return id;
}

bool World::is_height_field(uint32_t id) const {
    auto found = objects_.find(id);
    return found != objects_.end() && found->second.height_field;
}

bool World::inflatable(uint32_t id) const {
    auto found = objects_.find(id);
    return found != objects_.end() && !found->second.height_field &&
           found->second.object->collisionGeometryPtr()->getObjectType() == coal::OT_GEOM;
}

void World::set_inflation(uint32_t id, double radius) {
    Object &object = objects_.at(id);
    auto *shape = static_cast<coal::ShapeBase *>(object.object->collisionGeometry().get());
    shape->setSweptSphereRadius(radius);
    shape->computeLocalAABB();
    object.object->computeAABB();
    pairs_dirty_ = true;
}

bool World::set_heights(uint32_t id, const double *values, uint32_t count) {
    Object &object = objects_.at(id);
    auto *field = static_cast<coal::HeightField<coal::OBBRSS> *>(object.object->collisionGeometry().get());
    const auto rows = field->getHeights().rows(), cols = field->getHeights().cols();
    if (int64_t(count) != int64_t(rows) * cols) return false;
    for (uint32_t i = 0; i < count; ++i)
        if (!(values[i] >= field->getMinHeight())) return false;
    coal::MatrixXs heights(rows, cols);
    for (Eigen::Index r = 0; r < rows; ++r)
        for (Eigen::Index c = 0; c < cols; ++c) heights(r, c) = values[r * cols + c];
    field->updateHeights(heights);
    field->computeLocalAABB();
    object.object->computeAABB();
    return true;
}

void World::attach(uint32_t id, int32_t body, const double *offset) {
    Object &object = objects_.at(id);
    if (object.body >= 0) {
        auto &list = bodies_[size_t(object.body)].objects;
        list.erase(std::remove(list.begin(), list.end(), id), list.end());
    }
    object.body = body;
    if (body >= 0) bodies_[size_t(body)].objects.push_back(id);
    std::copy(offset, offset + 7, object.offset);
    pose(object);
    for (auto it = reference_.begin(); it != reference_.end();)
        it = it->first == id || it->second == id ? reference_.erase(it) : std::next(it);
    pairs_dirty_ = true;
}

void World::remove(uint32_t id) {
    const int32_t body = objects_.at(id).body;
    if (body >= 0) {
        auto &list = bodies_[size_t(body)].objects;
        list.erase(std::remove(list.begin(), list.end(), id), list.end());
    }
    objects_.erase(id);
    for (auto it = object_rules_.begin(); it != object_rules_.end();)
        it = it->first.first == id || it->first.second == id ? object_rules_.erase(it) : std::next(it);
    for (auto it = reference_.begin(); it != reference_.end();)
        it = it->first == id || it->second == id ? reference_.erase(it) : std::next(it);
    pairs_dirty_ = true;
}

void World::set_body_rule(int32_t a, int32_t b, int32_t rule, int32_t reason) {
    if (rule == CK_RULE_DEFAULT) body_rules_.erase(body_key(a, b));
    else body_rules_[body_key(a, b)] = Rule{rule, reason};
    pairs_dirty_ = true;
}

void World::set_object_rule(uint32_t a, uint32_t b, int32_t rule, int32_t reason) {
    if (rule == CK_RULE_DEFAULT) object_rules_.erase(key(a, b));
    else object_rules_[key(a, b)] = Rule{rule, reason};
    pairs_dirty_ = true;
}

int32_t World::declared_status(const Key &pair) const {
    auto rule = object_rules_.find(pair);
    if (rule != object_rules_.end()) return rule->second.rule == CK_RULE_ALLOW ? rule->second.reason : CK_PAIR_CHECKED;
    const int32_t a = objects_.at(pair.first).body, b = objects_.at(pair.second).body;
    if (a != b) {
        auto body_rule = body_rules_.find(body_key(a, b));
        if (body_rule != body_rules_.end())
            return body_rule->second.rule == CK_RULE_ALLOW ? body_rule->second.reason : CK_PAIR_CHECKED;
    }
    if (is_static(a) && is_static(b)) return CK_PAIR_STATIC;
    if (a == b) return CK_PAIR_RIGID;
    if (reference_.count(pair)) return CK_PAIR_OVERLAPS_AT_REFERENCE;
    return CK_PAIR_CHECKED;
}

int32_t World::status(uint32_t a, uint32_t b) const {
    const Key pair = key(a, b);
    const int32_t declared = declared_status(pair);
    if (declared != CK_PAIR_CHECKED) return declared;
    return supported(objects_.at(pair.first), objects_.at(pair.second)) ? CK_PAIR_CHECKED : CK_PAIR_UNSUPPORTED;
}

bool World::supported(const Object &first, const Object &second) {
    if (first.height_field && second.height_field) return false;
    // A height field is compared through convex cells.
    static const PointConvex cell(std::make_shared<std::vector<coal::Vec3s>>(std::vector<coal::Vec3s>{
        coal::Vec3s(0, 0, 0), coal::Vec3s(1, 0, 0), coal::Vec3s(0, 1, 0), coal::Vec3s(0, 0, 1)}));
    const coal::CollisionGeometry *a = first.height_field ? &cell : first.object->collisionGeometryPtr();
    const coal::CollisionGeometry *b = second.height_field ? &cell : second.object->collisionGeometryPtr();
    try {
        coal::ComputeCollision collide(a, b);
        coal::ComputeDistance distance(a, b);
    } catch (const std::invalid_argument &) {
        return false;
    }
    return true;
}

double World::field_distance(const coal::CollisionObject &field_object, const coal::CollisionObject &other,
                             double query, bool field_first, PairResult *out, double stop_at) {
    const auto &field = static_cast<const coal::HeightField<coal::OBBRSS> &>(*field_object.collisionGeometryPtr());
    const coal::MatrixXs &heights = field.getHeights();
    const coal::VecXs &xs = field.getXGrid(), &ys = field.getYGrid();
    const double floor = field.getMinHeight();
    const coal::Transform3s &placed = field_object.getTransform();
    // The other object's bounds in the field's frame, grown by the query distance.
    const coal::AABB &box = other.getAABB();
    coal::Vec3s low = coal::Vec3s::Constant(INFINITY), high = coal::Vec3s::Constant(-INFINITY);
    bool bounded = box.min_.allFinite() && box.max_.allFinite() && std::isfinite(query);
    if (bounded) {
        for (int corner = 0; corner < 8; ++corner) {
            const coal::Vec3s point(corner & 1 ? box.max_[0] : box.min_[0], corner & 2 ? box.max_[1] : box.min_[1],
                                    corner & 4 ? box.max_[2] : box.min_[2]);
            const coal::Vec3s local = placed.inverseTransform(point);
            low = low.cwiseMin(local);
            high = high.cwiseMax(local);
        }
        const double grow = std::max(query, 0.0);
        low.array() -= grow;
        high.array() += grow;
    }
    double best = INFINITY;
    coal::DistanceRequest request(true, true);
    for (Eigen::Index row = 0; row + 1 < heights.rows(); ++row) {
        // Rows run along -y.
        if (bounded && (ys[row] < low[1] || ys[row + 1] > high[1])) continue;
        for (Eigen::Index col = 0; col + 1 < heights.cols(); ++col) {
            if (bounded && (xs[col + 1] < low[0] || xs[col] > high[0])) continue;
            const double h00 = heights(row, col), h01 = heights(row, col + 1), h10 = heights(row + 1, col),
                         h11 = heights(row + 1, col + 1);
            if (bounded && (std::max({h00, h01, h10, h11}) < low[2] || floor > high[2])) continue;
            const double x0 = xs[col], x1 = xs[col + 1], y0 = ys[row], y1 = ys[row + 1];
            // The cell's two prisms, split as coal splits them for collision.
            const coal::Vec3s halves[2][3] = {
                {coal::Vec3s(x0, y0, h00), coal::Vec3s(x0, y1, h10), coal::Vec3s(x1, y0, h01)},
                {coal::Vec3s(x1, y1, h11), coal::Vec3s(x0, y1, h10), coal::Vec3s(x1, y0, h01)}};
            for (const auto &top : halves) {
                auto corners = std::make_shared<std::vector<coal::Vec3s>>();
                for (const coal::Vec3s &p : top) corners->emplace_back(p[0], p[1], floor);
                for (const coal::Vec3s &p : top) corners->push_back(p);
                PointConvex prism(std::move(corners));
                coal::DistanceResult result;
                const double distance = coal::distance(&prism, placed, other.collisionGeometryPtr(),
                                                       other.getTransform(), request, result);
                if (!(distance < best)) continue;
                best = distance;
                if (out) {
                    const int f = field_first ? 0 : 1, o = 1 - f;
                    const double sign = field_first ? 1.0 : -1.0;
                    for (int k = 0; k < 3; ++k) {
                        (f == 0 ? out->point_a : out->point_b)[k] = result.nearest_points[0][k];
                        (o == 0 ? out->point_a : out->point_b)[k] = result.nearest_points[1][k];
                        out->normal[k] = sign * result.normal[k];
                    }
                }
                if (best <= stop_at) return best;
            }
        }
    }
    return best;
}

void World::pose(Object &object) {
    if (object.body < 0) object.object->setTransform(transform_of(object.offset));
    else
        object.object->setTransform(transform_of(bodies_[size_t(object.body)].pose) * transform_of(object.offset));
    object.object->computeAABB();
}

bool World::make_pair(uint32_t a, uint32_t b, Pair &pair) {
    const Key ordered = key(a, b);
    const Object &first = objects_.at(ordered.first), &second = objects_.at(ordered.second);
    pair.a = ordered.first;
    pair.b = ordered.second;
    pair.body_a = first.body;
    pair.body_b = second.body;
    pair.first = first.object.get();
    pair.second = second.object.get();
    pair.field = first.height_field ? 1 : second.height_field ? 2 : 0;
    if (!supported(first, second)) return false;
    if (pair.field) return true;
    try {
        pair.collide = std::make_unique<coal::ComputeCollision>(pair.first->collisionGeometryPtr(),
                                                                pair.second->collisionGeometryPtr());
        pair.distance = std::make_unique<coal::ComputeDistance>(pair.first->collisionGeometryPtr(),
                                                                pair.second->collisionGeometryPtr());
    } catch (const std::invalid_argument &) {
        return false;
    }
    return true;
}

void World::rebuild_pairs() {
    if (!pairs_dirty_) return;
    pairs_.clear();
    unsupported_ = false;
    for (auto first = objects_.begin(); first != objects_.end(); ++first) {
        for (auto second = std::next(first); second != objects_.end(); ++second) {
            if (declared_status(Key(first->first, second->first)) != CK_PAIR_CHECKED) continue;
            Pair pair;
            if (!make_pair(first->first, second->first, pair)) {
                unsupported_ = true;
                continue;
            }
            pairs_.push_back(std::move(pair));
        }
    }
    pairs_dirty_ = false;
}

bool World::collides(Pair &pair, double margin, coal::CollisionRequest &request) {
    if (pair.field == 1) return field_distance(*pair.first, *pair.second, margin, true, nullptr, margin) <= margin;
    if (pair.field == 2) return field_distance(*pair.second, *pair.first, margin, false, nullptr, margin) <= margin;
    request.security_margin = margin;
    coal::CollisionResult result;
    (*pair.collide)(pair.first->getTransform(), pair.second->getTransform(), request, result);
    return result.isCollision();
}

void World::measure(Pair &pair, double query, PairResult &out) {
    out.a = pair.a;
    out.b = pair.b;
    out.body_a = pair.body_a;
    out.body_b = pair.body_b;
    if (pair.field) {
        out.distance = pair.field == 1 ? field_distance(*pair.first, *pair.second, query, true, &out, -INFINITY)
                                       : field_distance(*pair.second, *pair.first, query, false, &out, -INFINITY);
        return;
    }
    coal::DistanceRequest request(true, true);
    coal::DistanceResult result;
    out.distance = (*pair.distance)(pair.first->getTransform(), pair.second->getTransform(), request, result);
    for (int k = 0; k < 3; ++k) {
        out.point_a[k] = result.nearest_points[0][k];
        out.point_b[k] = result.nearest_points[1][k];
        out.normal[k] = result.normal[k];
    }
}

uint32_t World::allow_overlapping(const std::vector<int32_t> &bodies) {
    const std::set<int32_t> within(bodies.begin(), bodies.end());
    for (auto it = reference_.begin(); it != reference_.end();)
        it = within.count(objects_.at(it->first).body) && within.count(objects_.at(it->second).body)
                 ? reference_.erase(it)
                 : std::next(it);
    pairs_dirty_ = true;
    rebuild_pairs();
    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    std::vector<Key> overlapping;
    for (Pair &pair : pairs_) {
        if (pair.body_a == pair.body_b || !within.count(pair.body_a) || !within.count(pair.body_b)) continue;
        const Key ordered(pair.a, pair.b);
        if (object_rules_.count(ordered) || body_rules_.count(body_key(pair.body_a, pair.body_b))) continue;
        if (!pair.first->getAABB().overlap(pair.second->getAABB())) continue;
        if (collides(pair, 0.0, request)) overlapping.push_back(ordered);
    }
    reference_.insert(overlapping.begin(), overlapping.end());
    pairs_dirty_ = true;
    return uint32_t(overlapping.size());
}

bool World::check(double margin, std::vector<PairResult> &out) {
    out.clear();
    rebuild_pairs();
    if (unsupported_) return false;
    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    for (Pair &pair : pairs_) {
        const coal::AABB &a = pair.first->getAABB(), &b = pair.second->getAABB();
        if (margin > 0.0 ? a.distance(b) > margin : !a.overlap(b)) continue;
        if (!collides(pair, margin, request)) continue;
        PairResult found;
        found.a = pair.a;
        found.b = pair.b;
        found.body_a = pair.body_a;
        found.body_b = pair.body_b;
        found.required = margin;
        out.push_back(found);
    }
    return true;
}

bool World::distances(double query, std::vector<PairResult> &out) {
    out.clear();
    rebuild_pairs();
    if (unsupported_) return false;
    for (Pair &pair : pairs_) {
        // Boxes never report penetration: a negative query still needs every overlapping pair.
        if (pair.first->getAABB().distance(pair.second->getAABB()) > std::max(query, 0.0)) continue;
        PairResult found;
        measure(pair, query, found);
        if (found.distance < query) out.push_back(found);
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const PairResult &x, const PairResult &y) { return x.distance < y.distance; });
    return true;
}

bool World::pair_distance(uint32_t a, uint32_t b, PairResult &out) {
    Pair pair;
    if (!make_pair(a, b, pair)) return false;
    measure(pair, INFINITY, out);
    return true;
}

bool World::violation(const Margins &margins, const double *inflation, bool &found, PairResult &out) {
    found = false;
    rebuild_pairs();
    if (unsupported_) return false;
    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    for (Pair &pair : pairs_) {
        double required = margins.between(group_of(pair.body_a), group_of(pair.body_b));
        if (inflation) {
            if (pair.body_a >= 0) required += inflation[pair.body_a];
            if (pair.body_b >= 0) required += inflation[pair.body_b];
        }
        if (box_gap(*pair.first, *pair.second) > required) continue;
        if (!collides(pair, required, request)) continue;
        measure(pair, required, out);
        out.required = required;
        found = true;
        return true;
    }
    return true;
}

bool World::closest(const Margins &margins, bool &found, PairResult &out) {
    found = false;
    rebuild_pairs();
    if (unsupported_) return false;
    double best = INFINITY;
    for (Pair &pair : pairs_) {
        if (found && box_gap(*pair.first, *pair.second) >= best) continue;
        PairResult candidate;
        measure(pair, best, candidate);
        if (found && !(candidate.distance < best)) continue;
        candidate.required = margins.between(group_of(pair.body_a), group_of(pair.body_b));
        out = candidate;
        best = candidate.distance;
        found = true;
    }
    return true;
}

} // namespace ck
