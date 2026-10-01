#include "collision.hpp"

#include "kinematicskit.h"

#include <coal/hfield.h>

#include <algorithm>
#include <cmath>

namespace kk {

namespace {

coal::Transform3s transform_of(const double *pose) {
    return coal::Transform3s(coal::Quatf(pose[6], pose[3], pose[4], pose[5]),
                             coal::Vec3s(pose[0], pose[1], pose[2]));
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

CollisionWorld::CollisionWorld(const Model &model) : model_(model) {
    rigid_root_.assign(model_.body_count, -1);
    for (int32_t body : model_.body_order) {
        const int32_t joint = model_.body_parent_joint[body];
        rigid_root_[body] = joint >= 0 && model_.joint_kind[joint] == 0 ? rigid_root_[model_.joint_parent[joint]] : body;
    }
    std::vector<double> zeros(model_.dof_count, 0.0);
    model_.evaluate(zeros.data(), nullptr);
}

uint32_t CollisionWorld::add(int32_t body, const double *offset, std::shared_ptr<coal::CollisionGeometry> geometry,
                             bool height_field) {
    const uint32_t id = next_id_++;
    Object &object = objects_[id];
    object.body = body;
    std::copy(offset, offset + 7, object.offset);
    object.height_field = height_field;
    object.object = std::make_unique<coal::CollisionObject>(std::move(geometry));
    pose(object);
    pairs_dirty_ = true;
    return id;
}

bool CollisionWorld::is_height_field(uint32_t id) const {
    auto found = objects_.find(id);
    return found != objects_.end() && found->second.height_field;
}

bool CollisionWorld::set_heights(uint32_t id, const double *values, uint32_t count) {
    Object &object = objects_.at(id);
    auto *field = static_cast<coal::HeightField<coal::OBBRSS> *>(object.object->collisionGeometry().get());
    const auto rows = field->getHeights().rows(), cols = field->getHeights().cols();
    if (int64_t(count) != int64_t(rows) * cols) return false;
    coal::MatrixXs heights(rows, cols);
    for (Eigen::Index r = 0; r < rows; ++r)
        for (Eigen::Index c = 0; c < cols; ++c) heights(r, c) = values[r * cols + c];
    field->updateHeights(heights);
    field->computeLocalAABB();
    object.object->computeAABB();
    return true;
}

void CollisionWorld::attach(uint32_t id, int32_t body, const double *offset) {
    Object &object = objects_.at(id);
    object.body = body;
    std::copy(offset, offset + 7, object.offset);
    pose(object);
    pairs_dirty_ = true;
}

void CollisionWorld::remove(uint32_t id) {
    objects_.erase(id);
    for (auto it = rules_.begin(); it != rules_.end();)
        it = it->first.first == id || it->first.second == id ? rules_.erase(it) : std::next(it);
    for (auto it = reference_.begin(); it != reference_.end();)
        it = it->first == id || it->second == id ? reference_.erase(it) : std::next(it);
    pairs_dirty_ = true;
}

void CollisionWorld::set_rule(uint32_t a, uint32_t b, int32_t rule) {
    if (rule == KK_PAIR_RULE_DEFAULT) rules_.erase(key(a, b));
    else rules_[key(a, b)] = rule;
    pairs_dirty_ = true;
}

bool CollisionWorld::adjacent(int32_t a, int32_t b) const {
    const int32_t ga = rigid_root_[a], gb = rigid_root_[b];
    const int32_t ja = model_.body_parent_joint[ga], jb = model_.body_parent_joint[gb];
    return (ja >= 0 && rigid_root_[model_.joint_parent[ja]] == gb) ||
           (jb >= 0 && rigid_root_[model_.joint_parent[jb]] == ga);
}

int32_t CollisionWorld::declared_status(const Key &pair) const {
    auto rule = rules_.find(pair);
    if (rule != rules_.end()) return rule->second == KK_PAIR_RULE_ALLOW ? KK_PAIR_ALLOWED : KK_PAIR_CHECKED;
    const int32_t a = objects_.at(pair.first).body, b = objects_.at(pair.second).body;
    if (a < 0 && b < 0) return KK_PAIR_STATIC;
    if (a >= 0 && b >= 0) {
        if (rigid_root_[a] == rigid_root_[b]) return KK_PAIR_RIGID;
        if (adjacent(a, b)) return KK_PAIR_ADJACENT;
    }
    if (reference_.count(pair)) return KK_PAIR_OVERLAPS_AT_REFERENCE;
    return KK_PAIR_CHECKED;
}

int32_t CollisionWorld::status(uint32_t a, uint32_t b) const {
    const Key pair = key(a, b);
    const int32_t declared = declared_status(pair);
    if (declared != KK_PAIR_CHECKED) return declared;
    return supported(objects_.at(pair.first), objects_.at(pair.second)) ? KK_PAIR_CHECKED : KK_PAIR_UNSUPPORTED;
}

bool CollisionWorld::supported(const Object &first, const Object &second) {
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

double CollisionWorld::field_distance(const coal::CollisionObject &field_object, const coal::CollisionObject &other,
                                      double query, bool field_first, PairDistance *out, double stop_at) {
    const auto &field = static_cast<const coal::HeightField<coal::OBBRSS> &>(*field_object.collisionGeometryPtr());
    const coal::MatrixXs &heights = field.getHeights();
    const coal::VecXs &xs = field.getXGrid(), &ys = field.getYGrid();
    const double floor = field.getMinHeight();
    const coal::Transform3s &placed = field_object.getTransform();
    // The other object's bounds in the field's frame, grown by the query distance.
    const coal::AABB &box = other.getAABB();
    coal::Vec3s low = coal::Vec3s::Constant(INFINITY), high = coal::Vec3s::Constant(-INFINITY);
    bool bounded = box.min_.allFinite() && box.max_.allFinite();
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
    auto points = std::make_shared<std::vector<coal::Vec3s>>(6);
    PointConvex prism(points);
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
                prism.reset(std::move(corners));
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

void CollisionWorld::pose(Object &object) {
    if (object.body < 0) {
        object.object->setTransform(transform_of(object.offset));
    } else {
        double world[7];
        compose(model_.poses.data() + 7 * size_t(object.body), object.offset, world);
        object.object->setTransform(transform_of(world));
    }
    object.object->computeAABB();
}

void CollisionWorld::update(const double *q, const double *roots) {
    model_.evaluate(q, roots);
    for (auto &entry : objects_) pose(entry.second);
}

void CollisionWorld::rebuild_pairs() {
    if (!pairs_dirty_) return;
    pairs_.clear();
    unsupported_ = false;
    for (auto first = objects_.begin(); first != objects_.end(); ++first) {
        for (auto second = std::next(first); second != objects_.end(); ++second) {
            if (declared_status(Key(first->first, second->first)) != KK_PAIR_CHECKED) continue;
            Pair pair{first->first, second->first, first->second.object.get(), second->second.object.get(), nullptr,
                      nullptr, first->second.height_field ? 1 : second->second.height_field ? 2 : 0};
            if (!supported(first->second, second->second)) {
                unsupported_ = true;
                continue;
            }
            if (pair.field) {
                pairs_.push_back(std::move(pair));
                continue;
            }
            try {
                pair.collide = std::make_unique<coal::ComputeCollision>(pair.first->collisionGeometryPtr(),
                                                                        pair.second->collisionGeometryPtr());
                pair.distance = std::make_unique<coal::ComputeDistance>(pair.first->collisionGeometryPtr(),
                                                                        pair.second->collisionGeometryPtr());
            } catch (const std::invalid_argument &) {
                unsupported_ = true;
                continue;
            }
            pairs_.push_back(std::move(pair));
        }
    }
    pairs_dirty_ = false;
}

bool CollisionWorld::collides(Pair &pair, double margin, const coal::CollisionRequest &request) {
    if (pair.field == 1) return field_distance(*pair.first, *pair.second, margin, true, nullptr, margin) <= margin;
    if (pair.field == 2) return field_distance(*pair.second, *pair.first, margin, false, nullptr, margin) <= margin;
    coal::CollisionResult result;
    (*pair.collide)(pair.first->getTransform(), pair.second->getTransform(), request, result);
    return result.isCollision();
}

uint32_t CollisionWorld::allow_overlapping(const double *q, const double *roots) {
    reference_.clear();
    pairs_dirty_ = true;
    update(q, roots);
    rebuild_pairs();
    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    request.security_margin = 0.0;
    std::vector<Key> overlapping;
    for (Pair &pair : pairs_) {
        if (rules_.count(Key(pair.a, pair.b))) continue;
        if (!pair.first->getAABB().overlap(pair.second->getAABB())) continue;
        if (collides(pair, 0.0, request)) overlapping.emplace_back(pair.a, pair.b);
    }
    reference_.insert(overlapping.begin(), overlapping.end());
    pairs_dirty_ = true;
    return uint32_t(overlapping.size());
}

bool CollisionWorld::check(double margin, std::vector<std::pair<uint32_t, uint32_t>> &out) {
    out.clear();
    rebuild_pairs();
    if (unsupported_) return false;
    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    request.security_margin = margin;
    for (Pair &pair : pairs_) {
        const coal::AABB &a = pair.first->getAABB(), &b = pair.second->getAABB();
        if (margin > 0.0 ? a.distance(b) > margin : !a.overlap(b)) continue;
        if (collides(pair, margin, request)) out.emplace_back(pair.a, pair.b);
    }
    return true;
}

bool CollisionWorld::distances(double query, std::vector<PairDistance> &out) {
    out.clear();
    rebuild_pairs();
    if (unsupported_) return false;
    coal::DistanceRequest request(true, true);
    for (Pair &pair : pairs_) {
        // Boxes never report penetration: a negative query still needs every overlapping pair.
        if (pair.first->getAABB().distance(pair.second->getAABB()) > std::max(query, 0.0)) continue;
        PairDistance found;
        found.a = pair.a;
        found.b = pair.b;
        if (pair.field) {
            found.distance = pair.field == 1 ? field_distance(*pair.first, *pair.second, query, true, &found, -INFINITY)
                                             : field_distance(*pair.second, *pair.first, query, false, &found, -INFINITY);
        } else {
            coal::DistanceResult result;
            found.distance = (*pair.distance)(pair.first->getTransform(), pair.second->getTransform(), request, result);
            for (int k = 0; k < 3; ++k) {
                found.point_a[k] = result.nearest_points[0][k];
                found.point_b[k] = result.nearest_points[1][k];
                found.normal[k] = result.normal[k];
            }
        }
        if (found.distance < query) out.push_back(found);
    }
    std::stable_sort(out.begin(), out.end(),
                     [](const PairDistance &x, const PairDistance &y) { return x.distance < y.distance; });
    return true;
}

} // namespace kk
