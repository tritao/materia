#include "collisionkit.h"
#include "decompose.hpp"
#include "handles.hpp"
#include "world.hpp"

#include <coal/BVH/BVH_model.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include <cmath>
#include <functional>
#include <new>
#include <stdexcept>
#include <vector>

using ck::World;

namespace {

ck::Handles<World> worlds;
ck::Handles<ck::Decomposition> decompositions;

bool finite(const double *values, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i)
        if (!std::isfinite(values[i])) return false;
    return true;
}

/** A finite transform with a usable quaternion, normalized into `out`. */
bool read_pose(const double *pose, double *out) {
    if (!finite(pose, 7)) return false;
    const double norm = std::sqrt(pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] + pose[6] * pose[6]);
    if (!(norm > 1e-12)) return false;
    for (int k = 0; k < 3; ++k) out[k] = pose[k];
    for (int k = 3; k < 7; ++k) out[k] = pose[k] / norm;
    return true;
}

bool read_offset(const double *offset, uint32_t count, double *out) {
    return offset && count == 7 && read_pose(offset, out);
}

/** Runs `body` with the world's handle checked, mapping exceptions to results. */
template <typename F>
ck_result with_world(ck_world_handle handle, F body) {
    World *world = worlds.find(handle.id);
    if (!world) return CK_ERROR_INVALID_HANDLE;
    try {
        return body(*world);
    } catch (const std::bad_alloc &) {
        return CK_ERROR_OUT_OF_MEMORY;
    } catch (const std::invalid_argument &) {
        return CK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return CK_ERROR_INTERNAL;
    }
}

ck_result add_geometry(ck_world_handle handle, int32_t body, const double *offset, uint32_t offset_count,
                       uint32_t *out_object, bool height_field,
                       const std::function<std::shared_ptr<coal::CollisionGeometry>()> &make) {
    if (!out_object) return CK_ERROR_INVALID_ARGUMENT;
    *out_object = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        double pose[7];
        if (!world.has_body(body) || !read_offset(offset, offset_count, pose)) return CK_ERROR_INVALID_ARGUMENT;
        auto geometry = make();
        if (!geometry) return CK_ERROR_INVALID_ARGUMENT;
        *out_object = world.add(body, pose, std::move(geometry), height_field);
        return CK_OK;
    });
}

/** True when `points` (n x 3) span a solid: four of them are not in one plane. */
bool spans_volume(const std::vector<coal::Vec3s> &points) {
    const coal::Vec3s &p0 = points[0];
    size_t i1 = 0;
    double scale = 0.0;
    for (size_t i = 1; i < points.size(); ++i)
        if ((points[i] - p0).norm() > scale) {
            scale = (points[i] - p0).norm();
            i1 = i;
        }
    if (!(scale > 0.0)) return false;
    const coal::Vec3s axis = (points[i1] - p0) / scale;
    size_t i2 = 0;
    double off_line = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        const double d = (points[i] - p0).cross(axis).norm();
        if (d > off_line) {
            off_line = d;
            i2 = i;
        }
    }
    if (!(off_line > 1e-9 * scale)) return false;
    const coal::Vec3s normal = axis.cross(points[i2] - p0).normalized();
    for (const coal::Vec3s &p : points)
        if (std::abs((p - p0).dot(normal)) > 1e-9 * scale) return true;
    return false;
}

bool read_heights(const double *heights, uint32_t count, uint32_t rows, uint32_t cols, double min_height,
                  coal::MatrixXs &out) {
    if (!heights || rows < 2 || cols < 2 || uint64_t(rows) * cols != count || !finite(heights, count)) return false;
    out.resize(rows, cols);
    for (uint32_t r = 0; r < rows; ++r)
        for (uint32_t c = 0; c < cols; ++c) {
            const double h = heights[size_t(r) * cols + c];
            if (!(h >= min_height)) return false;
            out(r, c) = h;
        }
    return true;
}

/** A margin table over every body's group: square, finite and not negative. */
bool read_margins(const World &world, const double *margins, uint32_t count, ck::Margins &out) {
    if (!margins || count == 0 || !finite(margins, count)) return false;
    const uint32_t groups = uint32_t(std::lround(std::sqrt(double(count))));
    if (groups * groups != count || int64_t(world.max_group()) >= int64_t(groups)) return false;
    for (uint32_t i = 0; i < count; ++i)
        if (margins[i] < 0.0) return false;
    out.groups = groups;
    out.table = margins;
    return true;
}

bool read_inflation(const double *inflation, uint32_t count, uint32_t bodies) {
    if (count == 0) return true;
    if (!inflation || count != bodies || !finite(inflation, count)) return false;
    for (uint32_t i = 0; i < count; ++i)
        if (inflation[i] < 0.0) return false;
    return true;
}

bool reason_ok(int32_t rule, int32_t reason) {
    if (rule != CK_RULE_ALLOW) return true;
    return reason == CK_PAIR_RIGID || reason == CK_PAIR_ADJACENT || reason == CK_PAIR_CLOSURE ||
           reason == CK_PAIR_ALLOWED || reason == CK_PAIR_PROCESS_CONTACT || reason == CK_PAIR_DECLARED_CONTACT;
}

void write_pair(const ck::PairResult &pair, int32_t *out) {
    out[0] = int32_t(pair.a);
    out[1] = int32_t(pair.b);
    out[2] = pair.body_a;
    out[3] = pair.body_b;
}

void write_row(const ck::PairResult &pair, double *row) {
    row[0] = pair.distance;
    for (int k = 0; k < 3; ++k) {
        row[1 + k] = pair.point_a[k];
        row[4 + k] = pair.point_b[k];
        row[7 + k] = pair.normal[k];
    }
}

} // namespace

CK_API uint32_t CK_CALL ck_api_version(void) { return CK_API_VERSION; }

CK_API ck_result CK_CALL ck_world_create(ck_world_handle *out_world) {
    if (!out_world) return CK_ERROR_INVALID_ARGUMENT;
    out_world->id = 0;
    try {
        const uint32_t handle = worlds.store(std::make_unique<World>());
        if (!handle) return CK_ERROR_OUT_OF_MEMORY;
        out_world->id = handle;
        return CK_OK;
    } catch (const std::bad_alloc &) {
        return CK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return CK_ERROR_INTERNAL;
    }
}

CK_API void CK_CALL ck_world_destroy(ck_world_handle world) { worlds.release(world.id); }

CK_API ck_result CK_CALL ck_add_body(ck_world_handle handle, int32_t group, uint32_t *out_body) {
    if (!out_body) return CK_ERROR_INVALID_ARGUMENT;
    *out_body = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        if (group < 0) return CK_ERROR_INVALID_ARGUMENT;
        *out_body = world.add_body(group);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_set_body_group(ck_world_handle handle, uint32_t body, int32_t group) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (body >= world.body_count() || group < 0) return CK_ERROR_INVALID_ARGUMENT;
        world.set_body_group(body, group);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_set_body_static(ck_world_handle handle, uint32_t body, int32_t fixed) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (body >= world.body_count()) return CK_ERROR_INVALID_ARGUMENT;
        world.set_body_static(body, fixed != 0);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_set_body_poses(ck_world_handle handle, uint32_t first, const double *poses,
                                           uint32_t pose_count) {
    return with_world(handle, [&](World &world) -> ck_result {
        if ((pose_count && !poses) || pose_count % 7 != 0 || uint64_t(first) + pose_count / 7 > world.body_count())
            return CK_ERROR_INVALID_ARGUMENT;
        std::vector<double> normalized(pose_count);
        for (uint32_t i = 0; i < pose_count; i += 7)
            if (!read_pose(poses + i, normalized.data() + i)) return CK_ERROR_INVALID_ARGUMENT;
        for (uint32_t i = 0; i < pose_count / 7; ++i) world.set_body_pose(first + i, normalized.data() + 7 * i);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_add_shape(ck_world_handle world, int32_t body, const double *offset,
                                      uint32_t offset_count, int32_t kind, const double *params,
                                      uint32_t param_count, uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, false,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        if ((param_count && !params) || !finite(params, param_count)) return nullptr;
        const double *p = params;
        switch (kind) {
        case CK_SHAPE_BOX:
            if (param_count != 3 || !(p[0] > 0.0 && p[1] > 0.0 && p[2] > 0.0)) return nullptr;
            return std::make_shared<coal::Box>(2.0 * p[0], 2.0 * p[1], 2.0 * p[2]);
        case CK_SHAPE_SPHERE:
            if (param_count != 1 || !(p[0] > 0.0)) return nullptr;
            return std::make_shared<coal::Sphere>(p[0]);
        case CK_SHAPE_CAPSULE:
            if (param_count != 2 || !(p[0] > 0.0 && p[1] >= 0.0)) return nullptr;
            return std::make_shared<coal::Capsule>(p[0], 2.0 * p[1]);
        case CK_SHAPE_CYLINDER:
            if (param_count != 2 || !(p[0] > 0.0 && p[1] > 0.0)) return nullptr;
            return std::make_shared<coal::Cylinder>(p[0], 2.0 * p[1]);
        case CK_SHAPE_HALFSPACE: {
            if (param_count != 4) return nullptr;
            const double norm = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (!(norm > 1e-12)) return nullptr;
            return std::make_shared<coal::Halfspace>(p[0] / norm, p[1] / norm, p[2] / norm, p[3] / norm);
        }
        default:
            return nullptr;
        }
    });
}

CK_API ck_result CK_CALL ck_add_convex(ck_world_handle world, int32_t body, const double *offset,
                                       uint32_t offset_count, const double *points, uint32_t point_count,
                                       uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, false,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        if (!points || point_count < 12 || point_count % 3 != 0 || !finite(points, point_count)) return nullptr;
        auto vertices = std::make_shared<std::vector<coal::Vec3s>>();
        for (uint32_t i = 0; i < point_count; i += 3) vertices->emplace_back(points[i], points[i + 1], points[i + 2]);
        if (!spans_volume(*vertices)) return nullptr;
        return std::make_shared<ck::PointConvex>(std::move(vertices));
    });
}

CK_API ck_result CK_CALL ck_add_mesh(ck_world_handle world, int32_t body, const double *offset,
                                     uint32_t offset_count, const double *vertices, uint32_t vertex_count,
                                     const int32_t *indices, uint32_t index_count, uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, false,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        if (!vertices || !indices || vertex_count < 9 || vertex_count % 3 != 0 || index_count < 3 ||
            index_count % 3 != 0 || !finite(vertices, vertex_count))
            return nullptr;
        const uint32_t count = vertex_count / 3;
        std::vector<coal::Vec3s> points;
        for (uint32_t i = 0; i < vertex_count; i += 3) points.emplace_back(vertices[i], vertices[i + 1], vertices[i + 2]);
        std::vector<coal::Triangle> triangles;
        for (uint32_t i = 0; i < index_count; i += 3) {
            for (uint32_t k = 0; k < 3; ++k)
                if (indices[i + k] < 0 || uint32_t(indices[i + k]) >= count) return nullptr;
            triangles.emplace_back(indices[i], indices[i + 1], indices[i + 2]);
        }
        auto mesh = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
        if (mesh->beginModel() != coal::BVH_OK || mesh->addSubModel(points, triangles) != coal::BVH_OK ||
            mesh->endModel() != coal::BVH_OK)
            return nullptr;
        return mesh;
    });
}

CK_API ck_result CK_CALL ck_add_height_field(ck_world_handle world, int32_t body, const double *offset,
                                             uint32_t offset_count, double x_size, double y_size,
                                             const double *heights, uint32_t height_count, uint32_t rows,
                                             double min_height, uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, true,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        coal::MatrixXs grid;
        if (!(x_size > 0.0) || !(y_size > 0.0) || !std::isfinite(x_size) || !std::isfinite(y_size) ||
            !std::isfinite(min_height) || rows == 0 || height_count % rows != 0 ||
            !read_heights(heights, height_count, rows, height_count / rows, min_height, grid))
            return nullptr;
        return std::make_shared<coal::HeightField<coal::OBBRSS>>(x_size, y_size, grid, min_height);
    });
}

CK_API ck_result CK_CALL ck_set_heights(ck_world_handle handle, uint32_t object, const double *heights,
                                        uint32_t height_count) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (!world.is_height_field(object) || !heights || !finite(heights, height_count))
            return CK_ERROR_INVALID_ARGUMENT;
        return world.set_heights(object, heights, height_count) ? CK_OK : CK_ERROR_INVALID_ARGUMENT;
    });
}

CK_API ck_result CK_CALL ck_set_inflation(ck_world_handle handle, uint32_t object, double radius) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (!world.contains(object) || !std::isfinite(radius) || radius < 0.0) return CK_ERROR_INVALID_ARGUMENT;
        if (!world.inflatable(object)) return CK_ERROR_UNSUPPORTED;
        world.set_inflation(object, radius);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_attach(ck_world_handle handle, uint32_t object, int32_t body, const double *offset,
                                   uint32_t offset_count) {
    return with_world(handle, [&](World &world) -> ck_result {
        double pose[7];
        if (!world.contains(object) || !world.has_body(body) || !read_offset(offset, offset_count, pose))
            return CK_ERROR_INVALID_ARGUMENT;
        world.attach(object, body, pose);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_remove(ck_world_handle handle, uint32_t object) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (!world.contains(object)) return CK_ERROR_INVALID_ARGUMENT;
        world.remove(object);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_set_body_rule(ck_world_handle handle, int32_t a, int32_t b, int32_t rule,
                                          int32_t reason) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (a == b || !world.has_body(a) || !world.has_body(b) || rule < CK_RULE_DEFAULT || rule > CK_RULE_CHECK ||
            !reason_ok(rule, reason))
            return CK_ERROR_INVALID_ARGUMENT;
        world.set_body_rule(a, b, rule, reason);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_set_object_rule(ck_world_handle handle, uint32_t a, uint32_t b, int32_t rule,
                                            int32_t reason) {
    return with_world(handle, [&](World &world) -> ck_result {
        if (a == b || !world.contains(a) || !world.contains(b) || rule < CK_RULE_DEFAULT || rule > CK_RULE_CHECK ||
            !reason_ok(rule, reason))
            return CK_ERROR_INVALID_ARGUMENT;
        world.set_object_rule(a, b, rule, reason);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_pair_status(ck_world_handle handle, uint32_t a, uint32_t b, int32_t *out_status) {
    if (!out_status) return CK_ERROR_INVALID_ARGUMENT;
    return with_world(handle, [&](World &world) -> ck_result {
        if (a == b || !world.contains(a) || !world.contains(b)) return CK_ERROR_INVALID_ARGUMENT;
        *out_status = world.status(a, b);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_allow_overlapping(ck_world_handle handle, const int32_t *bodies, uint32_t body_count,
                                              uint32_t *out_count) {
    if (!out_count || (body_count && !bodies)) return CK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        std::vector<int32_t> set(bodies, bodies + body_count);
        for (int32_t body : set)
            if (body < 0 || !world.has_body(body)) return CK_ERROR_INVALID_ARGUMENT;
        *out_count = world.allow_overlapping(set);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_check(ck_world_handle handle, double margin, int32_t *out_pairs, uint32_t pair_capacity,
                                  uint32_t *out_count) {
    if (!out_count || (pair_capacity && !out_pairs)) return CK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        if (!std::isfinite(margin) || margin < 0.0) return CK_ERROR_INVALID_ARGUMENT;
        std::vector<ck::PairResult> pairs;
        if (!world.check(margin, pairs)) return CK_ERROR_UNSUPPORTED;
        *out_count = uint32_t(pairs.size());
        for (size_t i = 0; i < pairs.size() && 4 * i + 3 < pair_capacity; ++i) write_pair(pairs[i], out_pairs + 4 * i);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_distances(ck_world_handle handle, double query_distance, int32_t *out_pairs,
                                      uint32_t pair_capacity, double *out_results, uint32_t result_capacity,
                                      uint32_t *out_count) {
    if (!out_count || (pair_capacity && !out_pairs) || (result_capacity && !out_results))
        return CK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        if (!std::isfinite(query_distance)) return CK_ERROR_INVALID_ARGUMENT;
        std::vector<ck::PairResult> found;
        if (!world.distances(query_distance, found)) return CK_ERROR_UNSUPPORTED;
        *out_count = uint32_t(found.size());
        const size_t written = std::min({found.size(), size_t(pair_capacity / 4), size_t(result_capacity / 10)});
        for (size_t i = 0; i < written; ++i) {
            write_pair(found[i], out_pairs + 4 * i);
            write_row(found[i], out_results + 10 * i);
        }
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_pair_distances(ck_world_handle handle, const int32_t *pairs, uint32_t pair_count,
                                           double *out_results, uint32_t result_capacity) {
    if ((pair_count && !pairs) || pair_count % 2 != 0 || (result_capacity && !out_results) ||
        result_capacity < 5 * pair_count)
        return CK_ERROR_INVALID_ARGUMENT;
    return with_world(handle, [&](World &world) -> ck_result {
        for (uint32_t i = 0; i < pair_count; ++i)
            if (pairs[i] <= 0 || !world.contains(uint32_t(pairs[i]))) return CK_ERROR_INVALID_ARGUMENT;
        for (uint32_t i = 0; i < pair_count; i += 2)
            if (pairs[i] == pairs[i + 1]) return CK_ERROR_INVALID_ARGUMENT;
        for (uint32_t i = 0; i < pair_count / 2; ++i) {
            ck::PairResult result;
            if (!world.pair_distance(uint32_t(pairs[2 * i]), uint32_t(pairs[2 * i + 1]), result))
                return CK_ERROR_UNSUPPORTED;
            // Rows follow the order the pair was given in.
            if (uint32_t(pairs[2 * i]) != result.a) {
                for (int k = 0; k < 3; ++k) {
                    std::swap(result.point_a[k], result.point_b[k]);
                    result.normal[k] = -result.normal[k];
                }
            }
            write_row(result, out_results + 10 * i);
        }
        return CK_OK;
    });
}

namespace {

void report(bool found, const ck::PairResult &pair, int32_t *out_pair, double *out_result) {
    if (!found) return;
    write_pair(pair, out_pair);
    out_result[0] = pair.distance;
    out_result[1] = pair.required;
}

} // namespace

CK_API ck_result CK_CALL ck_violation(ck_world_handle handle, const double *margins, uint32_t margin_count,
                                      const double *inflation, uint32_t inflation_count, int32_t *out_pair,
                                      uint32_t pair_capacity, double *out_result, uint32_t result_capacity,
                                      int32_t *out_found) {
    if (!out_found || !out_pair || pair_capacity < 4 || !out_result || result_capacity < 2)
        return CK_ERROR_INVALID_ARGUMENT;
    *out_found = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        ck::Margins table;
        if (!read_margins(world, margins, margin_count, table) ||
            !read_inflation(inflation, inflation_count, world.body_count()))
            return CK_ERROR_INVALID_ARGUMENT;
        bool found = false;
        ck::PairResult pair;
        if (!world.violation(table, inflation_count ? inflation : nullptr, found, pair)) return CK_ERROR_UNSUPPORTED;
        *out_found = found ? 1 : 0;
        report(found, pair, out_pair, out_result);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_closest(ck_world_handle handle, const double *margins, uint32_t margin_count,
                                    int32_t *out_pair, uint32_t pair_capacity, double *out_result,
                                    uint32_t result_capacity, int32_t *out_found) {
    if (!out_found || !out_pair || pair_capacity < 4 || !out_result || result_capacity < 2)
        return CK_ERROR_INVALID_ARGUMENT;
    *out_found = 0;
    return with_world(handle, [&](World &world) -> ck_result {
        ck::Margins table;
        if (!read_margins(world, margins, margin_count, table)) return CK_ERROR_INVALID_ARGUMENT;
        bool found = false;
        ck::PairResult pair;
        if (!world.closest(table, found, pair)) return CK_ERROR_UNSUPPORTED;
        *out_found = found ? 1 : 0;
        report(found, pair, out_pair, out_result);
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_violation_batch(ck_world_handle handle, const double *poses, uint32_t pose_count,
                                            const double *inflation, uint32_t inflation_count,
                                            const double *margins, uint32_t margin_count, int32_t *out_pair,
                                            uint32_t pair_capacity, double *out_result, uint32_t result_capacity,
                                            int32_t *out_set) {
    if (!out_set || !out_pair || pair_capacity < 4 || !out_result || result_capacity < 2)
        return CK_ERROR_INVALID_ARGUMENT;
    *out_set = -1;
    return with_world(handle, [&](World &world) -> ck_result {
        const uint32_t bodies = world.body_count();
        ck::Margins table;
        if (!read_margins(world, margins, margin_count, table) || bodies == 0 || (pose_count && !poses) ||
            pose_count % (7 * bodies) != 0)
            return CK_ERROR_INVALID_ARGUMENT;
        const uint32_t sets = pose_count / (7 * bodies);
        if (inflation_count && (!inflation || inflation_count != sets * bodies)) return CK_ERROR_INVALID_ARGUMENT;
        for (uint32_t s = 0; s < sets; ++s)
            if (inflation_count && !read_inflation(inflation + size_t(s) * bodies, bodies, bodies))
                return CK_ERROR_INVALID_ARGUMENT;
        double pose[7];
        for (uint32_t i = 0; i < pose_count; i += 7)
            if (!read_pose(poses + i, pose)) return CK_ERROR_INVALID_ARGUMENT;
        for (uint32_t s = 0; s < sets; ++s) {
            for (uint32_t body = 0; body < bodies; ++body) {
                read_pose(poses + 7 * (size_t(s) * bodies + body), pose);
                world.set_body_pose(body, pose);
            }
            bool found = false;
            ck::PairResult pair;
            if (!world.violation(table, inflation_count ? inflation + size_t(s) * bodies : nullptr, found, pair))
                return CK_ERROR_UNSUPPORTED;
            if (found) {
                *out_set = int32_t(s);
                report(found, pair, out_pair, out_result);
                return CK_OK;
            }
        }
        return CK_OK;
    });
}

CK_API ck_result CK_CALL ck_decompose(const double *vertices, uint32_t vertex_count, const int32_t *indices,
                                      uint32_t index_count, uint32_t max_pieces, uint32_t resolution,
                                      uint32_t max_piece_vertices, double sample_spacing,
                                      ck_decomposition_handle *out_decomposition) {
    if (!out_decomposition) return CK_ERROR_INVALID_ARGUMENT;
    out_decomposition->id = 0;
    if (!vertices || !indices || vertex_count < 12 || vertex_count % 3 != 0 || index_count < 12 ||
        index_count % 3 != 0 || !finite(vertices, vertex_count) || max_pieces == 0 || resolution < 1000 ||
        max_piece_vertices < 4 || max_piece_vertices > 64 || !std::isfinite(sample_spacing) || sample_spacing < 0.0)
        return CK_ERROR_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < index_count; ++i)
        if (indices[i] < 0 || uint32_t(indices[i]) >= vertex_count / 3) return CK_ERROR_INVALID_ARGUMENT;
    try {
        auto result = std::make_unique<ck::Decomposition>();
        ck::DecomposeOptions options;
        options.max_pieces = max_pieces;
        options.resolution = resolution;
        options.max_piece_vertices = max_piece_vertices;
        options.sample_spacing = sample_spacing;
        if (!ck::decompose(vertices, vertex_count, indices, index_count, options, *result)) return CK_ERROR_INTERNAL;
        const uint32_t handle = decompositions.store(std::move(result));
        if (!handle) return CK_ERROR_OUT_OF_MEMORY;
        out_decomposition->id = handle;
        return CK_OK;
    } catch (const std::bad_alloc &) {
        return CK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return CK_ERROR_INTERNAL;
    }
}

CK_API void CK_CALL ck_decomposition_destroy(ck_decomposition_handle decomposition) {
    decompositions.release(decomposition.id);
}

CK_API ck_result CK_CALL ck_decomposition_info(ck_decomposition_handle handle, uint32_t *out_pieces,
                                               double *out_values, uint32_t value_capacity) {
    if (!out_pieces || !out_values || value_capacity < 3) return CK_ERROR_INVALID_ARGUMENT;
    const ck::Decomposition *decomposition = decompositions.find(handle.id);
    if (!decomposition) return CK_ERROR_INVALID_HANDLE;
    *out_pieces = uint32_t(decomposition->pieces.size());
    out_values[0] = decomposition->measured;
    out_values[1] = decomposition->spacing;
    out_values[2] = decomposition->inflation;
    return CK_OK;
}

CK_API ck_result CK_CALL ck_decomposition_piece(ck_decomposition_handle handle, uint32_t piece, double *out_points,
                                                uint32_t point_capacity, uint32_t *out_count) {
    if (!out_count || (point_capacity && !out_points)) return CK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    const ck::Decomposition *decomposition = decompositions.find(handle.id);
    if (!decomposition) return CK_ERROR_INVALID_HANDLE;
    if (piece >= decomposition->pieces.size()) return CK_ERROR_INVALID_ARGUMENT;
    const std::vector<double> &points = decomposition->pieces[piece];
    *out_count = uint32_t(points.size());
    std::copy(points.begin(), points.begin() + std::min<size_t>(points.size(), point_capacity), out_points);
    return CK_OK;
}
