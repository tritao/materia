#include "kinematicskit.h"
#include "collision.hpp"
#include "handles.hpp"
#include "model.hpp"
#include "qp.hpp"

#include <coal/BVH/BVH_model.h>
#include <coal/hfield.h>
#include <coal/shape/geometric_shapes.h>

#include <cmath>
#include <functional>
#include <new>
#include <stdexcept>
#include <vector>

using kk::Model;

namespace {
kk::Handles<Model> models;
kk::Handles<kk::QpStep> steps;
kk::Handles<kk::CollisionWorld> worlds;
}

KK_API uint32_t KK_CALL kk_api_version(void) { return KK_API_VERSION; }

KK_API kk_result KK_CALL kk_model_create(const int32_t *ints, uint32_t int_count, const double *reals,
                                         uint32_t real_count, kk_model_handle *out_model) {
    if (!out_model) return KK_ERROR_INVALID_ARGUMENT;
    out_model->id = 0;
    try {
        auto model = std::make_unique<Model>();
        if (!model->build(ints, int_count, reals, real_count)) return KK_ERROR_INVALID_ARGUMENT;
        const uint32_t handle = models.store(std::move(model));
        if (!handle) return KK_ERROR_OUT_OF_MEMORY;
        out_model->id = handle;
        return KK_OK;
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return KK_ERROR_INTERNAL;
    }
}

KK_API void KK_CALL kk_model_destroy(kk_model_handle model) { models.release(model.id); }

namespace {

bool finite(const double *values, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i)
        if (!std::isfinite(values[i])) return false;
    return true;
}

} // namespace

KK_API kk_result KK_CALL kk_forward(kk_model_handle handle, const double *q, uint32_t dof_count,
                                    const double *root_poses, uint32_t root_count, double *out_poses,
                                    uint32_t pose_count) {
    Model *model = models.find(handle.id);
    if (!model) return KK_ERROR_INVALID_HANDLE;
    if ((dof_count && !q) || dof_count != model->dof_count || !finite(q, dof_count) || !out_poses ||
        pose_count != 7 * model->body_count ||
        (root_count && (root_count != 7 * model->body_count || !root_poses || !finite(root_poses, root_count))))
        return KK_ERROR_INVALID_ARGUMENT;
    model->evaluate(q, root_count ? root_poses : nullptr);
    for (uint32_t i = 0; i < pose_count; ++i) out_poses[i] = model->poses[i];
    return KK_OK;
}

KK_API kk_result KK_CALL kk_point_jacobian(kk_model_handle handle, const double *q, uint32_t dof_count,
                                           uint32_t body, const double *point, uint32_t point_count,
                                           const int32_t *columns, uint32_t column_count,
                                           double *out_jacobian, uint32_t jacobian_count) {
    Model *model = models.find(handle.id);
    if (!model) return KK_ERROR_INVALID_HANDLE;
    if ((dof_count && !q) || dof_count != model->dof_count || !finite(q, dof_count) || body >= model->body_count ||
        !point || point_count != 3 || !finite(point, 3) || (column_count && !columns) ||
        !out_jacobian || jacobian_count != 6 * column_count)
        return KK_ERROR_INVALID_ARGUMENT;
    try {
        std::vector<int32_t> column_of_dof(model->dof_count, -1);
        for (uint32_t c = 0; c < column_count; ++c) {
            const int32_t dof = columns[c];
            if (dof < 0 || uint32_t(dof) >= model->dof_count || column_of_dof[dof] >= 0) return KK_ERROR_INVALID_ARGUMENT;
            column_of_dof[dof] = int32_t(c);
        }
        model->evaluate(q, nullptr);
        model->point_jacobian(body, point[0], point[1], point[2], column_of_dof.data(), column_count, out_jacobian);
        return KK_OK;
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    }
}

KK_API kk_result KK_CALL kk_qp_create(uint32_t width, kk_qp_handle *out_qp) {
    if (!out_qp) return KK_ERROR_INVALID_ARGUMENT;
    out_qp->id = 0;
    if (width == 0 || width > 4096) return KK_ERROR_INVALID_ARGUMENT;
    try {
        const uint32_t handle = steps.store(std::make_unique<kk::QpStep>(width));
        if (!handle) return KK_ERROR_OUT_OF_MEMORY;
        out_qp->id = handle;
        return KK_OK;
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return KK_ERROR_SOLVER;
    }
}

KK_API void KK_CALL kk_qp_destroy(kk_qp_handle qp) { steps.release(qp.id); }

KK_API kk_result KK_CALL kk_qp_solve(kk_qp_handle handle, const double *jacobian, uint32_t jacobian_count,
                                     const double *residual, uint32_t row_count, const double *lower,
                                     uint32_t lower_count, const double *upper, uint32_t upper_count, double damping,
                                     double tolerance, uint32_t max_iterations, double *out_step,
                                     uint32_t step_count, int32_t *out_status, uint32_t *out_iterations) {
    kk::QpStep *step = steps.find(handle.id);
    if (!step) return KK_ERROR_INVALID_HANDLE;
    const uint32_t width = step->width();
    if (!out_step || !out_status || !out_iterations || step_count != width || lower_count != width ||
        upper_count != width || !lower || !upper || (row_count && (!residual || !jacobian)) ||
        uint64_t(jacobian_count) != uint64_t(row_count) * width || !finite(jacobian, jacobian_count) ||
        !finite(residual, row_count) || !std::isfinite(damping) || damping < 0.0 || !std::isfinite(tolerance) ||
        tolerance <= 0.0 || max_iterations == 0)
        return KK_ERROR_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < width; ++i)
        if (std::isnan(lower[i]) || std::isnan(upper[i]) || lower[i] == INFINITY || upper[i] == -INFINITY)
            return KK_ERROR_INVALID_ARGUMENT;
    try {
        const auto outcome = step->solve(jacobian, row_count, residual, lower, upper, damping, tolerance,
                                         max_iterations, out_step);
        *out_status = outcome.status;
        *out_iterations = outcome.iterations;
        return KK_OK;
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return KK_ERROR_SOLVER;
    }
}

namespace {

using kk::CollisionWorld;

/** A finite offset transform with a usable quaternion, normalized into `out`. */
bool read_offset(const double *offset, uint32_t count, double *out) {
    if (!offset || count != 7 || !finite(offset, 7)) return false;
    const double norm = std::sqrt(offset[3] * offset[3] + offset[4] * offset[4] + offset[5] * offset[5] +
                                  offset[6] * offset[6]);
    if (!(norm > 1e-12)) return false;
    for (int k = 0; k < 3; ++k) out[k] = offset[k];
    for (int k = 3; k < 7; ++k) out[k] = offset[k] / norm;
    return true;
}

bool body_ok(const CollisionWorld &world, int32_t body) { return body >= -1 && body < int32_t(world.body_count()); }

/** Runs `body` with the world's handle checked, mapping exceptions to results. */
template <typename F>
kk_result with_world(kk_collision_world_handle handle, F body) {
    CollisionWorld *world = worlds.find(handle.id);
    if (!world) return KK_ERROR_INVALID_HANDLE;
    try {
        return body(*world);
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    } catch (const std::invalid_argument &) {
        return KK_ERROR_INVALID_ARGUMENT;
    } catch (...) {
        return KK_ERROR_INTERNAL;
    }
}

kk_result add_geometry(kk_collision_world_handle handle, int32_t body, const double *offset, uint32_t offset_count,
                       uint32_t *out_object, bool height_field,
                       const std::function<std::shared_ptr<coal::CollisionGeometry>()> &make) {
    if (!out_object) return KK_ERROR_INVALID_ARGUMENT;
    *out_object = 0;
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        double pose[7];
        if (!body_ok(world, body) || !read_offset(offset, offset_count, pose)) return KK_ERROR_INVALID_ARGUMENT;
        auto geometry = make();
        if (!geometry) return KK_ERROR_INVALID_ARGUMENT;
        *out_object = world.add(body, pose, std::move(geometry), height_field);
        return KK_OK;
    });
}

/** True when `points` (n x 3) span a solid: four of them are not in one plane. */
bool spans_volume(const std::vector<coal::Vec3s> &points) {
    const coal::Vec3s &p0 = points[0];
    size_t i1 = 0;
    double scale = 0.0;
    for (size_t i = 1; i < points.size(); ++i)
        if ((points[i] - p0).norm() > scale) { scale = (points[i] - p0).norm(); i1 = i; }
    if (!(scale > 0.0)) return false;
    const coal::Vec3s axis = (points[i1] - p0) / scale;
    size_t i2 = 0;
    double off_line = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        const double d = (points[i] - p0).cross(axis).norm();
        if (d > off_line) { off_line = d; i2 = i; }
    }
    if (!(off_line > 1e-9 * scale)) return false;
    const coal::Vec3s normal = axis.cross(points[i2] - p0).normalized();
    for (const coal::Vec3s &p : points)
        if (std::abs((p - p0).dot(normal)) > 1e-9 * scale) return true;
    return false;
}

bool read_heights(const double *heights, uint32_t count, uint32_t rows, uint32_t cols, coal::MatrixXs &out) {
    if (!heights || rows < 2 || cols < 2 || uint64_t(rows) * cols != count || !finite(heights, count)) return false;
    out.resize(rows, cols);
    for (uint32_t r = 0; r < rows; ++r)
        for (uint32_t c = 0; c < cols; ++c) out(r, c) = heights[size_t(r) * cols + c];
    return true;
}

} // namespace

KK_API kk_result KK_CALL kk_collision_world_create(kk_model_handle model_handle, kk_collision_world_handle *out_world) {
    if (!out_world) return KK_ERROR_INVALID_ARGUMENT;
    out_world->id = 0;
    Model *model = models.find(model_handle.id);
    if (!model) return KK_ERROR_INVALID_HANDLE;
    try {
        const uint32_t handle = worlds.store(std::make_unique<CollisionWorld>(*model));
        if (!handle) return KK_ERROR_OUT_OF_MEMORY;
        out_world->id = handle;
        return KK_OK;
    } catch (const std::bad_alloc &) {
        return KK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return KK_ERROR_INTERNAL;
    }
}

KK_API void KK_CALL kk_collision_world_destroy(kk_collision_world_handle world) { worlds.release(world.id); }

KK_API kk_result KK_CALL kk_collision_add_shape(kk_collision_world_handle world, int32_t body, const double *offset,
                                                uint32_t offset_count, int32_t kind, const double *params,
                                                uint32_t param_count, uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, false,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        if ((param_count && !params) || !finite(params, param_count)) return nullptr;
        const double *p = params;
        switch (kind) {
        case KK_SHAPE_BOX:
            if (param_count != 3 || !(p[0] > 0.0 && p[1] > 0.0 && p[2] > 0.0)) return nullptr;
            return std::make_shared<coal::Box>(2.0 * p[0], 2.0 * p[1], 2.0 * p[2]);
        case KK_SHAPE_SPHERE:
            if (param_count != 1 || !(p[0] > 0.0)) return nullptr;
            return std::make_shared<coal::Sphere>(p[0]);
        case KK_SHAPE_CAPSULE:
            if (param_count != 2 || !(p[0] > 0.0 && p[1] >= 0.0)) return nullptr;
            return std::make_shared<coal::Capsule>(p[0], 2.0 * p[1]);
        case KK_SHAPE_CYLINDER:
            if (param_count != 2 || !(p[0] > 0.0 && p[1] > 0.0)) return nullptr;
            return std::make_shared<coal::Cylinder>(p[0], 2.0 * p[1]);
        case KK_SHAPE_HALFSPACE: {
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

KK_API kk_result KK_CALL kk_collision_add_convex(kk_collision_world_handle world, int32_t body, const double *offset,
                                                 uint32_t offset_count, const double *points, uint32_t point_count,
                                                 uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, false,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        if (!points || point_count < 12 || point_count % 3 != 0 || !finite(points, point_count)) return nullptr;
        auto vertices = std::make_shared<std::vector<coal::Vec3s>>();
        for (uint32_t i = 0; i < point_count; i += 3) vertices->emplace_back(points[i], points[i + 1], points[i + 2]);
        if (!spans_volume(*vertices)) return nullptr;
        return std::make_shared<kk::PointConvex>(std::move(vertices));
    });
}

KK_API kk_result KK_CALL kk_collision_add_mesh(kk_collision_world_handle world, int32_t body, const double *offset,
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

KK_API kk_result KK_CALL kk_collision_add_height_field(kk_collision_world_handle world, int32_t body,
                                                       const double *offset, uint32_t offset_count, double x_size,
                                                       double y_size, const double *heights, uint32_t height_count,
                                                       uint32_t rows, double min_height, uint32_t *out_object) {
    return add_geometry(world, body, offset, offset_count, out_object, true,
                        [&]() -> std::shared_ptr<coal::CollisionGeometry> {
        coal::MatrixXs grid;
        if (!(x_size > 0.0) || !(y_size > 0.0) || !std::isfinite(x_size) || !std::isfinite(y_size) ||
            !std::isfinite(min_height) || rows == 0 || height_count % rows != 0 ||
            !read_heights(heights, height_count, rows, height_count / rows, grid))
            return nullptr;
        return std::make_shared<coal::HeightField<coal::OBBRSS>>(x_size, y_size, grid, min_height);
    });
}

KK_API kk_result KK_CALL kk_collision_set_heights(kk_collision_world_handle handle, uint32_t object,
                                                  const double *heights, uint32_t height_count) {
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!world.is_height_field(object) || !heights || !finite(heights, height_count))
            return KK_ERROR_INVALID_ARGUMENT;
        return world.set_heights(object, heights, height_count) ? KK_OK : KK_ERROR_INVALID_ARGUMENT;
    });
}

KK_API kk_result KK_CALL kk_collision_attach(kk_collision_world_handle handle, uint32_t object, int32_t body,
                                             const double *offset, uint32_t offset_count) {
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        double pose[7];
        if (!world.contains(object) || !body_ok(world, body) || !read_offset(offset, offset_count, pose))
            return KK_ERROR_INVALID_ARGUMENT;
        world.attach(object, body, pose);
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_remove(kk_collision_world_handle handle, uint32_t object) {
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!world.contains(object)) return KK_ERROR_INVALID_ARGUMENT;
        world.remove(object);
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_set_pair_rule(kk_collision_world_handle handle, uint32_t a, uint32_t b,
                                                    int32_t rule) {
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (a == b || !world.contains(a) || !world.contains(b) || rule < KK_PAIR_RULE_DEFAULT ||
            rule > KK_PAIR_RULE_CHECK)
            return KK_ERROR_INVALID_ARGUMENT;
        world.set_rule(a, b, rule);
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_pair_status(kk_collision_world_handle handle, uint32_t a, uint32_t b,
                                                  int32_t *out_status) {
    if (!out_status) return KK_ERROR_INVALID_ARGUMENT;
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (a == b || !world.contains(a) || !world.contains(b)) return KK_ERROR_INVALID_ARGUMENT;
        *out_status = world.status(a, b);
        return KK_OK;
    });
}

namespace {

bool configuration_ok(const CollisionWorld &world, const double *q, uint32_t dof_count, const double *roots,
                      uint32_t root_count) {
    return !(dof_count && !q) && dof_count == world.dof_count() && finite(q, dof_count) &&
           (!root_count || (root_count == 7 * world.body_count() && roots && finite(roots, root_count)));
}

} // namespace

KK_API kk_result KK_CALL kk_collision_allow_overlapping(kk_collision_world_handle handle, const double *q,
                                                        uint32_t dof_count, const double *root_poses,
                                                        uint32_t root_count, uint32_t *out_count) {
    if (!out_count) return KK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!configuration_ok(world, q, dof_count, root_poses, root_count)) return KK_ERROR_INVALID_ARGUMENT;
        *out_count = world.allow_overlapping(q, root_count ? root_poses : nullptr);
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_update(kk_collision_world_handle handle, const double *q, uint32_t dof_count,
                                             const double *root_poses, uint32_t root_count) {
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!configuration_ok(world, q, dof_count, root_poses, root_count)) return KK_ERROR_INVALID_ARGUMENT;
        world.update(q, root_count ? root_poses : nullptr);
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_check(kk_collision_world_handle handle, double margin, int32_t *out_pairs,
                                            uint32_t pair_capacity, uint32_t *out_count) {
    if (!out_count || (pair_capacity && !out_pairs)) return KK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!std::isfinite(margin) || margin < 0.0) return KK_ERROR_INVALID_ARGUMENT;
        std::vector<std::pair<uint32_t, uint32_t>> pairs;
        if (!world.check(margin, pairs)) return KK_ERROR_UNSUPPORTED;
        *out_count = uint32_t(pairs.size());
        for (size_t i = 0; i < pairs.size() && 2 * i + 1 < pair_capacity; ++i) {
            out_pairs[2 * i] = int32_t(pairs[i].first);
            out_pairs[2 * i + 1] = int32_t(pairs[i].second);
        }
        return KK_OK;
    });
}

KK_API kk_result KK_CALL kk_collision_distances(kk_collision_world_handle handle, double query_distance,
                                                int32_t *out_pairs, uint32_t pair_capacity, double *out_results,
                                                uint32_t result_capacity, uint32_t *out_count) {
    if (!out_count || (pair_capacity && !out_pairs) || (result_capacity && !out_results))
        return KK_ERROR_INVALID_ARGUMENT;
    *out_count = 0;
    return with_world(handle, [&](CollisionWorld &world) -> kk_result {
        if (!std::isfinite(query_distance)) return KK_ERROR_INVALID_ARGUMENT;
        std::vector<kk::PairDistance> found;
        if (!world.distances(query_distance, found)) return KK_ERROR_UNSUPPORTED;
        *out_count = uint32_t(found.size());
        const size_t written = std::min({found.size(), size_t(pair_capacity / 2), size_t(result_capacity / 10)});
        for (size_t i = 0; i < written; ++i) {
            const kk::PairDistance &pair = found[i];
            out_pairs[2 * i] = int32_t(pair.a);
            out_pairs[2 * i + 1] = int32_t(pair.b);
            double *row = out_results + 10 * i;
            row[0] = pair.distance;
            for (int k = 0; k < 3; ++k) {
                row[1 + k] = pair.point_a[k];
                row[4 + k] = pair.point_b[k];
                row[7 + k] = pair.normal[k];
            }
        }
        return KK_OK;
    });
}
