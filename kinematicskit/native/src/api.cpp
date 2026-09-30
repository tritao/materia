#include "kinematicskit.h"
#include "handles.hpp"
#include "model.hpp"
#include "qp.hpp"

#include <cmath>
#include <new>
#include <vector>

using kk::Model;

namespace {
kk::Handles<Model> models;
kk::Handles<kk::QpStep> steps;
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
