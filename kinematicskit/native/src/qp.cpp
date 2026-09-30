#include "qp.hpp"
#include "kinematicskit.h"

#include <algorithm>
#include <cmath>
#include <proxsuite/helpers/common.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace kk {

namespace dense = proxsuite::proxqp::dense;
using proxsuite::proxqp::QPSolverOutput;

struct QpStep::Impl {
    dense::QP<double> qp;
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient, lower, upper;
    bool initialized = false;

    explicit Impl(uint32_t width)
        : qp(Eigen::Index(width), 0, 0, true, proxsuite::proxqp::HessianType::Dense,
             proxsuite::proxqp::DenseBackend::PrimalDualLDLT),
          hessian(width, width), gradient(width), lower(width), upper(width) {
        qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        qp.settings.verbose = false;
    }
};

QpStep::QpStep(uint32_t width) : width_(width), impl_(std::make_unique<Impl>(width)) {}
QpStep::~QpStep() = default;

QpStep::Outcome QpStep::solve(const double *jacobian, uint32_t rows, const double *residual, const double *lower,
                              const double *upper, double damping, double tolerance, uint32_t max_iterations,
                              double *out_step) {
    Impl &s = *impl_;
    const Eigen::Index n = Eigen::Index(width_);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, rows, n);
    Eigen::Map<const Eigen::VectorXd> e(residual, rows);
    // ½ΔᵀHΔ + gᵀΔ with H = JᵀJ + λ²I and g = −Jᵀe.
    s.hessian.noalias() = J.transpose() * J;
    s.hessian.diagonal().array() += damping * damping;
    s.gradient.noalias() = -(J.transpose() * e);
    const double infinite = proxsuite::helpers::infinite_bound<double>::value();
    for (Eigen::Index i = 0; i < n; ++i) {
        s.lower[i] = std::isfinite(lower[i]) ? lower[i] : -infinite;
        s.upper[i] = std::isfinite(upper[i]) ? upper[i] : infinite;
    }
    s.qp.settings.eps_abs = tolerance;
    s.qp.settings.max_iter = Eigen::Index(max_iterations);
    if (!s.initialized) {
        s.qp.init(s.hessian, s.gradient, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
                  proxsuite::nullopt, proxsuite::nullopt, s.lower, s.upper);
        s.initialized = true;
    } else {
        s.qp.update(s.hessian, s.gradient, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt,
                    proxsuite::nullopt, proxsuite::nullopt, s.lower, s.upper);
    }
    s.qp.solve();
    const auto &results = s.qp.results;
    // ProxQP meets the bounds to its tolerance; project so they hold exactly
    // (joint and velocity limits are hard). This moves the step by at most
    // the solver tolerance.
    for (Eigen::Index i = 0; i < n; ++i) out_step[i] = std::min(std::max(results.x[i], lower[i]), upper[i]);
    int32_t status;
    switch (results.info.status) {
    case QPSolverOutput::PROXQP_SOLVED: status = KK_QP_SOLVED; break;
    case QPSolverOutput::PROXQP_MAX_ITER_REACHED: status = KK_QP_MAX_ITERATIONS; break;
    case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
    case QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE: status = KK_QP_INFEASIBLE; break;
    default: status = KK_QP_FAILED; break;
    }
    return {status, uint32_t(results.info.iter)};
}

} // namespace kk
