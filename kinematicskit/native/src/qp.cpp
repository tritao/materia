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

/** A QP with box bounds and inequality rows, rebuilt when its row count changes. */
struct QpStep::Rows {
    Eigen::Index variables = 0, rows = -1;
    std::unique_ptr<dense::QP<double>> qp;
    bool initialized = false;

    void shape(Eigen::Index n, Eigen::Index m) {
        if (qp && n == variables && m == rows) return;
        variables = n;
        rows = m;
        qp = std::make_unique<dense::QP<double>>(n, 0, m, true, proxsuite::proxqp::HessianType::Dense,
                                                  proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
        qp->settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        qp->settings.verbose = false;
        initialized = false;
    }

    int32_t run(const Eigen::MatrixXd &h, const Eigen::VectorXd &g, const Eigen::MatrixXd &c,
                const Eigen::VectorXd &l, const Eigen::VectorXd &u, const Eigen::VectorXd &lbox,
                const Eigen::VectorXd &ubox, double tolerance, uint32_t max_iterations, uint32_t &iterations) {
        qp->settings.eps_abs = tolerance;
        qp->settings.max_iter = Eigen::Index(max_iterations);
        if (!initialized) {
            qp->init(h, g, proxsuite::nullopt, proxsuite::nullopt, c, l, u, lbox, ubox);
            initialized = true;
        } else {
            qp->update(h, g, proxsuite::nullopt, proxsuite::nullopt, c, l, u, lbox, ubox);
        }
        qp->solve();
        iterations += uint32_t(qp->results.info.iter);
        switch (qp->results.info.status) {
        case QPSolverOutput::PROXQP_SOLVED: return KK_QP_SOLVED;
        case QPSolverOutput::PROXQP_MAX_ITER_REACHED: return KK_QP_MAX_ITERATIONS;
        case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
        case QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE: return KK_QP_INFEASIBLE;
        default: return KK_QP_FAILED;
        }
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

QpStep::Outcome QpStep::solve_rows(const double *jacobian, uint32_t rows, const double *residual,
                                   const double *lower, const double *upper, const double *constraint,
                                   uint32_t constraint_rows, const double *row_lower, const double *row_upper,
                                   double damping, double tolerance, uint32_t max_iterations, double *out_step,
                                   int32_t *out_relaxed) {
    const Eigen::Index n = Eigen::Index(width_), m = Eigen::Index(constraint_rows);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, rows, n);
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> C(constraint, m, n);
    Eigen::Map<const Eigen::VectorXd> e(residual, rows);
    const double infinite = proxsuite::helpers::infinite_bound<double>::value();
    auto bound = [&](double value) { return std::isfinite(value) ? value : (value > 0 ? infinite : -infinite); };
    Eigen::MatrixXd h = J.transpose() * J;
    h.diagonal().array() += damping * damping;
    const Eigen::VectorXd g = -(J.transpose() * e);
    Eigen::VectorXd lbox(n), ubox(n), l(m), u(m);
    for (Eigen::Index i = 0; i < n; ++i) {
        lbox[i] = bound(lower[i]);
        ubox[i] = bound(upper[i]);
    }
    for (Eigen::Index i = 0; i < m; ++i) {
        l[i] = bound(row_lower[i]);
        u[i] = bound(row_upper[i]);
        out_relaxed[i] = 0;
    }
    uint32_t iterations = 0;
    if (!hard_) hard_ = std::make_unique<Rows>();
    hard_->shape(n, m);
    const Eigen::MatrixXd c = C;
    int32_t status = hard_->run(h, g, c, l, u, lbox, ubox, tolerance, max_iterations, iterations);
    Eigen::VectorXd x;
    if (status == KK_QP_SOLVED) {
        x = hard_->qp->results.x;
    } else {
        // Relax every row by a slack s >= 0 (C*D + s >= l, C*D - s <= u), penalized far above the task.
        if (!soft_) soft_ = std::make_unique<Rows>();
        soft_->shape(n + m, 2 * m);
        const double penalty = 1e6 * std::max(1.0, h.diagonal().maxCoeff());
        Eigen::MatrixXd hs = Eigen::MatrixXd::Zero(n + m, n + m);
        hs.topLeftCorner(n, n) = h;
        hs.bottomRightCorner(m, m).diagonal().setConstant(penalty);
        Eigen::VectorXd gs = Eigen::VectorXd::Zero(n + m);
        gs.head(n) = g;
        Eigen::MatrixXd cs = Eigen::MatrixXd::Zero(2 * m, n + m);
        Eigen::VectorXd ls(2 * m), us(2 * m), lboxs(n + m), uboxs(n + m);
        for (Eigen::Index i = 0; i < m; ++i) {
            cs.row(i).head(n) = C.row(i);
            cs(i, n + i) = 1.0;
            ls[i] = l[i];
            us[i] = infinite;
            cs.row(m + i).head(n) = C.row(i);
            cs(m + i, n + i) = -1.0;
            ls[m + i] = -infinite;
            us[m + i] = u[i];
        }
        lboxs << lbox, Eigen::VectorXd::Zero(m);
        uboxs << ubox, Eigen::VectorXd::Constant(m, infinite);
        status = soft_->run(hs, gs, cs, ls, us, lboxs, uboxs, tolerance, max_iterations, iterations);
        x = soft_->qp->results.x.head(n);
        const auto slack = soft_->qp->results.x.tail(m);
        for (Eigen::Index i = 0; i < m; ++i) out_relaxed[i] = slack[i] > 10.0 * tolerance ? 1 : 0;
    }
    for (Eigen::Index i = 0; i < n; ++i) out_step[i] = std::min(std::max(x[i], lower[i]), upper[i]);
    return {status, iterations};
}

} // namespace kk
