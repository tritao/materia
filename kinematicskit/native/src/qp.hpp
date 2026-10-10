#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <vector>

namespace kk {

/**
 * The bounded damped least-squares step
 *   minimize ½‖J·Δ − e‖² + ½λ²‖Δ‖²  subject to  lower ≤ Δ ≤ upper,
 * as a dense box-constrained QP on ProxQP. One instance per problem width,
 * reused across ticks: each solve warm-starts from the previous result.
 */
class QpStep {
public:
    explicit QpStep(uint32_t width);
    ~QpStep();

    uint32_t width() const { return width_; }

    struct Outcome {
        int32_t status;       // KK_QP_*
        uint32_t iterations;
    };

    /** `jacobian` is rows x width, row-major; infinite bounds mean unbounded. */
    Outcome solve(const double *jacobian, uint32_t rows, const double *residual, const double *lower,
                  const double *upper, double damping, double tolerance, uint32_t max_iterations, double *out_step);

    /**
     * As `solve`, with general rows `row_lower <= C*D <= row_upper` (`constraint`
     * is `constraint_rows` x width, row-major). When they and the bounds admit
     * no step, each row is relaxed by a slack penalized far above the task
     * (as mink relaxes its collision rows); `out_relaxed` then flags the rows
     * whose slack is in use.
     */
    Outcome solve_rows(const double *jacobian, uint32_t rows, const double *residual, const double *lower,
                       const double *upper, const double *constraint, uint32_t constraint_rows,
                       const double *row_lower, const double *row_upper, double damping, double tolerance,
                       uint32_t max_iterations, double *out_step, int32_t *out_relaxed);

    /** Rows kept for `kk_qp_solve` (see `kk_qp_set_rows`), and the relaxation flags of the last solve. */
    std::vector<double> constraint, row_lower, row_upper;
    std::vector<int32_t> relaxed;

private:
    struct Impl;
    struct Rows;
    uint32_t width_;
    std::unique_ptr<Impl> impl_;
    std::unique_ptr<Rows> hard_, soft_;
};

} // namespace kk
