#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>

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

private:
    struct Impl;
    uint32_t width_;
    std::unique_ptr<Impl> impl_;
};

} // namespace kk
