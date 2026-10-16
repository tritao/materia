#ifndef TRAJECTORY_MERGE_HPP
#define TRAJECTORY_MERGE_HPP
#include "trajectory_core.hpp"

namespace motionkit {
/** Borrowed polynomial storage; each joint begins coefficient_stride bytes after the previous joint.
 * Each row contains degree+1 aligned doubles. The source stays immutable until merge_polynomials returns.
 */
struct PolynomialSegmentView {
    int64_t start_ns, duration_ns;
    uint32_t degree, joint_count;
    const void *coefficients;
    size_t coefficient_stride;
    const double *joint(uint32_t index) const {
        return reinterpret_cast<const double *>(static_cast<const unsigned char *>(coefficients) + index * coefficient_stride);
    }
};
/** Segments must be contiguous, finite, and span at most INT64_MAX nanoseconds. */
struct PolynomialSource {
    const void *context;
    uint32_t count, joint_count;
    PolynomialSegmentView (*segment)(const void *, uint32_t);
};
/** Output remains owned by the caller. The inclusive source range identifies changed pieces.
 * A failure may leave a prefix in the output; the caller discards it transactionally.
 */
struct PolynomialOutput {
    void *context;
    mk_result (*append)(void *, const mk_segment &, uint32_t first, uint32_t last);
};
/** Approximation tolerances apply continuously on every original polynomial.
 * Inconclusive numerical bounds refuse a candidate rather than spend unproved error.
 * Zero requests exact preservation, including stationary joints; it is not an absent claim.
 */
struct MergeOptions {
    const double *position_tolerance, *velocity_tolerance, *acceleration_tolerance;
    const double *velocity_limit, *acceleration_limit, *position_lower, *position_upper;
    double max_duration_seconds;
    int64_t merge_below_ns = INT64_MAX;
};
TC_API mk_result merge_polynomials(const PolynomialSource &, const MergeOptions &, const PolynomialOutput &);
TC_API mk_result merge_trajectory(mk_trajectory_handle, const MergeOptions &, mk_trajectory_handle *);
/** Derive derivative-error bounds from each candidate's actual duration and position_tolerance.
 * velocity_tolerance and acceleration_tolerance are unused and may be null. Physical limits still apply.
 */
TC_API mk_result merge_polynomials_position_budget(const PolynomialSource &, const MergeOptions &, const PolynomialOutput &);
TC_API mk_result merge_trajectory_position_budget(mk_trajectory_handle, const MergeOptions &, mk_trajectory_handle *);
}
#endif
