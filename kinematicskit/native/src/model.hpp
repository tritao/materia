#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace kk {

/** A compiled kinematic model and its evaluation buffers (the native twin of the Haxe snapshot). */
struct Model {
    uint32_t body_count = 0, joint_count = 0, dof_count = 0, frame_count = 0;
    std::vector<int32_t> body_parent_joint, body_order;
    std::vector<int32_t> joint_kind, joint_parent, joint_child, joint_dof, joint_source;
    std::vector<int32_t> joint_order, joint_value_order, frame_body;
    std::vector<double> root_pose, parent_t_joint, joint_t_child, axis, ratio, offset, scale, frame_offset;
    std::vector<std::vector<int32_t>> body_chain;

    // Evaluation state, reused across calls.
    std::vector<double> values, poses, origins, axes;

    bool build(const int32_t *ints, uint32_t int_count, const double *reals, uint32_t real_count);
    /** Forward kinematics into `poses`; `roots` (7 per body) overrides root poses when non-null. */
    void evaluate(const double *q, const double *roots);
    /** 6 x width Jacobian of a world point on `body`, after `evaluate`. */
    void point_jacobian(uint32_t body, double px, double py, double pz, const int32_t *column_of_dof,
                        uint32_t width, double *out) const;
};

/** `out = a · b` for seven-double transforms; `out` may alias neither input. */
void compose(const double *a, const double *b, double *out);
/** `out` (3) = rotation of `a` applied to `(vx, vy, vz)`. */
void rotate(const double *a, double vx, double vy, double vz, double *out);


} // namespace kk
