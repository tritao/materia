#include "motionkit.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace {
using V = Eigen::Vector3d;
using T = Eigen::Isometry3d;
V vector(const double *p) { return V(p[0], p[1], p[2]); }
Eigen::Quaterniond quaternion(const double *p) { return Eigen::Quaterniond(p[3], p[0], p[1], p[2]); }
bool finite(const double *p, unsigned n) {
    for (unsigned i = 0; i < n; ++i) if (!std::isfinite(p[i])) return false;
    return true;
}
Eigen::Matrix3d translation_basis(const mk_analytic_cartesian_model &m) {
    Eigen::Matrix3d b;
    for (unsigned i = 0; i < 3; ++i) b.col(i) = vector(m.translation_axes + 3 * i);
    return b;
}
bool valid(const mk_analytic_cartesian_model *m) {
    if (!m || m->struct_size != sizeof(*m) || m->joint_count < 3 || m->joint_count > 5 ||
        !finite(m->translation_axes, 9) || !finite(m->home_position, 3) ||
        !finite(m->home_quaternion, 4) || std::abs(quaternion(m->home_quaternion).norm() - 1) > 1e-8)
        return false;
    auto b = translation_basis(*m);
    if ((b.transpose() * b - Eigen::Matrix3d::Identity()).norm() > 1e-8) return false;
    for (unsigned i = 0; i < m->joint_count - 3; ++i)
        if (!finite(m->rotary_axes + 3 * i, 3) || !finite(m->rotary_origins + 3 * i, 3) ||
            std::abs(vector(m->rotary_axes + 3 * i).norm() - 1) > 1e-8) return false;
    return m->joint_count != 5 || std::abs(vector(m->rotary_axes).dot(vector(m->rotary_axes + 3))) < 1e-8;
}
T home(const mk_analytic_cartesian_model &m) {
    T t = T::Identity();
    t.linear() = quaternion(m.home_quaternion).toRotationMatrix();
    t.translation() = vector(m.home_position);
    return t;
}
T rotation(const V &axis, const V &origin, double angle) {
    T t = T::Identity();
    t.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    t.translation() = origin - t.linear() * origin;
    return t;
}
T head(const mk_analytic_cartesian_model &m, const double *q) {
    T t = T::Identity();
    for (unsigned i = 0; i < m.joint_count - 3; ++i)
        t = t * rotation(vector(m.rotary_axes + 3 * i), vector(m.rotary_origins + 3 * i), q[3 + i]);
    return t * home(m);
}
void write_pose(const T &t, mk_analytic_pose &p) {
    p = {}; p.struct_size = sizeof(p);
    Eigen::Quaterniond q(t.linear());
    for (unsigned i = 0; i < 3; ++i) p.position[i] = t.translation()[i];
    p.quaternion[0] = q.x(); p.quaternion[1] = q.y(); p.quaternion[2] = q.z(); p.quaternion[3] = q.w();
}
double alignment(const V &a, const V &b, const V &axis) {
    return std::atan2(axis.dot(a.cross(b)), a.dot(b));
}
}

extern "C" mk_result MK_CALL mk_analytic_cartesian_forward(const mk_analytic_cartesian_model *m,
    const double *q, uint32_t n, mk_analytic_pose *out) {
    if (!valid(m) || !q || n != m->joint_count || !finite(q, n) || !out) return MK_ERROR_INVALID_ARGUMENT;
    T t = head(*m, q);
    t.translation() += translation_basis(*m) * vector(q);
    write_pose(t, *out);
    return MK_OK;
}

extern "C" mk_result MK_CALL mk_analytic_cartesian_inverse(const mk_analytic_cartesian_model *m,
    const mk_analytic_pose *target, uint32_t axis_only, double seed, mk_analytic_solution *out,
    uint32_t capacity, uint32_t *count) {
    if (!valid(m) || !target || target->struct_size != sizeof(*target) || !out || capacity < 2 ||
        !count || axis_only > 1 || !std::isfinite(seed) || !finite(target->position, 3) ||
        !finite(target->quaternion, 4) || std::abs(quaternion(target->quaternion).norm() - 1) > 1e-8)
        return MK_ERROR_INVALID_ARGUMENT;
    *count = 0;
    const auto wanted = quaternion(target->quaternion).toRotationMatrix();
    const V direction = wanted.col(2), zero = home(*m).linear().col(2);
    double tilts[2] = {0, 0}; unsigned branches = 1;
    if (m->joint_count == 5) {
        V c = vector(m->rotary_axes), a = vector(m->rotary_axes + 3);
        double cosine = c.dot(zero), sine = c.dot(a.cross(zero));
        double radius = std::hypot(cosine, sine), value = c.dot(direction);
        if (radius < 1e-10) return MK_ERROR_INVALID_ARGUMENT; // Tool lies on A: unsupported head.
        if (std::abs(value) > radius + 1e-9) return MK_OK;
        double phase = std::atan2(sine, cosine), angle = std::acos(std::clamp(value / radius, -1.0, 1.0));
        tilts[0] = phase + angle; tilts[1] = phase - angle;
        branches = angle < 1e-10 || std::abs(angle - std::acos(-1.0)) < 1e-10 ? 1 : 2;
    }
    for (unsigned branch = 0; branch < branches; ++branch) {
        mk_analytic_solution s = {}; s.struct_size = sizeof(s); s.branch = branch;
        s.joints[4] = tilts[branch];
        if (m->joint_count >= 4) {
            const V c = vector(m->rotary_axes);
            V before = zero;
            if (m->joint_count == 5) before = Eigen::AngleAxisd(tilts[branch], vector(m->rotary_axes + 3)) * before;
            if (std::abs(c.dot(before) - c.dot(direction)) > 1e-8) continue;
            V u = before - c * c.dot(before), v = direction - c * c.dot(direction);
            if (u.norm() < 1e-9) {
                s.singular = 1; s.joints[3] = seed;
                if (!axis_only) {
                    Eigen::Matrix3d before_c = home(*m).linear();
                    if (m->joint_count == 5)
                        before_c = Eigen::AngleAxisd(tilts[branch], vector(m->rotary_axes + 3)).toRotationMatrix() * before_c;
                    Eigen::Matrix3d relative = wanted * before_c.transpose();
                    // Resolve the otherwise-free C from the full requested rotation.
                    Eigen::AngleAxisd aa(relative);
                    s.joints[3] = aa.angle() * (aa.axis().dot(c) < 0 ? -1 : 1);
                }
            } else s.joints[3] = alignment(u, v, c);
        }
        T t = head(*m, s.joints);
        if (axis_only ? (t.linear().col(2) - direction).norm() > 1e-8 : (t.linear() - wanted).norm() > 1e-8) continue;
        V q = translation_basis(*m).transpose() * (vector(target->position) - t.translation());
        for (unsigned i = 0; i < 3; ++i) s.joints[i] = q[i];
        out[(*count)++] = s;
    }
    return MK_OK;
}
