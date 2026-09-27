#include "motionkit.h"

#include <opw_kinematics/opw_kinematics.h>
#include <opw_kinematics/opw_utilities.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace {
bool parameters_valid(const mk_opw_parameters *value) {
    if (!value || value->struct_size != sizeof(*value)) return false;
    const double dimensions[] = {value->a1, value->a2, value->b, value->c1,
        value->c2, value->c3, value->c4};
    for (double dimension : dimensions) if (!std::isfinite(dimension)) return false;
    if (value->c2 <= 0.0 || value->c3 <= 0.0 || value->c4 < 0.0) return false;
    for (unsigned i = 0; i < 6; ++i)
        if (!std::isfinite(value->offsets[i]) ||
            (value->sign_corrections[i] != 1 && value->sign_corrections[i] != -1))
            return false;
    return true;
}

opw_kinematics::Parameters<double> convert(const mk_opw_parameters &value) {
    opw_kinematics::Parameters<double> result;
    result.a1 = value.a1; result.a2 = value.a2; result.b = value.b;
    result.c1 = value.c1; result.c2 = value.c2;
    result.c3 = value.c3; result.c4 = value.c4;
    for (unsigned i = 0; i < 6; ++i) {
        result.offsets[i] = value.offsets[i];
        result.sign_corrections[i] = value.sign_corrections[i];
    }
    return result;
}

bool pose_valid(const mk_opw_pose *pose) {
    if (!pose || pose->struct_size != sizeof(*pose)) return false;
    for (double value : pose->position) if (!std::isfinite(value)) return false;
    double squared_norm = 0.0;
    for (double value : pose->quaternion) {
        if (!std::isfinite(value)) return false;
        squared_norm += value * value;
    }
    return std::abs(squared_norm - 1.0) <= 1e-6;
}

opw_kinematics::Transform<double> convert(const mk_opw_pose &value) {
    opw_kinematics::Transform<double> result = opw_kinematics::Transform<double>::Identity();
    result.translation() = Eigen::Vector3d(value.position[0], value.position[1], value.position[2]);
    const Eigen::Quaterniond rotation(value.quaternion[3], value.quaternion[0],
        value.quaternion[1], value.quaternion[2]);
    result.linear() = rotation.toRotationMatrix();
    return result;
}

void convert(const opw_kinematics::Transform<double> &value, mk_opw_pose &result) {
    const Eigen::Quaterniond rotation(value.rotation());
    for (unsigned i = 0; i < 3; ++i) result.position[i] = value.translation()[i];
    result.quaternion[0] = rotation.x(); result.quaternion[1] = rotation.y();
    result.quaternion[2] = rotation.z(); result.quaternion[3] = rotation.w();
}
}  // namespace

extern "C" mk_result MK_CALL mk_opw_forward(const mk_opw_parameters *parameters,
    const double *joints, uint32_t joint_count, mk_opw_pose *out_pose) {
    if (!parameters_valid(parameters) || !joints || joint_count != 6 || !out_pose)
        return MK_ERROR_INVALID_ARGUMENT;
    out_pose->struct_size = sizeof(*out_pose);
    std::array<double, 6> q;
    for (unsigned i = 0; i < 6; ++i) {
        if (!std::isfinite(joints[i])) return MK_ERROR_INVALID_ARGUMENT;
        q[i] = joints[i];
    }
    convert(opw_kinematics::forward(convert(*parameters), q), *out_pose);
    return MK_OK;
}

extern "C" mk_result MK_CALL mk_opw_inverse(const mk_opw_parameters *parameters,
    const mk_opw_pose *pose, mk_opw_solution *out_solutions, uint32_t solution_count) {
    if (!parameters_valid(parameters) || !pose_valid(pose) || !out_solutions || solution_count != 8)
        return MK_ERROR_INVALID_ARGUMENT;
    const auto geometry = convert(*parameters);
    const auto solutions = opw_kinematics::inverse(geometry, convert(*pose));
    const auto wrist_center = convert(*pose).translation() -
        parameters->c4 * convert(*pose).rotation().col(2);
    const double scale = std::max({1.0, std::abs(parameters->c1),
        std::abs(parameters->c2), std::abs(parameters->c3)});
    const bool shoulder_singular = wrist_center.head<2>().norm() < 1e-6 * scale;
    for (unsigned i = 0; i < 8; ++i) {
        auto &output = out_solutions[i];
        output.struct_size = sizeof(output);
        output.valid = opw_kinematics::isValid(solutions[i]) ? 1 : 0;
        output.singular = 0;
        for (unsigned j = 0; j < 6; ++j)
            output.joints[j] = output.valid ? solutions[i][j] : 0.0;
        if (output.valid) {
            const double wrist_angle = solutions[i][4] * parameters->sign_corrections[4] -
                parameters->offsets[4];
            output.singular = shoulder_singular || std::abs(std::sin(wrist_angle)) < 1e-6;
        }
    }
    return MK_OK;
}
