#include "motionkit.h"

#include <cassert>
#include <cmath>

namespace {

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-12);
}

mk_trajectory_handle cubic() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = 0;
    segment.duration_ns = 1'000'000'000;
    segment.degree = 3;
    segment.joint_count = 1;
    segment.coefficients[0].value[2] = 1.5;
    segment.coefficients[0].value[3] = -1.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    return trajectory;
}

void velocity_extremum_and_plan_rejection() {
    const auto trajectory = cubic();
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.max_velocity[0] = 0.7;
    limits.max_acceleration[0] = 4.0;
    limits.model_revision = 12;
    limits.calibration_revision = 3;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_VELOCITY].value, 0.75);
    near(report.checks[MK_CHECK_VELOCITY].time_seconds, 0.5);
    near(report.checks[MK_CHECK_VELOCITY].limit, 0.7);
    assert(report.checks[MK_CHECK_JERK].status == MK_CHECK_UNCHECKED);
    assert(report.model_revision == 12 && report.calibration_revision == 3);
    assert(report.trajectory_revision == 1);
    assert(report.assumption_count > 0);

    mk_plan_spec spec{};
    spec.struct_size = sizeof(spec);
    spec.plan_id = 44;
    spec.model_revision = 12;
    spec.calibration_revision = 3;
    spec.required_capabilities = MK_CAP_TIMED_TRAJECTORY;
    spec.planning_authority = MK_AUTHORITY_MATERIA;
    spec.start_state.struct_size = sizeof(spec.start_state);
    spec.start_state.joint_count = 1;
    spec.start_state.position_tolerance[0] = 0.01;
    spec.start_state.velocity_tolerance[0] = 0.01;
    spec.start_state.acceleration_tolerance[0] = 0.01;
    mk_plan_handle plan{};
    assert(mk_plan_create(trajectory, &spec, &limits, &plan, &report) == MK_ERROR_LIMIT);
    assert(plan.id == 0);
    assert(report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_FAILED);

    limits.max_velocity[0] = 0.8;
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_PASSED);
    assert(mk_plan_create(trajectory, &spec, &limits, &plan, &report) == MK_OK);
    mk_plan_info info{};
    info.struct_size = sizeof(info);
    assert(mk_plan_get_info(plan, &info) == MK_OK);
    assert(info.plan_id == 44 && info.model_revision == 12 &&
        info.calibration_revision == 3 && info.trajectory_revision == 1);
    assert(info.required_capabilities == MK_CAP_TIMED_TRAJECTORY);
    assert(info.planning_authority == MK_AUTHORITY_MATERIA);
    mk_start_state stored_start{};
    stored_start.struct_size = sizeof(stored_start);
    assert(mk_plan_get_start_state(plan, &stored_start) == MK_OK);
    near(stored_start.position_tolerance[0], 0.01);
    mk_validation_report stored_report{};
    stored_report.struct_size = sizeof(stored_report);
    assert(mk_plan_get_report(plan, &stored_report) == MK_OK);
    assert(stored_report.checks[MK_CHECK_VELOCITY].status == MK_CHECK_PASSED);
    mk_plan_destroy(plan);
    mk_trajectory_destroy(trajectory);
}

void position_extremum_between_samples() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 2;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 1.0;
    segment.coefficients[0].value[2] = -1.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.position_claimed[0] = 1;
    limits.position_lower[0] = 0.0;
    limits.position_upper[0] = 0.2;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_POSITION].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_POSITION].value, 0.25);
    near(report.checks[MK_CHECK_POSITION].time_seconds, 0.5);
    mk_trajectory_destroy(trajectory);
}

void quintic_quartic_critical_point() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 5;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.5;
    segment.coefficients[0].value[2] = 0.25;
    segment.coefficients[0].value[3] = -0.5;
    segment.coefficients[0].value[4] = -0.625;
    segment.coefficients[0].value[5] = -0.2;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.position_claimed[0] = 1;
    limits.position_lower[0] = -1.0;
    limits.position_upper[0] = 0.2;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_POSITION].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_POSITION].time_seconds, 0.5);
    near(report.checks[MK_CHECK_POSITION].value, 0.2046875);
    mk_trajectory_destroy(trajectory);
}

void explicit_quantization_tolerance() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1;
    segment.degree = 3;
    segment.joint_count = 1;
    segment.coefficients[0].value[2] = (2.0 + 1.8e-9) / 2.0;
    segment.coefficients[0].value[3] = -5.0 / 6.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.max_acceleration[0] = 2.0;
    limits.max_jerk[0] = 5.0;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    const auto &check = report.checks[MK_CHECK_ACCELERATION];
    assert(check.status == MK_CHECK_PASSED);
    near(check.value, 2.0 + 1.8e-9);
    near(check.margin, -1.8e-9);
    near(check.tolerance, 2.5e-9);
    mk_plan_spec spec{};
    spec.struct_size = sizeof(spec);
    spec.plan_id = 71;
    spec.planning_authority = MK_AUTHORITY_MATERIA;
    spec.start_state.struct_size = sizeof(spec.start_state);
    spec.start_state.joint_count = 1;
    mk_plan_handle plan{};
    assert(mk_plan_create(trajectory, &spec, &limits, &plan, &report) == MK_OK);
    assert(plan.id != 0);
    mk_plan_destroy(plan);
    limits.max_acceleration[0] = 2.0 - 1e-7;
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status == MK_CHECK_FAILED);
    mk_trajectory_destroy(trajectory);
}

void nonfinite_extrema_rejected() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 2;
    segment.joint_count = 1;
    segment.coefficients[0].value[2] = 1e308;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.max_acceleration[0] = 1.0;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_ERROR_INVALID_ARGUMENT);
    mk_trajectory_destroy(trajectory);
}

} // namespace

int main() {
    velocity_extremum_and_plan_rejection();
    position_extremum_between_samples();
    quintic_quartic_critical_point();
    explicit_quantization_tolerance();
    nonfinite_extrema_rejected();
}
