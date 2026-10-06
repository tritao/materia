#include "motionkit.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <initializer_list>
#include <string>

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
    limits.max_velocity[0] = (limits.derivative_claimed[0] |= 1, 0.7);
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 4.0);
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
    assert(report.checks[MK_CHECK_TASK_SPACE].status == MK_CHECK_UNCHECKED);
    assert(report.executor_time_resolution_ns == 1);
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

    limits.max_velocity[0] = (limits.derivative_claimed[0] |= 1, 0.8);
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
    // The plan's segments read in place, until it is destroyed.
    assert(mk_plan_segment_array_count(plan) == 1);
    assert(mk_plan_segment_starts(plan)[0] == 0);
    assert(mk_plan_segment_durations(plan)[0] == 1'000'000'000);
    assert(mk_plan_segment_degrees(plan)[0] == 3);
    assert(mk_plan_coefficient_array_count(plan) == MK_MAX_DEGREE + 1);
    const double *coefficients = mk_plan_segment_coefficients(plan);
    near(coefficients[2], 1.5);
    near(coefficients[3], -1.0);
    near(coefficients[0], 0.0);
    near(coefficients[MK_MAX_DEGREE], 0.0);
    mk_plan_destroy(plan);
    assert(mk_plan_segment_array_count(plan) == 0 && mk_plan_segment_starts(plan) == nullptr);
    assert(mk_plan_coefficient_array_count(plan) == 0 && mk_plan_segment_coefficients(plan) == nullptr);
    spec.required_capabilities |= MK_CAP_EVENTS;
    spec.event_count = 1;
    spec.events[0].time_ns = 0;
    std::strcpy(spec.events[0].channel, "sprayer.flow");
    spec.events[0].value.kind = MK_EVENT_DIGITAL;
    spec.events[0].value.digital = 1;
    spec.events[0].hold_policy = MK_EVENT_RESTORE_ON_RESUME;
    assert(mk_plan_create(trajectory, &spec, &limits, &plan, &report) == MK_OK);
    assert(mk_plan_get_info(plan, &info) == MK_OK && info.event_count == 1);
    mk_timed_event stored_event{};
    assert(mk_plan_get_event(plan, 0, &stored_event) == MK_OK);
    assert(stored_event.value.digital == 1 && stored_event.time_ns == 0);
    assert(mk_plan_get_event(plan, 1, &stored_event) == MK_ERROR_INVALID_ARGUMENT);
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
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 2.0);
    limits.max_jerk[0] = (limits.derivative_claimed[0] |= 4, 5.0);
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
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 2.0 - 1e-7);
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
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 1.0);
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_ERROR_INVALID_ARGUMENT);
    mk_trajectory_destroy(trajectory);
}

void executor_resolution_and_tolerance_cap() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 3;
    segment.joint_count = 1;
    segment.coefficients[0].value[2] = (2.0 + 1.8e-9) / 2.0;
    segment.coefficients[0].value[3] = -5.0 / 6.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 2.0);
    limits.executor_time_resolution_ns = 2;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.executor_time_resolution_ns == 2);
    near(report.checks[MK_CHECK_ACCELERATION].tolerance, 5e-9);
    limits.executor_time_resolution_ns = 1;
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    near(report.checks[MK_CHECK_ACCELERATION].tolerance, 2.5e-9);
    limits.struct_size = offsetof(mk_limits, executor_time_resolution_ns);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.executor_time_resolution_ns == 1);
    mk_trajectory_destroy(trajectory);

    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    segment = {};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1;
    segment.degree = 3;
    segment.joint_count = 1;
    // Acceleration rises from 1.1 to 2.1 in one tick. The uncapped
    // jerk-derived 0.5 tolerance would incorrectly accept the 0.1 excess.
    segment.coefficients[0].value[2] = 1.1 / 2.0;
    segment.coefficients[0].value[3] = 1e9 / 6.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    limits.struct_size = sizeof(limits);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_ACCELERATION].tolerance, 2e-6);
    mk_trajectory_destroy(trajectory);
}

void zero_origin_real_overshoot_fails() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 0;
    segment.joint_count = 1;
    segment.coefficients[0].value[0] = -1e-4;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.position_claimed[0] = 1;
    limits.position_lower[0] = 0.0;
    limits.position_upper[0] = 2.0;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_POSITION].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_POSITION].tolerance, 2e-9);
    limits.position_upper[0] = 0.0;
    assert(mk_validate(trajectory, &limits, &report) == MK_OK);
    assert(report.checks[MK_CHECK_POSITION].status == MK_CHECK_FAILED);
    near(report.checks[MK_CHECK_POSITION].tolerance, 1e-12);
    mk_trajectory_destroy(trajectory);
}

void task_space_slot_writer() {
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    assert(report.checks[MK_CHECK_TASK_SPACE].status == MK_CHECK_UNCHECKED);
    assert(mk_report_set_task_space(&report, MK_CHECK_FAILED, 0.006, 0.75,
        0.005, 1'000'000) == MK_OK);
    const auto &check = report.checks[MK_CHECK_TASK_SPACE];
    assert(check.status == MK_CHECK_FAILED);
    assert(check.joint == UINT32_MAX);
    assert(check.derivative_order == 0);
    assert(check.method == MK_CHECK_METHOD_SAMPLED);
    near(check.value, 0.006);
    near(check.time_seconds, 0.75);
    near(check.limit, 0.005);
    near(check.margin, -0.001);
    assert(check.resolution_ns == 1'000'000);
    assert(mk_report_set_task_space(&report, MK_CHECK_PASSED, 0.004, 0.5,
        0.005, 500'000) == MK_OK);
    assert(check.status == MK_CHECK_PASSED);
    near(check.value, 0.004);
    near(check.limit, 0.005);
    assert(check.resolution_ns == 500'000);
    assert(mk_report_set_task_space(&report, MK_CHECK_PASSED, 0.0, 0.0,
        0.005, 0) == MK_ERROR_INVALID_ARGUMENT);
    assert(mk_report_set_task_space_bound(&report,0.0001,0.005)==MK_OK);
    assert(check.status==MK_CHECK_PASSED && check.method==MK_CHECK_METHOD_BOUND);
    near(check.value,0.0001);near(check.margin,0.0049);
    assert(check.time_seconds==0 && check.resolution_ns==0);
    assert(mk_report_set_task_space_bound(&report,0.006,0.005)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_report_set_task_space_bound(&report,-1,0.005)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_report_set_task_space_bound(&report,NAN,0.005)==MK_ERROR_INVALID_ARGUMENT);
}

void execution_certificate_parity() {
    auto mode=[](bool audit){
#ifdef _WIN32
        _putenv_s("PROCESS_PATH_VERIFY_EXECUTION",audit?"1":"0");
#else
        setenv("PROCESS_PATH_VERIFY_EXECUTION",audit?"1":"0",1);
#endif
    };
    const char *previous=std::getenv("PROCESS_PATH_VERIFY_EXECUTION");
    const bool had_previous=previous!=nullptr;const std::string saved=previous?previous:"";
    for(unsigned fixture=0;fixture<200;++fixture){
        mk_trajectory_handle trajectory{};assert(mk_trajectory_create(2,&trajectory)==MK_OK);
        mk_segment segment{};segment.struct_size=sizeof(segment);segment.joint_count=2;
        segment.duration_ns=100'000'000+fixture*10'000'000;segment.degree=5;
        for(unsigned j=0;j<2;++j)for(unsigned k=0;k<6;++k)
            segment.coefficients[j].value[k]=fixture<100 ? (k?(j==0?1.0:-0.1)*(fixture%7+1)/(k+1):0.0) : std::sin((fixture+1)*(j+2)*(k+3))*5;
        assert(mk_trajectory_append_segment(trajectory,&segment)==MK_OK);
        mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=2;
        for(unsigned j=0;j<2;++j){limits.position_claimed[j]=1;limits.position_lower[j]=-4;limits.position_upper[j]=4;
            limits.derivative_claimed[j]=7;limits.max_velocity[j]=5;limits.max_acceleration[j]=10;limits.max_jerk[j]=30;}
        mk_validation_report certified{},reference{};certified.struct_size=sizeof(certified);reference.struct_size=sizeof(reference);
        mode(false);assert(mk_validate(trajectory,&limits,&certified)==MK_OK);
        mode(true);assert(mk_validate(trajectory,&limits,&reference)==MK_OK);
        for(unsigned i=0;i<MK_CHECK_COUNT;++i){auto &a=certified.checks[i],&b=reference.checks[i];
            assert(a.status==b.status && a.method==b.method && a.joint==b.joint && a.derivative_order==b.derivative_order);
            assert(std::abs(a.value-b.value)<=1e-9 && std::abs(a.time_seconds-b.time_seconds)<=1e-9);
            assert(std::abs(a.tolerance-b.tolerance)<=1e-9 && std::abs(a.margin-b.margin)<=1e-9);}
        mk_trajectory_destroy(trajectory);
    }
#ifdef _WIN32
    _putenv_s("PROCESS_PATH_VERIFY_EXECUTION",had_previous?saved.c_str():"");
#else
    if(had_previous)setenv("PROCESS_PATH_VERIFY_EXECUTION",saved.c_str(),1);else unsetenv("PROCESS_PATH_VERIFY_EXECUTION");
#endif
}

} // namespace

/*
 * Plans are shared across threads: one thread evaluates a plan while another creates and destroys
 * plans, and destroying a plan another thread holds waits for no lock held during validation.
 */
void plans_across_threads() {
    const auto trajectory = cubic();
    mk_limits limits{};
    limits.struct_size = sizeof(limits);
    limits.joint_count = 1;
    limits.max_velocity[0] = (limits.derivative_claimed[0] |= 1, 0.8);
    limits.max_acceleration[0] = (limits.derivative_claimed[0] |= 2, 4.0);
    mk_plan_spec spec{};
    spec.struct_size = sizeof(spec);
    spec.plan_id = 1;
    spec.required_capabilities = MK_CAP_TIMED_TRAJECTORY;
    spec.planning_authority = MK_AUTHORITY_MATERIA;
    spec.start_state.struct_size = sizeof(spec.start_state);
    spec.start_state.joint_count = 1;
    mk_validation_report report{};
    report.struct_size = sizeof(report);
    mk_plan_handle evaluated{};
    assert(mk_plan_create(trajectory, &spec, &limits, &evaluated, &report) == MK_OK);
    std::atomic<bool> done{false};
    std::atomic<int> failures{0};
    std::thread planner([&] {
        for (int index = 0; index < 200; ++index) {
            mk_validation_report own{};
            own.struct_size = sizeof(own);
            mk_plan_handle plan{};
            if (mk_plan_create(trajectory, &spec, &limits, &plan, &own) != MK_OK ||
                mk_plan_segment_array_count(plan) != 1) ++failures;
            mk_plan_destroy(plan);
        }
        done = true;
    });
    int evaluations = 0;
    while (!done || evaluations == 0) {
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        if (mk_plan_evaluate(evaluated, 500'000'000, &state) != MK_OK) ++failures;
        else near(state.position[0], 1.5 * 0.25 - 0.125);
        ++evaluations;
    }
    planner.join();
    assert(failures == 0);
    mk_plan_destroy(evaluated);
    mk_trajectory_destroy(trajectory);
}

void continuity_position_units_and_origin() {
    for (const double extent : {251.0, 2e9})
    for (const double scale : {1.0, 1.0 / 3141.592653589793})
        for (const double origin : {0.0, 100.0})
            for (const double gap : {1.37e-7, 1e-4}) {
                mk_trajectory_handle trajectory{};
                assert(mk_trajectory_create(1, &trajectory) == MK_OK);
                mk_segment segment{};
                segment.struct_size = sizeof(segment);
                segment.joint_count = 1;
                segment.degree = 1;
                segment.duration_ns = 1'000'000'000;
                segment.coefficients[0].value[0] = origin;
                segment.coefficients[0].value[1] = 137.0 * scale;
                assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
                segment.t0_ns = segment.duration_ns;
                segment.coefficients[0].value[0] = origin + (137.0 + gap) * scale;
                segment.coefficients[0].value[1] = 0;
                assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
                mk_limits limits{};
                limits.struct_size = sizeof(limits);
                limits.joint_count = 1;
                limits.position_claimed[0] = 1;
                limits.position_lower[0] = origin;
                limits.position_upper[0] = origin + extent * scale;
                limits.max_continuity_jump[0] = 1e-9 * scale;
                limits.executor_time_resolution_ns = 1;
                mk_validation_report report{};
                report.struct_size = sizeof(report);
                assert(mk_validate(trajectory, &limits, &report) == MK_OK);
                assert(report.checks[MK_CHECK_CONTINUITY].status ==
                    (gap < 1e-6 ? MK_CHECK_UNCHECKED : MK_CHECK_FAILED));
                assert(report.checks[MK_CHECK_CONTINUITY].tolerance < 1e-6 * scale);
                mk_trajectory_destroy(trajectory);
            }
}

int main() {
    continuity_position_units_and_origin();
    velocity_extremum_and_plan_rejection();
    position_extremum_between_samples();
    quintic_quartic_critical_point();
    explicit_quantization_tolerance();
    executor_resolution_and_tolerance_cap();
    zero_origin_real_overshoot_fails();
    nonfinite_extrema_rejected();
    task_space_slot_writer();
    execution_certificate_parity();
#ifndef __EMSCRIPTEN__
    // This single-threaded Node target has no pthread worker pool.
    plans_across_threads();
#endif
}
