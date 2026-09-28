#include "virtual_device_endpoint.hpp"
#include "motionkit.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>

using namespace robotkit;

struct RunResult {
    double position;
    rk_robot_state state;
    std::vector<VirtualStepRecord6> steps;
};

RunResult run(VirtualDeviceConfig6 config, bool cut = false, bool replace = false,
    bool hold = false) {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 10;
    blueprint.joints[0].max_acceleration = 10;
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    assert(endpoint->sample(0, state) == RK_ERROR_STALE_STATE);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000) {
        const auto result = endpoint->sample(now, state);
        assert(result == RK_OK || result == RK_ERROR_STALE_STATE);
    }
    assert(endpoint->sample(100'000'000, state) == RK_OK);
    for (std::uint64_t now = 102'000'000; now <= 120'000'000; now += 2'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 8;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segment_count = 1;
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.5;
    const auto submitted = endpoint->submit_device_plan(plan, 0, 120'000'000,
        20'000'000, blueprint);
    if (submitted != RK_OK) std::fprintf(stderr, "submit=%d diagnostic=%d\n",
        submitted, endpoint->diagnostic_code());
    assert(submitted == RK_OK);
    if (replace) {
        // Wait for device status proving it received the segment boundary.
        for (std::uint64_t now = 130'000'000; now <= 180'000'000; now += 10'000'000)
            assert(endpoint->sample(now, state) == RK_OK);
        auto late = plan;
        late.plan_id = 9;
        late.replace_after_plan_id = 8;
        late.start_position[0] = 0.25;
        late.segments.segments[0].coefficients[0].value[0] = 0.25;
        assert(endpoint->submit_device_plan(late, 500'000'000, 120'000'000,
            520'000'000, blueprint) == RK_ERROR_INVALID_STATE);
        auto next = plan;
        next.plan_id = 10;
        next.replace_after_plan_id = 8;
        next.start_position[0] = 0.5;
        next.segments.segments[0].coefficients[0].value[0] = 0.5;
        assert(endpoint->submit_device_plan(next, 1'000'000'000, 120'000'000,
            1'020'000'000, blueprint) == RK_OK);
    }
    const auto end_ns = replace ? 2'220'000'000ULL : hold ? 1'420'000'000ULL : 1'220'000'000ULL;
    for (std::uint64_t now = replace ? 190'000'000 : 130'000'000;
         now <= end_ns; now += 10'000'000) {
        if (cut && now == 330'000'000) endpoint->cut_link(true);
        if (hold && (now == 330'000'000 || now == 530'000'000)) {
            rk_robot_command command{};
            command.kind = now == 330'000'000 ? RK_COMMAND_HOLD : RK_COMMAND_RESUME;
            assert(endpoint->apply(command) == RK_OK);
        }
        assert(endpoint->sample(now, state) == RK_OK);
    }
    const auto positions = endpoint->actuator_positions();
    assert(positions.size() == 1);
    assert(std::abs(state.position[0] - positions[0]) <= 1e-6);
    return {positions[0], state, endpoint->step_log()};
}

void dual_drive_layout() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -1;
    blueprint.joints[0].upper_limit = 1;
    blueprint.joints[0].max_velocity = 0.02;
    blueprint.joints[0].max_acceleration = 1;
    VirtualDeviceConfig6 config;
    config.fingerprint.fill(7);
    config.clock_bound_ns = 5'000'000;
    config.actuators = {{0, 1.0, 0.0, 400'000.0, 0.02, 2, 4e-6},
                        {0, 2.0, 0.0, 400'000.0, 0.04, 2, 4e-6}};
    config.actuators[0].id = "gantry.left";
    config.actuators[1].id = "gantry.right";
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    auto changed = config;
    changed.actuators[1].ratio = 3.0;
    auto other = VirtualDeviceEndpoint::create(blueprint, changed);
    assert(other && endpoint->fingerprint() != other->fingerprint());
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 50;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segment_count = 1;
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.01;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 1'220'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    auto positions = endpoint->actuator_positions();
    assert(positions.size() == 2);
    assert(std::abs(positions[0] - 0.01) <= 1.0 / 400'000 + 1e-6);
    assert(std::abs(positions[1] - 0.02) <= 1.0 / 400'000 + 1e-6);
    assert(std::abs(state.position[0] - 0.01) <= 1.0 / 400'000 + 1e-6);
    int first_steps = 0, second_steps = 0;
    for (const auto &record : endpoint->step_log()) {
        if (record.actuator == 0) ++first_steps;
        if (record.actuator == 1) ++second_steps;
    }
    assert(std::abs(first_steps - 4'000) <= 1);
    assert(std::abs(second_steps - 8'000) <= 1);

    auto missed = VirtualDeviceEndpoint::create(blueprint, config);
    assert(missed);
    missed->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        missed->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        missed->sample(now, state);
    assert(missed->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 320'000'000; now += 10'000'000)
        assert(missed->sample(now, state) == RK_OK);
    assert(missed->miss_next_steps(1, 100));
    for (std::uint64_t now = 330'000'000; now <= 380'000'000; now += 10'000'000)
        assert(missed->sample(now, state) == RK_OK);
    assert(state.safety == RK_SAFETY_FAULT);
    assert(missed->diagnostic_code() == RK_FAULT_DUAL_DRIVE_SKEW);
}

void lead_screw_carriage_coupling() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 2;
    blueprint.owner_period_ns = 10'000'000;
    for (auto &joint : blueprint.joints) {
        joint.lower_limit = -10;
        joint.upper_limit = 10;
        joint.max_velocity = 10;
        joint.max_acceleration = 10;
    }
    // One screw revolution advances the carriage by 8 mm, from a 1 mm offset.
    blueprint.coupling_count = 1;
    blueprint.couplings[0] = {0, 1, 0.008, 0.001};
    VirtualDeviceConfig6 config;
    config.fingerprint.fill(9);
    config.clock_bound_ns = 5'000'000;
    config.actuators = {{0, 1.0, 0.0, 3'200.0, 1.0},
                        {1, 1.0, 0.001, 400'000.0, 0.01}};
    config.actuators[0].id = "screw";
    config.actuators[1].id = "carriage";
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 55;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.start_position[1] = 0.001;
    plan.segments.segment_count = 1;
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 2;
    segment.coefficients[0].value[1] = 0.01;
    segment.coefficients[1].value[0] = 0.001;
    segment.coefficients[1].value[1] = 0.00008;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 1'220'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    const auto positions = endpoint->actuator_positions();
    assert(positions.size() == 2);
    int screw_steps = 0, carriage_steps = 0;
    for (const auto &record : endpoint->step_log()) {
        if (record.actuator == 0) ++screw_steps;
        if (record.actuator == 1) ++carriage_steps;
    }
    assert(std::abs(screw_steps - 32) <= 1);
    assert(std::abs(carriage_steps - 32) <= 1);
    assert(std::abs(0.001 + positions[1] - (0.001 + 0.008 * positions[0])) <=
        1.0 / 400'000 + 0.008 / 3'200);

    auto invalid = plan;
    invalid.plan_id = 56;
    invalid.segments.segments[0].coefficients[1].value[1] += 0.001;
    auto fresh = VirtualDeviceEndpoint::create(blueprint, config);
    assert(fresh);
    fresh->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        fresh->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        fresh->sample(now, state);
    assert(fresh->submit_device_plan(invalid, 0, 120'000'000, 20'000'000,
        blueprint) != RK_OK);
}

void minimal_midstream_replacement() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 1;
    blueprint.joints[0].max_acceleration = 10;
    VirtualDeviceConfig6 config;
    config.profile = 2;
    config.fingerprint.fill(4);
    config.clock_bound_ns = 5'000'000;
    config.steps_per_unit = {1'000};
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 70;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segment_count = 1;
    plan.segments.segments[0].duration_ns = 1'000'000'000;
    plan.segments.segments[0].degree = 1;
    plan.segments.segments[0].joint_count = 1;
    plan.segments.segments[0].coefficients[0].value[1] = 0.5;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 320'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    const auto boundary = state.committed_until_ns;
    assert(boundary >= 80'000'000 && boundary < 1'000'000'000);
    auto replacement = plan;
    replacement.plan_id = 71;
    replacement.replace_after_plan_id = 70;
    replacement.start_position[0] = 0.5 * static_cast<double>(boundary) / 1e9;
    replacement.start_velocity[0] = 0.5;
    replacement.segments.segments[0].duration_ns = 300'000'000;
    replacement.segments.segments[0].coefficients[0].value[0] =
        replacement.start_position[0];
    assert(endpoint->submit_device_plan(replacement, boundary, 320'000'000,
        boundary + 20'000'000, blueprint) == RK_OK);
    for (std::uint64_t now = 330'000'000; now <= 1'700'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(state.safety == RK_SAFETY_READY);
    assert(std::abs(state.position[0] - (replacement.start_position[0] + 0.15)) < 1e-4);
}

std::vector<VirtualEventRecord6> run_event_pair(bool hold, bool stop) {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 1;
    blueprint.joints[0].max_acceleration = 10;
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    VirtualDeviceConfig6 config;
    config.fingerprint.fill(8);
    config.steps_per_unit = {1'000};
    config.clock_bound_ns = 5'000'000;
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    rk_plan_submission plan{};
    plan.struct_size = sizeof(plan);
    plan.plan_id = 60;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segment_count = 1;
    plan.segments.segments[0].duration_ns = 1'000'000'000;
    plan.segments.segments[0].degree = 1;
    plan.segments.segments[0].joint_count = 1;
    plan.segments.segments[0].coefficients[0].value[1] = 0.5;
    plan.event_count = 2;
    plan.events[0].time_ns = 250'000'000;
    std::strcpy(plan.events[0].channel, "sprayer.flow");
    plan.events[0].value.kind = RK_EVENT_DIGITAL;
    plan.events[0].value.digital = 1;
    plan.events[0].hold_policy = RK_EVENT_RESTORE_ON_RESUME;
    plan.events[1] = plan.events[0];
    plan.events[1].time_ns = 750'000'000;
    plan.events[1].value.digital = 0;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 500'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(endpoint->event_log().size() == 1);
    assert(endpoint->channel_values()[0] == 1.0f);
    if (hold || stop) {
        rk_robot_command command{};
        command.kind = hold ? RK_COMMAND_HOLD : RK_COMMAND_STOP;
        assert(endpoint->apply(command) == RK_OK);
    }
    for (std::uint64_t now = 510'000'000; now <= 900'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    if (hold) {
        assert(endpoint->event_log().size() == 1);
        assert(endpoint->channel_values()[0] == 0.0f);
        rk_robot_command resume{};
        resume.kind = RK_COMMAND_RESUME;
        assert(endpoint->apply(resume) == RK_OK);
    }
    for (std::uint64_t now = 910'000'000; now <= 1'900'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    auto events = endpoint->event_log();
    assert(events.size() == (stop ? 1u : 2u));
    assert(endpoint->channel_values()[0] == 0.0f);
    for (const auto &event : events) {
        assert(event.plan_id == 60 && event.channel == 0 && event.kind == RK_EVENT_DIGITAL);
        assert(event.applied_path_ticks >= event.scheduled_path_ticks);
        assert(event.applied_path_ticks - event.scheduled_path_ticks <= 25);
    }
    return events;
}

int main() {
    dual_drive_layout();
    lead_screw_carriage_coupling();
    minimal_midstream_replacement();
    const auto ordinary_events = run_event_pair(false, false);
    const auto held_events = run_event_pair(true, false);
    run_event_pair(false, true);
    assert(held_events[1].device_ticks > ordinary_events[1].device_ticks);
    VirtualDeviceConfig6 config;
    config.fingerprint.fill(7);
    config.steps_per_unit = {1'000};
    config.clock_bound_ns = 5'000'000;
    const auto baseline = run(config);
    assert(std::abs(baseline.position - 0.5) <= 0.00101);
    assert(baseline.state.safety == RK_SAFETY_READY);
    assert(!baseline.steps.empty());
    assert(baseline.steps == run(config).steps);
    auto minimal = config;
    minimal.profile = 2;
    const auto setpoint_run = run(minimal);
    assert(std::abs(setpoint_run.position - 0.5) <= 1e-5);
    assert(setpoint_run.steps.empty());
    const auto held_setpoints = run(minimal, false, false, true);
    assert(std::abs(held_setpoints.position - 0.5) <= 1e-5);
    const auto lost_link = run(minimal, true);
    assert(lost_link.position < 0.5 && lost_link.state.safety == RK_SAFETY_FAULT);
    const auto replaced = run(config, false, true);
    assert(std::abs(replaced.position - 1.0) <= 0.00101);
    assert(replaced.state.safety == RK_SAFETY_READY);
    config.drift_ppm = 2'000;
    config.jitter_ns = 500'000;
    config.frame_drop_rate = 0.05;
    const auto disturbed = run(config);
    assert(std::abs(disturbed.position - 0.5) <= 0.00101);
    assert(disturbed.state.safety == RK_SAFETY_READY);
    assert(disturbed.steps == run(config).steps);
    const auto interrupted = run(VirtualDeviceConfig6{.fingerprint = config.fingerprint,
        .steps_per_unit = config.steps_per_unit, .clock_bound_ns = config.clock_bound_ns}, true);
    assert(interrupted.position < 0.5);
    assert(interrupted.state.safety == RK_SAFETY_FAULT);

    config.drift_ppm = 0;
    config.jitter_ns = 0;
    config.frame_drop_rate = 0;

    mk_state_to_state_request request{};
    request.struct_size = sizeof(request);
    request.joint_count = 1;
    request.synchronization = MK_SYNCHRONIZATION_TIME;
    request.target_position[0] = 0.5;
    request.max_velocity[0] = 1.0;
    request.max_acceleration[0] = 2.0;
    request.max_jerk[0] = 5.0;
    mk_trajectory_handle trajectory{};
    int32_t generator_result = 0;
    assert(mk_generate_state_to_state(&request, &trajectory, &generator_result) == MK_OK);
    assert(trajectory.id != 0);
    std::uint32_t count = 0;
    std::int64_t duration_ns = 0;
    assert(mk_trajectory_segment_count(trajectory, &count) == MK_OK);
    assert(mk_trajectory_duration_ns(trajectory, &duration_ns) == MK_OK);
    assert(count > 0 && count <= RK_MAX_TRAJECTORY_SEGMENTS);
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 10;
    blueprint.joints[0].max_acceleration = 10;
    for (const auto profile : {1, 2}) {
        config.profile = profile;
        config.target_error = profile == 2 ? 0.002 : 1e-5;
        auto ruckig_device = VirtualDeviceEndpoint::create(blueprint, config);
        assert(ruckig_device);
        rk_robot_state state{};
        assert(ruckig_device->sample(0, state) == RK_ERROR_STALE_STATE);
        for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
            assert(ruckig_device->sample(now, state) == RK_OK);
        for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
            assert(ruckig_device->sample(now, state) == RK_OK);
        rk_plan_submission plan{};
        plan.struct_size = sizeof(plan);
        plan.plan_id = 40;
        plan.sequence = 1;
        plan.ends_at_rest = 1;
        plan.segments.segment_count = count;
        for (std::uint32_t i = 0; i < count; ++i) {
            mk_segment segment{};
            segment.struct_size = sizeof(segment);
            assert(mk_trajectory_get_segment(trajectory, i, &segment) == MK_OK);
            auto &target = plan.segments.segments[i];
            target.time_from_start_ns = segment.t0_ns;
            target.duration_ns = segment.duration_ns;
            target.degree = segment.degree;
            target.joint_count = 1;
            for (std::uint32_t d = 0; d <= segment.degree; ++d)
                target.coefficients[0].value[d] = segment.coefficients[0].value[d];
        }
        assert(ruckig_device->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
            blueprint) == RK_OK);
        for (std::uint64_t now = 130'000'000;
             now <= 120'000'000 + static_cast<std::uint64_t>(duration_ns) + 100'000'000;
             now += 10'000'000) {
            assert(ruckig_device->sample(now, state) == RK_OK);
            if (state.trajectory_active && state.trajectory_time_ns <= static_cast<std::uint64_t>(duration_ns)) {
                mk_trajectory_state reference{};
                reference.struct_size = sizeof(reference);
                assert(mk_trajectory_evaluate(trajectory, state.trajectory_time_ns, &reference) == MK_OK);
                if (std::abs(state.position[0] - reference.position[0]) > 0.002)
                    std::fprintf(stderr, "profile=%d t=%llu actual=%f expected=%f safety=%d\n", profile,
                        static_cast<unsigned long long>(state.trajectory_time_ns), state.position[0],
                        reference.position[0], state.safety);
                assert(std::abs(state.position[0] - reference.position[0]) <= 0.002);
            }
        }
        assert(std::abs(ruckig_device->actuator_positions()[0] - 0.5) <= 0.00101);
        assert(state.safety == RK_SAFETY_READY);
        if (profile == 2) assert(ruckig_device->step_log().empty());
    }
    mk_trajectory_destroy(trajectory);
}
