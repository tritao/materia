#include "virtual_device_endpoint.hpp"
#include "robotkit_runtime.hpp"
#include "motionkit.h"
#include <cassert>
#include <chrono>
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
    bool hold = false, bool append = false, bool resynced = false) {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 10);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
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
    robotkit::PlanRequest plan{};
    plan.plan_id = 8;
    plan.sequence = 1;
    plan.ends_at_rest = append ? 0 : 1;
    plan.segments.segments.resize(1);
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
        // Wait for device status proving it received the segment boundary, and,
        // when resynced, for time syncs to refine the clock past what the plan used.
        const std::uint64_t settled_ns = resynced ? 380'000'000 : 180'000'000;
        for (std::uint64_t now = 130'000'000; now <= settled_ns; now += 10'000'000)
            assert(endpoint->sample(now, state) == RK_OK);
        auto late = plan;
        late.plan_id = 9;
        late.replace_after_plan_id = 8;
        late.start_position[0] = 0.25;
        late.segments.segments[0].coefficients[0].value[0] = 0.25;
        assert(endpoint->submit_device_plan(late, 500'000'000, settled_ns + 10'000'000,
            520'000'000, blueprint) == RK_ERROR_INVALID_STATE);
        auto next = plan;
        next.plan_id = 10;
        next.replace_after_plan_id = 8;
        next.start_position[0] = 0.5;
        next.segments.segments[0].coefficients[0].value[0] = 0.5;
        assert(endpoint->submit_device_plan(next, 1'000'000'000, settled_ns + 10'000'000,
            1'020'000'000, blueprint) == RK_OK);
    }
    if (append) {
        // Time syncs refine the clock before the continuation arrives; it must still
        // start exactly where the queued path ends on the device.
        for (std::uint64_t now = 130'000'000; now <= 380'000'000; now += 10'000'000)
            assert(endpoint->sample(now, state) == RK_OK);
        auto next = plan;
        next.plan_id = 10;
        next.ends_at_rest = 1;
        next.start_position[0] = 0.5;
        next.start_velocity[0] = 0.5;
        next.segments.segments[0].coefficients[0].value[0] = 0.5;
        assert(endpoint->submit_device_plan(next, 1'000'000'000, 390'000'000,
            290'000'000, blueprint) == RK_OK);
    }
    const auto end_ns = replace || append ? 2'220'000'000ULL :
        hold ? 1'420'000'000ULL : 1'220'000'000ULL;
    for (std::uint64_t now = replace ? (resynced ? 390'000'000 : 190'000'000) :
            append ? 390'000'000 : 130'000'000;
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
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 0.02);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 1);
    VirtualDeviceConfig6 config;
    config.controller.fill(7);
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
    assert(other && endpoint->controller() == other->controller());
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    robotkit::PlanRequest plan{};
    plan.plan_id = 50;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.01;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 1'220'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(!state.trajectory_active);
    assert(state.trajectory_tag == 50);
    assert(state.trajectory_tag_time_ns >= 1'000'000'000);
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

void motor_feedback_reconstructs_leaders() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 5;
    blueprint.owner_period_ns = 10'000'000;
    for (auto &joint : blueprint.joints) {
        joint.lower_limit = -10;
        joint.upper_limit = 10;
        joint.max_velocity = 10;
        joint.max_acceleration = 10;
        joint.limit_flags = RK_LIMIT_VELOCITY | RK_LIMIT_ACCELERATION;
    }
    blueprint.coupling_count = 5;
    blueprint.couplings[0] = {0, 2, 2.0, 0.02};
    blueprint.couplings[1] = {1, 2, 3.0, 0.0};
    blueprint.couplings[2] = {0, 3, 2.0, -0.03};
    blueprint.couplings[3] = {1, 3, -3.0, 0.0};
    blueprint.couplings[4] = {2, 4, 0.5, 0.01};
    VirtualDeviceConfig6 config;
    config.clock_bound_ns = 5'000'000;
    config.actuators = {{2, 1.0, 0.02, 100'000.0, 0.1},
                        {3, 1.0, -0.03, 100'000.0, 0.1}};
    config.actuators[0].id = "motor.a";
    config.actuators[1].id = "motor.b";
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    PlanRequest plan{};
    plan.plan_id = 57;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.start_position[2] = 0.02;
    plan.start_position[3] = -0.03;
    plan.start_position[4] = 0.02;
    plan.segments.segments.resize(1);
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 5;
    segment.coefficients[0].value[1] = 0.01;
    segment.coefficients[1].value[1] = 0.02;
    segment.coefficients[2].value[0] = 0.02;
    segment.coefficients[2].value[1] = 0.08;
    segment.coefficients[3].value[0] = -0.03;
    segment.coefficients[3].value[1] = -0.04;
    segment.coefficients[4].value[0] = 0.02;
    segment.coefficients[4].value[1] = 0.04;
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000, blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 1'220'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(std::abs(state.position[0] - 0.01) < 1e-5);
    assert(std::abs(state.position[1] - 0.02) < 1e-5);
    assert(std::abs(state.position[4] - 0.06) < 1e-5);
    const auto joints = endpoint->joint_positions();
    assert(std::abs(joints[0] - 0.01) < 1e-5);
    assert(std::abs(joints[1] - 0.02) < 1e-5);
    assert(std::abs(joints[4] - 0.06) < 1e-5);
    // One motor cannot observe both independent CoreXY coordinates.
    config.actuators.resize(1);
    assert(!VirtualDeviceEndpoint::create(blueprint, config));
}

void lead_screw_carriage_coupling() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 2;
    blueprint.owner_period_ns = 10'000'000;
    for (auto &joint : blueprint.joints) {
        joint.lower_limit = -10;
        joint.upper_limit = 10;
        joint.max_velocity = (joint.limit_flags |= RK_LIMIT_VELOCITY, 10);
        joint.max_acceleration = (joint.limit_flags |= RK_LIMIT_ACCELERATION, 10);
    }
    // One screw revolution advances the carriage by 8 mm, from a 1 mm offset.
    blueprint.coupling_count = 1;
    blueprint.couplings[0] = {0, 1, 0.008, 0.001};
    VirtualDeviceConfig6 config;
    config.controller.fill(9);
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
    robotkit::PlanRequest plan{};
    plan.plan_id = 55;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.start_position[1] = 0.001;
    plan.segments.segments.resize(1);
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
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 1);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    VirtualDeviceConfig6 config;
    config.profile = 2;
    config.controller.fill(4);
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
    robotkit::PlanRequest plan{};
    plan.plan_id = 70;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
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
    assert(state.trajectory_tag == 71);
    assert(state.trajectory_tag_time_ns >= 300'000'000);
    assert(std::abs(state.position[0] - (replacement.start_position[0] + 0.15)) < 1e-4);
}

void host_stall_keeps_device_moving() {
    // The host stops pumping for 300 ms mid-plan, as for a long garbage collection:
    // what it committed beforehand keeps the device moving.
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 10);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    VirtualDeviceConfig6 config;
    config.controller.fill(5);
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
    robotkit::PlanRequest plan{};
    plan.plan_id = 80;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(8);
    for (std::size_t k = 0; k < 8; ++k) {
        auto &segment = plan.segments.segments[k];
        segment.time_from_start_ns = k * 125'000'000;
        segment.duration_ns = 125'000'000;
        segment.degree = 1;
        segment.joint_count = 1;
        segment.coefficients[0].value[0] = 0.0625 * k;
        segment.coefficients[0].value[1] = 0.5;
    }
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000, blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 400'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    // A 300 ms owner stall must fit inside the remaining committed horizon,
    // even when it begins halfway between old replenishment boundaries.
    assert(state.committed_until_ns >= state.trajectory_time_ns + 400'000'000);
    for (std::uint64_t now = 700'000'000; now <= 1'800'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    if (state.safety != RK_SAFETY_READY)
        std::fprintf(stderr, "host stall fault: %s, position=%g, path=%llu, commit=%llu\n",
            endpoint->fault_reason() ? endpoint->fault_reason() : "none", state.position[0],
            static_cast<unsigned long long>(state.trajectory_time_ns),
            static_cast<unsigned long long>(state.committed_until_ns));
    assert(state.safety == RK_SAFETY_READY);
    assert(std::abs(endpoint->actuator_positions()[0] - 0.5) <= 0.00101);

    // A simulation reset rewinds host time while preserving the endpoint object
    // held by Runtime. Recreate the board, stream and clock-sync state together.
    assert(endpoint->reset());
    assert(endpoint->actuator_positions()[0] == 0.0);
    assert(endpoint->step_log().empty());
    assert(endpoint->sample(0, state) == RK_ERROR_STALE_STATE);
    for (std::uint64_t now = 2'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000, blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 1'400'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(state.safety == RK_SAFETY_READY);
    assert(std::abs(endpoint->actuator_positions()[0] - 0.5) <= 0.00101);
}

void midsegment_replacement_keeps_events() {
    // A replacement inside a segment the device holds: the endpoint reopens the queue at
    // that segment's start, sends it again cut short at the boundary, and sends again the
    // events the replaced plan scheduled in that stretch. The clock drifts, so syncs move
    // the estimate between the plan and its replacement.
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 1);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    VirtualDeviceConfig6 config;
    config.controller.fill(8);
    config.steps_per_unit = {1'000};
    config.clock_bound_ns = 5'000'000;
    config.drift_ppm = 2'000;
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    rk_robot_state state{};
    endpoint->sample(0, state);
    for (std::uint64_t now = 2'000'000; now <= 20'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    for (std::uint64_t now = 100'000'000; now <= 120'000'000; now += 2'000'000)
        endpoint->sample(now, state);
    robotkit::PlanRequest plan{};
    plan.plan_id = 60;
    plan.sequence = 1;
    plan.segments.segments.resize(8);
    for (std::size_t k = 0; k < 8; ++k) {
        auto &segment = plan.segments.segments[k];
        segment.time_from_start_ns = k * 250'000'000;
        segment.duration_ns = 250'000'000;
        segment.degree = 1;
        segment.joint_count = 1;
        segment.coefficients[0].value[0] = 0.125 * k;
        segment.coefficients[0].value[1] = 0.5;
    }
    auto event = [](std::uint64_t time_ns, bool on) {
        rk_timed_event value{};
        value.time_ns = time_ns;
        std::strcpy(value.channel, "sprayer.flow");
        value.value.kind = RK_EVENT_DIGITAL;
        value.value.digital = on ? 1 : 0;
        return value;
    };
    plan.events = {event(1'300'000'000, true), event(1'450'000'000, false)};
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000, blueprint) == RK_OK);
    for (std::uint64_t now = 130'000'000; now <= 380'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    robotkit::PlanRequest next{};
    next.plan_id = 61;
    next.sequence = 2;
    next.ends_at_rest = 1;
    next.replace_after_plan_id = 60;
    next.start_position[0] = 0.6875;
    next.start_velocity[0] = 0.5;
    next.segments.segments.resize(1);
    next.segments.segments[0].duration_ns = 500'000'000;
    next.segments.segments[0].degree = 1;
    next.segments.segments[0].joint_count = 1;
    next.segments.segments[0].coefficients[0].value[0] = 0.6875;
    next.segments.segments[0].coefficients[0].value[1] = 0.5;
    next.events = {event(100'000'000, false)};
    // 1.375 s lies inside the sixth segment, beyond what is committed half a second ahead.
    assert(endpoint->submit_device_plan(next, 1'375'000'000, 390'000'000, 410'000'000,
        blueprint) == RK_OK);
    for (std::uint64_t now = 390'000'000; now <= 2'800'000'000; now += 10'000'000)
        assert(endpoint->sample(now, state) == RK_OK);
    assert(state.safety == RK_SAFETY_READY);
    assert(std::abs(endpoint->actuator_positions()[0] - 0.9375) <= 0.00101);
    const auto events = endpoint->event_log();
    assert(events.size() == 2 && events[0].plan_id == 60 && events[0].digital == 1 &&
        events[1].plan_id == 61 && events[1].digital == 0);
    assert(endpoint->channel_values()[0] == 0.0f);
}

/**
 * A plan switches a channel on then off; `hold` or `stop` interrupts it while on. With
 * `keep_on_stop` the channel's output survives a commanded stop on the device, as a vacuum holding a
 * part must, while an `emergency` stop still makes it safe.
 */
std::vector<VirtualEventRecord6> run_event_pair(bool hold, bool stop, bool keep_on_stop = false,
                                                bool emergency = false) {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 1);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    if (keep_on_stop) blueprint.channels[0].stop_policy = RK_CHANNEL_KEEP_ON_STOP;
    VirtualDeviceConfig6 config;
    config.controller.fill(8);
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
    robotkit::PlanRequest plan{};
    plan.plan_id = 60;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    plan.segments.segments[0].duration_ns = 1'000'000'000;
    plan.segments.segments[0].degree = 1;
    plan.segments.segments[0].joint_count = 1;
    plan.segments.segments[0].coefficients[0].value[1] = 0.5;
    plan.events.resize(2);
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
        command.kind = hold ? RK_COMMAND_HOLD : emergency ? RK_COMMAND_EMERGENCY_STOP : RK_COMMAND_STOP;
        assert(endpoint->apply(command) == RK_OK);
    }
    // An emergency stop latches a fault the samples then report.
    for (std::uint64_t now = 510'000'000; now <= 900'000'000; now += 10'000'000) {
        const auto status = endpoint->sample(now, state);
        assert(emergency || status == RK_OK);
    }
    if (hold) {
        assert(endpoint->event_log().size() == 1);
        assert(endpoint->channel_values()[0] == 0.0f);
        rk_robot_command resume{};
        resume.kind = RK_COMMAND_RESUME;
        assert(endpoint->apply(resume) == RK_OK);
    }
    for (std::uint64_t now = 910'000'000; now <= 1'900'000'000; now += 10'000'000) {
        const auto status = endpoint->sample(now, state);
        assert(emergency || status == RK_OK);
    }
    auto events = endpoint->event_log();
    assert(events.size() == (stop ? 1u : 2u));
    assert(endpoint->channel_values()[0] == (stop && keep_on_stop && !emergency ? 1.0f : 0.0f));
    for (const auto &event : events) {
        assert(event.plan_id == 60 && event.channel == 0 && event.kind == RK_EVENT_DIGITAL);
        assert(event.applied_path_ticks >= event.scheduled_path_ticks);
        assert(event.applied_path_ticks - event.scheduled_path_ticks <= 25);
    }
    return events;
}

void runtime_hold_rest_resume_fires_final_event() {
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.revision = 1;
    blueprint.joint_count = 1;
    blueprint.link_count = 2;
    blueprint.owner_period_ns = 10'000'000;
    for (std::uint32_t i = 0; i < blueprint.link_count; ++i) {
        auto &link = blueprint.links[i];
        link.mass = 1.0;
        link.inertia_tensor[0] = link.inertia_tensor[4] = link.inertia_tensor[8] = 1.0;
    }
    auto &joint = blueprint.joints[0];
    joint.type = RK_RUNTIME_JOINT_PRISMATIC;
    joint.parent_link = 0;
    joint.child_link = 1;
    joint.parent_frame_rotation[3] = joint.child_frame_rotation[3] = 1.0;
    joint.axis[0] = 1.0;
    joint.lower_limit = -1.0;
    joint.upper_limit = 1.0;
    joint.max_velocity = (joint.limit_flags |= RK_LIMIT_VELOCITY, 1.0);
    joint.max_acceleration = (joint.limit_flags |= RK_LIMIT_ACCELERATION, 2.0);
    blueprint.channel_count = 1;
    std::strcpy(blueprint.channels[0].id, "sprayer.flow");
    blueprint.channels[0].kind = RK_EVENT_DIGITAL;
    blueprint.channels[0].safe_value.kind = RK_EVENT_DIGITAL;

    VirtualDeviceConfig6 config;
    config.controller.fill(11);
    config.steps_per_unit = {1'000};
    config.clock_bound_ns = 5'000'000;
    auto endpoint = VirtualDeviceEndpoint::create(blueprint, config);
    assert(endpoint);
    robotkit::RobotRuntime runtime(blueprint, endpoint,
        std::chrono::milliseconds(10));
    runtime.set_externally_driven(true);
    std::uint64_t now = 0;
    auto cycle = [&] {
        now += 10'000'000;
        assert(runtime.apply_pending_commands(now) == RK_OK);
        assert(runtime.publish_sample(now) == RK_OK);
        rk_robot_snapshot snapshot{};
        assert(runtime.snapshot_full(snapshot) == RK_OK);
        return snapshot;
    };
    for (int i = 0; i < 15; ++i) cycle();

    robotkit::PlanRequest plan{};
    plan.sequence = 1;
    plan.plan_id = 80;
    plan.model_revision = blueprint.revision;
    plan.required_capabilities = RK_PLAN_CAPABILITY_EVENTS;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    plan.segments.tag = 80;
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.5;
    plan.events.resize(2);
    std::strcpy(plan.events[0].channel, "sprayer.flow");
    plan.events[0].time_ns = 250'000'000;
    plan.events[0].value.kind = RK_EVENT_DIGITAL;
    plan.events[0].value.digital = 1;
    plan.events[0].hold_policy = RK_EVENT_RESTORE_ON_RESUME;
    plan.events[1] = plan.events[0];
    plan.events[1].time_ns = segment.duration_ns;
    plan.events[1].value.digital = 0;
    assert(runtime.submit_plan(plan) == RK_OK);

    rk_robot_snapshot snapshot{};
    for (int i = 0; i < 40; ++i) snapshot = cycle();
    assert(snapshot.trajectory_active != 0);
    assert(runtime.submit({.struct_size = sizeof(rk_robot_command),
        .sequence = 2, .kind = RK_COMMAND_HOLD}) == RK_OK);
    bool held = false;
    for (int i = 0; i < 100; ++i) {
        snapshot = cycle();
        if (snapshot.session_state == RK_SESSION_HELD) {
            held = true;
            break;
        }
    }
    assert(held && snapshot.trajectory_active != 0);
    const auto held_time = snapshot.trajectory_time_ns;
    // The endpoint must stay held too. Without runtime lifecycle forwarding,
    // its clock reaches the final event while the host still reports Held.
    for (int i = 0; i < 100; ++i) snapshot = cycle();
    assert(snapshot.session_state == RK_SESSION_HELD);
    assert(snapshot.trajectory_time_ns == held_time);
    assert(endpoint->event_log().size() == 1);
    assert(endpoint->channel_values()[0] == 0.0f);
    assert(runtime.submit({.struct_size = sizeof(rk_robot_command),
        .sequence = 3, .kind = RK_COMMAND_RESUME}) == RK_OK);

    bool completed = false;
    for (int i = 0; i < 200; ++i) {
        snapshot = cycle();
        if (!snapshot.trajectory_active && snapshot.trajectory_queue_depth == 0) {
            completed = true;
            break;
        }
    }
    assert(completed && snapshot.fault_code == 0 && snapshot.trajectory_tag == 80);
    // Finished means the runtime's copy of the queue has run out too: the setpoint is the
    // plan's end, and the next plan starts afresh rather than joining a finished queue.
    assert(std::abs(snapshot.setpoint_position[0] - 0.5) < 1e-12);
    for (int i = 0; i < 20; ++i) cycle();
    const auto events = endpoint->event_log();
    assert(events.size() == 2);
    assert(events[0].plan_id == 80 && events[0].digital == 1);
    assert(events[1].plan_id == 80 && events[1].digital == 0);
    assert(events[1].scheduled_path_ticks > events[0].scheduled_path_ticks);
    assert(events[1].applied_path_ticks >= events[1].scheduled_path_ticks);
    assert(endpoint->channel_values()[0] == 0.0f);
}

int main() {
    motor_feedback_reconstructs_leaders();
    dual_drive_layout();
    lead_screw_carriage_coupling();
    minimal_midstream_replacement();
    const auto ordinary_events = run_event_pair(false, false);
    const auto held_events = run_event_pair(true, false);
    runtime_hold_rest_resume_fires_final_event();
    run_event_pair(false, true);
    run_event_pair(false, true, true);
    run_event_pair(false, true, true, true);
    midsegment_replacement_keeps_events();
    host_stall_keeps_device_moving();
    assert(held_events[1].device_ticks > ordinary_events[1].device_ticks);
    VirtualDeviceConfig6 config;
    config.controller.fill(7);
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
    const auto appended = run(config, false, false, false, true);
    assert(std::abs(appended.position - 1.0) <= 0.00101);
    assert(appended.state.safety == RK_SAFETY_READY);
    // So does a replacement: its boundary is where the queued path has it on the device.
    const auto replaced_disturbed = run(config, false, true, false, false, true);
    assert(std::abs(replaced_disturbed.position - 1.0) <= 0.00101);
    assert(replaced_disturbed.state.safety == RK_SAFETY_READY);
    assert(disturbed.steps == run(config).steps);
    const auto interrupted = run(VirtualDeviceConfig6{.controller = config.controller,
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
    assert(count > 0 && count <= RK_MAX_TRAJECTORY_QUEUE_POINTS);
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = (blueprint.joints[0].limit_flags |= RK_LIMIT_VELOCITY, 10);
    blueprint.joints[0].max_acceleration = (blueprint.joints[0].limit_flags |= RK_LIMIT_ACCELERATION, 10);
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
        robotkit::PlanRequest plan{};
        plan.plan_id = 40;
        plan.sequence = 1;
        plan.ends_at_rest = 1;
        plan.segments.segments.resize(count);
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
