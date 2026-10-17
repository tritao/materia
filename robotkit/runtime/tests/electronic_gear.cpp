#include "robotkit_runtime.hpp"
#include <cassert>
#include <cstring>
#include <memory>
#include <numbers>
#include <iostream>
using robotkit::ElectronicGear;
static rk_event_value value(const char *command, double argument = 0) {
    rk_event_value v{}; v.kind = RK_EVENT_PROCESS;
    std::strncpy(v.command, command, sizeof(v.command) - 1); v.argument = argument; return v;
}
static void engage(ElectronicGear &gear, double ratio, bool predict = true) {
    assert(gear.event(1, value("gear.begin"), 3));
    assert(gear.event(1, value("gear.loop", 4000), 3));
    assert(gear.event(1, value(predict ? "gear.term/0/1000000/1" : "gear.term/0/1000000/0", ratio), 3));
    assert(gear.event(1, value("gear.quantum", 2 * std::numbers::pi / 8388608), 3));
    assert(gear.event(1, value("gear.engage"), 3));
}
class Encoders final : public robotkit::RobotEndpoint {
public:
    double follower_velocity = 0;
    double rate = 100 * std::numbers::pi, follower = 0; // B 3000 rpm, C 15 rpm.
    rk_robot_command last{};
    bool supports_trajectory_queue() const noexcept override { return true; }
    rk_result apply(const rk_robot_command &command) override {
        last = command;
        if (command.kind == RK_COMMAND_JOINT_TARGETS) for (unsigned i = 0; i < command.target_count; ++i)
            if (command.targets[i].joint == 1) {
                if (command.targets[i].mode == RK_TARGET_SERVO && command.servos[i].stiffness <= 0 && command.servos[i].damping <= 0) continue;
                follower = command.targets[i].target; follower_velocity = command.servos[i].velocity;
            }
        return RK_OK;
    }
    rk_result sample(uint64_t t, rk_robot_state &state) override {
        state.struct_size = sizeof(state); state.joint_count = 2; state.source_timestamp_ns = t;
        // Encoder transport delay is explicit, in addition to the owner sample age.
        state.position[0] = rate * (t * 1e-9 - .001); state.velocity[0] = rate;
        state.position[1] = follower; state.velocity[1] = follower_velocity;
        return RK_OK;
    }
};
int main() {
    ElectronicGear gear;
    assert(ElectronicGear::follower("motion.gear/1") == 1);
    assert(ElectronicGear::follower("motion.gear/1oops") == -1);
    assert(!gear.event(1, value("gear.engage"), 3));
    const double omega = 100 * std::numbers::pi, ratio = 1.0 / 200, arcsec = std::numbers::pi / 648000;
    auto measured = std::make_unique<rk_robot_state>(); measured->joint_count = 3;
    double worst = 0;
    engage(gear, ratio);
    for (unsigned tick = 1; tick <= 8000; ++tick) {
        double t = tick / 4000.0;
        measured->position[0] = omega * (t - .001); measured->velocity[0] = omega;
        worst = std::max(worst, std::abs(gear.offset(1, *measured, 0) - ratio * omega * t));
    }
    assert(worst < arcsec);
    engage(gear, ratio, false);
    double lag = ratio * omega * 2 - gear.offset(1, *measured, 0);
    assert(std::abs(lag - ratio * omega * .001) < 2e-9);
    assert(std::abs(lag * .03 * 1e6 - 47.1238898) < .0001);
    engage(gear, 1.0 / 100); // New program, same machine and observations.
    assert(std::abs(gear.offset(1, *measured, 0) - 2 * ratio * omega * 2) < 4e-9);
    // Multiple measured masters and additive motion, without recursively gearing observations.
    assert(gear.event(1, value("gear.begin"), 3));
    assert(gear.event(1, value("gear.loop", 4000), 3));
    assert(gear.event(1, value("gear.term/0/0/1", .01), 3));
    assert(gear.event(1, value("gear.term/2/0/1", -.5), 3));
    assert(gear.event(1, value("gear.engage"), 3));
    measured->position[2] = .3; measured->velocity[2] = .2;
    assert(std::abs(gear.offset(1, *measured, .002) - (.01 * (measured->position[0] + omega * .002) - .5 * .3004)) < 1e-12);
    assert(gear.event(0, value("gear.begin"), 3));
    assert(gear.event(0, value("gear.loop", 4000), 3));
    assert(gear.event(0, value("gear.term/1/0/1", 1), 3));
    assert(!gear.event(0, value("gear.engage"), 3)); // Cycle must not replace the old relation.
    assert(!gear.event(1, value("gear.term/999/0/1", 1), 3));
    assert(gear.event(1, value("gear.release"), 3)); assert(!gear.relations[1].active);

    auto spec = std::make_unique<rk_robot_runtime_blueprint>();
    spec->struct_size = sizeof(*spec); spec->revision = 1; spec->joint_count = 2; spec->link_count = 3;
    for (unsigned i = 0; i < 3; ++i) {
        spec->links[i].mass = 1;
        spec->links[i].inertia_tensor[0] = spec->links[i].inertia_tensor[4] = spec->links[i].inertia_tensor[8] = .01;
    }
    for (unsigned i = 0; i < 2; ++i) {
        spec->joints[i] = {i, RK_RUNTIME_JOINT_REVOLUTE, 0, i + 1, -100000, 100000, 100};
        spec->joints[i].axis[2] = 1;
        spec->joints[i].parent_frame_rotation[3] = spec->joints[i].child_frame_rotation[3] = 1;
        spec->joint_servo[i] = {100, 10};
    }
    spec->channel_count = 1; std::strcpy(spec->channels[0].id, "motion.gear/1");
    spec->channels[0].kind = RK_EVENT_PROCESS; spec->channels[0].safe_value = value("gear.release");
    assert(rk_robot_runtime_blueprint_validate(spec.get()) == RK_OK);
    auto encoders = std::make_shared<Encoders>(); encoders->follower = .2;
    auto runtime = std::make_unique<robotkit::RobotRuntime>(*spec, encoders, std::chrono::microseconds(250));
    assert(runtime->publish_sample(1'000'000) == RK_OK);
    robotkit::PlanRequest plan{}; plan.sequence = 1; plan.plan_id = 1; plan.model_revision = 1;
    plan.required_capabilities = RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE | RK_PLAN_CAPABILITY_EVENTS;
    plan.control_acceleration[0] = plan.control_acceleration[1] = 10000;
    plan.segments.segments.resize(1);
    auto &segment = plan.segments.segments[0]; segment.joint_count = 2; segment.degree = 1; segment.duration_ns = 1'000'000'000;
    segment.coefficients[1].value[0] = .2; plan.start_position[1] = .2;
    for (auto v : {value("gear.begin"), value("gear.loop", 4000), value("gear.term/0/1000000/1", ratio), value("gear.quantum", 2 * std::numbers::pi / 8388608), value("gear.engage")}) {
        rk_timed_event event{}; std::strcpy(event.channel, "motion.gear/1"); event.value = v; plan.events.push_back(event);
    }
    // Simulation's first command epoch is genuinely zero, not an unspecified
    // clock request. Keep transport-delay compensation without adding a tick.
    auto zero_encoders = std::make_shared<Encoders>(); zero_encoders->follower = .2;
    auto zero_runtime = std::make_unique<robotkit::RobotRuntime>(*spec, zero_encoders, std::chrono::microseconds(250));
    assert(zero_runtime->publish_sample(1'000'000) == RK_OK);
    assert(zero_runtime->submit_plan(plan) == RK_OK);
    assert(zero_runtime->apply_pending_commands(0) == RK_OK);
    assert(std::abs(zero_encoders->follower - .2 - ratio * omega * .001) < arcsec);
    assert(runtime->submit_plan(plan) == RK_OK);
    for (uint64_t tick = 5; tick < 405; ++tick) {
        assert(runtime->apply_pending_commands(tick * 250000) == RK_OK);
        assert(std::abs(encoders->follower - .2 - ratio * omega * tick / 4000.0) < arcsec);
        assert(runtime->publish_sample(tick * 250000) == RK_OK);
    }
    // Feed hold stops additive travel while C remains synchronized to a running process master.
    rk_robot_command lifecycle{}; lifecycle.struct_size = sizeof(lifecycle);
    lifecycle.sequence = 2; lifecycle.kind = RK_COMMAND_HOLD;
    assert(runtime->submit(lifecycle) == RK_OK);
    for (uint64_t tick = 405; tick < 425; ++tick) {
        assert(runtime->apply_pending_commands(tick * 250000) == RK_OK);
        assert(std::abs(encoders->follower - .2 - ratio * omega * tick / 4000.0) < arcsec);
        assert(runtime->publish_sample(tick * 250000) == RK_OK);
    }
    lifecycle.sequence = 3; lifecycle.kind = RK_COMMAND_STOP;
    assert(runtime->submit(lifecycle) == RK_OK);
    double before_stop = encoders->follower;
    assert(runtime->apply_pending_commands(425 * 250000) == RK_OK);
    assert(std::abs(encoders->follower - before_stop) < .01); // no jump back to additive zero
    auto records = std::make_unique<rk_event_record_batch>(); records->struct_size = sizeof(*records);
    assert(runtime->poll_events(*records) == RK_OK); assert(records->count > 0);
    bool saw_sync = false;
    for (unsigned i = 0; i < records->count; ++i) if (!std::strcmp(records->records[i].value.command, "sync")) { saw_sync = true; assert(std::abs(records->records[i].value.argument) < arcsec); }
    assert(saw_sync);
    // Reference servo: assumed J=.02 kg m², 200 Hz critically damped position response,
    // sampled at the declared 4 kHz, already at 15 rpm when the steady cutting interval starts.
    auto servo_error = [&](bool predict) {
        ElectronicGear relation; engage(relation, ratio, predict);
        double p = 0, v = ratio * omega, worst_error = 0;
        const double step = .00025, bandwidth = 2 * std::numbers::pi * 200, inertia = .02;
        for (unsigned tick = 0; tick < 8000; ++tick) {
            const double t = tick * step;
            measured->position[0] = omega * (t - .001); measured->velocity[0] = omega;
            const double target = relation.offset(1, *measured, 0);
            const double torque = inertia * (bandwidth * bandwidth * (target - p) +
                2 * bandwidth * (relation.velocity(1, *measured) - v));
            v += torque / inertia * step; p += v * step;
            if (tick > 4000) worst_error = std::max(worst_error, std::abs(p - ratio * omega * (t + step)));
        }
        return worst_error;
    };
    const double servo_prediction = servo_error(true), servo_lag = servo_error(false);
    assert(servo_prediction < arcsec);
    assert(std::abs(servo_lag - ratio * omega * .001) < 1e-8);
    std::cout << "Reference servo sync " << servo_prediction / arcsec << " arcsec; disabled prediction " << servo_lag * .03 * 1e6 << " um\n";
    // Invalid partial packets and a slow owner are rejected before changing the queue.
    auto bad_plan = plan; bad_plan.sequence = 4; bad_plan.plan_id = 4;
    bad_plan.events.pop_back();
    assert(runtime->submit_plan(bad_plan) == RK_ERROR_INVALID_ARGUMENT);
    auto slow = std::make_unique<robotkit::RobotRuntime>(*spec, std::make_shared<Encoders>(), std::chrono::milliseconds(1));
    assert(slow->submit_plan(plan) == RK_ERROR_LIMIT);
    auto invalid_spec = std::make_unique<rk_robot_runtime_blueprint>(*spec);
    invalid_spec->channels[0].safe_value = value("gear.engage");
    assert(rk_robot_runtime_blueprint_validate(invalid_spec.get()) == RK_ERROR_INVALID_ARGUMENT);
    // A drained position plan still has an independently running measured master.
    auto barrier_encoders = std::make_shared<Encoders>(); barrier_encoders->follower = .2;
    auto barrier = std::make_unique<robotkit::RobotRuntime>(*spec, barrier_encoders, std::chrono::microseconds(250));
    assert(barrier->publish_sample(1'000'000) == RK_OK);
    auto short_plan = plan; short_plan.ends_at_rest = true;
    short_plan.segments.segments[0].duration_ns = 10'000'000;
    assert(barrier->submit_plan(short_plan) == RK_OK);
    for (uint64_t tick = 5; tick < 100; ++tick) {
        assert(barrier->apply_pending_commands(tick * 250000) == RK_OK);
        assert(std::abs(barrier_encoders->follower - .2 - ratio * omega * tick / 4000.0) < arcsec);
        assert(barrier->publish_sample(tick * 250000) == RK_OK);
    }
    short_plan.sequence = 2; short_plan.plan_id = 2;
    assert(barrier->submit_plan(short_plan) == RK_OK); // additive anchor, despite physical C advancing
    // A computed-torque follower has no physical PD gains. Empty-queue gear
    // following must retain position mode rather than create a zero-gain servo.
    auto computed_spec=std::make_unique<rk_robot_runtime_blueprint>(*spec);computed_spec->joint_servo[1]={};
    auto computed_encoders=std::make_shared<Encoders>();computed_encoders->follower=.2;
    auto computed=std::make_unique<robotkit::RobotRuntime>(*computed_spec,computed_encoders,std::chrono::microseconds(250));
    assert(computed->publish_sample(1'000'000)==RK_OK);
    short_plan.sequence=1;short_plan.plan_id=1;assert(computed->submit_plan(short_plan)==RK_OK);
    for(uint64_t tick=5;tick<100;++tick){
      assert(computed->apply_pending_commands(tick*250000)==RK_OK);
      assert(std::abs(computed_encoders->follower-.2-ratio*omega*tick/4000.0)<arcsec);
      assert(computed->publish_sample(tick*250000)==RK_OK);
    }
    assert(computed_encoders->last.targets[0].mode==RK_TARGET_POSITION);
    std::cout << "Electronic gear: encoder prediction worst " << worst / arcsec << " arcsec; uncompensated " << lag * .03 * 1e6 << " um; owner events and per-tick trace passed\n";
}
