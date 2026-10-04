#include "virtual_device_endpoint.hpp"
#include "robotkit_runtime.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>
using namespace robotkit;

static void run(int stop) {
    auto blueprint = std::make_unique<rk_robot_runtime_blueprint>();
    blueprint->struct_size = sizeof(*blueprint);
    blueprint->joint_count = 1;
    blueprint->owner_period_ns = 10'000'000;
    blueprint->joints[0].lower_limit = -10;
    blueprint->joints[0].upper_limit = 10;
    blueprint->joints[0].max_velocity = 1;
    blueprint->joints[0].max_acceleration = 10;
    blueprint->joints[0].limit_flags = RK_LIMIT_VELOCITY | RK_LIMIT_ACCELERATION;
    blueprint->channel_count = 3;
    const char *names[] = {"torch", "wire", "voltage"};
    for (int i = 0; i < 3; ++i) {
        std::strcpy(blueprint->channels[i].id, names[i]);
        blueprint->channels[i].kind = i == 0 ? RK_EVENT_DIGITAL : RK_EVENT_ANALOG;
        blueprint->channels[i].safe_value.kind = blueprint->channels[i].kind;
        blueprint->channels[i].stop_policy = i == 2 ? RK_CHANNEL_KEEP_ON_STOP : RK_CHANNEL_SAFE_ON_STOP;
    }
    VirtualDeviceConfig6 config;
    config.peripheral_kind = 1;
    config.external_sensor_mask = 1;
    config.peripheral_parameters = {0, 0, 1, 2, 0.002, 0.1, 0.9};
    config.clock_bound_ns = 5'000'000;
    auto endpoint = VirtualDeviceEndpoint::create(*blueprint, config);
    assert(endpoint);
    assert(!endpoint->ready_for_plans());
    assert(!endpoint->set_peripheral_input(1, 1));
    assert(!endpoint->set_peripheral_input(0, 0.5));
    assert(endpoint->set_peripheral_input(0, 1));
    auto state = std::make_unique<rk_robot_state>();
    for (uint64_t now = 0; now <= 120'000'000; now += 2'000'000) endpoint->sample(now, *state);
    assert(endpoint->ready_for_plans());
    PlanRequest plan{};
    plan.plan_id = 1; plan.sequence = 1; plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    auto &segment = plan.segments.segments[0];
    // Link loss must also safe an arc whose ignition dwell has already ended at rest.
    segment.duration_ns = stop == 0 ? 50'000'000 : 2'000'000'000; segment.degree = 1; segment.joint_count = 1;
    plan.events.resize(3);
    for (int i = 0; i < 3; ++i) {
        std::strcpy(plan.events[i].channel, names[i]);
        plan.events[i].value.kind = i == 0 ? RK_EVENT_DIGITAL : RK_EVENT_ANALOG;
        plan.events[i].value.digital = i == 0;
        plan.events[i].value.analog = i == 1 ? 8 : 24;
    }
    assert(endpoint->submit_device_plan(plan, 0, 120'000'000, 20'000'000, *blueprint) == RK_OK);
    for (uint64_t now = 130'000'000; now <= 350'000'000; now += 10'000'000) endpoint->sample(now, *state);
    rk_sensor_sample sample{}; std::array<double, RK_MAX_SENSOR_VALUES> values{};
    assert(endpoint->sensor_sample(0, sample, values));
    assert(sample.sequence && sample.value_count == 6 && values[0] == 1 && values[1] == 240 && values[2] == 24);
    assert(state->sensor_count == 0); // Authored external samples must not be mislabeled as native encoders.
    if (stop == -2) { assert(endpoint->stop_device());
        const auto safe = endpoint->channel_values(); assert(safe[0] == 0 && safe[1] == 0);
    }
    else if (stop == 0) endpoint->cut_link(true);
    else if (stop == -1) assert(endpoint->set_peripheral_input(0, 0));
    else { rk_robot_command command{}; command.struct_size = sizeof(command); command.sequence = 2;
        command.kind = static_cast<rk_command_kind>(stop); assert(endpoint->apply(command) == RK_OK); }
    for (uint64_t now = 360'000'000; now <= 1'200'000'000; now += 10'000'000) endpoint->sample(now, *state);
    const auto outputs = endpoint->channel_values();
    assert(outputs[0] == 0 && outputs[1] == 0);
    {
        assert(endpoint->sensor_sample(0, sample, values)); assert(values[0] == 0);
        if (stop == -1) assert(values[4] == 2);
    }
    assert(endpoint->reset());
    assert(!endpoint->ready_for_plans());
    assert(endpoint->set_peripheral_input(0, 1));
}
int main() {
    for (int stop : {static_cast<int>(RK_COMMAND_STOP), static_cast<int>(RK_COMMAND_ABORT),
            static_cast<int>(RK_COMMAND_EMERGENCY_STOP), 0, -1, -2}) run(stop);
    std::puts("Host RKD6 welder: ignition, numeric sensor, stop, abort, estop, link loss, arc loss and reset passed");
}
