#include "robotkit_runtime.hpp"
#include <cassert>
#include <cstring>
#include <memory>

class BrakeFeedbackEndpoint final : public robotkit::RobotEndpoint {
public:
    bool released = false;
    bool fresh = true;
    bool owns_path_clock = false;
    uint64_t sequence = 0;
    double position = 0.0;
    rk_robot_command last{};
    bool executes_trajectory_queue() const noexcept override { return owns_path_clock; }
    rk_result apply(const rk_robot_command &command) override {
        last = command;
        if (command.kind == RK_COMMAND_JOINT_TARGETS && command.target_count)
            position = command.targets[0].target;
        return RK_OK;
    }
    rk_result sample(uint64_t timestamp, rk_robot_state &state) override {
        state.struct_size = sizeof(state);
        state.source_timestamp_ns = timestamp;
        state.joint_count = 1;
        state.position[0] = position;
        state.sensor_count = 1;
        if (fresh) ++sequence;
        state.sensors[0].sequence = sequence;
        state.sensors[0].source_timestamp_ns = timestamp;
        state.sensors[0].value_count = 1;
        state.sensors[0].value_offset = 0;
        state.sensor_values[0] = released ? 1.0 : 0.0;
        return RK_OK;
    }
};

std::unique_ptr<rk_robot_runtime_blueprint> blueprint() {
    auto value = std::make_unique<rk_robot_runtime_blueprint>();
    value->struct_size = sizeof(*value);
    value->revision = 1;
    value->joint_count = 1;
    value->link_count = 2;
    value->collision_approximation = RK_COLLISION_APPROXIMATION_BOUNDS_BOX;
    for (uint32_t i = 0; i < 2; ++i) {
        value->links[i].mass = 1.0;
        value->links[i].inertia_tensor[0] = value->links[i].inertia_tensor[4] = value->links[i].inertia_tensor[8] = 1.0;
    }
    value->joints[0] = {0, RK_RUNTIME_JOINT_REVOLUTE, 0, 1, -2.0, 2.0, 10.0};
    value->joints[0].axis[2] = 1.0;
    value->joints[0].parent_frame_rotation[3] = value->joints[0].child_frame_rotation[3] = 1.0;
    value->joint_servo[0] = {100.0, 10.0};
    value->channel_count = 1;
    std::strcpy(value->channels[0].id, "z.brake.release");
    value->channels[0].kind = value->channels[0].safe_value.kind = RK_EVENT_DIGITAL;
    value->channels[0].stop_policy = RK_CHANNEL_KEEP_ON_STOP;
    value->sensor_count = 1;
    value->sensors[0].kind = RK_SENSOR_BRAKE_RELEASED;
    value->sensors[0].rotation[3] = 1.0;
    value->sensor_joint[0] = 0;
    value->brake_count = 1;
    value->brakes[0] = {0, 0, 0, 0, 250'000'000, 30'000'000};
    assert(rk_robot_runtime_blueprint_validate(value.get()) == RK_OK);
    return value;
}

rk_robot_command lifecycle(uint64_t sequence, rk_command_kind kind) {
    rk_robot_command command{};
    command.struct_size = sizeof(command);
    command.sequence = sequence;
    command.kind = kind;
    return command;
}

bool release_coil(robotkit::RobotRuntime &runtime) {
    rk_event_value output{};
    assert(runtime.channel_value("z.brake.release", output) == RK_OK);
    assert(output.kind == RK_EVENT_DIGITAL);
    return output.digital != 0;
}

int main() {
    auto spec = blueprint();
    auto endpoint = std::make_shared<BrakeFeedbackEndpoint>();
    robotkit::RobotRuntime runtime(*spec, endpoint, std::chrono::milliseconds(10));
    robotkit::SegmentBatch batch{};
    batch.tag = 1;
    batch.segments.resize(1);
    auto &segment = batch.segments[0];
    segment.joint_count = 1;
    segment.duration_ns = 1'000'000'000;
    segment.degree = 3;
    segment.coefficients[0].value[3] = 1.0;
    robotkit::PlanRequest plan{};
    plan.sequence = plan.plan_id = plan.model_revision = 1;
    plan.segments = batch;
    assert(robotkit::validate_plan_for_blueprint(plan, *spec) == RK_OK);
    rk_timed_event forbidden{};
    std::strcpy(forbidden.channel, "z.brake.release");
    forbidden.value.kind = RK_EVENT_DIGITAL;
    forbidden.value.digital = 1;
    plan.events.push_back(forbidden);
    assert(robotkit::validate_plan_for_blueprint(plan, *spec) == RK_ERROR_INVALID_ARGUMENT);
    plan.events.clear();
    auto queued_endpoint = std::make_shared<BrakeFeedbackEndpoint>();
    queued_endpoint->owns_path_clock = true;
    robotkit::RobotRuntime queued(*spec, queued_endpoint, std::chrono::milliseconds(10));
    assert(queued.submit_plan(plan) == RK_ERROR_UNSUPPORTED);
    assert(queued.submit_segments(lifecycle(1, RK_COMMAND_TRAJECTORY_SEGMENTS), batch) == RK_ERROR_UNSUPPORTED);
    assert(runtime.submit_segments(lifecycle(1, RK_COMMAND_TRAJECTORY_SEGMENTS), batch) == RK_OK);
    uint64_t tick = 0;
    assert(runtime.publish_sample(tick) == RK_OK);
    for (int i = 0; i < 6; ++i) {
        tick += 10'000'000;
        assert(runtime.apply_pending_commands(tick) == RK_OK);
        assert(release_coil(runtime));
        if (i == 0) {
            rk_event_record_batch events{};
            events.struct_size = sizeof(events);
            assert(runtime.poll_events(events) == RK_OK);
            assert(events.count == 1 && events.records[0].cause == RK_EVENT_CONTROLLER);
        }
        assert(endpoint->last.targets[0].target == 0.0);
        assert(endpoint->last.servos[0].velocity == 0.0 && endpoint->last.servos[0].feedforward == 0.0);
        assert(endpoint->last.reference_end_position[0] == 0.0 && endpoint->last.reference_end_velocity[0] == 0.0);
        rk_robot_state state{};
        state.struct_size = sizeof(state);
        assert(runtime.snapshot(state) == RK_OK);
        assert(state.trajectory_time_ns == 0);
        assert(runtime.publish_sample(tick) == RK_OK);
    }
    endpoint->released = true;
    assert(runtime.publish_sample(tick) == RK_OK);
    for (int i = 0; i < 3; ++i) {
        tick += 10'000'000;
        assert(runtime.apply_pending_commands(tick) == RK_OK);
        assert(runtime.publish_sample(tick) == RK_OK);
    }
    assert(endpoint->last.targets[0].target > 0.0);
    assert(runtime.submit(lifecycle(2, RK_COMMAND_STOP)) == RK_OK);
    tick += 10'000'000;
    assert(runtime.apply_pending_commands(tick) == RK_OK);
    assert(release_coil(runtime));
    assert(runtime.publish_sample(tick) == RK_OK);
    assert(runtime.submit(lifecycle(3, RK_COMMAND_EMERGENCY_STOP)) == RK_OK);
    tick += 10'000'000;
    assert(runtime.apply_pending_commands(tick) == RK_OK);
    assert(!release_coil(runtime));
    assert(runtime.publish_sample(tick) == RK_OK);
    assert(runtime.submit(lifecycle(4, RK_COMMAND_RESET_SAFETY)) == RK_OK);
    tick += 10'000'000;
    assert(runtime.apply_pending_commands(tick) == RK_OK);
    assert(release_coil(runtime));
    endpoint->fresh = false;
    tick += 250'000'000;
    assert(runtime.apply_pending_commands(tick) == RK_ERROR_SAFETY_STOPPED);
    assert(!release_coil(runtime));
    rk_robot_snapshot fault{};
    fault.struct_size = sizeof(fault);
    assert(runtime.snapshot_full(fault) == RK_OK);
    assert(fault.fault_code == RK_FAULT_BRAKE_RELEASE_TIMEOUT);

    for (bool stale : {false, true}) {
        auto feedback = std::make_shared<BrakeFeedbackEndpoint>();
        feedback->released = true;
        robotkit::RobotRuntime checked(*spec, feedback, std::chrono::milliseconds(10));
        assert(checked.publish_sample(0) == RK_OK);
        assert(checked.apply_pending_commands(10'000'000) == RK_OK);
        assert(checked.publish_sample(10'000'000) == RK_OK);
        assert(checked.apply_pending_commands(20'000'000) == RK_OK);
        assert(release_coil(checked));
        if (stale) feedback->fresh = false;
        else feedback->released = false;
        assert(checked.publish_sample(20'000'000) == RK_OK);
        assert(checked.apply_pending_commands(stale ? 41'000'000 : 30'000'000) == RK_ERROR_SAFETY_STOPPED);
        assert(!release_coil(checked));
        assert(checked.snapshot_full(fault) == RK_OK);
        assert(fault.fault_code == RK_FAULT_BRAKE_FEEDBACK);
    }
}
