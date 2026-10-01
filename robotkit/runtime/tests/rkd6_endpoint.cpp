#include "rkd6_endpoint.hpp"
#include "device_frame6.hpp"
#include <type_traits>
#include <cassert>
#include <deque>
#include <string>

using namespace robotkit;

class MockLink final : public Rkd6Transport {
public:
    int segment_frames = 0;
    int commit_frames = 0;
    int queue_begin_frames = 0;
    std::array<std::uint8_t, 16> fingerprint{};
    bool minimal = false;
    std::deque<std::vector<std::uint8_t>> incoming;
    std::vector<std::uint8_t> delayed;
    bool delay_once = false;
    std::uint64_t jump_ticks = 0;
    /** Bytes received since the session began: this device takes in everything sent at once. */
    std::uint32_t received_bytes = 0;
    /** Bytes the device reports it has not received yet, such as ones held in a USB adapter. */
    std::uint32_t unreceived = 0;
    unsigned baud() const noexcept override { return 921'600; }
    template<class T> void push(std::uint8_t kind, const T &value) {
        std::vector<std::uint8_t> payload(T::SIZE);
        if constexpr (std::is_same_v<T, device_wire6::QueueStatus6>) {
            auto reported = value;
            reported.received_bytes = received_bytes - unreceived;
            assert(device_wire6::encode(reported, payload));
        } else {
            assert(device_wire6::encode(value, payload));
        }
        std::vector<std::uint8_t> frame;
        assert(device_frame6::encode(kind, payload, frame));
        incoming.push_back(std::move(frame));
    }
    bool send(std::span<const std::uint8_t> frame) override {
        device_frame6::Frame decoded{};
        assert(device_frame6::decode(frame, decoded));
        received_bytes += static_cast<std::uint32_t>(frame.size());
        if (decoded.kind == 1) {
            received_bytes = 0;
            device_wire6::SessionBegin6 begin{};
            assert(device_wire6::decode(decoded.payload.first(begin.SIZE), begin));
            device_wire6::SessionAck6 ack{};
            ack.session = begin.session;
            ack.protocol_version = device_wire6::PROTOCOL_VERSION;
            ack.device_fingerprint = fingerprint;
            ack.status = 1;
            ack.device_tick_hz = 1'000'000;
            ack.step_tick_hz = 40'000;
            ack.segment_capacity = minimal ? 8 : 4;
            ack.event_capacity = 4;
            ack.max_degree = minimal ? 1 : 5;
            ack.actuator_count = begin.actuator_count;
            ack.profile = minimal ? 2 : 1;
            push(2, ack);
            device_wire6::State6Header state{};
            state.session = begin.session;
            state.actuator_count = begin.actuator_count;
            state.safety = 0;
            std::vector<std::uint8_t> body(state.SIZE +
                begin.actuator_count * device_wire6::ActuatorState6::SIZE);
            assert(device_wire6::encode(state, std::span(body).first(state.SIZE)));
            for (std::size_t i = 0; i < begin.actuator_count; ++i) {
                device_wire6::ActuatorState6 actuator{};
                actuator.effort = 2.0f;
                assert(device_wire6::encode(actuator, std::span(body).subspan(
                    state.SIZE + i * actuator.SIZE, actuator.SIZE)));
            }
            std::vector<std::uint8_t> framed;
            assert(device_frame6::encode(15, body, framed));
            incoming.push_back(std::move(framed));
        } else if (decoded.kind == 3) {
            device_wire6::TimeSyncRequest request{};
            assert(device_wire6::decode(decoded.payload, request));
            const auto ticks = 50'000 + request.host_send_ns / 1'000 + 100 + jump_ticks;
            device_wire6::TimeSyncReply reply{request.host_send_ns, ticks, ticks};
            std::vector<std::uint8_t> body(reply.SIZE);
            assert(device_wire6::encode(reply, body));
            assert(device_frame6::encode(4, body, delayed));
            delay_once = true;
        } else if (decoded.kind == 5) {
            ++queue_begin_frames;
        } else if (decoded.kind == 6) {
            ++segment_frames;
        } else if (decoded.kind == 7) {
            ++commit_frames;
        }
        return true;
    }
    bool receive(std::vector<std::uint8_t> &frame) override {
        if (!incoming.empty()) {
            frame = std::move(incoming.front());
            incoming.pop_front();
            return true;
        }
        if (delay_once) { delay_once = false; return false; }
        if (!delayed.empty()) {
            frame = std::move(delayed);
            delayed.clear();
            return true;
        }
        return false;
    }
};

void unseen_backlog_holds_segments(const rk_robot_runtime_blueprint &blueprint) {
    // Bytes the host cannot see waiting, such as in a USB adapter, still count: the device's
    // status says what it has received, and segments wait while the rest would delay a commit.
    auto link = std::make_unique<MockLink>();
    auto *observed = link.get();
    observed->fingerprint.fill(7);
    auto endpoint = Rkd6Endpoint::attach(std::move(link), blueprint, observed->fingerprint,
        77, 1e-6, 500'000, 100'000);
    assert(endpoint);
    rk_robot_state state{};
    for (const std::uint64_t now : {0ull, 200'000ull, 100'000'000ull, 100'200'000ull})
        assert(endpoint->sample(now, state) == RK_OK);
    robotkit::PlanRequest plan{};
    plan.plan_id = 8;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    plan.segments.segments[0].duration_ns = 1'000'000'000;
    plan.segments.segments[0].degree = 1;
    plan.segments.segments[0].joint_count = 1;
    plan.segments.segments[0].coefficients[0].value[1] = 0.5;
    device_wire6::QueueStatus6 status{};
    status.remaining_segments = 4;
    observed->unreceived = 4'000;
    observed->push(14, status);
    assert(endpoint->sample(100'300'000, state) == RK_OK);
    assert(endpoint->submit_device_plan(plan, 0, 100'300'000, 20'000'000, blueprint) == RK_OK);
    assert(observed->queue_begin_frames == 1 && observed->segment_frames == 0);
    observed->unreceived = 0;
    observed->push(14, status);
    assert(endpoint->sample(100'400'000, state) == RK_OK);
    assert(observed->segment_frames == 1);
}

int main() {
    assert(Rkd6Endpoint::minimum_baud(64, 10'000'000, 2'000'000) > 921'600);
    assert(Rkd6Endpoint::minimum_queue_depth(921'600, 1, 10'000'000,
        100'000, 500'000) <= 4);
    assert(Rkd6Endpoint::minimum_baud(2, 10'000'000, 2'000'000) <= 921'600);
    assert(Rkd6Endpoint::minimum_queue_depth(921'600, 2, 10'000'000,
        100'000, 30'000'000) <= 8);
    rk_robot_runtime_blueprint blueprint{};
    blueprint.struct_size = sizeof(blueprint);
    blueprint.joint_count = 1;
    blueprint.owner_period_ns = 10'000'000;
    blueprint.joints[0].lower_limit = -10;
    blueprint.joints[0].upper_limit = 10;
    blueprint.joints[0].max_velocity = 10;
    blueprint.joints[0].max_acceleration = 10;
    unseen_backlog_holds_segments(blueprint);
    {
        auto wrong = std::make_unique<MockLink>();
        wrong->fingerprint.fill(4);
        rk_result reason = RK_OK;
        auto rejected = Rkd6Endpoint::attach(std::move(wrong), blueprint,
            std::array<std::uint8_t, 16>{7, 7, 7, 7, 7, 7, 7, 7,
                7, 7, 7, 7, 7, 7, 7, 7},
            89, 1e-6, 500'000, 100'000, 40'000, 500'000'000, {}, &reason);
        assert(!rejected && reason == RK_ERROR_MODEL_MISMATCH);
        auto fast = blueprint;
        fast.owner_period_ns = 1'000'000;
        auto short_queue = std::make_unique<MockLink>();
        short_queue->fingerprint.fill(4);
        reason = RK_OK;
        rejected = Rkd6Endpoint::attach(std::move(short_queue), fast,
            std::array<std::uint8_t, 16>{4, 4, 4, 4, 4, 4, 4, 4,
                4, 4, 4, 4, 4, 4, 4, 4},
            90, 1e-6, 500'000, 100'000, 40'000, 500'000'000, {}, &reason);
        assert(!rejected && reason == RK_ERROR_UNSUPPORTED);
    }
    {
        auto mismatched = std::make_unique<MockLink>();
        auto *device = mismatched.get();
        device->fingerprint.fill(7);
        auto endpoint = Rkd6Endpoint::attach(std::move(mismatched), blueprint,
            device->fingerprint, 88, 1e-6, 500'000, 100'000);
        assert(endpoint);
        rk_robot_state state{};
        assert(endpoint->sample(0, state) == RK_OK);
        device_wire6::QueueStatus6 wrong{};
        wrong.queue_revision = 7;
        device->push(14, wrong);
        assert(endpoint->sample(200'000, state) == RK_OK);
        assert(endpoint->diagnostic_code() != 0);
        assert(std::string(endpoint->fault_reason()) == "queue_revision_mismatch");
    }
    {
        auto bench = blueprint;
        bench.joint_count = 2;
        bench.joints[1] = bench.joints[0];
        auto minimal_link = std::make_unique<MockLink>();
        minimal_link->minimal = true;
        minimal_link->fingerprint.fill(9);
        auto qualified = Rkd6Endpoint::attach(std::move(minimal_link), bench,
            std::array<std::uint8_t, 16>{9, 9, 9, 9, 9, 9, 9, 9,
                9, 9, 9, 9, 9, 9, 9, 9},
            78, 1e-5, 30'000'000, 100'000);
        assert(qualified);
    }
    auto link = std::make_unique<MockLink>();
    auto *observed = link.get();
    observed->fingerprint.fill(7);
    auto endpoint = Rkd6Endpoint::attach(std::move(link), blueprint,
        observed->fingerprint, 77, 1e-6, 500'000, 100'000);
    assert(endpoint);
    rk_robot_state state{};
    assert(endpoint->sample(0, state) == RK_OK);
    assert(state.effort[0] == 2.0);
    assert(endpoint->sample(200'000, state) == RK_OK);
    assert(state.effort[0] == 2.0);
    assert(endpoint->sample(100'000'000, state) == RK_OK);
    assert(endpoint->sample(100'200'000, state) == RK_OK);
    robotkit::PlanRequest plan{};
    plan.plan_id = 8;
    plan.sequence = 1;
    plan.ends_at_rest = 1;
    plan.segments.segments.resize(1);
    auto &segment = plan.segments.segments[0];
    segment.duration_ns = 1'000'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 0.5;
    auto too_fast = plan;
    too_fast.segments.segments[0].duration_ns = 1'000'000;
    assert(endpoint->submit_device_plan(too_fast, 0, 100'200'000, 20'000'000,
        blueprint) == RK_ERROR_LIMIT);
    assert(endpoint->submit_device_plan(plan, 0, 100'200'000, 20'000'000,
        blueprint) == RK_OK);
    assert(observed->queue_begin_frames == 1);
    assert(observed->segment_frames == 1);
    assert(observed->commit_frames == 1);
    assert(endpoint->committed_until_ticks() > 0);
    device_wire6::QueueStatus6 status{};
    status.queue_revision = 1;
    status.committed_until_ticks = endpoint->committed_until_ticks();
    // The device holds plan 8's one segment, which the commit runs through.
    status.received_until_ticks = status.committed_until_ticks;
    status.executing_plan_id = 8;
    status.path_clock_ticks = status.committed_until_ticks - 100;
    status.rate = 1.0f;
    status.remaining_segments = 3;
    observed->push(14, status);
    assert(endpoint->sample(100'400'000, state) == RK_OK);
    assert(state.trajectory_active == 1 && state.active_plan_id == 8);
    assert(state.trajectory_queue_depth == 1 && state.committed_until_ns > 0);
    auto late = plan;
    late.replace_after_plan_id = 8;
    assert(endpoint->submit_device_plan(late, 500'000'000, 100'400'000,
        520'000'000, blueprint) == RK_ERROR_INVALID_STATE);
    // The device has only received the first plan's segment; an anchor beyond
    // its received horizon cannot be repaired by later pending host segments.
    assert(endpoint->submit_device_plan(late, 2'000'000'000,
        100'400'000, 2'020'000'000, blueprint) == RK_ERROR_INVALID_STATE);
    auto replacement = plan;
    replacement.plan_id = 9;
    replacement.replace_after_plan_id = 8;
    replacement.start_position[0] = 0.5;
    replacement.segments.segments[0].degree = 2;
    replacement.segments.segments[0].coefficients[0].value[0] = 0.5;
    assert(endpoint->submit_device_plan(replacement, 1'000'000'000,
        100'400'000, 1'020'000'000, blueprint) == RK_OK);
    // The boundary goes at once; the segment waits until the line clears within an
    // owner period, so a commit never queues behind a backlog of segments.
    assert(observed->queue_begin_frames == 2 && observed->segment_frames == 1);
    status.queue_revision = 2;
    status.path_clock_ticks = endpoint->committed_until_ticks() + 2'000'000;
    status.remaining_segments = 4;
    observed->push(14, status);
    assert(endpoint->sample(150'000'000, state) == RK_OK);
    assert(observed->segment_frames == 2);
    // The next status retires the segment the device has since run past.
    observed->push(14, status);
    assert(endpoint->sample(150'200'000, state) == RK_OK);
    assert(endpoint->bookkeeping_counts().first == 0);
    assert(endpoint->bookkeeping_counts().second == 1);
    observed->jump_ticks = 10'000;
    assert(endpoint->sample(200'000'000, state) == RK_OK);
    assert(endpoint->sample(200'200'000, state) == RK_OK);
    assert(endpoint->diagnostic_code() == 0);
    assert(endpoint->sample(300'000'000, state) == RK_OK);
    assert(endpoint->sample(300'200'000, state) == RK_OK);
    assert(endpoint->sample(400'000'000, state) == RK_OK);
    assert(endpoint->sample(400'200'000, state) == RK_OK);
    assert(endpoint->diagnostic_code() == RK_FAULT_CLOCK_SYNC_LOST);
    assert(std::string(endpoint->fault_reason()) == "clock_sync_lost");
    assert(endpoint->submit_device_plan(plan, 0, 200'200'000, 20'000'000,
        blueprint) == RK_ERROR_INVALID_STATE);
}
