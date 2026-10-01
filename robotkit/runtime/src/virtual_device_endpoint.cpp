#include "virtual_device_endpoint.hpp"
#include "device_frame6.hpp"
#include "rkd_virtual.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <random>

namespace robotkit {

class VirtualDeviceEndpoint::Link final : public Rkd6Transport {
public:
    Link(const VirtualDeviceConfig6 &config, std::uint32_t count)
        : config_(config), random_(config.seed), count_(count) {
        std::vector<double> scale;
        if (!config.actuators.empty()) {
            for (const auto &actuator : config.actuators) scale.push_back(actuator.steps_per_unit);
        }
        if (scale.empty()) scale.assign(count, 1'000.0);
        if (scale.size() != count) return;
        device_ = rkd_virtual_create(config.device_tick_hz, config.step_tick_hz,
            config.offset_ticks, config.drift_ppm, count, scale.data(),
            config.fingerprint.data(), config.profile);
    }
    ~Link() override { rkd_virtual_destroy(device_); }
    bool valid() const noexcept { return device_ != nullptr; }
    bool miss_next_steps(std::uint32_t actuator, std::uint32_t count) {
        return rkd_virtual_miss_next_steps(device_, actuator, count) != 0;
    }
    unsigned baud() const noexcept override { return config_.baud; }
    std::uint64_t received_at_ns() const noexcept override { return received_at_ns_; }
    void cut(bool value) {
        cut_ = value;
        if (value) pending_host_.clear();
    }
    std::vector<double> positions() const {
        std::vector<double> result(count_);
        if (rkd_virtual_actuator_positions(device_, result.data(), result.size()) != count_)
            return {};
        return result;
    }
    std::vector<float> channels() const {
        std::vector<float> result(32);
        if (rkd_virtual_channel_values(device_, result.data(), result.size()) != result.size())
            return {};
        return result;
    }
    std::vector<VirtualStepRecord6> steps() const {
        const auto count = rkd_virtual_step_log(device_, nullptr, 0);
        std::vector<rkd_virtual_step_record> rows(count);
        if (rkd_virtual_step_log(device_, rows.data(), rows.size()) != count) return {};
        std::vector<VirtualStepRecord6> result;
        result.reserve(count);
        for (const auto &row : rows) result.push_back({row.ticks, row.actuator, row.forward != 0});
        return result;
    }
    std::vector<VirtualEventRecord6> events() const {
        const auto count = rkd_virtual_event_log(device_, nullptr, 0);
        std::vector<rkd_virtual_event_record> rows(count);
        if (rkd_virtual_event_log(device_, rows.data(), rows.size()) != count) return {};
        std::vector<VirtualEventRecord6> result;
        result.reserve(count);
        for (const auto &row : rows)
            result.push_back({row.plan_id, row.scheduled_path_ticks,
                row.applied_path_ticks, row.device_ticks, row.channel,
                row.kind, row.digital});
        return result;
    }
    bool send(std::span<const std::uint8_t> frame) override {
        if (!device_) return false;
        device_frame6::Frame decoded{};
        if (!device_frame6::decode(frame, decoded)) return false;
        if (cut_) return true;
        if (decoded.kind == 1) {
            if (!rkd_virtual_link_host_to_device(device_, frame.data(), frame.size())) return false;
            drain_device(true);
            return true;
        }
        // The line sends one frame at a time; the simulated UART retries a damaged
        // frame after another packet time, keeping the transport reliable under
        // sampled line errors.
        auto transmit = transmit_ns(frame.size());
        for (unsigned tries = 0; tries < 16 && disturbed(); ++tries)
            transmit += transmit_ns(frame.size()) + config_.latency_ns;
        host_tx_ready_ns_ = std::max(now_ns_, host_tx_ready_ns_) + transmit;
        // A serial line delivers in order, whatever the jitter.
        host_delivered_ns_ = std::max({host_tx_ready_ns_ + latency_ns(), now_ns_ + 1, host_delivered_ns_});
        pending_host_.push_back({host_delivered_ns_,
            std::vector<std::uint8_t>(frame.begin(), frame.end())});
        std::stable_sort(pending_host_.begin(), pending_host_.end(),
            [](const Packet &a, const Packet &b) { return a.at_ns < b.at_ns; });
        return true;
    }
    bool receive(std::vector<std::uint8_t> &frame) override {
        if (pending_device_.empty() || pending_device_.front().at_ns > now_ns_) return false;
        received_at_ns_ = pending_device_.front().at_ns;
        frame = std::move(pending_device_.front().frame);
        pending_device_.pop_front();
        return true;
    }
    bool advance(std::uint64_t host_ns) {
        if (!device_ || host_ns < now_ns_) return false;
        while (!pending_host_.empty() && pending_host_.front().at_ns <= host_ns) {
            auto packet = std::move(pending_host_.front());
            pending_host_.pop_front();
            now_ns_ = packet.at_ns;
            if (!rkd_virtual_step(device_, now_ns_)) return false;
            drain_device(false);
            if (!cut_) rkd_virtual_link_host_to_device(device_, packet.frame.data(), packet.frame.size());
            drain_device(false);
        }
        now_ns_ = host_ns;
        if (!rkd_virtual_step(device_, now_ns_)) return false;
        drain_device(false);
        return true;
    }

private:
    struct Packet { std::uint64_t at_ns; std::vector<std::uint8_t> frame; };
    bool disturbed() {
        const auto value = std::generate_canonical<double, 53>(random_);
        return value < config_.frame_drop_rate + config_.corruption_rate;
    }
    std::uint64_t transmit_ns(std::size_t frame_size) const {
        return static_cast<std::uint64_t>(std::ceil(
            10.0L * frame_size * 1e9L / std::max(1u, config_.baud)));
    }
    /** Latency after a frame leaves the line: frames pipeline it, so it is not serialized. */
    std::uint64_t latency_ns() {
        const auto magnitude = config_.jitter_ns;
        const auto jitter = magnitude == 0 ? 0 : static_cast<std::int64_t>(
            random_() % (2 * magnitude + 1)) - static_cast<std::int64_t>(magnitude);
        return static_cast<std::uint64_t>(std::max<std::int64_t>(0,
            static_cast<std::int64_t>(config_.latency_ns) + jitter));
    }
    void drain_device(bool session) {
        std::array<std::uint8_t, device_frame6::MAX_FRAME_SIZE> buffer{};
        for (;;) {
            const auto size = rkd_virtual_link_device_to_host(device_, buffer.data(), buffer.size());
            if (!size) break;
            device_frame6::Frame decoded{};
            if (!device_frame6::decode(std::span(buffer.data(), size), decoded)) continue;
            std::uint64_t at = now_ns_;
            if (!session || decoded.kind != 2) {
                auto transmit = transmit_ns(size);
                for (unsigned tries = 0; tries < 16 && disturbed(); ++tries)
                    transmit += transmit_ns(size) + config_.latency_ns;
                device_tx_ready_ns_ = std::max(now_ns_, device_tx_ready_ns_) + transmit;
                device_delivered_ns_ = std::max({device_tx_ready_ns_ + latency_ns(), now_ns_ + 1,
                    device_delivered_ns_});
                at = device_delivered_ns_;
            }
            pending_device_.push_back({at, std::vector<std::uint8_t>(buffer.begin(), buffer.begin() + size)});
        }
        std::stable_sort(pending_device_.begin(), pending_device_.end(),
            [](const Packet &a, const Packet &b) { return a.at_ns < b.at_ns; });
    }
    VirtualDeviceConfig6 config_;
    std::mt19937_64 random_;
    std::uint32_t count_;
    rkd_virtual_device *device_ = nullptr;
    bool cut_ = false;
    std::uint64_t now_ns_ = 0;
    std::uint64_t received_at_ns_ = 0;
    std::uint64_t host_tx_ready_ns_ = 0;
    std::uint64_t device_tx_ready_ns_ = 0;
    std::uint64_t host_delivered_ns_ = 0;
    std::uint64_t device_delivered_ns_ = 0;
    std::deque<Packet> pending_host_;
    std::deque<Packet> pending_device_;
};

std::shared_ptr<VirtualDeviceEndpoint> VirtualDeviceEndpoint::create(
    const rk_robot_runtime_blueprint &blueprint, VirtualDeviceConfig6 config) {
    if (config.actuators.empty()) {
        for (std::uint32_t i = 0; i < blueprint.joint_count; ++i) {
            DeviceActuator6 actuator{static_cast<std::uint8_t>(i)};
            actuator.id = "joint." + std::to_string(i);
            if (i < config.steps_per_unit.size())
                actuator.steps_per_unit = config.steps_per_unit[i];
            config.actuators.push_back(actuator);
        }
    }
    const auto count = config.actuators.empty() ? blueprint.joint_count : config.actuators.size();
    if (blueprint.joint_count == 0 || count > device_wire6::MAX_ACTUATORS ||
        config.baud == 0 || config.frame_drop_rate < 0 || config.corruption_rate < 0 ||
        config.frame_drop_rate + config.corruption_rate >= 1 ||
        config.clock_bound_ns == 0 || config.step_tick_hz == 0) return {};
    for (std::size_t i = 0; i < config.actuators.size(); ++i) {
        if (config.actuators[i].id.empty()) return {};
        for (std::size_t j = 0; j < i; ++j)
            if (config.actuators[j].id == config.actuators[i].id) return {};
    }
    const auto base_fingerprint = config.fingerprint;
    config.fingerprint = fingerprint_device_layout6(base_fingerprint, config.actuators,
        std::span(blueprint.channels, blueprint.channel_count));
    auto transport = std::make_unique<Link>(config, count);
    if (!transport->valid()) return {};
    auto *link = transport.get();
    auto inner = Rkd6Endpoint::attach(std::move(transport), blueprint, base_fingerprint,
        config.seed ? config.seed : 1, config.target_error, config.clock_bound_ns,
        config.latency_ns, config.step_tick_hz, config.link_loss_timeout_ns, config.actuators);
    if (!inner) return {};
    return std::shared_ptr<VirtualDeviceEndpoint>(new VirtualDeviceEndpoint(std::move(inner), link,
        std::move(config.actuators), blueprint.joint_count, config.fingerprint));
}

rk_result VirtualDeviceEndpoint::apply(const rk_robot_command &command) {
    return inner_->apply(command);
}

rk_result VirtualDeviceEndpoint::sample(std::uint64_t timestamp_ns, rk_robot_state &state) {
    if (!link_->advance(timestamp_ns)) return RK_ERROR_BACKEND;
    return inner_->sample(timestamp_ns, state);
}

int32_t VirtualDeviceEndpoint::diagnostic_code() const noexcept {
    return inner_->diagnostic_code();
}

rk_result VirtualDeviceEndpoint::submit_device_plan(const PlanRequest &plan,
    std::uint64_t base_time_ns, std::uint64_t owner_now_ns,
    std::uint64_t committed_through_ns, const rk_robot_runtime_blueprint &blueprint) {
    return inner_->submit_device_plan(plan, base_time_ns, owner_now_ns,
        committed_through_ns, blueprint);
}

void VirtualDeviceEndpoint::cut_link(bool cut) { link_->cut(cut); }
bool VirtualDeviceEndpoint::miss_next_steps(std::uint32_t actuator, std::uint32_t count) {
    return link_->miss_next_steps(actuator, count);
}
std::vector<double> VirtualDeviceEndpoint::actuator_positions() const { return link_->positions(); }
std::vector<double> VirtualDeviceEndpoint::joint_positions() const {
    auto positions = link_->positions();
    if (actuators_.empty()) return positions;
    std::vector<double> joints(joint_count_);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto &mapping = actuators_[i];
        if (mapping.joint < joints.size())
            joints[mapping.joint] = positions[i] / mapping.ratio + mapping.offset;
    }
    return joints;
}
std::vector<float> VirtualDeviceEndpoint::channel_values() const { return link_->channels(); }
std::vector<VirtualStepRecord6> VirtualDeviceEndpoint::step_log() const { return link_->steps(); }
std::vector<VirtualEventRecord6> VirtualDeviceEndpoint::event_log() const { return link_->events(); }

} // namespace robotkit
