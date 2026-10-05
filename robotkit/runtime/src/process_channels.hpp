#ifndef ROBOTKIT_PROCESS_CHANNELS_HPP
#define ROBOTKIT_PROCESS_CHANNELS_HPP

#include "robotkit_runtime.h"

#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <utility>

namespace robotkit {

/** A failed typed read belongs to the owning robot, not to the shared physics tick. */
struct ProcessChannelFault {
    enum class Reason { Missing, Type, Value, Endpoint };
    Reason reason;
    std::string channel;
    rk_result source_result;
    int32_t code() const noexcept {
        switch (reason) {
        case Reason::Missing: return RK_FAULT_PROCESS_CHANNEL_MISSING;
        case Reason::Type: return RK_FAULT_PROCESS_CHANNEL_TYPE;
        case Reason::Value: return RK_FAULT_PROCESS_CHANNEL_VALUE;
        case Reason::Endpoint: return RK_FAULT_ENDPOINT;
        }
        return RK_FAULT_ENDPOINT;
    }
};

template <typename Value> struct ProcessChannelReading {
    Value value{};
    std::optional<ProcessChannelFault> fault;
};

/** Converts native event values once, at the channel boundary. */
class ProcessChannelReader {
public:
    using Read = std::function<rk_result(const char *, rk_event_value &)>;
    explicit ProcessChannelReader(Read read) : read_(std::move(read)) {}

    ProcessChannelReading<bool> digital(const std::string &channel) const {
        rk_event_value value{};
        if (auto fault = read(channel, RK_EVENT_DIGITAL, value)) return {false, std::move(fault)};
        if (value.digital > 1)
            return {false, ProcessChannelFault{ProcessChannelFault::Reason::Value, channel, RK_OK}};
        return {value.digital != 0, std::nullopt};
    }
    ProcessChannelReading<double> analog(const std::string &channel) const {
        rk_event_value value{};
        if (auto fault = read(channel, RK_EVENT_ANALOG, value)) return {0.0, std::move(fault)};
        if (!std::isfinite(value.analog))
            return {0.0, ProcessChannelFault{ProcessChannelFault::Reason::Value, channel, RK_OK}};
        return {value.analog, std::nullopt};
    }
private:
    std::optional<ProcessChannelFault> read(const std::string &channel, rk_event_kind kind,
                                           rk_event_value &value) const {
        const auto result = read_(channel.c_str(), value);
        if (result != RK_OK) return ProcessChannelFault{ProcessChannelFault::Reason::Missing, channel, result};
        if (value.kind != kind) return ProcessChannelFault{ProcessChannelFault::Reason::Type, channel, RK_OK};
        return std::nullopt;
    }
    Read read_;
};

} // namespace robotkit
#endif
