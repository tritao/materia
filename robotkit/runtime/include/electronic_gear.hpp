#ifndef ROBOTKIT_ELECTRONIC_GEAR_HPP
#define ROBOTKIT_ELECTRONIC_GEAR_HPP
#include "robotkit_runtime.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace robotkit {
/** Owner-thread state. Inputs are encoder observations, never commanded master positions.
 * A relation is installed atomically by begin/term/engage events at one path time.
 * Extra latency describes delay not already represented by the sample timestamp.
 */
class ElectronicGear {
public:
    static constexpr unsigned max_terms = 8;
    struct Term { unsigned master{}; double ratio{}, latency{}, quantum{}; bool predict{}; };
    struct Relation { bool active{}, staging{}; unsigned count{}; uint64_t time{}; double loop_hz{}; std::array<Term, max_terms> terms{}; };
    std::array<Relation, RK_MAX_SERVO_JOINTS> relations{}, staged{};
    static int follower(const char *channel) {
        unsigned index; int end = 0;
        if (std::sscanf(channel, "motion.gear/%u%n", &index, &end) != 1 || !end || channel[end]) return -1;
        return index < RK_MAX_SERVO_JOINTS ? static_cast<int>(index) : -1;
    }
    bool event(unsigned joint, const rk_event_value &value, unsigned joints, uint64_t time = 0) {
        if (joint >= joints || joint >= relations.size() || value.kind != RK_EVENT_PROCESS) return false;
        auto &draft = staged[joint];
        if (!std::strcmp(value.command, "gear.release")) { relations[joint] = {}; draft = {}; return true; }
        if (!std::strcmp(value.command, "gear.begin")) { draft = {}; draft.staging = true; draft.time = time; return true; }
        if (!draft.staging || draft.time != time) return false;
        if (!std::strcmp(value.command, "gear.engage")) {
            if (!draft.staging || !draft.count || !(draft.loop_hz > 0)) return false;
            // Measured master graphs may not contain cycles, even through another follower.
            auto previous = relations[joint]; relations[joint] = draft; relations[joint].active = true;
            std::array<unsigned, RK_MAX_JOINTS> color{};
            for (unsigned i = 0; i < joints; ++i) if (cycle(i, color)) { relations[joint] = previous; return false; }
            draft = {}; return true;
        }
        if (!std::strcmp(value.command, "gear.loop") && std::isfinite(value.argument) && value.argument > 0 && value.argument <= 1e6) {
            draft.loop_hz = value.argument; return true;
        }
        unsigned master, predict; unsigned long long latency; int end = 0;
        if (std::sscanf(value.command, "gear.term/%u/%llu/%u%n", &master, &latency, &predict, &end) == 3 &&
            end && !value.command[end] && master < joints && master != joint && predict <= 1 &&
            latency <= 1'000'000'000ULL && draft.staging && draft.count < max_terms &&
            std::isfinite(value.argument) && value.argument != 0.0) {
            for (unsigned i = 0; i < draft.count; ++i) if (draft.terms[i].master == master) return false;
            draft.terms[draft.count++] = {master, value.argument, latency * 1e-9, 0.0, predict != 0};
            return true;
        }
        // Quantum is separate so ratio and resolution retain full double precision in the event ABI.
        if (!std::strcmp(value.command, "gear.quantum") && draft.staging && draft.count &&
            std::isfinite(value.argument) && value.argument >= 0.0) {
            draft.terms[draft.count - 1].quantum = value.argument; return true;
        }
        return false;
    }
    double offset(unsigned joint, const rk_robot_state &measured, double age, bool predict = true) const {
        double result = 0;
        for (unsigned i = 0; i < relations[joint].count; ++i) {
            const auto &t = relations[joint].terms[i];
            const double p = t.quantum > 0 ? std::round(measured.position[t.master] / t.quantum) * t.quantum : measured.position[t.master];
            result += t.ratio * (p + (predict && t.predict ? measured.velocity[t.master] * (age + t.latency) : 0.0));
        }
        return result;
    }
    double velocity(unsigned joint, const rk_robot_state &measured) const {
        double result = 0;
        for (unsigned i = 0; i < relations[joint].count; ++i) {
            const auto &t = relations[joint].terms[i]; result += t.ratio * measured.velocity[t.master];
        }
        return result;
    }
private:
    bool cycle(unsigned joint, std::array<unsigned, RK_MAX_JOINTS> &color) const {
        if (color[joint] == 1) return true;
        if (color[joint] == 2) return false;
        color[joint] = 1;
        if (joint < relations.size() && relations[joint].active) for (unsigned i = 0; i < relations[joint].count; ++i)
            if (cycle(relations[joint].terms[i].master, color)) return true;
        color[joint] = 2; return false;
    }
};
}
#endif
