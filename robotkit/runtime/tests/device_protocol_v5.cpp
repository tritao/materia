#include "robotkit_device_protocol_v5.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); std::abort(); } } while (false)

namespace v5 = robotkit::v5;
namespace wire = robotkit::device_wire;

static std::map<std::string, std::vector<std::uint8_t>> fixtures(const char *path) {
    std::ifstream input(path);
    CHECK(input.good());
    std::map<std::string, std::vector<std::uint8_t>> result;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto tab = line.find('\t');
        CHECK(tab != std::string::npos);
        std::vector<std::uint8_t> bytes;
        for (std::size_t at = tab + 1; at < line.size(); at += 2)
            bytes.push_back(static_cast<std::uint8_t>(std::stoul(line.substr(at, 2), nullptr, 16)));
        result.emplace(line.substr(0, tab), bytes);
    }
    return result;
}

template<class T>
static std::vector<std::uint8_t> payload(const T &value) {
    std::vector<std::uint8_t> bytes(T::SIZE);
    CHECK(wire::encode(value, bytes));
    return bytes;
}

static std::vector<std::uint8_t> frame(std::uint8_t type, const std::vector<std::uint8_t> &body) {
    std::vector<std::uint8_t> bytes(v5::header_size + body.size() + v5::crc_size);
    std::size_t written = 0;
    CHECK(v5::encode_frame(type, body, bytes, written));
    CHECK(written == bytes.size());
    return bytes;
}

static void refresh_crc(std::vector<std::uint8_t> &bytes) {
    const auto crc = v5::crc32(std::span<const std::uint8_t>(bytes.data(), bytes.size() - 4));
    for (unsigned index = 0; index < 4; ++index)
        bytes[bytes.size() - 4 + index] = static_cast<std::uint8_t>(crc >> (8 * index));
}

static std::vector<std::uint8_t> command(std::uint64_t session, std::uint64_t sequence,
    std::uint8_t kind, const std::vector<wire::JointTarget> &targets = {}) {
    CHECK(targets.size() <= 64);
    wire::CommandHeader head{session, sequence, kind, static_cast<std::uint8_t>(targets.size()), 0};
    auto bytes = payload(head);
    for (const auto &target : targets) {
        auto encoded = payload(target);
        bytes.insert(bytes.end(), encoded.begin(), encoded.end());
    }
    return frame(3, bytes);
}

struct Collector {
    std::size_t sessions = 0;
    std::vector<v5::Command> commands;
    bool accept = true;
    static bool session(void *context, std::uint64_t) {
        auto &self = *static_cast<Collector *>(context);
        ++self.sessions;
        return self.accept;
    }
    static bool command(void *context, const v5::Command &value) {
        auto &self = *static_cast<Collector *>(context);
        if (!self.accept) return false;
        self.commands.push_back(value);
        return true;
    }
};

static std::size_t feed(v5::CommandDecoder &decoder, const std::vector<std::uint8_t> &bytes,
                         std::uint64_t time, Collector &collector) {
    return decoder.feed(bytes, time, Collector::session, Collector::command, &collector);
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const auto vectors = fixtures(argv[1]);
    const std::uint64_t session = 0x0102030405060708ULL;
    std::array<std::uint8_t, 16> fingerprint{};
    for (std::size_t index = 0; index < fingerprint.size(); ++index)
        fingerprint[index] = static_cast<std::uint8_t>(index);
    const auto begin = frame(1, payload(wire::SessionBegin{session, fingerprint}));
    const auto reset = command(session, 1, 4);
    const auto target = command(session, 2, 1, {{1, 2, 0, 1.5f}});
    CHECK(begin == vectors.at("session_begin"));
    CHECK(reset == vectors.at("command_reset"));
    CHECK(target == vectors.at("command_target"));
    CHECK(frame(4, [&] {
        auto state = payload(wire::StateHeader{session, 123456789, 2, 1, 0, 1, 0});
        auto joint = payload(wire::JointState{1.0f, -2.0f, 0.5f});
        state.insert(state.end(), joint.begin(), joint.end());
        return state;
    }()) == vectors.at("state"));

    // Every split point in a concatenated stream must decode identically.
    auto stream = begin;
    stream.insert(stream.end(), reset.begin(), reset.end());
    stream.insert(stream.end(), target.begin(), target.end());
    for (std::size_t split = 0; split <= stream.size(); ++split) {
        v5::CommandDecoder decoder(2, fingerprint);
        Collector collector;
        const auto first = decoder.feed(std::span(stream.data(), split), 100,
            Collector::session, Collector::command, &collector);
        const auto second = decoder.feed(std::span(stream.data() + split, stream.size() - split), 100,
            Collector::session, Collector::command, &collector);
        CHECK(first + second == 2);
        CHECK(collector.sessions == 1 && collector.commands.size() == 2);
        CHECK(decoder.last_sequence() == 2 && !decoder.latched());
    }

    v5::CommandDecoder decoder(2, fingerprint);
    Collector collector;
    CHECK(feed(decoder, begin, 10, collector) == 0);
    CHECK(decoder.latched() && decoder.model_matches() && decoder.watchdog_expired(10, 100));
    CHECK(feed(decoder, target, 11, collector) == 0); // Motion before reset.
    CHECK(decoder.last_sequence() == 0 && decoder.watchdog_expired(11, 100));
    CHECK(feed(decoder, reset, 20, collector) == 1);
    CHECK(!decoder.latched() && decoder.last_sequence() == 1);
    CHECK(feed(decoder, target, 30, collector) == 1);
    CHECK(!decoder.watchdog_expired(129, 100) && decoder.watchdog_expired(130, 100));

    auto flagged = command(session, 3, 2);
    flagged[5] = 1;
    refresh_crc(flagged);
    CHECK(feed(decoder, flagged, 140, collector) == 0);
    auto reserved = command(session, 3, 2);
    reserved[8 + 18] = 1;
    refresh_crc(reserved);
    CHECK(feed(decoder, reserved, 140, collector) == 0);
    CHECK(feed(decoder, command(session, 3, 1, {{1, 9, 0, 1.0f}}), 140, collector) == 0);
    collector.accept = false;
    CHECK(feed(decoder, command(session, 3, 2), 140, collector) == 0);
    collector.accept = true;

    CHECK(feed(decoder, command(session, 2, 2), 140, collector) == 0); // Replay.
    CHECK(feed(decoder, command(session + 1, 3, 2), 140, collector) == 0);
    CHECK(feed(decoder, command(session, 3, 1, {{1, 2, 0, std::nanf("")}}), 140, collector) == 0);
    CHECK(feed(decoder, command(session, 3, 1, {{1, 2, 0, 1.0f}, {1, 2, 0, 2.0f}}), 140, collector) == 0);
    CHECK(decoder.last_sequence() == 2 && decoder.watchdog_expired(140, 100));

    auto bad_crc = command(session, 3, 2);
    bad_crc.back() ^= 0x40;
    auto valid = command(session, 3, 2);
    bad_crc.insert(bad_crc.end(), valid.begin(), valid.end());
    CHECK(feed(decoder, bad_crc, 141, collector) == 1);
    CHECK(decoder.statistics().bad_crc == 1 && decoder.last_sequence() == 3);

    auto bogus_length = begin;
    bogus_length[6] = 0xff;
    bogus_length[7] = 0xff;
    bogus_length.insert(bogus_length.end(), begin.begin(), begin.end());
    CHECK(feed(decoder, bogus_length, 150, collector) == 0);
    CHECK(decoder.statistics().bad_length >= 1 && decoder.latched());
    CHECK(decoder.last_sequence() == 0 && decoder.watchdog_expired(150, 100));

    auto wrong = fingerprint;
    wrong[0] ^= 1;
    CHECK(feed(decoder, frame(1, payload(wire::SessionBegin{session + 2, wrong})), 160, collector) == 0);
    CHECK(!decoder.model_matches() && decoder.latched());
    CHECK(feed(decoder, command(session + 2, 1, 4), 170, collector) == 0);
    CHECK(decoder.last_sequence() == 0 && decoder.watchdog_expired(170, 100));

    constexpr std::array<unsigned, 4> rates{115200, 230400, 460800, 921600};
    CHECK(v5::max_command_payload == 532 && v5::max_state_payload == 796 && v5::max_frame_size == 808);
    for (auto baud : rates) {
        const auto command_us = (10ULL * (v5::header_size + v5::max_command_payload + v5::crc_size) * 1000000 + baud - 1) / baud;
        const auto state_us = (10ULL * v5::max_frame_size * 1000000 + baud - 1) / baud;
        CHECK(v5::response_deadline_us(baud) + 1 >= command_us + state_us + 30000);
        CHECK(v5::response_deadline_us(baud) <= command_us + state_us + 30000);
    }
    CHECK(v5::response_deadline_us(9600) == 0);
    CHECK(v5::response_deadline_us(115200) == 147362);
}
