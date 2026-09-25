#include "device_wire.hpp"

#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <map>
#include <span>
#include <string>
#include <vector>

#define CHECK(condition) do { if (!(condition)) std::abort(); } while (false)

using namespace robotkit::device_wire;

static std::map<std::string, std::vector<std::uint8_t>> read_vectors(const char *path) {
    std::ifstream file(path);
    CHECK(file.good());
    std::map<std::string, std::vector<std::uint8_t>> result;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        const auto tab = line.find('\t');
        CHECK(tab != std::string::npos);
        std::vector<std::uint8_t> bytes;
        for (std::size_t i = tab + 1; i < line.size(); i += 2)
            bytes.push_back(static_cast<std::uint8_t>(std::stoul(line.substr(i, 2), nullptr, 16)));
        result.emplace(line.substr(0, tab), bytes);
    }
    return result;
}

template<class T>
static void check(const std::map<std::string, std::vector<std::uint8_t>> &vectors,
                  const std::string &name, const T &value) {
    const auto &expected = vectors.at(name);
    CHECK(expected.size() == T::SIZE);
    std::vector<std::uint8_t> bytes(T::SIZE);
    CHECK(encode(value, bytes));
    CHECK(bytes == expected);
    T decoded{};
    CHECK(decode(bytes, decoded));
    std::vector<std::uint8_t> roundtrip(T::SIZE);
    CHECK(encode(decoded, roundtrip));
    CHECK(roundtrip == expected);
    CHECK(!encode(value, std::span<std::uint8_t>(bytes.data(), bytes.size() - 1)));
    CHECK(!decode(std::span<const std::uint8_t>(bytes.data(), bytes.size() - 1), decoded));
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    const auto vectors = read_vectors(argv[1]);
    const std::uint64_t session = 0x0102030405060708ULL;
    std::array<std::uint8_t, 16> fingerprint{};
    for (std::size_t i = 0; i < fingerprint.size(); ++i) fingerprint[i] = static_cast<std::uint8_t>(i);
    check(vectors, "SessionBegin", SessionBegin{session, fingerprint});
    check(vectors, "SessionAck", SessionAck{session, fingerprint, 1, {0, 0, 0}});
    check(vectors, "CommandHeader", CommandHeader{session, 9, 1, 2, 0});
    check(vectors, "JointTarget", JointTarget{0x1234, 2, 0, 1.5f});
    check(vectors, "StateHeader", StateHeader{session, 123456789, 9, 1, 0, 2, 0});
    check(vectors, "JointState", JointState{1.0f, -2.0f, 0.5f});
}
