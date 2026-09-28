#pragma once

#include "profile.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace stockkit {

enum Zone : uint32_t { kZoneCutting = 0, kZoneShank = 1, kZoneHolder = 2 };
constexpr uint32_t kZoneCount = 3;

/** A stretch of the tool above its flutes that must not touch stock. */
struct Band {
    uint32_t zone;
    Profile profile;
};

/**
 * A tool whose profile segments are marked cutting, shank or holder. The
 * cutting segments run from the tip; only they remove material. The
 * segments above them form bands, one per run of segments in the same zone,
 * whose contact with stock is measured.
 */
struct Tool {
    Profile cutting;
    std::vector<Band> bands;

    static bool build(const std::vector<Segment> &segments, const std::vector<uint32_t> &zones, Tool &out,
        std::string &error);
};

} // namespace stockkit
