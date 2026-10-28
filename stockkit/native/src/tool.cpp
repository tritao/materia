#include "tool.hpp"

namespace stockkit {

bool Tool::build(const std::vector<Segment> &segments, const std::vector<uint32_t> &zones, Tool &out,
    std::string &error) {
    if (segments.size() != zones.size()) {
        error = "every profile segment needs a zone";
        return false;
    }
    // The whole profile must be valid on its own.
    Profile whole;
    if (!Profile::build(segments, whole, error)) return false;
    size_t flutes = 0;
    while (flutes < segments.size() && zones[flutes] == kZoneCutting) ++flutes;
    if (flutes == 0) {
        error = "tool profile must start with its cutting zone at the tip";
        return false;
    }
    for (size_t k = flutes; k < segments.size(); ++k)
        if (zones[k] == kZoneCutting) {
            error = "tool cutting zone must run from the tip without gaps";
            return false;
        } else if (zones[k] >= kZoneCount) {
            error = "unknown tool zone";
            return false;
        }
    Tool tool;
    if (!Profile::build({segments.begin(), segments.begin() + flutes}, tool.cutting, error)) return false;
    for (size_t k = flutes; k < segments.size();) {
        size_t end = k + 1;
        while (end < segments.size() && zones[end] == zones[k]) ++end;
        Band band{zones[k], {}};
        std::vector<Segment> part(segments.begin() + k, segments.begin() + end);
        // A band that is only a flat step (a shoulder) has no height of its own.
        if (part.back().z1 > part.front().z0) {
            if (!Profile::build(part, band.profile, error, false)) return false;
            tool.bands.push_back(std::move(band));
        }
        k = end;
    }
    out = std::move(tool);
    return true;
}

} // namespace stockkit
