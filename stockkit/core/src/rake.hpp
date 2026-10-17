#pragma once
#include "ring.hpp"
#include <array>
#include <vector>

namespace stockkit {
using RakePoint = std::array<double,3>;
struct RakeMesh {
    std::vector<RakePoint> vertices;
    std::vector<std::array<uint32_t,3>> triangles;
};
struct RakeTool { std::vector<RakeMesh> gashes; };
bool build_rake_tool(const double *vertices, uint32_t vertex_count, const sk_rake_region *regions,
    uint32_t region_count, uint32_t gashes, RakeTool &out);
void sweep_rake(RingStock &stock, const RakeTool &tool, uint32_t gash, uint32_t pass,
    const sk_rake_pose *poses, uint32_t pose_count);
}
