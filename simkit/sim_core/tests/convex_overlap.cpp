#include "../src/convex_overlap.hpp"
#include <cstdlib>

using namespace nksim::geometry;

static void require(bool condition) { if (!condition) std::abort(); }

int main() {
    // Captured screw/fixture support hull: the ordinary affine projection
    // repeats a support point. Exact rational projection of the final triangle
    // gives distance 2.137696117298869 nm. Its plane separates every vertex.
    const std::vector<Point> captured{
        {0.04201410460508315, -0.0016689360377503681, -2.604275683992796e-09},
        {-0.03475909277744876, -0.0016690977056313394, -0.023999988083404256},
        {-0.034577356960917016, -0.0007556376190060443, 4.542784370897834e-09},
        {0.03962753869986024, 0.00071762916305812, -0.00400001981799758},
        {0.03962753869986024, 0.00071762916305812, -1.1711765890431991e-08},
        {-0.03237245820069823, 0.00071762916305812, -1.1711765890431991e-08},
        {0.03830163401827169, 0.00031542008410101463, -5.181478468507805e-09},
        {-0.03369836288228678, 0.00031542008410101463, -5.181478468507805e-09},
        {-0.034356814713867034, -0.0003430319418026295, 1.1380007491901267e-09},
        {-0.03406001471915021, 1.8619982651735256e-05, -2.145481015070061e-09}
    };
    const Point plane{-4.732703602640838e-9, -9.075231636383676e-6, -0.9999999999588199};
    for (const auto &point : captured) require(dot(plane, point) > 2e-9);
    auto support = [&](const Point &direction) {
        auto best = captured.front();
        for (const auto &point : captured)
            if (dot(point, direction) > dot(best, direction)) best = point;
        return Simplex::SupportPoint{best, best, Point{}};
    };
    Point witness{};
    require(overlap_with_witness(support, witness) == Overlap::separated);
    // Moving the same hull one nanometre toward contact crosses the unchanged
    // two-nanometre threshold; it must not become an unconditional separation.
    auto touching = [&](const Point &direction) {
        auto point = support(direction);
        point.difference[2] += 1e-9;
        point.first[2] += 1e-9;
        return point;
    };
    require(overlap_with_witness(touching, witness) == Overlap::intersecting);
}
