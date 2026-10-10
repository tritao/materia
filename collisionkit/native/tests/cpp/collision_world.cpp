// The collision world through the C ABI: bodies posed by the caller, pair statuses (static,
// rigid, declared rules with reasons, overlapping at reference within one articulation,
// unsupported), signed distances with closest points, inflation, margin and inflation
// violations (single and batched), and scene changes (re-attaching, height-field updates).
#include "collisionkit.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char *what) {
    if (!condition) {
        std::printf("FAILED: %s\n", what);
        ++failures;
    }
}

void near(double actual, double expected, double tolerance, const char *what) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::printf("FAILED: %s: %.9f, expected %.9f\n", what, actual, expected);
        ++failures;
    }
}

const double kIdentity[7] = {0, 0, 0, 0, 0, 0, 1};

std::vector<double> at(double x, double y, double z) { return {x, y, z, 0, 0, 0, 1}; }

/** A pose at (x, y, z) turned by `angle` about z. */
std::vector<double> turned(double x, double y, double z, double angle) {
    return {x, y, z, 0, 0, std::sin(angle / 2), std::cos(angle / 2)};
}

/**
 * Base (body 0) -> revolute z -> link1 (body 1) -> revolute z at x = 1 -> link2 (body 2)
 * -> fixed at x = 1 -> tool (body 3): every body's pose for (q0, q1), as the caller's
 * kinematics would give them.
 */
std::vector<double> arm_poses(double q0, double q1) {
    std::vector<double> poses = at(0, 0, 0);
    auto upper = turned(0, 0, 0, q0);
    auto fore = turned(std::cos(q0), std::sin(q0), 0, q0 + q1);
    auto tool = turned(std::cos(q0) + std::cos(q0 + q1), std::sin(q0) + std::sin(q0 + q1), 0, q0 + q1);
    for (auto *pose : {&upper, &fore, &tool}) poses.insert(poses.end(), pose->begin(), pose->end());
    return poses;
}

uint32_t shape(ck_world_handle world, int32_t body, const std::vector<double> &offset, int32_t kind,
               std::vector<double> params) {
    uint32_t id = 0;
    const ck_result result =
        ck_add_shape(world, body, offset.data(), 7, kind, params.data(), uint32_t(params.size()), &id);
    expect(result == CK_OK && id != 0, "shape added");
    return id;
}

int32_t status(ck_world_handle world, uint32_t a, uint32_t b) {
    int32_t value = -1;
    expect(ck_pair_status(world, a, b, &value) == CK_OK, "pair status");
    return value;
}

void update(ck_world_handle world, double q0, double q1) {
    const auto poses = arm_poses(q0, q1);
    expect(ck_set_body_poses(world, 0, poses.data(), uint32_t(poses.size())) == CK_OK, "poses set");
}

/** The distance row (10 doubles) for pair (a, b) within `query`, or an empty vector; `bodies` gets its bodies. */
std::vector<double> distance_of(ck_world_handle world, uint32_t a, uint32_t b, double query,
                                std::vector<int32_t> *bodies = nullptr) {
    std::vector<int32_t> pairs(128);
    std::vector<double> rows(320);
    uint32_t count = 0;
    const ck_result result =
        ck_distances(world, query, pairs.data(), uint32_t(pairs.size()), rows.data(), uint32_t(rows.size()), &count);
    if (result != CK_OK) std::printf("distances returned %d\n", result);
    expect(result == CK_OK, "distances");
    for (uint32_t i = 0; i < count && i < 32; ++i)
        if (uint32_t(pairs[4 * i]) == (a < b ? a : b) && uint32_t(pairs[4 * i + 1]) == (a < b ? b : a)) {
            if (bodies) *bodies = {pairs[4 * i + 2], pairs[4 * i + 3]};
            return std::vector<double>(rows.begin() + 10 * i, rows.begin() + 10 * i + 10);
        }
    return {};
}

std::vector<std::pair<uint32_t, uint32_t>> colliding(ck_world_handle world, double margin,
                                                     ck_result *result = nullptr) {
    std::vector<int32_t> pairs(128);
    uint32_t count = 0;
    const ck_result status = ck_check(world, margin, pairs.data(), uint32_t(pairs.size()), &count);
    if (result) *result = status;
    else expect(status == CK_OK, "check");
    std::vector<std::pair<uint32_t, uint32_t>> out;
    for (uint32_t i = 0; i < count && i < 32; ++i) out.emplace_back(uint32_t(pairs[4 * i]), uint32_t(pairs[4 * i + 1]));
    return out;
}

/** A violation: found, the four ids and (distance, required). */
struct Violation {
    bool found = false;
    int32_t pair[4] = {0, 0, 0, 0};
    double result[2] = {0, 0};
};

Violation violation(ck_world_handle world, const std::vector<double> &margins, const std::vector<double> &inflation) {
    Violation v;
    int32_t found = 0;
    expect(ck_violation(world, margins.data(), uint32_t(margins.size()), inflation.data(), uint32_t(inflation.size()),
                        v.pair, 4, v.result, 2, &found) == CK_OK,
           "violation query");
    v.found = found != 0;
    return v;
}

} // namespace

int main() {
    ck_world_handle world{0};
    expect(ck_world_create(&world) == CK_OK, "world created");
    uint32_t body = 99;
    for (int32_t i = 0; i < 4; ++i) {
        expect(ck_add_body(world, 0, &body) == CK_OK && body == uint32_t(i), "bodies numbered in order");
    }
    expect(ck_add_body(world, -1, &body) == CK_ERROR_INVALID_ARGUMENT, "a negative group is refused");
    // The caller's kinematics says which bodies never move apart or are one joint apart (CL-D3).
    expect(ck_set_body_static(world, 0, 1) == CK_OK, "base is static");
    expect(ck_set_body_rule(world, 2, 3, CK_RULE_ALLOW, CK_PAIR_RIGID) == CK_OK, "fore-tool rigid");
    expect(ck_set_body_rule(world, 0, 1, CK_RULE_ALLOW, CK_PAIR_ADJACENT) == CK_OK, "base-upper adjacent");
    expect(ck_set_body_rule(world, 1, 2, CK_RULE_ALLOW, CK_PAIR_ADJACENT) == CK_OK, "upper-fore adjacent");
    expect(ck_set_body_rule(world, 1, 3, CK_RULE_ALLOW, CK_PAIR_ADJACENT) == CK_OK, "upper-tool adjacent");
    expect(ck_set_body_rule(world, 1, 3, CK_RULE_ALLOW, CK_PAIR_CHECKED) == CK_ERROR_INVALID_ARGUMENT,
           "an allow rule needs a reason");
    expect(ck_set_body_rule(world, 1, 1, CK_RULE_ALLOW, CK_PAIR_ALLOWED) == CK_ERROR_INVALID_ARGUMENT,
           "a body has no rule with itself");

    const uint32_t base = shape(world, 0, at(0, 0, 0), CK_SHAPE_BOX, {0.1, 0.1, 0.1});
    const uint32_t upper = shape(world, 1, at(0.5, 0, 0), CK_SHAPE_SPHERE, {0.1});
    const uint32_t fore = shape(world, 2, at(0.5, 0, 0), CK_SHAPE_CAPSULE, {0.05, 0.2});
    const uint32_t tool = shape(world, 3, at(0, 0, 0), CK_SHAPE_BOX, {0.1, 0.1, 0.1});
    const uint32_t obstacle = shape(world, -1, at(2.5, 0, 0), CK_SHAPE_BOX, {0.2, 0.2, 0.2});
    const uint32_t wall = shape(world, -1, at(-3, 0, 0), CK_SHAPE_CYLINDER, {0.5, 1.0});

    // Statuses from the declared rules, static bodies and shared bodies.
    expect(status(world, fore, tool) == CK_PAIR_RIGID, "forearm and tool are rigidly joined");
    expect(status(world, upper, fore) == CK_PAIR_ADJACENT, "upper and forearm are one joint apart");
    expect(status(world, upper, tool) == CK_PAIR_ADJACENT, "upper and tool are one joint apart through a fixed joint");
    expect(status(world, base, fore) == CK_PAIR_CHECKED, "base and forearm are checked");
    expect(status(world, obstacle, wall) == CK_PAIR_STATIC, "two world objects are static");
    expect(status(world, base, wall) == CK_PAIR_STATIC, "a static body and the world are static");
    expect(status(world, tool, obstacle) == CK_PAIR_CHECKED, "tool and obstacle are checked");
    {
        const uint32_t second = shape(world, 2, at(0.2, 0, 0), CK_SHAPE_SPHERE, {0.01});
        expect(status(world, fore, second) == CK_PAIR_RIGID, "two objects on one body are rigid");
        expect(ck_remove(world, second) == CK_OK, "removed");
    }

    // Stretched along +x: the tool box (1.9..2.1) is 0.2 from the obstacle (2.3..2.7).
    update(world, 0.0, 0.0);
    std::vector<int32_t> bodies;
    auto row = distance_of(world, tool, obstacle, 0.5, &bodies);
    expect(row.size() == 10, "tool-obstacle within query");
    if (row.size() == 10) {
        near(row[0], 0.2, 1e-9, "tool-obstacle distance");
        near(row[1], 2.1, 1e-9, "closest point on tool");
        near(row[4], 2.3, 1e-9, "closest point on obstacle");
        near(row[7], 1.0, 1e-9, "normal from tool to obstacle");
        expect(bodies.size() == 2 && bodies[0] == 3 && bodies[1] == -1, "results name the bodies");
    }
    expect(colliding(world, 0.0).empty(), "nothing collides stretched out");
    {
        auto within = colliding(world, 0.25);
        expect(within.size() == 1 && within[0] == std::make_pair(tool, obstacle), "margin 0.25 reports the tool");
    }

    // Elbow at 90 degrees: the tool box moves to (1, 1, 0), 1.2 from the obstacle along x and 0.7 along y.
    update(world, 0.0, M_PI / 2);
    row = distance_of(world, tool, obstacle, 10.0);
    expect(row.size() == 10, "tool-obstacle within large query");
    if (row.size() == 10)
        near(row[0], std::hypot(2.3 - 1.1, 0.9 - 0.2), 1e-9, "tool-obstacle distance with the elbow bent");
    expect(distance_of(world, tool, obstacle, 0.5).empty(), "outside a small query");

    // Chosen pairs, whatever their status, in the order given.
    {
        const int32_t pairs[4] = {int32_t(obstacle), int32_t(tool), int32_t(fore), int32_t(tool)};
        double rows[20];
        expect(ck_pair_distances(world, pairs, 4, rows, 20) == CK_OK, "pair distances");
        near(rows[0], std::hypot(2.3 - 1.1, 0.9 - 0.2), 1e-9, "obstacle-tool distance by pair");
        expect(rows[7] < 0.0, "the normal points from the first object given");
        near(rows[10], 0.35, 1e-9, "a rigid pair is measured when asked");
    }

    // Re-attach the obstacle into the tool's path: a collision with a negative signed distance.
    const auto overlap = at(2.05, 0, 0);
    expect(ck_attach(world, obstacle, -1, overlap.data(), 7) == CK_OK, "obstacle moved");
    update(world, 0.0, 0.0);
    {
        auto hits = colliding(world, 0.0);
        expect(hits.size() == 1 && hits[0] == std::make_pair(tool, obstacle), "tool collides with the obstacle");
        row = distance_of(world, tool, obstacle, 0.0);
        expect(row.size() == 10 && row[0] < -0.2, "penetration is a negative distance");
        expect(distance_of(world, tool, obstacle, -0.2).size() == 10, "a negative query finds deep penetration");
        expect(distance_of(world, tool, obstacle, -0.3).empty(), "but not deeper than it is");
    }

    // Declared rules on objects override body rules and defaults.
    expect(ck_set_object_rule(world, tool, obstacle, CK_RULE_ALLOW, CK_PAIR_PROCESS_CONTACT) == CK_OK, "allow rule");
    expect(status(world, tool, obstacle) == CK_PAIR_PROCESS_CONTACT, "pair allowed for process contact");
    expect(colliding(world, 0.0).empty(), "allowed pair not reported");
    expect(ck_set_object_rule(world, tool, obstacle, CK_RULE_DEFAULT, 0) == CK_OK, "default rule");
    expect(ck_set_object_rule(world, fore, tool, CK_RULE_CHECK, 0) == CK_OK, "check rule");
    expect(status(world, fore, tool) == CK_PAIR_CHECKED, "an object check rule overrides a body rule");
    expect(ck_set_object_rule(world, fore, tool, CK_RULE_DEFAULT, 0) == CK_OK, "default rule again");

    // Overlap at reference applies within the articulation only: the obstacle stays checked.
    {
        const auto into = at(0.5, 0, 0);
        const uint32_t sleeve = shape(world, 2, into, CK_SHAPE_SPHERE, {0.1});
        expect(ck_set_body_rule(world, 1, 2, CK_RULE_DEFAULT, 0) == CK_OK, "upper-fore without a rule");
        expect(status(world, upper, sleeve) == CK_PAIR_CHECKED, "upper and sleeve checked");
        const int32_t arm[4] = {0, 1, 2, 3};
        uint32_t marked = 0;
        expect(ck_allow_overlapping(world, arm, 4, &marked) == CK_OK, "allow overlapping");
        // The sleeve sits on the forearm at (1.5, 0, 0), clear of the upper arm's sphere at (0.5, 0, 0).
        expect(marked == 0, "nothing overlaps within the arm stretched out");
        const auto inside = at(-0.5, 0, 0);
        expect(ck_attach(world, sleeve, 2, inside.data(), 7) == CK_OK, "sleeve moved onto the upper sphere");
        expect(ck_allow_overlapping(world, arm, 4, &marked) == CK_OK && marked == 1, "one arm pair overlaps");
        expect(status(world, upper, sleeve) == CK_PAIR_OVERLAPS_AT_REFERENCE, "marked as overlapping at reference");
        expect(status(world, tool, obstacle) == CK_PAIR_CHECKED, "the environment overlap stays checked");
        auto hits = colliding(world, 0.0);
        expect(hits.size() == 1 && hits[0] == std::make_pair(tool, obstacle), "and is reported as a layout error");
        const int32_t other[1] = {0};
        expect(ck_allow_overlapping(world, other, 1, &marked) == CK_OK && marked == 0, "another set");
        expect(status(world, upper, sleeve) == CK_PAIR_OVERLAPS_AT_REFERENCE, "leaves the arm's marks");
        const auto away = at(0.5, 0, 0);
        expect(ck_attach(world, sleeve, 2, away.data(), 7) == CK_OK, "sleeve moved back");
        expect(status(world, upper, sleeve) == CK_PAIR_CHECKED, "re-attaching drops its marks");
        expect(ck_remove(world, sleeve) == CK_OK, "sleeve removed");
        expect(ck_set_body_rule(world, 1, 2, CK_RULE_ALLOW, CK_PAIR_ADJACENT) == CK_OK, "upper-fore adjacent again");
    }
    const auto clear = at(2.5, 0, 0);
    expect(ck_attach(world, obstacle, -1, clear.data(), 7) == CK_OK, "obstacle back");

    // A held part follows its body's rules: grasped by the tool, it is rigid with the forearm.
    {
        const uint32_t part = shape(world, -1, at(0, 2, 0), CK_SHAPE_BOX, {0.05, 0.05, 0.05});
        expect(status(world, part, fore) == CK_PAIR_CHECKED, "a part on the table is checked against the arm");
        const auto grip = at(0.15, 0, 0);
        expect(ck_attach(world, part, 3, grip.data(), 7) == CK_OK, "grasped");
        expect(status(world, part, fore) == CK_PAIR_RIGID, "held: rigid with the forearm through the tool's rule");
        expect(status(world, part, upper) == CK_PAIR_ADJACENT, "held: adjacent to the upper arm");
        expect(status(world, part, tool) == CK_PAIR_RIGID, "held: on the tool's body");
        expect(status(world, part, base) == CK_PAIR_CHECKED, "held: checked against the base");
        update(world, 0.0, 0.0);
        row = distance_of(world, part, obstacle, 1.0);
        expect(row.size() == 10, "held part near the obstacle");
        if (row.size() == 10) near(row[0], 2.3 - 2.2, 1e-9, "the held part rides on the tool");
        expect(ck_remove(world, part) == CK_OK, "part removed");
    }

    // Inflation: every distance to an inflated primitive or convex shrinks by exactly its radius.
    {
        update(world, 0.0, 0.0);
        expect(ck_set_inflation(world, tool, 0.05) == CK_OK, "tool inflated");
        row = distance_of(world, tool, obstacle, 0.5);
        expect(row.size() == 10, "inflated tool within query");
        if (row.size() == 10) near(row[0], 0.15, 1e-9, "inflated box to box");
        expect(ck_set_inflation(world, obstacle, 0.02) == CK_OK, "obstacle inflated");
        row = distance_of(world, tool, obstacle, 0.5);
        if (row.size() == 10) near(row[0], 0.13, 1e-9, "two inflated boxes");
        auto hits = colliding(world, 0.14);
        expect(hits.size() == 1, "an inflated pair is within a margin it was not");
        expect(ck_set_inflation(world, tool, 0.0) == CK_OK && ck_set_inflation(world, obstacle, 0.0) == CK_OK,
               "inflation removed");
        expect(ck_set_inflation(world, tool, -0.1) == CK_ERROR_INVALID_ARGUMENT, "negative inflation refused");
    }

    // Violations: margins per group pair, plus each body's inflation; the closest pair.
    {
        update(world, 0.0, 0.0);
        expect(ck_set_body_group(world, 3, 1) == CK_OK, "the tool is its own group");
        const std::vector<double> none;
        std::vector<double> margins = {0.1, 0.1, 0.0, 0.1};
        expect(!violation(world, margins, none).found, "the tool clears 0.1 from the obstacle");
        std::vector<double> inflation = {0.0, 0.0, 0.0, 0.11};
        Violation v = violation(world, margins, inflation);
        expect(v.found && v.pair[0] == int32_t(tool) && v.pair[1] == int32_t(obstacle) && v.pair[2] == 3 &&
                   v.pair[3] == -1,
               "the tool's inflation brings it within the margin");
        near(v.result[0], 0.2, 1e-9, "violation distance");
        near(v.result[1], 0.21, 1e-12, "required: the group margin plus the tool's inflation");
        margins = {0.1, 0.3, 0.0, 0.1};
        v = violation(world, margins, none);
        expect(v.found && v.pair[0] == int32_t(tool), "the tool's group margin to the world group");
        near(v.result[1], 0.3, 1e-12, "required from the table's upper triangle");
        margins = {0.1, 0.1, 0.5, 0.1};
        expect(!violation(world, margins, none).found, "entries below the diagonal are not read");
        int32_t pair[4];
        double result[2];
        int32_t found = 0;
        expect(ck_closest(world, margins.data(), 4, pair, 4, result, 2, &found) == CK_OK && found == 1, "closest");
        expect(pair[0] == int32_t(tool) && pair[1] == int32_t(obstacle), "the tool is closest to the obstacle");
        near(result[0], 0.2, 1e-9, "closest distance");
        const std::vector<double> small = {0.1};
        expect(ck_violation(world, small.data(), 1, nullptr, 0, pair, 4, result, 2, &found) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "the margin table must cover every group");
        expect(ck_violation(world, margins.data(), 3, nullptr, 0, pair, 4, result, 2, &found) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "the margin table is square");
        expect(ck_violation(world, margins.data(), 4, inflation.data(), 3, pair, 4, result, 2, &found) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "inflation covers every body");

        // Batched: three pose sets; the elbow bent, stretched, then stretched with inflation.
        std::vector<double> poses = arm_poses(0.0, M_PI / 2);
        const std::vector<double> stretched = arm_poses(0.0, 0.0);
        for (int copy = 0; copy < 2; ++copy) poses.insert(poses.end(), stretched.begin(), stretched.end());
        std::vector<double> inflations(12, 0.0);
        inflations[11] = 0.11;
        int32_t set = 99;
        margins = {0.1, 0.1, 0.0, 0.1};
        expect(ck_violation_batch(world, poses.data(), uint32_t(poses.size()), inflations.data(), 12, margins.data(), 4,
                                  pair, 4, result, 2, &set) == CK_OK,
               "batch");
        expect(set == 2 && pair[0] == int32_t(tool), "the third set fails, on its inflation");
        near(result[1], 0.21, 1e-12, "batch required");
        expect(ck_violation_batch(world, poses.data(), 28, nullptr, 0, margins.data(), 4, pair, 4, result, 2, &set) ==
                   CK_OK && set == -1,
               "the bent set alone is clear");
        expect(ck_violation_batch(world, poses.data(), 20, nullptr, 0, margins.data(), 4, pair, 4, result, 2, &set) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "a pose set covers every body");
        expect(ck_set_body_group(world, 3, 0) == CK_OK, "the tool back in group 0");
    }

    // A triangle mesh floor, a convex block, a half-space and a height field.
    {
        update(world, 0.0, 0.0);
        const std::vector<double> floor = {-5, -5, -0.5, 5, -5, -0.5, 5, 5, -0.5, -5, 5, -0.5};
        const std::vector<int32_t> triangles = {0, 1, 2, 0, 2, 3};
        uint32_t mesh = 0;
        expect(ck_add_mesh(world, -1, kIdentity, 7, floor.data(), uint32_t(floor.size()), triangles.data(),
                           uint32_t(triangles.size()), &mesh) == CK_OK,
               "mesh added");
        expect(ck_set_inflation(world, mesh, 0.1) == CK_ERROR_UNSUPPORTED, "a mesh cannot be inflated");
        expect(ck_set_body_static(world, 0, 0) == CK_OK, "base moving, to see it against the world");
        row = distance_of(world, base, mesh, 1.0);
        expect(row.size() == 10, "base-floor within query");
        if (row.size() == 10) near(row[0], 0.4, 1e-6, "base above the floor mesh");

        std::vector<double> cube;
        for (int i = 0; i < 8; ++i) {
            cube.push_back(i & 1 ? 0.1 : -0.1);
            cube.push_back(i & 2 ? 0.1 : -0.1);
            cube.push_back(i & 4 ? 0.1 : -0.1);
        }
        uint32_t block = 0;
        const auto block_at = at(0.5, 0.5, 0);
        expect(ck_add_convex(world, -1, block_at.data(), 7, cube.data(), uint32_t(cube.size()), &block) == CK_OK,
               "convex added");
        row = distance_of(world, upper, block, 1.0);
        expect(row.size() == 10, "upper-block within query");
        if (row.size() == 10) near(row[0], 0.3, 1e-6, "sphere to convex block");
        expect(ck_set_inflation(world, block, 0.04) == CK_OK, "convex inflated");
        row = distance_of(world, upper, block, 1.0);
        if (row.size() == 10) near(row[0], 0.26, 1e-6, "sphere to inflated convex block");
        expect(ck_set_inflation(world, block, 0.0) == CK_OK, "convex back");
        const std::vector<double> flat = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        uint32_t rejected = 0;
        expect(ck_add_convex(world, -1, kIdentity, 7, flat.data(), uint32_t(flat.size()), &rejected) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "coplanar points are not a convex solid");

        uint32_t below = shape(world, -1, at(0, 0, 0), CK_SHAPE_HALFSPACE, {0, 0, 1, -2});
        row = distance_of(world, base, below, 2.0);
        expect(row.size() == 10, "base-half-space within query");
        if (row.size() == 10) near(row[0], 1.9, 1e-6, "base above the half-space");

        const std::vector<double> zeros(9, 0.0);
        uint32_t terrain = 0;
        const auto terrain_at = at(0, 0, -1);
        expect(ck_add_height_field(world, -1, terrain_at.data(), 7, 2.0, 2.0, zeros.data(), 9, 3, -1.0, &terrain) ==
                   CK_OK,
               "height field added");
        const std::vector<double> deep(9, -2.0);
        uint32_t refused = 0;
        expect(ck_add_height_field(world, -1, terrain_at.data(), 7, 2.0, 2.0, deep.data(), 9, 3, -1.0, &refused) ==
                   CK_ERROR_INVALID_ARGUMENT,
               "heights below the minimum are refused");
        row = distance_of(world, base, terrain, 2.0);
        expect(row.size() == 10, "base-terrain within query");
        if (row.size() == 10) near(row[0], 0.9, 1e-6, "base above flat terrain");
        // Height field against a mesh, through the field's cells (both are static, so force the check).
        expect(ck_set_object_rule(world, mesh, terrain, CK_RULE_CHECK, 0) == CK_OK, "check floor-terrain");
        row = distance_of(world, mesh, terrain, 1.0);
        expect(row.size() == 10, "floor-terrain within query");
        if (row.size() == 10) {
            near(row[0], 0.5, 1e-6, "floor mesh above flat terrain");
            near(row[9], -1.0, 1e-6, "normal from the floor down to the terrain");
        }
        const std::vector<double> raised(9, 0.5);
        expect(ck_set_heights(world, terrain, raised.data(), 9) == CK_OK, "heights updated");
        row = distance_of(world, base, terrain, 2.0);
        if (row.size() == 10) near(row[0], 0.4, 1e-6, "base above raised terrain");
        expect(ck_set_heights(world, terrain, raised.data(), 8) == CK_ERROR_INVALID_ARGUMENT,
               "a height update keeps the grid");
        std::vector<double> dug(9, 0.5);
        dug[4] = -1.5;
        expect(ck_set_heights(world, terrain, dug.data(), 9) == CK_ERROR_INVALID_ARGUMENT,
               "a dig below the field's minimum is refused, not clamped");
        row = distance_of(world, base, terrain, 2.0);
        if (row.size() == 10) near(row[0], 0.4, 1e-6, "a refused update leaves the heights");

        // Penetrating terrain: the raised surface (z = -0.4) cuts the floor mesh's plane.
        const std::vector<double> buried(9, 0.6);
        expect(ck_set_heights(world, terrain, buried.data(), 9) == CK_OK, "heights above the floor");
        {
            auto hits = colliding(world, 0.0);
            bool found = false;
            for (auto &hit : hits) found = found || hit == std::make_pair(mesh, terrain);
            expect(found, "terrain above the floor collides with it");
        }
        expect(ck_set_heights(world, terrain, raised.data(), 9) == CK_OK, "heights back");
        expect(ck_set_object_rule(world, mesh, terrain, CK_RULE_DEFAULT, 0) == CK_OK, "floor-terrain static");

        // Two height fields cannot be compared: a check on them is refused, not skipped.
        uint32_t other_terrain = 0;
        expect(ck_add_height_field(world, 3, kIdentity, 7, 1.0, 1.0, zeros.data(), 9, 3, -1.0, &other_terrain) ==
                   CK_OK,
               "a height field on the tool");
        expect(status(world, other_terrain, terrain) == CK_PAIR_UNSUPPORTED, "height field pairs are unsupported");
        ck_result result = CK_OK;
        colliding(world, 0.0, &result);
        expect(result == CK_ERROR_UNSUPPORTED, "queries refuse an unsupported checked pair");
        {
            const std::vector<double> margins = {0.0};
            int32_t pair[4];
            double out[2];
            int32_t found = 0;
            expect(ck_violation(world, margins.data(), 1, nullptr, 0, pair, 4, out, 2, &found) == CK_ERROR_UNSUPPORTED,
                   "violations refuse it too");
        }
        expect(ck_set_object_rule(world, other_terrain, terrain, CK_RULE_ALLOW, CK_PAIR_ALLOWED) == CK_OK, "allow it");
        colliding(world, 0.0, &result);
        expect(result == CK_OK, "queries run once it is allowed");
        expect(ck_remove(world, other_terrain) == CK_OK, "height field removed");
        expect(ck_pair_status(world, other_terrain, terrain, nullptr) == CK_ERROR_INVALID_ARGUMENT,
               "a removed object has no status");
    }

    // A convex given by more points than coal's large-convex threshold (32), as CAD hulls are (up to 64): coal
    // must scan them rather than climb a neighbour graph it does not have (fixed in our fork, CL-D13).
    {
        std::vector<double> ball;
        const int n = 60;
        for (int i = 0; i < n; ++i) {
            const double z = 1.0 - 2.0 * (i + 0.5) / n, r = std::sqrt(1.0 - z * z), a = 2.399963 * i;
            ball.insert(ball.end(), {0.1 * r * std::cos(a), 0.1 * r * std::sin(a), 0.1 * z});
        }
        double top = -1;
        for (int i = 0; i < n; ++i) top = std::max(top, ball[3 * i + 2]);
        uint32_t big = 0;
        const auto big_at = at(0, 0, 3);
        expect(ck_add_convex(world, -1, big_at.data(), 7, ball.data(), uint32_t(ball.size()), &big) == CK_OK,
               "a 60-point convex");
        const auto plate_at = at(0, 0, 3.5);
        const uint32_t plate = shape(world, -1, plate_at, CK_SHAPE_BOX, {1, 1, 0.1});
        const int32_t pair[2] = {int32_t(big), int32_t(plate)};
        double row[10];
        expect(ck_pair_distances(world, pair, 2, row, 10) == CK_OK, "distance to the 60-point convex");
        near(row[0], 0.4 - top, 1e-9, "the 60-point convex's top below the plate");
        expect(ck_remove(world, big) == CK_OK && ck_remove(world, plate) == CK_OK, "removed");
    }

    // Arguments and handles.
    {
        uint32_t id = 0;
        const double bad[7] = {0, 0, 0, 0, 0, 0, 0};
        const double radius = 0.1;
        expect(ck_add_shape(world, 0, bad, 7, CK_SHAPE_SPHERE, &radius, 1, &id) == CK_ERROR_INVALID_ARGUMENT,
               "a zero quaternion is refused");
        expect(ck_add_shape(world, 4, kIdentity, 7, CK_SHAPE_SPHERE, &radius, 1, &id) == CK_ERROR_INVALID_ARGUMENT,
               "an unknown body is refused");
        expect(ck_add_shape(world, 0, kIdentity, 7, 99, &radius, 1, &id) == CK_ERROR_INVALID_ARGUMENT,
               "an unknown kind is refused");
        expect(ck_set_body_poses(world, 3, kIdentity, 14) == CK_ERROR_INVALID_ARGUMENT, "poses past the last body");
        expect(ck_set_body_poses(world, 0, bad, 7) == CK_ERROR_INVALID_ARGUMENT, "a bad pose is refused");
    }
    ck_world_destroy(world);
    ck_world_handle stale = world;
    expect(ck_set_body_poses(stale, 0, nullptr, 0) == CK_ERROR_INVALID_HANDLE, "destroyed world");

    if (failures) {
        std::printf("%d collision world checks failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("collision world: all checks passed\n");
    return EXIT_SUCCESS;
}
