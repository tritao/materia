// The collision world through the C ABI: shapes posed by forward kinematics, pair statuses
// (rigid, adjacent, static, declared, overlapping at reference, unsupported), signed
// distances with closest points, and scene changes (re-attaching, height-field updates).
#include "kinematicskit.h"

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

/**
 * Base (body 0) -> revolute z -> link1 (body 1) -> revolute z at x = 1 -> link2 (body 2)
 * -> fixed at x = 1 -> tool (body 3).
 */
kk_model_handle make_arm() {
    std::vector<int32_t> ints = {1, 4, 3, 2, 0,
                                 -1, 0, 1, 2,
                                 0, 1, 2, 3,
                                 1, 0, 1, 0, -1,
                                 1, 1, 2, 1, -1,
                                 0, 2, 3, -1, -1,
                                 0, 1, 2,
                                 0, 1, 2};
    std::vector<double> reals;
    for (int body = 0; body < 4; ++body) reals.insert(reals.end(), kIdentity, kIdentity + 7);
    const double joint_x[3] = {0.0, 1.0, 1.0};
    for (int joint = 0; joint < 3; ++joint) {
        auto parent = at(joint_x[joint], 0, 0);
        reals.insert(reals.end(), parent.begin(), parent.end());
        reals.insert(reals.end(), kIdentity, kIdentity + 7);
        reals.insert(reals.end(), {0.0, 0.0, 1.0, 1.0, 0.0, 1.0});
    }
    kk_model_handle model{0};
    const kk_result created = kk_model_create(ints.data(), uint32_t(ints.size()), reals.data(),
                                              uint32_t(reals.size()), &model);
    expect(created == KK_OK, "model created");
    return model;
}

uint32_t shape(kk_collision_world_handle world, int32_t body, const std::vector<double> &offset, int32_t kind,
               std::vector<double> params) {
    uint32_t id = 0;
    const kk_result result = kk_collision_add_shape(world, body, offset.data(), 7, kind, params.data(),
                                                    uint32_t(params.size()), &id);
    expect(result == KK_OK && id != 0, "shape added");
    return id;
}

int32_t status(kk_collision_world_handle world, uint32_t a, uint32_t b) {
    int32_t value = -1;
    expect(kk_collision_pair_status(world, a, b, &value) == KK_OK, "pair status");
    return value;
}

void update(kk_collision_world_handle world, double q0, double q1) {
    const double q[2] = {q0, q1};
    expect(kk_collision_update(world, q, 2, nullptr, 0) == KK_OK, "update");
}

/** The distance row (10 doubles) for pair (a, b) within `query`, or an empty vector. */
std::vector<double> distance_of(kk_collision_world_handle world, uint32_t a, uint32_t b, double query) {
    std::vector<int32_t> pairs(64);
    std::vector<double> rows(320);
    uint32_t count = 0;
    const kk_result result = kk_collision_distances(world, query, pairs.data(), uint32_t(pairs.size()), rows.data(),
                                                    uint32_t(rows.size()), &count);
    if (result != KK_OK) std::printf("distances returned %d\n", result);
    expect(result == KK_OK, "distances");
    for (uint32_t i = 0; i < count && i < 32; ++i)
        if (uint32_t(pairs[2 * i]) == (a < b ? a : b) && uint32_t(pairs[2 * i + 1]) == (a < b ? b : a))
            return std::vector<double>(rows.begin() + 10 * i, rows.begin() + 10 * i + 10);
    return {};
}

std::vector<std::pair<uint32_t, uint32_t>> colliding(kk_collision_world_handle world, double margin,
                                                     kk_result *result = nullptr) {
    std::vector<int32_t> pairs(64);
    uint32_t count = 0;
    const kk_result status = kk_collision_check(world, margin, pairs.data(), uint32_t(pairs.size()), &count);
    if (result) *result = status;
    else expect(status == KK_OK, "check");
    std::vector<std::pair<uint32_t, uint32_t>> out;
    for (uint32_t i = 0; i < count && i < 32; ++i) out.emplace_back(uint32_t(pairs[2 * i]), uint32_t(pairs[2 * i + 1]));
    return out;
}

} // namespace

int main() {
    kk_model_handle model = make_arm();
    kk_collision_world_handle world{0};
    expect(kk_collision_world_create(model, &world) == KK_OK, "world created");

    const uint32_t base = shape(world, 0, at(0, 0, 0), KK_SHAPE_BOX, {0.1, 0.1, 0.1});
    const uint32_t upper = shape(world, 1, at(0.5, 0, 0), KK_SHAPE_SPHERE, {0.1});
    const uint32_t fore = shape(world, 2, at(0.5, 0, 0), KK_SHAPE_CAPSULE, {0.05, 0.2});
    const uint32_t tool = shape(world, 3, at(0, 0, 0), KK_SHAPE_BOX, {0.1, 0.1, 0.1});
    const uint32_t obstacle = shape(world, -1, at(2.5, 0, 0), KK_SHAPE_BOX, {0.2, 0.2, 0.2});
    const uint32_t wall = shape(world, -1, at(-3, 0, 0), KK_SHAPE_CYLINDER, {0.5, 1.0});

    // Statuses from the model's structure.
    expect(status(world, fore, tool) == KK_PAIR_RIGID, "forearm and tool are rigidly joined");
    expect(status(world, upper, fore) == KK_PAIR_ADJACENT, "upper and forearm are one joint apart");
    expect(status(world, upper, tool) == KK_PAIR_ADJACENT, "upper and tool are one joint apart through a fixed joint");
    expect(status(world, base, fore) == KK_PAIR_CHECKED, "base and forearm are checked");
    expect(status(world, obstacle, wall) == KK_PAIR_STATIC, "two world objects are static");
    expect(status(world, tool, obstacle) == KK_PAIR_CHECKED, "tool and obstacle are checked");

    // Stretched along +x: the tool box (1.9..2.1) is 0.2 from the obstacle (2.3..2.7).
    update(world, 0.0, 0.0);
    auto row = distance_of(world, tool, obstacle, 0.5);
    expect(row.size() == 10, "tool-obstacle within query");
    if (row.size() == 10) {
        near(row[0], 0.2, 1e-9, "tool-obstacle distance");
        near(row[1], 2.1, 1e-9, "closest point on tool");
        near(row[4], 2.3, 1e-9, "closest point on obstacle");
        near(row[7], 1.0, 1e-9, "normal from tool to obstacle");
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
    if (row.size() == 10) near(row[0], std::hypot(2.3 - 1.1, 0.9 - 0.2), 1e-9, "tool-obstacle distance with the elbow bent");
    expect(distance_of(world, tool, obstacle, 0.5).empty(), "outside a small query");

    // Re-attach the obstacle into the tool's path: a collision with a negative signed distance.
    const auto overlap = at(2.05, 0, 0);
    expect(kk_collision_attach(world, obstacle, -1, overlap.data(), 7) == KK_OK, "obstacle moved");
    update(world, 0.0, 0.0);
    {
        auto hits = colliding(world, 0.0);
        expect(hits.size() == 1 && hits[0] == std::make_pair(tool, obstacle), "tool collides with the obstacle");
        row = distance_of(world, tool, obstacle, 0.0);
        expect(row.size() == 10 && row[0] < -0.2, "penetration is a negative distance");
        expect(distance_of(world, tool, obstacle, -0.2).size() == 10, "a negative query finds deep penetration");
        expect(distance_of(world, tool, obstacle, -0.3).empty(), "but not deeper than it is");
    }

    // Declared rules, then overlap at reference.
    expect(kk_collision_set_pair_rule(world, tool, obstacle, KK_PAIR_RULE_ALLOW) == KK_OK, "allow rule");
    expect(status(world, tool, obstacle) == KK_PAIR_ALLOWED, "pair allowed");
    expect(colliding(world, 0.0).empty(), "allowed pair not reported");
    expect(kk_collision_set_pair_rule(world, tool, obstacle, KK_PAIR_RULE_DEFAULT) == KK_OK, "default rule");
    expect(kk_collision_set_pair_rule(world, fore, tool, KK_PAIR_RULE_CHECK) == KK_OK, "check rule");
    expect(status(world, fore, tool) == KK_PAIR_CHECKED, "a check rule overrides rigidity");
    expect(kk_collision_set_pair_rule(world, fore, tool, KK_PAIR_RULE_DEFAULT) == KK_OK, "default rule again");
    {
        const double q[2] = {0.0, 0.0};
        uint32_t marked = 0;
        expect(kk_collision_allow_overlapping(world, q, 2, nullptr, 0, &marked) == KK_OK && marked == 1,
               "one pair overlaps at reference");
        expect(status(world, tool, obstacle) == KK_PAIR_OVERLAPS_AT_REFERENCE, "marked as overlapping at reference");
        const auto away = at(2.5, 0, 0);
        expect(kk_collision_attach(world, obstacle, -1, away.data(), 7) == KK_OK, "obstacle moved away");
        expect(kk_collision_allow_overlapping(world, q, 2, nullptr, 0, &marked) == KK_OK && marked == 0,
               "marks are replaced");
        expect(status(world, tool, obstacle) == KK_PAIR_CHECKED, "checked again");
    }

    // A triangle mesh floor, a convex block, a half-space and a height field.
    {
        const std::vector<double> floor = {-5, -5, -0.5, 5, -5, -0.5, 5, 5, -0.5, -5, 5, -0.5};
        const std::vector<int32_t> triangles = {0, 1, 2, 0, 2, 3};
        uint32_t mesh = 0;
        expect(kk_collision_add_mesh(world, -1, kIdentity, 7, floor.data(), uint32_t(floor.size()), triangles.data(),
                                     uint32_t(triangles.size()), &mesh) == KK_OK,
               "mesh added");
        update(world, 0.0, 0.0);
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
        expect(kk_collision_add_convex(world, -1, block_at.data(), 7, cube.data(), uint32_t(cube.size()), &block) == KK_OK,
               "convex added");
        row = distance_of(world, upper, block, 1.0);
        expect(row.size() == 10, "upper-block within query");
        if (row.size() == 10) near(row[0], 0.3, 1e-6, "sphere to convex block");
        const std::vector<double> flat = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        uint32_t rejected = 0;
        expect(kk_collision_add_convex(world, -1, kIdentity, 7, flat.data(), uint32_t(flat.size()), &rejected) ==
                   KK_ERROR_INVALID_ARGUMENT,
               "coplanar points are not a convex solid");

        uint32_t below = shape(world, -1, at(0, 0, 0), KK_SHAPE_HALFSPACE, {0, 0, 1, -2});
        row = distance_of(world, base, below, 2.0);
        expect(row.size() == 10, "base-half-space within query");
        if (row.size() == 10) near(row[0], 1.9, 1e-6, "base above the half-space");

        const std::vector<double> zeros(9, 0.0);
        uint32_t terrain = 0;
        const auto terrain_at = at(0, 0, -1);
        expect(kk_collision_add_height_field(world, -1, terrain_at.data(), 7, 2.0, 2.0, zeros.data(), 9, 3, -1.0,
                                             &terrain) == KK_OK,
               "height field added");
        row = distance_of(world, base, terrain, 2.0);
        expect(row.size() == 10, "base-terrain within query");
        if (row.size() == 10) near(row[0], 0.9, 1e-6, "base above flat terrain");
        // Height field against a mesh, through the field's cells (both are static, so force the check).
        expect(kk_collision_set_pair_rule(world, mesh, terrain, KK_PAIR_RULE_CHECK) == KK_OK, "check floor-terrain");
        row = distance_of(world, mesh, terrain, 1.0);
        expect(row.size() == 10, "floor-terrain within query");
        if (row.size() == 10) {
            near(row[0], 0.5, 1e-6, "floor mesh above flat terrain");
            near(row[9], -1.0, 1e-6, "normal from the floor down to the terrain");
        }
        const std::vector<double> raised(9, 0.5);
        expect(kk_collision_set_heights(world, terrain, raised.data(), 9) == KK_OK, "heights updated");
        row = distance_of(world, base, terrain, 2.0);
        if (row.size() == 10) near(row[0], 0.4, 1e-6, "base above raised terrain");
        expect(kk_collision_set_heights(world, terrain, raised.data(), 8) == KK_ERROR_INVALID_ARGUMENT,
               "a height update keeps the grid");

        // Penetrating terrain: the raised surface (z = -0.5) cuts the floor mesh's plane.
        const std::vector<double> buried(9, 0.6);
        expect(kk_collision_set_heights(world, terrain, buried.data(), 9) == KK_OK, "heights above the floor");
        {
            auto hits = colliding(world, 0.0);
            bool found = false;
            for (auto &hit : hits) found = found || hit == std::make_pair(mesh, terrain);
            expect(found, "terrain above the floor collides with it");
        }
        expect(kk_collision_set_heights(world, terrain, raised.data(), 9) == KK_OK, "heights back");
        expect(kk_collision_set_pair_rule(world, mesh, terrain, KK_PAIR_RULE_DEFAULT) == KK_OK, "floor-terrain static");

        // Two height fields cannot be compared: a check on them is refused, not skipped.
        uint32_t other_terrain = 0;
        expect(kk_collision_add_height_field(world, 3, kIdentity, 7, 1.0, 1.0, zeros.data(), 9, 3, -1.0,
                                             &other_terrain) == KK_OK,
               "a height field on the tool");
        expect(status(world, other_terrain, terrain) == KK_PAIR_UNSUPPORTED, "height field pairs are unsupported");
        kk_result result = KK_OK;
        colliding(world, 0.0, &result);
        expect(result == KK_ERROR_UNSUPPORTED, "queries refuse an unsupported checked pair");
        expect(kk_collision_set_pair_rule(world, other_terrain, terrain, KK_PAIR_RULE_ALLOW) == KK_OK, "allow it");
        colliding(world, 0.0, &result);
        expect(result == KK_OK, "queries run once it is allowed");
        expect(kk_collision_remove(world, other_terrain) == KK_OK, "height field removed");
        expect(kk_collision_pair_status(world, other_terrain, terrain, nullptr) == KK_ERROR_INVALID_ARGUMENT,
               "a removed object has no status");
    }

    // Arguments and handles.
    {
        uint32_t id = 0;
        const double bad[7] = {0, 0, 0, 0, 0, 0, 0};
        const double radius = 0.1;
        expect(kk_collision_add_shape(world, 0, bad, 7, KK_SHAPE_SPHERE, &radius, 1, &id) == KK_ERROR_INVALID_ARGUMENT,
               "a zero quaternion is refused");
        expect(kk_collision_add_shape(world, 4, kIdentity, 7, KK_SHAPE_SPHERE, &radius, 1, &id) ==
                   KK_ERROR_INVALID_ARGUMENT,
               "an unknown body is refused");
        expect(kk_collision_add_shape(world, 0, kIdentity, 7, 99, &radius, 1, &id) == KK_ERROR_INVALID_ARGUMENT,
               "an unknown kind is refused");
        const double q[1] = {0.0};
        expect(kk_collision_update(world, q, 1, nullptr, 0) == KK_ERROR_INVALID_ARGUMENT, "wrong DOF count");
    }
    kk_collision_world_destroy(world);
    kk_collision_world_handle stale = world;
    expect(kk_collision_update(stale, nullptr, 0, nullptr, 0) == KK_ERROR_INVALID_HANDLE, "destroyed world");
    kk_model_destroy(model);

    if (failures) {
        std::printf("%d collision world checks failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("collision world: all checks passed\n");
    return EXIT_SUCCESS;
}
