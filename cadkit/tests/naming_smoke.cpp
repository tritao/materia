// Topological names in the core (plans/TOPOLOGICAL_NAMING.md, TN1).
#include "cadkit.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

namespace {

std::vector<std::string> names(cad_shape shape, cad_shape_kind kind) {
    std::uint32_t capacity = 0;
    const auto probe = cad_shape_copy_element_names_bytes(shape, kind, nullptr, &capacity);
    assert(probe == CAD_OK || probe == CAD_ERROR_BUFFER_TOO_SMALL);
    std::string text(capacity, '\0');
    assert(cad_shape_copy_element_names_bytes(shape, kind, reinterpret_cast<std::uint8_t*>(text.data()), &capacity) == CAD_OK);
    std::vector<std::string> result;
    std::uint32_t count = 0;
    assert(cad_shape_subshape_count(shape, kind, &count) == CAD_OK);
    if (count == 0) return result;
    std::string current;
    for (char c : text) {
        if (c == '\n') {
            result.push_back(current);
            current.clear();
        } else {
            current += c;
        }
    }
    result.push_back(current);
    assert(result.size() == count);
    return result;
}

std::set<std::string> name_set(cad_shape shape, cad_shape_kind kind) {
    auto list = names(shape, kind);
    return std::set<std::string>(list.begin(), list.end());
}

bool weak(const std::string& name) {
    return name.find_first_of("#~@") != std::string::npos;
}

void expect_unique_strong(cad_shape shape, cad_shape_kind kind) {
    auto list = names(shape, kind);
    std::set<std::string> unique(list.begin(), list.end());
    if (unique.size() != list.size()) {
        for (const auto& name : list) std::fprintf(stderr, "  %s\n", name.c_str());
    }
    assert(unique.size() == list.size());
    for (const auto& name : list) {
        if (weak(name)) std::fprintf(stderr, "weak: %s\n", name.c_str());
        assert(!weak(name));
    }
}

bool contains(const std::set<std::string>& set, const std::string& name) {
    return set.count(name) != 0;
}

cad_shape box() {
    cad_shape shape = 0;
    assert(cad_box(10, 20, 30, &shape) == CAD_OK);
    return shape;
}

void check_primitives() {
    std::uint32_t version = 0;
    assert(cad_naming_scheme_version(&version) == CAD_OK && version == 1);

    auto block = box();
    assert(name_set(block, CAD_SHAPE_FACE) ==
           (std::set<std::string>{"box.+x", "box.-x", "box.+y", "box.-y", "box.+z", "box.-z"}));
    const auto edges = name_set(block, CAD_SHAPE_EDGE);
    assert(edges.size() == 12 && contains(edges, "E(box.+x|box.+z)") && contains(edges, "E(box.-y|box.-z)"));
    const auto vertices = name_set(block, CAD_SHAPE_VERTEX);
    assert(vertices.size() == 8 && contains(vertices, "V(box.+x|box.+y|box.+z)"));
    expect_unique_strong(block, CAD_SHAPE_EDGE);
    expect_unique_strong(block, CAD_SHAPE_VERTEX);
    cad_shape_destroy(block);

    cad_shape cylinder = 0;
    assert(cad_cylinder(3, 5, &cylinder) == CAD_OK);
    assert(name_set(cylinder, CAD_SHAPE_FACE) == (std::set<std::string>{"cyl.side", "cyl.top", "cyl.bottom"}));
    const auto rims = name_set(cylinder, CAD_SHAPE_EDGE);
    assert(contains(rims, "E(cyl.side|cyl.top)") && contains(rims, "E(cyl.bottom|cyl.side)") &&
           contains(rims, "E(cyl.side|seam)"));
    expect_unique_strong(cylinder, CAD_SHAPE_EDGE);
    expect_unique_strong(cylinder, CAD_SHAPE_VERTEX);
    cad_shape_destroy(cylinder);

    cad_shape sphere = 0;
    assert(cad_sphere(4, &sphere) == CAD_OK);
    assert(names(sphere, CAD_SHAPE_FACE) == (std::vector<std::string>{"sphere"}));
    cad_shape_destroy(sphere);
}

// Rigid moves and copies keep every name: identity, not position.
void check_carried() {
    auto block = box();
    const auto faces = names(block, CAD_SHAPE_FACE);
    const auto edges = names(block, CAD_SHAPE_EDGE);

    cad_shape moved = 0, turned = 0, mirrored = 0, cloned = 0, placed = 0;
    assert(cad_shape_translate(block, cad_vec3{5, 6, 7}, &moved) == CAD_OK);
    assert(cad_shape_rotate(block, cad_vec3{0, 0, 1}, 0.7, &turned) == CAD_OK);
    assert(cad_shape_mirror(block, cad_vec3{0, 0, 0}, cad_vec3{1, 0, 0}, &mirrored) == CAD_OK);
    assert(cad_shape_clone(block, &cloned) == CAD_OK);
    assert(cad_shape_place(block, cad_vec3{1, 2, 3}, cad_vec3{0, 1, 0}, cad_vec3{0, 0, 1}, &placed) == CAD_OK);
    for (auto shape : {moved, turned, mirrored, cloned, placed}) {
        assert(name_set(shape, CAD_SHAPE_FACE) == std::set<std::string>(faces.begin(), faces.end()));
        assert(name_set(shape, CAD_SHAPE_EDGE) == std::set<std::string>(edges.begin(), edges.end()));
        cad_shape_destroy(shape);
    }

    cad_operation operation = 0;
    cad_shape result = 0;
    assert(cad_shape_translate_operation(block, cad_vec3{1, 0, 0}, &operation) == CAD_OK);
    assert(cad_operation_result_shape(operation, &result) == CAD_OK);
    assert(name_set(result, CAD_SHAPE_FACE) == std::set<std::string>(faces.begin(), faces.end()));
    cad_shape_destroy(result);
    cad_operation_destroy(operation);

    // Names sort bytewise, so `+` comes before `-`.
    // An extracted face keeps its name, and its (now boundary) edges keep theirs.
    const auto top = static_cast<std::uint32_t>(std::find(faces.begin(), faces.end(), "box.+z") - faces.begin());
    cad_shape face = 0;
    assert(cad_shape_subshape_at(block, CAD_SHAPE_FACE, top, &face) == CAD_OK);
    assert(names(face, CAD_SHAPE_FACE) == (std::vector<std::string>{"box.+z"}));
    assert(name_set(face, CAD_SHAPE_EDGE) ==
           (std::set<std::string>{"E(box.+x|box.+z)", "E(box.+z|box.-x)", "E(box.+y|box.+z)", "E(box.+z|box.-y)"}));
    cad_shape_destroy(face);
    cad_shape_destroy(block);
}

// Two copies of one shape in a compound collide: each name says which input it came from, and is weak.
void check_collisions() {
    auto block = box();
    cad_shape copy = 0, compound = 0;
    assert(cad_shape_translate(block, cad_vec3{50, 0, 0}, &copy) == CAD_OK);
    cad_shape_ref refs[2] = {{block}, {copy}};
    assert(cad_compound(refs, 2, &compound) == CAD_OK);
    const auto faces = name_set(compound, CAD_SHAPE_FACE);
    assert(faces.size() == 12 && contains(faces, "box.+z@0") && contains(faces, "box.+z@1"));
    cad_shape_destroy(compound);
    cad_shape_destroy(copy);
    cad_shape_destroy(block);
}

// A tag marks what is new relative to the inputs; what passes through keeps its name.
void check_stamp() {
    auto block = box();
    cad_shape stamped = 0;
    assert(cad_shape_stamp_names(block, "f3", nullptr, 0, &stamped) == CAD_OK);
    assert(contains(name_set(stamped, CAD_SHAPE_FACE), "f3:box.+z"));
    assert(contains(name_set(stamped, CAD_SHAPE_EDGE), "E(f3:box.+x|f3:box.+z)"));

    cad_shape moved = 0, again = 0;
    assert(cad_shape_translate(stamped, cad_vec3{0, 0, 9}, &moved) == CAD_OK);
    cad_shape_ref input{stamped};
    assert(cad_shape_stamp_names(moved, "f4", &input, 1, &again) == CAD_OK);
    assert(name_set(again, CAD_SHAPE_FACE) == name_set(stamped, CAD_SHAPE_FACE));

    cad_shape bad = 0;
    assert(cad_shape_stamp_names(block, "", nullptr, 0, &bad) == CAD_ERROR_INVALID_ARGUMENT);
    for (auto shape : {again, moved, stamped, block}) cad_shape_destroy(shape);
}

// Sketch-style starting names: seeded edges carry through wire and face building.
void check_seeds() {
    const cad_vec3 corners[4] = {{0, 0, 0}, {10, 0, 0}, {10, 5, 0}, {0, 5, 0}};
    cad_shape polyline = 0;
    assert(cad_polyline(corners, 4, 1, &polyline) == CAD_OK);
    assert(name_set(polyline, CAD_SHAPE_EDGE) == (std::set<std::string>{"seg.0", "seg.1", "seg.2", "seg.3"}));
    assert(name_set(polyline, CAD_SHAPE_VERTEX) == (std::set<std::string>{"pt.0", "pt.1", "pt.2", "pt.3"}));
    cad_shape_destroy(polyline);

    cad_shape lines[4] = {};
    cad_shape_ref refs[4] = {};
    const char* ids[4] = {"bottom", "right side", "top", "left"};
    for (int i = 0; i < 4; ++i) {
        cad_shape line = 0;
        assert(cad_line(corners[i], corners[(i + 1) % 4], &line) == CAD_OK);
        assert(cad_shape_seed_names(line, CAD_SHAPE_EDGE, ids[i], &lines[i]) == CAD_OK);
        cad_shape_destroy(line);
        refs[i].shape = lines[i];
    }
    assert(names(lines[1], CAD_SHAPE_EDGE) == (std::vector<std::string>{"right%20side"}));
    cad_shape wire = 0, face = 0, region = 0;
    assert(cad_wire(refs, 4, &wire) == CAD_OK);
    assert(name_set(wire, CAD_SHAPE_EDGE) == (std::set<std::string>{"bottom", "right%20side", "top", "left"}));
    assert(cad_planar_face(wire, nullptr, 0, &face) == CAD_OK);
    assert(name_set(face, CAD_SHAPE_EDGE) == (std::set<std::string>{"bottom", "right%20side", "top", "left"}));
    assert(weak(names(face, CAD_SHAPE_FACE).front()));
    assert(cad_shape_seed_names(face, CAD_SHAPE_FACE, "r.outer", &region) == CAD_OK);
    assert(names(region, CAD_SHAPE_FACE) == (std::vector<std::string>{"r.outer"}));

    cad_shape bad = 0;
    assert(cad_shape_seed_names(face, CAD_SHAPE_EDGE, "a\nb", &bad) == CAD_ERROR_INVALID_ARGUMENT);
    for (auto shape : {region, face, wire, lines[0], lines[1], lines[2], lines[3]}) cad_shape_destroy(shape);
}

// Operations without an adapter yet (TN2) still name every face, weakly.
void check_unadapted() {
    auto block = box();
    cad_shape cylinder = 0, fused = 0;
    assert(cad_cylinder(2, 40, &cylinder) == CAD_OK);
    assert(cad_fuse(block, cylinder, &fused) == CAD_OK);
    for (const auto& name : names(fused, CAD_SHAPE_FACE)) assert(!name.empty());
    for (const auto& name : names(fused, CAD_SHAPE_EDGE)) assert(!name.empty());
    for (auto shape : {fused, cylinder, block}) cad_shape_destroy(shape);
}

}  // namespace

int main() {
    check_primitives();
    check_carried();
    check_collisions();
    check_stamp();
    check_seeds();
    check_unadapted();
    std::puts("cadkit naming smoke passed");
    return 0;
}
