// Topological names in the core (plans/TOPOLOGICAL_NAMING.md, TN1).
#include "cadkit.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
    assert(names(face, CAD_SHAPE_FACE) == (std::vector<std::string>{"face"}));
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


cad_shape tagged(cad_shape shape, const char* tag) {
    cad_shape result = 0;
    assert(cad_shape_stamp_names(shape, tag, nullptr, 0, &result) == CAD_OK);
    cad_shape_destroy(shape);
    return result;
}

cad_shape moved(cad_shape shape, cad_vec3 delta) {
    cad_shape result = 0;
    assert(cad_shape_translate(shape, delta, &result) == CAD_OK);
    cad_shape_destroy(shape);
    return result;
}

// A closed polyline profile face named like a sketch region.
cad_shape profile(const std::vector<cad_vec3>& corners, const char* region) {
    cad_shape wire = 0, face = 0, named = 0;
    assert(cad_polyline(corners.data(), static_cast<std::uint32_t>(corners.size()), 1, &wire) == CAD_OK);
    assert(cad_planar_face(wire, nullptr, 0, &face) == CAD_OK);
    assert(cad_shape_seed_names(face, CAD_SHAPE_FACE, region, &named) == CAD_OK);
    cad_shape_destroy(face);
    cad_shape_destroy(wire);
    return named;
}

bool verbose() {
    return std::getenv("CADKIT_NAMING_VERBOSE") != nullptr;
}

// TN2 completeness: every face the operation made is accounted for by a rule, so none is weak, and none repeats.
void expect_complete(const char* operation, cad_shape shape) {
    const auto faces = names(shape, CAD_SHAPE_FACE);
    if (verbose()) {
        std::fprintf(stderr, "%s faces:\n", operation);
        for (const auto& name : faces) std::fprintf(stderr, "  %s\n", name.c_str());
        std::fprintf(stderr, "%s edges:\n", operation);
        for (const auto& name : names(shape, CAD_SHAPE_EDGE)) std::fprintf(stderr, "  %s\n", name.c_str());
    }
    std::set<std::string> unique(faces.begin(), faces.end());
    bool complete = unique.size() == faces.size();
    for (const auto& name : faces) complete = complete && !weak(name);
    if (!complete) {
        std::fprintf(stderr, "%s: incomplete face names\n", operation);
        for (const auto& name : faces) std::fprintf(stderr, "  %s\n", name.c_str());
    }
    assert(complete);
}

void check_operations() {
    // Booleans: a through hole leaves the top face whole; a slot splits it.
    cad_shape plate = 0, drill = 0, slot = 0, holed = 0, slotted = 0;
    assert(cad_box(60, 40, 10, &plate) == CAD_OK);
    plate = tagged(plate, "f1");
    assert(cad_cylinder(3, 30, &drill) == CAD_OK);
    drill = moved(tagged(drill, "f2"), cad_vec3{15, 20, -5});
    assert(cad_box(4, 60, 30, &slot) == CAD_OK);
    slot = moved(tagged(slot, "f3"), cad_vec3{28, -10, -5});
    assert(cad_cut(plate, drill, &holed) == CAD_OK);
    expect_complete("cut (hole)", holed);
    assert(contains(name_set(holed, CAD_SHAPE_FACE), "f1:box.+z") && contains(name_set(holed, CAD_SHAPE_FACE), "f2:cyl.side"));
    assert(cad_cut(holed, slot, &slotted) == CAD_OK);
    expect_complete("cut (slot)", slotted);
    const auto slottedFaces = name_set(slotted, CAD_SHAPE_FACE);
    // Each piece is named by the faces that bound only it: the left one has the hole.
    assert(contains(slottedFaces, "f1:box.+z{f1:box.-x,f2:cyl.side,f3:box.-x}") &&
           contains(slottedFaces, "f1:box.+z{f1:box.+x,f3:box.+x}"));
    // Stamping a feature's result leaves split pieces of its inputs' faces untagged: they keep that identity.
    cad_shape_ref cutInputs[2] = {{holed}, {slot}};
    cad_shape restamped = 0;
    assert(cad_shape_stamp_names(slotted, "f4", cutInputs, 2, &restamped) == CAD_OK);
    assert(contains(name_set(restamped, CAD_SHAPE_FACE), "f1:box.+z{f1:box.+x,f3:box.+x}"));
    cad_shape_destroy(restamped);
    cad_shape fused = 0, common = 0;
    assert(cad_fuse(plate, slot, &fused) == CAD_OK);
    expect_complete("fuse", fused);
    assert(cad_common(plate, slot, &common) == CAD_OK);
    expect_complete("common", common);

    // Extrude and revolve a sketch-like profile: sides from its segments, caps from its region.
    auto rectangle = profile({{0, 0, 0}, {20, 0, 0}, {20, 10, 0}, {0, 10, 0}}, "r");
    cad_shape prism = 0, ring = 0;
    assert(cad_shape_extrude(rectangle, cad_vec3{0, 0, 5}, &prism) == CAD_OK);
    expect_complete("extrude", prism);
    assert(name_set(prism, CAD_SHAPE_FACE) ==
           (std::set<std::string>{"start(r)", "end(r)", "side(seg.0)", "side(seg.1)", "side(seg.2)", "side(seg.3)"}));
    auto offset = profile({{5, 0, 0}, {15, 0, 0}, {15, 0, 10}, {5, 0, 10}}, "r");
    assert(cad_shape_revolve(offset, cad_vec3{0, 0, 0}, cad_vec3{0, 0, 1}, 1.5, &ring) == CAD_OK);
    expect_complete("revolve (part turn)", ring);
    cad_shape whole = 0;
    assert(cad_shape_revolve(offset, cad_vec3{0, 0, 0}, cad_vec3{0, 0, 1}, 6.283185307179586, &whole) == CAD_OK);
    expect_complete("revolve (full turn)", whole);

    // Fillet and chamfer: blend faces from the edges, neighbours keep their names.
    auto block = tagged(box(), "f1");
    const auto edges = names(block, CAD_SHAPE_EDGE);
    const auto rim = static_cast<std::uint32_t>(std::find(edges.begin(), edges.end(), "E(f1:box.+x|f1:box.+z)") - edges.begin());
    cad_shape edge = 0, filleted = 0, chamfered = 0, rounded = 0;
    assert(cad_shape_subshape_at(block, CAD_SHAPE_EDGE, rim, &edge) == CAD_OK);
    cad_shape_ref selected{edge};
    assert(cad_shape_fillet_edges(block, &selected, 1, 1.0, &filleted) == CAD_OK);
    expect_complete("fillet (one edge)", filleted);
    assert(contains(name_set(filleted, CAD_SHAPE_FACE), "fillet(E(f1:box.+x|f1:box.+z))"));
    assert(cad_shape_chamfer_edges(block, &selected, 1, 1.0, &chamfered) == CAD_OK);
    expect_complete("chamfer (one edge)", chamfered);
    assert(cad_shape_fillet(block, 1.0, &rounded) == CAD_OK);
    expect_complete("fillet (all edges)", rounded);

    // Shell, loft, sweep.
    cad_shape top = 0, shelled = 0;
    const auto faces = names(block, CAD_SHAPE_FACE);
    const auto lid = static_cast<std::uint32_t>(std::find(faces.begin(), faces.end(), "f1:box.+z") - faces.begin());
    assert(cad_shape_subshape_at(block, CAD_SHAPE_FACE, lid, &top) == CAD_OK);
    cad_shape_ref removed{top};
    assert(cad_shell(block, &removed, 1, -1.0, &shelled) == CAD_OK);
    expect_complete("shell", shelled);

    const cad_vec3 lower[4] = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}};
    const cad_vec3 upper[4] = {{2, 2, 8}, {8, 2, 8}, {8, 8, 8}, {2, 8, 8}};
    cad_shape a = 0, b = 0, lofted = 0;
    assert(cad_polyline(lower, 4, 1, &a) == CAD_OK);
    assert(cad_polyline(upper, 4, 1, &b) == CAD_OK);
    cad_shape_ref sections[2] = {{a}, {b}};
    assert(cad_loft(sections, 2, 1, 1, &lofted) == CAD_OK);
    expect_complete("loft", lofted);

    const cad_vec3 path[3] = {{0, 0, 0}, {0, 0, 20}, {10, 0, 30}};
    cad_shape spine = 0, swept = 0;
    assert(cad_polyline(path, 3, 0, &spine) == CAD_OK);
    spine = tagged(spine, "f8");
    auto square = tagged(profile({{-2, -2, 0}, {2, -2, 0}, {2, 2, 0}, {-2, 2, 0}}, "s"), "f9");
    assert(cad_sweep(square, spine, &swept) == CAD_OK);
    expect_complete("sweep", swept);
    // One side face per (profile edge, path segment).
    assert(contains(name_set(swept, CAD_SHAPE_FACE), "side(f8:seg.1,f9:seg.2)"));

    for (auto shape : {plate, drill, slot, holed, slotted, fused, common, rectangle, prism, offset, ring, whole, block, edge,
                       filleted, chamfered, rounded, top, shelled, a, b, lofted, spine, square, swept})
        cad_shape_destroy(shape);
}


std::vector<double> scores(const char* reference, const char* candidates) {
    std::uint32_t capacity = 0;
    cad_element_name_match_bytes(reference, candidates, nullptr, &capacity);
    std::vector<double> result(capacity / sizeof(double));
    assert(cad_element_name_match_bytes(reference, candidates, reinterpret_cast<std::uint8_t*>(result.data()), &capacity) == CAD_OK);
    return result;
}

// Matching is text only: exact, exact but weak, relatives graded by their split pieces, none.
void check_matching() {
    const auto graded = scores("f1:box.+z", "f1:box.+z\nf1:box.+z{f3:box.-x}\nf1:box.+x\nface#2");
    assert(graded.size() == 4 && graded[0] == 3.0 && graded[1] == 1.0 && graded[2] == 0.0 && graded[3] == 0.0);
    assert(scores("face#2", "face#2").front() == 2.0);
    const auto pieces = scores("f1:box.+z{f2:cyl.side,f3:box.-x}", "f1:box.+z{f1:box.-x,f2:cyl.side,f3:box.-x}\nf1:box.+z{f1:box.+x,f3:box.+x}");
    assert(pieces[0] > pieces[1] && pieces[1] >= 1.0);
    assert(scores("E(f1:box.+x|f1:box.+z)", "E(f1:box.+x|f1:box.+z{f3:box.+x})").front() == 1.0);
    assert(scores("x", "").empty());
}

}  // namespace

int main() {
    check_primitives();
    check_carried();
    check_collisions();
    check_stamp();
    check_seeds();
    check_unadapted();
    check_operations();
    check_matching();
    std::puts("cadkit naming smoke passed");
    return 0;
}
