#include "cadkit.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// Planar regions for slicing: section, offset, booleans and exact boundaries.
namespace {
void check_at(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "planar test failed at line %d: %s (%s)\n", line, expression, cad_last_error());
        std::abort();
    }
}
#define check(condition) check_at((condition), #condition, __LINE__)
void ok(cad_result result) { check(result == CAD_OK); }
std::vector<cad_shape> owned;
cad_shape keep(cad_shape shape) { owned.push_back(shape); return shape; }
void near(double actual, double expected, double tolerance = 1e-6) {
    if (!(std::abs(actual - expected) < tolerance)) std::fprintf(stderr, "%g vs %g\n", actual, expected);
    check(std::abs(actual - expected) < tolerance);
}
double area(cad_shape shape) { double value = 0; ok(cad_shape_area(shape, &value)); return value; }
uint32_t faces(cad_shape shape) { uint32_t count = 0; ok(cad_shape_subshape_count(shape, CAD_SHAPE_FACE, &count)); return count; }
std::vector<double> boundary(cad_shape shape) {
    uint32_t bytes = 0;
    check(cad_planar_copy_boundary_bytes(shape, 1e-5, nullptr, &bytes) == CAD_ERROR_BUFFER_TOO_SMALL || bytes == 0);
    std::vector<double> values(bytes / sizeof(double));
    ok(cad_planar_copy_boundary_bytes(shape, 1e-5, reinterpret_cast<uint8_t*>(values.data()), &bytes));
    return values;
}
}

int main() {
    // A 20 mm cube with a 6 mm through hole, sectioned halfway up and just above its top.
    cad_shape cube = 0, hole = 0, part = 0;
    ok(cad_box(20, 20, 20, &cube)); keep(cube);
    ok(cad_cylinder(3, 40, &hole)); keep(hole);
    cad_shape moved = 0; ok(cad_shape_translate(hole, {10, 10, -10}, &moved)); keep(moved);
    ok(cad_cut(cube, moved, &part)); keep(part);

    cad_shape section = 0;
    ok(cad_shape_section(part, {0, 0, 7.5}, {0, 0, 1}, &section)); keep(section);
    check(faces(section) == 1);
    near(area(section), 400.0 - M_PI * 9.0, 1e-6);

    cad_shape above = 0;
    ok(cad_shape_section(part, {0, 0, 25}, {0, 0, 1}, &above)); keep(above);
    check(faces(above) == 0);

    // Exact boundaries: one outer wire of four lines, one hole of a single full arc, in the first face.
    auto records = boundary(section);
    check(records.size() % 14 == 0);
    int lines = 0, arcs = 0, outer = 0, inner = 0;
    for (size_t i = 0; i < records.size(); i += 14) {
        const double* r = &records[i];
        check(r[0] == 0.0);
        if (r[3] == 0.0) lines++;
        else {
            arcs++;
            near(r[11], 3.0, 1e-9);
            near(std::abs(r[13]), 2 * M_PI, 1e-9);
            near(r[9], 10.0, 1e-9);
            near(r[10], 10.0, 1e-9);
        }
        near(r[6], 7.5, 1e-9);
        if (r[2] == 1.0) outer++; else inner++;
    }
    check(lines == 4 && arcs == 1 && outer == 4 && inner == 1);

    // Offsets: inward shrinks the square and grows the hole; the area follows the perimeter.
    cad_shape inset = 0;
    ok(cad_planar_offset(section, -1.0, &inset)); keep(inset);
    check(faces(inset) == 1);
    near(area(inset), 18.0 * 18.0 - M_PI * 16.0, 1e-6);
    cad_shape grown = 0;
    ok(cad_planar_offset(section, 1.0, &grown)); keep(grown);
    near(area(grown), 22.0 * 22.0 - M_PI * 4.0 + 4.0 * 1.0 * 1.0 * (M_PI / 4.0 - 1.0) * 0.0, 5.0);
    check(area(grown) > area(section));
    // A region thinner than the offset vanishes.
    cad_shape gone = 0;
    ok(cad_planar_offset(section, -11.0, &gone)); keep(gone);
    check(faces(gone) == 0);
    // Round trip: shrinking and growing back stays inside the original and loses only the corners.
    cad_shape back = 0;
    ok(cad_planar_offset(inset, 1.0, &back)); keep(back);
    check(area(back) <= area(section) + 1e-6 && area(back) > area(section) - 4.0);

    // Booleans between regions: the band between the section and its inset.
    cad_shape band = 0;
    ok(cad_planar_cut(section, inset, &band)); keep(band);
    near(area(band), area(section) - area(inset), 1e-6);
    cad_shape common = 0;
    ok(cad_planar_common(section, inset, &common)); keep(common);
    near(area(common), area(inset), 1e-6);
    cad_shape fused = 0;
    ok(cad_planar_fuse(band, inset, &fused)); keep(fused);
    near(area(fused), area(section), 1e-6);
    check(faces(fused) == 1);
    cad_shape nothing = 0;
    ok(cad_planar_common(inset, gone, &nothing)); keep(nothing);
    check(faces(nothing) == 0);

    // Pinching: a dumbbell inset enough to cut the neck becomes two faces.
    cad_vec3 shape[] = {{0, 0, 0}, {10, 0, 0}, {10, 4, 0}, {14, 4, 0}, {14, 0, 0}, {24, 0, 0}, {24, 10, 0}, {14, 10, 0}, {14, 6, 0}, {10, 6, 0}, {10, 10, 0}, {0, 10, 0}};
    cad_shape wire = 0, base = 0, dumbbell = 0;
    ok(cad_polyline(shape, 12, 1, &wire)); keep(wire);
    ok(cad_planar_face(wire, nullptr, 0, &base)); keep(base);
    cad_shape solid = 0; ok(cad_shape_extrude(base, {0, 0, 2}, &solid)); keep(solid);
    ok(cad_shape_section(solid, {0, 0, 1}, {0, 0, 1}, &dumbbell)); keep(dumbbell);
    check(faces(dumbbell) == 1);
    cad_shape pinched = 0;
    ok(cad_planar_offset(dumbbell, -1.5, &pinched)); keep(pinched);
    check(faces(pinched) == 2);

    for (auto it = owned.rbegin(); it != owned.rend(); ++it) cad_shape_destroy(*it);
    std::puts("planar regions passed");
    return 0;
}
