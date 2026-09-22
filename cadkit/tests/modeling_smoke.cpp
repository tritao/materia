#include "cadkit.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <vector>

namespace {
void check_at(bool condition, const char* expression, int line) {
    if (!condition) {
        std::fprintf(stderr, "modeling test failed at line %d: %s (%s)\n", line, expression, cad_last_error());
        std::abort();
    }
}
#define check(condition) check_at((condition), #condition, __LINE__)
void ok(cad_result result) { check(result == CAD_OK); }
std::vector<cad_shape> owned;
cad_shape keep(cad_shape shape) { owned.push_back(shape); return shape; }
cad_shape circle(double x, double y, double z, double radius) {
    cad_shape edge=0, wire=0;
    ok(cad_circle({x,y,z},{0,0,1},radius,&edge)); keep(edge);
    cad_shape_ref refs[]={{edge}};
    ok(cad_wire(refs,1,&wire)); return keep(wire);
}
void valid(cad_shape shape) { uint8_t result=0; ok(cad_shape_valid(shape,&result)); check(result==1); }
void near(double actual,double expected,double tolerance=1e-6) { check(std::abs(actual-expected)<tolerance); }
}
int main() {
    cad_vec3 points[]={{-40,-25,0},{40,-25,0},{40,25,0},{-40,25,0}};
    cad_shape outer=0; ok(cad_polyline(points,4,1,&outer)); keep(outer);
    cad_shape_ref holes[4]; int i=0;
    for(double x:{-30.,30.}) for(double y:{-15.,15.}) holes[i++]={circle(x,y,0,3)};
    cad_shape face=0; ok(cad_planar_face(outer,holes,4,&face)); keep(face); valid(face);
    double area=0; ok(cad_shape_area(face,&area)); near(area,4000-36*std::acos(-1));
    cad_shape plate=0; ok(cad_shape_extrude(face,{0,0,6},&plate)); keep(plate); valid(plate);
    double volume=0; ok(cad_shape_volume(plate,&volume)); near(volume,area*6);
    // Pick only the four outer vertical edges, excluding cylindrical seam edges.
    uint32_t edge_count=0; ok(cad_shape_subshape_count(plate,CAD_SHAPE_EDGE,&edge_count));
    std::vector<cad_shape_ref> selected;
    for(uint32_t n=0;n<edge_count;++n) {
        cad_shape edge=0; ok(cad_shape_subshape_at(plate,CAD_SHAPE_EDGE,n,&edge)); keep(edge);
        cad_curve_kind kind; ok(cad_edge_curve_kind(edge,&kind));
        if(kind!=CAD_CURVE_LINE) continue;
        cad_vec3 tangent; ok(cad_edge_tangent_at(edge,0.5,&tangent));
        cad_bounds bounds; ok(cad_shape_bounds(edge,&bounds));
        if(std::abs(tangent.z)>0.99 && std::abs(bounds.min.x)>39) selected.push_back({edge});
    }
    check(selected.size()==4);
    cad_shape rounded=0; ok(cad_shape_fillet_edges(plate,selected.data(),selected.size(),2,&rounded)); keep(rounded); valid(rounded);
    double rounded_volume=0; ok(cad_shape_volume(rounded,&rounded_volume)); check(rounded_volume<volume);
    const auto path=std::filesystem::temp_directory_path()/"cadkit-modeling-smoke.step";
    ok(cad_step_export(rounded,path.string().c_str()));
    cad_shape imported=0; ok(cad_step_import(path.string().c_str(),&imported)); keep(imported); valid(imported);
    double imported_volume=0; ok(cad_shape_volume(imported,&imported_volume)); near(imported_volume,rounded_volume);
    std::filesystem::remove(path);
    // Reversed hole winding is accepted; disjoint, intersecting, nonplanar and open boundaries fail.
    cad_shape bad=99;
    cad_shape_ref outside[]={{circle(100,0,0,2)}};
    check(cad_planar_face(outer,outside,1,&bad)!=CAD_OK && bad==0);
    cad_shape_ref elevated[]={{circle(0,0,1,2)}};
    check(cad_planar_face(outer,elevated,1,&bad)!=CAD_OK && bad==0);
    cad_shape_ref overlap[]={{holes[0].shape},{holes[0].shape}};
    check(cad_planar_face(outer,overlap,2,&bad)!=CAD_OK && bad==0);
    cad_shape_ref touching[]={{circle(38,0,0,2)}};
    check(cad_planar_face(outer,touching,1,&bad)!=CAD_OK && bad==0);
    cad_vec3 reversed_points[]={{-2,-2,0},{-2,2,0},{2,2,0},{2,-2,0}};
    cad_shape reversed=0; ok(cad_polyline(reversed_points,4,1,&reversed)); keep(reversed);
    cad_shape_ref reversed_hole[]={{reversed}};
    cad_shape reverse_face=0; ok(cad_planar_face(outer,reversed_hole,1,&reverse_face)); keep(reverse_face);
    ok(cad_shape_area(reverse_face,&area)); near(area,3984);
    cad_shape a=0,b=0; ok(cad_line({0,0,0},{1,0,0},&a)); keep(a);
    ok(cad_line({3,0,0},{4,0,0},&b)); keep(b);
    cad_shape_ref disconnected[]={{a},{b}};
    check(cad_wire(disconnected,2,&bad)!=CAD_OK && bad==0);
    cad_shape open=0; ok(cad_polyline(points,4,0,&open)); keep(open);
    check(cad_planar_face(open,nullptr,0,&bad)!=CAD_OK && bad==0);
    points[2].z=1;
    cad_shape skew=0; ok(cad_polyline(points,4,1,&skew)); keep(skew);
    check(cad_planar_face(skew,nullptr,0,&bad)!=CAD_OK && bad==0);
    check(cad_circle({0,0,0},{0,0,0},1,&bad)==CAD_ERROR_INVALID_ARGUMENT && bad==0);
    check(cad_line({0,0,0},{0,0,0},&bad)==CAD_ERROR_INVALID_ARGUMENT && bad==0);
    check(cad_line({0,0,0},{NAN,0,0},&bad)==CAD_ERROR_INVALID_ARGUMENT && bad==0);
    check(cad_polyline(nullptr,2,0,&bad)==CAD_ERROR_INVALID_ARGUMENT && bad==0);
    cad_shape_ref stale[]={{0}};
    check(cad_wire(stale,1,&bad)==CAD_ERROR_INVALID_HANDLE && bad==0);
    check(cad_shape_place(plate,{0,0,0},{1,0,0},{1,0,0},&bad)==CAD_ERROR_INVALID_ARGUMENT && bad==0);
    cad_shape placed=0; ok(cad_shape_place(plate,{0,0,0},{1,0,0},{0,-1,0},&placed)); keep(placed); valid(placed);
    cad_bounds bounds; ok(cad_shape_bounds(placed,&bounds)); near(bounds.min.y,-6); near(bounds.max.z,25);
    cad_shape arc=0; ok(cad_arc({1,0,0},{0,1,0},{-1,0,0},&arc)); keep(arc);
    double length=0; ok(cad_edge_length(arc,&length)); near(length,std::acos(-1));
    check(cad_arc({0,0,0},{1,0,0},{2,0,0},&bad)!=CAD_OK && bad==0);
    cad_vec3 spline_points[]={{0,0,0},{1,1,0},{2,0,0}};
    cad_shape spline=0; ok(cad_spline(spline_points,3,&spline)); keep(spline); valid(spline);
    cad_shape_ref loft_wires[]={{circle(0,0,0,2)},{circle(0,0,5,2)}};
    cad_shape loft=0; ok(cad_loft(loft_wires,2,1,0,&loft)); keep(loft); valid(loft);
    ok(cad_shape_volume(loft,&volume)); near(volume,20*std::acos(-1));
    cad_shape disk=0; ok(cad_planar_face(loft_wires[0].shape,nullptr,0,&disk)); keep(disk);
    cad_vec3 path_points[]={{0,0,0},{0,0,5}};
    cad_shape spine=0; ok(cad_polyline(path_points,2,0,&spine)); keep(spine);
    cad_shape pipe=0; ok(cad_sweep(disk,spine,&pipe)); keep(pipe); valid(pipe);
    ok(cad_shape_volume(pipe,&volume)); near(volume,20*std::acos(-1));
    cad_shape offset=0; ok(cad_wire_offset(loft_wires[0].shape,1,&offset)); keep(offset); valid(offset);
    cad_shape box=0; ok(cad_box(10,10,10,&box)); keep(box);
    uint32_t face_count=0; ok(cad_shape_subshape_count(box,CAD_SHAPE_FACE,&face_count));
    cad_shape top=0;
    for(uint32_t n=0;n<face_count;++n) {
        cad_shape f=0; ok(cad_shape_subshape_at(box,CAD_SHAPE_FACE,n,&f)); keep(f);
        cad_vec3 center; ok(cad_face_center(f,&center)); if(center.z>9.9) top=f;
    }
    check(top!=0); cad_shape_ref removed[]={{top}};
    cad_shape shell=0; ok(cad_shell(box,removed,1,-1,&shell)); keep(shell); valid(shell);
    ok(cad_shape_volume(shell,&volume)); near(volume,1000-8*8*9);
    cad_shape projected=0; ok(cad_project(loft_wires[1].shape,disk,{0,0,-1},&projected)); keep(projected); valid(projected);
    // History owns its result independently of caller-owned handles.
    cad_operation operation=0;
    ok(cad_loft_operation(loft_wires,2,1,0,&operation));
    uint32_t generated=0; ok(cad_operation_history_count(operation,CAD_HISTORY_GENERATED,&generated));
    check(generated>0);
    cad_shape operation_result=0; ok(cad_operation_result_shape(operation,&operation_result)); keep(operation_result);
    cad_operation_destroy(operation); valid(operation_result);
    operation=99;
    check(cad_sweep_operation(0,spine,&operation)==CAD_ERROR_INVALID_HANDLE && operation==0);
    check(cad_loft_operation(loft_wires,1,1,0,&operation)==CAD_ERROR_INVALID_ARGUMENT && operation==0);
    check(cad_shell_operation(box,stale,1,-1,&operation)==CAD_ERROR_INVALID_HANDLE && operation==0);
    check(cad_shape_place_operation(box,{0,0,0},{1,0,0},{0,0,1},nullptr)==CAD_ERROR_INVALID_ARGUMENT);
    cad_shape empty=0; ok(cad_compound(nullptr,0,&empty)); keep(empty); valid(empty);
    check(cad_shape_valid(0,nullptr)==CAD_ERROR_INVALID_ARGUMENT);
    for(auto shape:owned) cad_shape_destroy(shape);
}
