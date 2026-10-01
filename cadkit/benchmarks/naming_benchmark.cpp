// What topological naming costs next to the operations it follows (plans/TOPOLOGICAL_NAMING.md, TN2).
// Times OCCT and the naming step separately on a plate drilled hole by hole, then filleted.
#include "../core/src/naming.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>

#include <chrono>
#include <cstdio>

namespace naming = cadkit_naming;
using Clock = std::chrono::steady_clock;

double since(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

int main() {
    double occt = 0, names = 0, derive = 0;
    auto started = Clock::now();
    naming::NamedShape plate{BRepPrimAPI_MakeBox(200, 200, 10).Shape(), nullptr};
    plate.names = naming::box_names(plate.shape);
    names += since(started);

    const int holes = 64;
    for (int i = 0; i < holes; ++i) {
        const double x = 12 + (i % 8) * 24, y = 12 + (i / 8) * 24;
        started = Clock::now();
        BRepPrimAPI_MakeCylinder drill(gp_Ax2(gp_Pnt(x, y, -1), gp_Dir(0, 0, 1)), 3, 12);
        BRepAlgoAPI_Cut cut(plate.shape, drill.Shape());
        cut.SetToFillHistory(true);
        cut.Build();
        occt += since(started);
        started = Clock::now();
        auto& faces = drill.Cylinder();
        naming::NamedShape tool{drill.Shape(), naming::role_names(drill.Shape(), {{faces.LateralFace(), "cyl.side"},
                                                                                 {faces.TopFace(), "cyl.top"},
                                                                                 {faces.BottomFace(), "cyl.bottom"}})};
        tool.names = naming::stamp(tool.shape, *tool.names, "f" + std::to_string(i + 2), {});
        naming::AlgorithmHistory<BRepAlgoAPI_Cut> history(cut);
        plate = {cut.Shape(), naming::propagate({plate, tool}, &history, cut.Shape())};
        names += since(started);
    }
    started = Clock::now();
    const auto& edges = plate.names->names(plate.shape, naming::ElementKind::Edge);
    plate.names->names(plate.shape, naming::ElementKind::Vertex);
    derive += since(started);

    // Fillet every edge of the top face.
    started = Clock::now();
    BRepFilletAPI_MakeFillet fillet(plate.shape);
    naming::ShapeIndex all;
    TopExp::MapShapes(plate.shape, TopAbs_EDGE, all);
    for (int i = 1; i <= all.Extent(); ++i) {
        if (edges[static_cast<std::size_t>(i - 1)].find("box.+z") != std::string::npos) fillet.Add(0.5, TopoDS::Edge(all(i)));
    }
    fillet.Build();
    occt += since(started);
    started = Clock::now();
    naming::LambdaHistory history;
    history.on_modified = [&](const TopoDS_Shape& s) {
        return s.ShapeType() == TopAbs_FACE ? naming::shapes_of(fillet.Modified(s)) : std::vector<TopoDS_Shape>{};
    };
    history.on_deleted = [&](const TopoDS_Shape& s) { return fillet.IsDeleted(s); };
    history.on_generated = [&](const TopoDS_Shape& s, std::size_t) {
        naming::Generated result;
        if (s.ShapeType() == TopAbs_EDGE)
            for (const auto& face : naming::shapes_of(fillet.Generated(s))) result.emplace_back(face, "fillet");
        return result;
    };
    auto filleted = naming::propagate({plate}, &history, fillet.Shape());
    names += since(started);
    started = Clock::now();
    filleted->names(fillet.Shape(), naming::ElementKind::Edge);
    filleted->names(fillet.Shape(), naming::ElementKind::Vertex);
    derive += since(started);

    std::size_t longest = 0, total = 0;
    for (const auto& name : filleted->names(fillet.Shape(), naming::ElementKind::Edge)) {
        longest = std::max(longest, name.size());
        total += name.size();
    }
    const auto count = filleted->names(fillet.Shape(), naming::ElementKind::Edge).size();
    std::printf("faces %zu, edges %zu\n", filleted->faces.size(), count);
    std::printf("occt %.1f ms, propagation %.1f ms (%.1f%%), derived edge/vertex names %.1f ms (%.1f%%)\n", occt, names,
                100 * names / occt, derive, 100 * derive / occt);
    std::printf("edge names: longest %zu bytes, mean %.0f\n", longest, static_cast<double>(total) / count);
    return 0;
}
