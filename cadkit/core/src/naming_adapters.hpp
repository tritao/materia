// Per-operation naming adapters (plans/TOPOLOGICAL_NAMING.md, TN2): how each
// OCCT algorithm's history names its result. Included by cadkit.cpp only.
#pragma once

#include "naming.hpp"

#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <TopExp.hxx>

namespace cadkit_naming {

// Extrude and revolve: each profile edge sweeps a side face and each vertex a
// lateral edge; the caps come only from FirstShape/LastShape. The profile is
// copied, so nothing passes through unchanged.
template <typename Sweep>
ElementMapPtr sweep_names(const NamedShape& profile, Sweep& sweep) {
    LambdaHistory history;
    history.on_deleted = [](const TopoDS_Shape&) { return true; };
    history.on_generated = [&](const TopoDS_Shape& source, std::size_t) {
        Generated result;
        const auto type = source.ShapeType();
        if (type == TopAbs_EDGE)
            for (const auto& face : shapes_of(sweep.Generated(source))) result.emplace_back(face, "side");
        if (type == TopAbs_VERTEX)
            for (const auto& edge : shapes_of(sweep.Generated(source))) result.emplace_back(edge, "lateral");
        if (type == TopAbs_FACE || type == TopAbs_EDGE || type == TopAbs_VERTEX) {
            const auto first = sweep.FirstShape(source);
            const auto last = sweep.LastShape(source);
            if (!first.IsNull()) result.emplace_back(first, "start");
            if (!last.IsNull() && !last.IsSame(first)) result.emplace_back(last, "end");
        }
        return result;
    };
    return propagate({profile}, &history, sweep.Shape());
}

inline ElementMapPtr operation_names(const NamedShape& profile, BRepPrimAPI_MakePrism& sweep) {
    return sweep_names(profile, sweep);
}

inline ElementMapPtr operation_names(const NamedShape& profile, BRepPrimAPI_MakeRevol& sweep) {
    return sweep_names(profile, sweep);
}

// Fillet and chamfer: blend faces from edges, corner faces from vertices, and
// trimmed (possibly split) faces from the faces they touch.
template <typename Finish>
ElementMapPtr finish_names(const NamedShape& source, Finish& finish, const char* role) {
    LambdaHistory history;
    history.on_modified = [&](const TopoDS_Shape& shape) {
        return shape.ShapeType() == TopAbs_FACE ? shapes_of(finish.Modified(shape)) : std::vector<TopoDS_Shape>{};
    };
    history.on_deleted = [&](const TopoDS_Shape& shape) { return finish.IsDeleted(shape); };
    history.on_generated = [&](const TopoDS_Shape& shape, std::size_t) {
        Generated result;
        const auto type = shape.ShapeType();
        if (type != TopAbs_EDGE && type != TopAbs_VERTEX) return result;
        for (const auto& face : shapes_of(finish.Generated(shape)))
            if (face.ShapeType() == TopAbs_FACE) result.emplace_back(face, type == TopAbs_EDGE ? role : "corner");
        return result;
    };
    return propagate({source}, &history, finish.Shape());
}

inline ElementMapPtr operation_names(const NamedShape& source, BRepFilletAPI_MakeFillet& finish) {
    return finish_names(source, finish, "fillet");
}

inline ElementMapPtr operation_names(const NamedShape& source, BRepFilletAPI_MakeChamfer& finish) {
    return finish_names(source, finish, "chamfer");
}

// Loft: side faces are named by the first section's edges; the caps come from no single element.
inline ElementMapPtr loft_names(const std::vector<NamedShape>& sections, BRepOffsetAPI_ThruSections& loft) {
    LambdaHistory history;
    history.on_deleted = [](const TopoDS_Shape&) { return true; };
    history.on_generated = [&](const TopoDS_Shape& source, std::size_t slot) {
        Generated result;
        if (slot != 0 || source.ShapeType() != TopAbs_EDGE) return result;
        for (const auto& face : shapes_of(loft.Generated(source)))
            if (face.ShapeType() == TopAbs_FACE) result.emplace_back(face, "side");
        return result;
    };
    auto propagated = propagate(sections, &history, loft.Shape());
    return with_face_roles(loft.Shape(), *propagated, {{loft.FirstShape(), "start"}, {loft.LastShape(), "end"}});
}

// Sweep: a side face per (profile edge, spine edge); the one-argument Generated lumps them.
inline ElementMapPtr pipe_names(const NamedShape& profile, const NamedShape& spine, BRepOffsetAPI_MakePipe& pipe) {
    ShapeIndex sectionEdges, spineEdges;
    TopExp::MapShapes(profile.shape, TopAbs_EDGE, sectionEdges);
    TopExp::MapShapes(spine.shape, TopAbs_EDGE, spineEdges);
    LambdaHistory history;
    history.on_deleted = [](const TopoDS_Shape&) { return true; };
    history.on_generated = [&](const TopoDS_Shape& source, std::size_t slot) {
        Generated result;
        if (source.ShapeType() == TopAbs_EDGE) {
            const auto& others = slot == 0 ? spineEdges : sectionEdges;
            for (int i = 1; i <= others.Extent(); ++i) {
                const auto face = slot == 0 ? pipe.Generated(others(i), source) : pipe.Generated(source, others(i));
                if (!face.IsNull() && face.ShapeType() == TopAbs_FACE) result.emplace_back(face, "side");
            }
        }
        if (slot == 0 && source.ShapeType() == TopAbs_FACE) {
            result.emplace_back(pipe.FirstShape(), "start");
            result.emplace_back(pipe.LastShape(), "end");
        }
        return result;
    };
    return propagate({profile, spine}, &history, pipe.Shape());
}

// Wire offset: each edge's offset is `offset(edge)`.
inline ElementMapPtr offset_names(const NamedShape& wire, BRepOffsetAPI_MakeOffset& offset) {
    LambdaHistory history;
    history.on_deleted = [](const TopoDS_Shape&) { return true; };
    history.on_generated = [&](const TopoDS_Shape& edge, std::size_t) {
        Generated result;
        if (edge.ShapeType() != TopAbs_EDGE) return result;
        for (const auto& target : shapes_of(offset.Generated(edge)))
            if (target.ShapeType() == TopAbs_EDGE) result.emplace_back(target, "offset");
        return result;
    };
    return propagate({wire}, &history, offset.Shape());
}

// Shell: kept faces keep their names; each face's offset copy is `inner(face)`.
inline ElementMapPtr shell_names(const NamedShape& solid, BRepOffsetAPI_MakeThickSolid& shell) {
    LambdaHistory history;
    history.on_modified = [&](const TopoDS_Shape& shape) { return shapes_of(shell.Modified(shape)); };
    history.on_deleted = [&](const TopoDS_Shape& shape) { return shell.IsDeleted(shape); };
    history.on_generated = [&](const TopoDS_Shape& shape, std::size_t) {
        Generated result;
        for (const auto& target : shapes_of(shell.Generated(shape)))
            if (target.ShapeType() == TopAbs_FACE) result.emplace_back(target, shape.ShapeType() == TopAbs_FACE ? "inner" : "wall");
        return result;
    };
    return propagate({solid}, &history, shell.Shape());
}

}  // namespace cadkit_naming
