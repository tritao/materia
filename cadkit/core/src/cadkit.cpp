#include "cadkit.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Bnd_Box.hxx>
#include <DESTEP_ConfigurationNode.hxx>
#include <DESTEP_Provider.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomAbs_CurveType.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopExp.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <new>
#include <optional>
#include <string>
#include <vector>

namespace {

struct ShapeEntry {
    std::uint16_t generation = 1;
    std::optional<TopoDS_Shape> shape;
};

struct MeshData {
    std::vector<cad_vec3> vertices;
    std::vector<cad_vec3> normals;
    std::vector<std::uint32_t> indices;
    std::vector<cad_mesh_face_range> face_ranges;
};

struct MeshEntry {
    std::uint16_t generation = 1;
    std::optional<MeshData> mesh;
};

struct HistoryRelation {
    TopoDS_Shape source;
    TopoDS_Shape target;
};

struct OperationData {
    TopoDS_Shape result;
    std::vector<HistoryRelation> generated;
    std::vector<HistoryRelation> modified;
    std::vector<HistoryRelation> deleted;
};

struct OperationEntry {
    std::uint16_t generation = 1;
    std::optional<OperationData> operation;
};

std::mutex g_shapes_mutex;
std::vector<ShapeEntry> g_shapes;
std::vector<std::uint16_t> g_free_slots;
std::mutex g_meshes_mutex;
std::vector<MeshEntry> g_meshes;
std::vector<std::uint16_t> g_free_mesh_slots;
std::mutex g_operations_mutex;
std::vector<OperationEntry> g_operations;
std::vector<std::uint16_t> g_free_operation_slots;
thread_local std::string g_last_error;

constexpr std::uint32_t kSlotMask = 0xffffu;
constexpr std::uint32_t kGenerationShift = 16u;

void clear_error() {
    g_last_error.clear();
}

cad_result fail(cad_result result, const char* message) {
    g_last_error = message;
    return result;
}

cad_result fail(cad_result result, const std::exception& error) {
    g_last_error = error.what();
    return result;
}

cad_result fail_occt(cad_result result, const Standard_Failure& error) {
    g_last_error = error.what();
    if (g_last_error.empty()) {
        g_last_error = "OCCT operation failed";
    }
    return result;
}

cad_shape encode_handle(std::uint16_t slot, std::uint16_t generation) {
    return (static_cast<cad_shape>(generation) << kGenerationShift) | slot;
}

bool decode_handle(cad_shape handle, std::uint16_t& slot, std::uint16_t& generation) {
    slot = static_cast<std::uint16_t>(handle & kSlotMask);
    generation = static_cast<std::uint16_t>(handle >> kGenerationShift);
    return slot != 0 && generation != 0;
}

ShapeEntry* lookup_locked(cad_shape handle) {
    std::uint16_t slot = 0;
    std::uint16_t generation = 0;
    if (!decode_handle(handle, slot, generation)) {
        return nullptr;
    }

    const auto index = static_cast<std::size_t>(slot - 1);
    if (index >= g_shapes.size()) {
        return nullptr;
    }

    auto& entry = g_shapes[index];
    if (entry.generation != generation || !entry.shape.has_value()) {
        return nullptr;
    }
    return &entry;
}

MeshEntry* lookup_mesh_locked(cad_mesh handle) {
    std::uint16_t slot = 0;
    std::uint16_t generation = 0;
    if (!decode_handle(handle, slot, generation)) {
        return nullptr;
    }

    const auto index = static_cast<std::size_t>(slot - 1);
    if (index >= g_meshes.size()) {
        return nullptr;
    }

    auto& entry = g_meshes[index];
    if (entry.generation != generation || !entry.mesh.has_value()) {
        return nullptr;
    }
    return &entry;
}

OperationEntry* lookup_operation_locked(cad_operation handle) {
    std::uint16_t slot = 0;
    std::uint16_t generation = 0;
    if (!decode_handle(handle, slot, generation)) {
        return nullptr;
    }

    const auto index = static_cast<std::size_t>(slot - 1);
    if (index >= g_operations.size()) {
        return nullptr;
    }

    auto& entry = g_operations[index];
    if (entry.generation != generation || !entry.operation.has_value()) {
        return nullptr;
    }
    return &entry;
}

cad_result insert_shape(TopoDS_Shape shape, cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }

    std::lock_guard lock(g_shapes_mutex);
    try {
        std::uint16_t slot = 0;
        if (!g_free_slots.empty()) {
            slot = g_free_slots.back();
            g_free_slots.pop_back();
            auto& entry = g_shapes[slot - 1];
            entry.shape.emplace(std::move(shape));
            *out_shape = encode_handle(slot, entry.generation);
            return CAD_OK;
        }

        if (g_shapes.size() >= kSlotMask) {
            return fail(CAD_ERROR_OUT_OF_MEMORY, "shape handle table is full");
        }

        g_shapes.emplace_back();
        auto& entry = g_shapes.back();
        entry.shape.emplace(std::move(shape));
        slot = static_cast<std::uint16_t>(g_shapes.size());
        *out_shape = encode_handle(slot, entry.generation);
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

cad_result copy_shape(cad_shape handle, TopoDS_Shape& out_shape) {
    std::lock_guard lock(g_shapes_mutex);
    auto* entry = lookup_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "shape handle is invalid or stale");
    }

    try {
        out_shape = *entry->shape;
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

cad_result import_step_shape(const char* path, cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (path == nullptr || path[0] == '\0') {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "STEP path must not be null or empty");
    }

    try {
        TopoDS_Shape imported;
        DESTEP_Provider provider(new DESTEP_ConfigurationNode());
        if (!provider.Read(TCollection_AsciiString(path), imported) || imported.IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "STEP import failed");
        }
        return insert_shape(std::move(imported), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result export_step_shape(cad_shape handle, const char* path) {
    if (path == nullptr || path[0] == '\0') {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "STEP path must not be null or empty");
    }

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        DESTEP_Provider provider(new DESTEP_ConfigurationNode());
        if (!provider.Write(TCollection_AsciiString(path), source)) {
            return fail(CAD_ERROR_OPERATION_FAILED, "STEP export failed");
        }
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

void release_shape_locked(cad_shape handle) {
    auto* entry = lookup_locked(handle);
    if (entry == nullptr) {
        return;
    }

    const auto slot = static_cast<std::uint16_t>(handle & kSlotMask);
    entry->shape.reset();
    entry->generation = static_cast<std::uint16_t>(entry->generation + 1);
    if (entry->generation == 0) {
        entry->generation = 1;
    }
    g_free_slots.push_back(slot);
}

bool shape_kind_to_occt(cad_shape_kind kind, TopAbs_ShapeEnum& out_kind) {
    switch (kind) {
    case CAD_SHAPE_COMPOUND:
        out_kind = TopAbs_COMPOUND;
        return true;
    case CAD_SHAPE_COMPSOLID:
        out_kind = TopAbs_COMPSOLID;
        return true;
    case CAD_SHAPE_SOLID:
        out_kind = TopAbs_SOLID;
        return true;
    case CAD_SHAPE_SHELL:
        out_kind = TopAbs_SHELL;
        return true;
    case CAD_SHAPE_FACE:
        out_kind = TopAbs_FACE;
        return true;
    case CAD_SHAPE_WIRE:
        out_kind = TopAbs_WIRE;
        return true;
    case CAD_SHAPE_EDGE:
        out_kind = TopAbs_EDGE;
        return true;
    case CAD_SHAPE_VERTEX:
        out_kind = TopAbs_VERTEX;
        return true;
    case CAD_SHAPE_UNKNOWN:
        return false;
    }
    return false;
}

cad_shape_kind shape_kind_from_occt(TopAbs_ShapeEnum kind) {
    switch (kind) {
    case TopAbs_COMPOUND:
        return CAD_SHAPE_COMPOUND;
    case TopAbs_COMPSOLID:
        return CAD_SHAPE_COMPSOLID;
    case TopAbs_SOLID:
        return CAD_SHAPE_SOLID;
    case TopAbs_SHELL:
        return CAD_SHAPE_SHELL;
    case TopAbs_FACE:
        return CAD_SHAPE_FACE;
    case TopAbs_WIRE:
        return CAD_SHAPE_WIRE;
    case TopAbs_EDGE:
        return CAD_SHAPE_EDGE;
    case TopAbs_VERTEX:
        return CAD_SHAPE_VERTEX;
    case TopAbs_SHAPE:
        return CAD_SHAPE_UNKNOWN;
    }
    return CAD_SHAPE_UNKNOWN;
}

cad_surface_kind surface_kind_from_occt(const Handle(Geom_Surface)& surface) {
    if (surface.IsNull()) {
        return CAD_SURFACE_UNKNOWN;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_Plane))) {
        return CAD_SURFACE_PLANE;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface))) {
        return CAD_SURFACE_CYLINDER;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_ConicalSurface))) {
        return CAD_SURFACE_CONE;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_SphericalSurface))) {
        return CAD_SURFACE_SPHERE;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_ToroidalSurface))) {
        return CAD_SURFACE_TORUS;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_BezierSurface))) {
        return CAD_SURFACE_BEZIER;
    }
    if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
        return CAD_SURFACE_BSPLINE;
    }
    return CAD_SURFACE_UNKNOWN;
}

cad_curve_kind curve_kind_from_occt(GeomAbs_CurveType kind) {
    switch (kind) {
    case GeomAbs_Line:
        return CAD_CURVE_LINE;
    case GeomAbs_Circle:
        return CAD_CURVE_CIRCLE;
    case GeomAbs_Ellipse:
        return CAD_CURVE_ELLIPSE;
    case GeomAbs_Hyperbola:
        return CAD_CURVE_HYPERBOLA;
    case GeomAbs_Parabola:
        return CAD_CURVE_PARABOLA;
    case GeomAbs_BezierCurve:
        return CAD_CURVE_BEZIER;
    case GeomAbs_BSplineCurve:
        return CAD_CURVE_BSPLINE;
    case GeomAbs_OffsetCurve:
        return CAD_CURVE_OFFSET;
    case GeomAbs_OtherCurve:
        return CAD_CURVE_UNKNOWN;
    }
    return CAD_CURVE_UNKNOWN;
}

cad_result collect_subshapes(
    const TopoDS_Shape& shape,
    cad_shape_kind kind,
    std::vector<TopoDS_Shape>& out_subshapes) {
    TopAbs_ShapeEnum occt_kind = TopAbs_SHAPE;
    if (!shape_kind_to_occt(kind, occt_kind)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape kind is invalid");
    }

    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> subshape_map;
    TopExp::MapShapes(shape, occt_kind, subshape_map);
    out_subshapes.reserve(static_cast<std::size_t>(subshape_map.Extent()));
    for (int index = 1; index <= subshape_map.Extent(); ++index) {
        out_subshapes.push_back(subshape_map(index));
    }
    return CAD_OK;
}

cad_result make_transformed_shape(
    cad_shape handle,
    const gp_Trsf& transform,
    cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        auto transformed = BRepBuilderAPI_Transform(source, transform, true).Shape();
        return insert_shape(std::move(transformed), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result insert_operation(OperationData operation, cad_operation* out_operation);

template <typename Operation>
cad_result collect_operation_history(
    Operation& operation,
    const TopoDS_Shape& first,
    const TopoDS_Shape* second,
    OperationData& out_data);

cad_result make_extruded_shape(
    cad_shape handle,
    const gp_Vec& delta,
    cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        BRepPrimAPI_MakePrism operation(source, delta, true);
        if (operation.Shape().IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "extrusion produced a null shape");
        }
        return insert_shape(operation.Shape(), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result make_revolved_shape(
    cad_shape handle,
    const gp_Ax1& axis,
    double angle,
    cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        BRepPrimAPI_MakeRevol operation(source, axis, angle, true);
        if (operation.Shape().IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "revolution produced a null shape");
        }
        return insert_shape(operation.Shape(), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result make_extrusion_operation(
    cad_shape handle,
    const gp_Vec& delta,
    cad_operation* out_operation) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        BRepPrimAPI_MakePrism operation(source, delta, true);
        if (operation.Shape().IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "extrusion produced a null shape");
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, source, nullptr, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result make_revolution_operation(
    cad_shape handle,
    const gp_Ax1& axis,
    double angle,
    cad_operation* out_operation) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        BRepPrimAPI_MakeRevol operation(source, axis, angle, true);
        if (operation.Shape().IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "revolution produced a null shape");
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, source, nullptr, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

template <typename Builder>
cad_result configure_all_edge_finish(
    Builder& operation,
    const TopoDS_Shape& source,
    double amount,
    const char* operation_name) {
    bool has_edge = false;
    for (TopExp_Explorer explorer(source, TopAbs_EDGE); explorer.More(); explorer.Next()) {
        operation.Add(amount, TopoDS::Edge(explorer.Current()));
        has_edge = true;
    }
    if (!has_edge) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "edge finish requires at least one edge");
    }

    operation.Build();
    if (!operation.IsDone() || operation.Shape().IsNull()) {
        return fail(CAD_ERROR_OPERATION_FAILED, operation_name);
    }
    return CAD_OK;
}

cad_result collect_selected_edges(
    const TopoDS_Shape& source,
    const cad_shape_ref* edge_handles,
    std::uint32_t edge_count,
    std::vector<TopoDS_Shape>& out_edges) {
    if (edge_handles == nullptr || edge_count == 0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "selected edge list must not be empty");
    }

    std::vector<TopoDS_Shape> source_edges;
    for (TopExp_Explorer explorer(source, TopAbs_EDGE); explorer.More(); explorer.Next()) {
        source_edges.push_back(explorer.Current());
    }
    if (source_edges.empty()) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "edge finish requires at least one edge");
    }

    out_edges.reserve(edge_count);
    for (std::uint32_t index = 0; index < edge_count; ++index) {
        TopoDS_Shape selected;
        const auto copy_result = copy_shape(edge_handles[index].shape, selected);
        if (copy_result != CAD_OK) {
            return copy_result;
        }
        if (selected.IsNull() || selected.ShapeType() != TopAbs_EDGE) {
            return fail(CAD_ERROR_INVALID_ARGUMENT, "selected edge handle is not an edge");
        }

        bool belongs_to_source = false;
        for (const auto& source_edge : source_edges) {
            if (selected.IsSame(source_edge)) {
                belongs_to_source = true;
                break;
            }
        }
        if (!belongs_to_source) {
            return fail(CAD_ERROR_INVALID_ARGUMENT, "selected edge does not belong to shape");
        }

        for (const auto& previous : out_edges) {
            if (selected.IsSame(previous)) {
                return fail(CAD_ERROR_INVALID_ARGUMENT, "selected edge list contains duplicates");
            }
        }
        out_edges.push_back(std::move(selected));
    }
    return CAD_OK;
}

template <typename Builder>
cad_result configure_selected_edge_finish(
    Builder& operation,
    const TopoDS_Shape& source,
    const cad_shape_ref* edge_handles,
    std::uint32_t edge_count,
    double amount,
    const char* operation_name) {
    std::vector<TopoDS_Shape> selected_edges;
    const auto collect_result = collect_selected_edges(
        source, edge_handles, edge_count, selected_edges);
    if (collect_result != CAD_OK) {
        return collect_result;
    }

    for (const auto& selected_edge : selected_edges) {
        operation.Add(amount, TopoDS::Edge(selected_edge));
    }
    operation.Build();
    if (!operation.IsDone() || operation.Shape().IsNull()) {
        return fail(CAD_ERROR_OPERATION_FAILED, operation_name);
    }
    return CAD_OK;
}

template <typename Builder>
cad_result make_edge_finish_shape(
    cad_shape handle,
    double amount,
    cad_shape* out_shape,
    const char* operation_name) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        Builder operation(source);
        const auto configure_result = configure_all_edge_finish(
            operation, source, amount, operation_name);
        if (configure_result != CAD_OK) {
            return configure_result;
        }
        return insert_shape(operation.Shape(), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown edge finish exception");
    }
}

template <typename Builder>
cad_result make_edge_finish_operation(
    cad_shape handle,
    double amount,
    cad_operation* out_operation,
    const char* operation_name) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        Builder operation(source);
        const auto configure_result = configure_all_edge_finish(
            operation, source, amount, operation_name);
        if (configure_result != CAD_OK) {
            return configure_result;
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, source, nullptr, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown edge finish exception");
    }
}

template <typename Builder>
cad_result make_selected_edge_finish_shape(
    cad_shape handle,
    const cad_shape_ref* edge_handles,
    std::uint32_t edge_count,
    double amount,
    cad_shape* out_shape,
    const char* operation_name) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        Builder operation(source);
        const auto configure_result = configure_selected_edge_finish(
            operation,
            source,
            edge_handles,
            edge_count,
            amount,
            operation_name);
        if (configure_result != CAD_OK) {
            return configure_result;
        }
        return insert_shape(operation.Shape(), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown edge finish exception");
    }
}

template <typename Builder>
cad_result make_selected_edge_finish_operation(
    cad_shape handle,
    const cad_shape_ref* edge_handles,
    std::uint32_t edge_count,
    double amount,
    cad_operation* out_operation,
    const char* operation_name) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        Builder operation(source);
        const auto configure_result = configure_selected_edge_finish(
            operation,
            source,
            edge_handles,
            edge_count,
            amount,
            operation_name);
        if (configure_result != CAD_OK) {
            return configure_result;
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, source, nullptr, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown edge finish exception");
    }
}

template <typename Operation>
cad_result make_boolean_shape(
    cad_shape first_handle,
    cad_shape second_handle,
    cad_shape* out_shape) {
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape first;
    const auto first_result = copy_shape(first_handle, first);
    if (first_result != CAD_OK) {
        return first_result;
    }
    TopoDS_Shape second;
    const auto second_result = copy_shape(second_handle, second);
    if (second_result != CAD_OK) {
        return second_result;
    }

    try {
        Operation operation(first, second);
        if (!operation.IsDone() || operation.HasErrors()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "OCCT boolean operation failed");
        }
        return insert_shape(operation.Shape(), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result insert_operation(OperationData operation, cad_operation* out_operation);

template <typename Operation>
cad_result collect_operation_history(
    Operation& operation,
    const TopoDS_Shape& first,
    const TopoDS_Shape* second,
    OperationData& out_data) {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> source_map;
    source_map.Add(first);
    TopExp::MapShapes(first, source_map);
    if (second != nullptr) {
        source_map.Add(*second);
        TopExp::MapShapes(*second, source_map);
    }

    for (int index = 1; index <= source_map.Extent(); ++index) {
        const auto source = source_map(index);

        const auto& generated = operation.Generated(source);
        for (NCollection_List<TopoDS_Shape>::Iterator iterator(generated);
             iterator.More();
             iterator.Next()) {
            if (!iterator.Value().IsNull()) {
                out_data.generated.push_back({source, iterator.Value()});
            }
        }

        const auto& modified = operation.Modified(source);
        for (NCollection_List<TopoDS_Shape>::Iterator iterator(modified);
             iterator.More();
             iterator.Next()) {
            if (!iterator.Value().IsNull()) {
                out_data.modified.push_back({source, iterator.Value()});
            }
        }

        if (operation.IsDeleted(source)) {
            out_data.deleted.push_back({source, TopoDS_Shape()});
        }
    }
    return CAD_OK;
}

template <typename Operation>
cad_result make_boolean_operation(
    cad_shape first_handle,
    cad_shape second_handle,
    cad_operation* out_operation) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape first;
    const auto first_result = copy_shape(first_handle, first);
    if (first_result != CAD_OK) {
        return first_result;
    }
    TopoDS_Shape second;
    const auto second_result = copy_shape(second_handle, second);
    if (second_result != CAD_OK) {
        return second_result;
    }

    try {
        Operation operation;
        NCollection_List<TopoDS_Shape> arguments;
        arguments.Append(first);
        NCollection_List<TopoDS_Shape> tools;
        tools.Append(second);
        operation.SetArguments(arguments);
        operation.SetTools(tools);
        operation.SetNonDestructive(true);
        operation.SetToFillHistory(true);
        operation.Build();
        if (!operation.IsDone() || operation.HasErrors()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "OCCT boolean operation failed");
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, first, &second, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result make_transform_operation(
    cad_shape handle,
    const gp_Trsf& transform,
    cad_operation* out_operation) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        BRepBuilderAPI_Transform operation(source, transform, true);
        if (!operation.IsDone()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "OCCT transform operation failed");
        }

        OperationData data;
        data.result = operation.Shape();
        const auto history_result = collect_operation_history(operation, source, nullptr, data);
        if (history_result != CAD_OK) {
            return history_result;
        }
        return insert_operation(std::move(data), out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

const std::vector<HistoryRelation>* history_relations(
    const OperationData& operation,
    cad_history_relation relation) {
    switch (relation) {
    case CAD_HISTORY_GENERATED:
        return &operation.generated;
    case CAD_HISTORY_MODIFIED:
        return &operation.modified;
    case CAD_HISTORY_DELETED:
        return &operation.deleted;
    }
    return nullptr;
}

cad_result copy_operation_result(
    cad_operation handle,
    TopoDS_Shape& out_shape) {
    std::lock_guard lock(g_operations_mutex);
    auto* entry = lookup_operation_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "operation handle is invalid or stale");
    }
    try {
        out_shape = entry->operation->result;
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

cad_result copy_operation_history_shape(
    cad_operation handle,
    cad_history_relation relation,
    std::uint32_t index,
    bool target,
    TopoDS_Shape& out_shape) {
    std::lock_guard lock(g_operations_mutex);
    auto* entry = lookup_operation_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "operation handle is invalid or stale");
    }
    const auto* relations = history_relations(*entry->operation, relation);
    if (relations == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "history relation is invalid");
    }
    if (target && relation == CAD_HISTORY_DELETED) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "deleted history entries have no target");
    }
    if (index >= relations->size()) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "history index is out of range");
    }

    try {
        out_shape = target ? (*relations)[index].target : (*relations)[index].source;
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

template <typename Measure>
cad_result measure_shape(
    cad_shape handle,
    double* output,
    Measure measure) {
    if (output == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "output must not be null");
    }
    *output = 0.0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(handle, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        GProp_GProps properties;
        measure(source, properties);
        const auto value = properties.Mass();
        if (!std::isfinite(value)) {
            return fail(CAD_ERROR_OPERATION_FAILED, "OCCT measurement is not finite");
        }
        *output = value;
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

cad_result insert_mesh(MeshData mesh, cad_mesh* out_mesh) {
    if (out_mesh == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_mesh must not be null");
    }

    std::lock_guard lock(g_meshes_mutex);
    try {
        std::uint16_t slot = 0;
        if (!g_free_mesh_slots.empty()) {
            slot = g_free_mesh_slots.back();
            g_free_mesh_slots.pop_back();
            auto& entry = g_meshes[slot - 1];
            entry.mesh.emplace(std::move(mesh));
            *out_mesh = encode_handle(slot, entry.generation);
            return CAD_OK;
        }

        if (g_meshes.size() >= kSlotMask) {
            return fail(CAD_ERROR_OUT_OF_MEMORY, "mesh handle table is full");
        }

        g_meshes.emplace_back();
        auto& entry = g_meshes.back();
        entry.mesh.emplace(std::move(mesh));
        slot = static_cast<std::uint16_t>(g_meshes.size());
        *out_mesh = encode_handle(slot, entry.generation);
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

cad_result insert_operation(OperationData operation, cad_operation* out_operation) {
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }

    std::lock_guard lock(g_operations_mutex);
    try {
        std::uint16_t slot = 0;
        if (!g_free_operation_slots.empty()) {
            slot = g_free_operation_slots.back();
            g_free_operation_slots.pop_back();
            auto& entry = g_operations[slot - 1];
            entry.operation.emplace(std::move(operation));
            *out_operation = encode_handle(slot, entry.generation);
            return CAD_OK;
        }

        if (g_operations.size() >= kSlotMask) {
            return fail(CAD_ERROR_OUT_OF_MEMORY, "operation handle table is full");
        }

        g_operations.emplace_back();
        auto& entry = g_operations.back();
        entry.operation.emplace(std::move(operation));
        slot = static_cast<std::uint16_t>(g_operations.size());
        *out_operation = encode_handle(slot, entry.generation);
        return CAD_OK;
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    }
}

void release_operation_locked(cad_operation handle) {
    auto* entry = lookup_operation_locked(handle);
    if (entry == nullptr) {
        return;
    }

    const auto slot = static_cast<std::uint16_t>(handle & kSlotMask);
    entry->operation.reset();
    entry->generation = static_cast<std::uint16_t>(entry->generation + 1);
    if (entry->generation == 0) {
        entry->generation = 1;
    }
    g_free_operation_slots.push_back(slot);
}

cad_result build_mesh(
    const TopoDS_Shape& shape,
    const cad_mesh_options& options,
    MeshData& out_mesh) {
    BRepMesh_IncrementalMesh mesher(
        shape,
        options.linear_deflection,
        false,
        options.angular_deflection,
        false);
    if (!mesher.IsDone()) {
        return fail(CAD_ERROR_OPERATION_FAILED, "OCCT tessellation did not complete");
    }

    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> face_map;
    TopExp::MapShapes(shape, TopAbs_FACE, face_map);

    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
        const auto face = TopoDS::Face(explorer.Current());
        const auto face_index = face_map.FindIndex(face);
        if (face_index <= 0) {
            return fail(CAD_ERROR_OPERATION_FAILED, "tessellated face is missing from topology map");
        }

        const auto first_index = out_mesh.indices.size();
        TopLoc_Location location;
        const auto& triangulation = BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull() || !triangulation->HasGeometry()) {
            out_mesh.face_ranges.push_back({
                static_cast<std::uint32_t>(face_index - 1),
                static_cast<std::uint32_t>(first_index),
                0u});
            continue;
        }

        const auto location_transform = location.Transformation();
        for (int triangle_index = 1;
             triangle_index <= triangulation->NbTriangles();
             ++triangle_index) {
            int node1 = 0;
            int node2 = 0;
            int node3 = 0;
            triangulation->Triangle(triangle_index).Get(node1, node2, node3);

            gp_Pnt point1 = triangulation->Node(node1);
            gp_Pnt point2 = triangulation->Node(node2);
            gp_Pnt point3 = triangulation->Node(node3);
            point1.Transform(location_transform);
            point2.Transform(location_transform);
            point3.Transform(location_transform);

            if (face.Orientation() == TopAbs_REVERSED) {
                std::swap(point2, point3);
            }

            const gp_Vec edge1(point1, point2);
            const gp_Vec edge2(point1, point3);
            auto normal = edge1.Crossed(edge2);
            if (normal.SquareMagnitude() <= 1e-30) {
                continue;
            }
            normal.Normalize();

            if (out_mesh.vertices.size() >
                std::numeric_limits<std::uint32_t>::max() - 3u) {
                return fail(CAD_ERROR_OPERATION_FAILED, "mesh exceeds 32-bit index range");
            }

            const auto first_index = static_cast<std::uint32_t>(out_mesh.vertices.size());
            out_mesh.vertices.push_back({point1.X(), point1.Y(), point1.Z()});
            out_mesh.vertices.push_back({point2.X(), point2.Y(), point2.Z()});
            out_mesh.vertices.push_back({point3.X(), point3.Y(), point3.Z()});
            out_mesh.normals.push_back({normal.X(), normal.Y(), normal.Z()});
            out_mesh.normals.push_back({normal.X(), normal.Y(), normal.Z()});
            out_mesh.normals.push_back({normal.X(), normal.Y(), normal.Z()});
            out_mesh.indices.push_back(first_index);
            out_mesh.indices.push_back(first_index + 1u);
            out_mesh.indices.push_back(first_index + 2u);
        }

        const auto face_index_count = out_mesh.indices.size() - first_index;
        if (face_index_count > std::numeric_limits<std::uint32_t>::max()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "mesh face range exceeds 32-bit index range");
        }
        out_mesh.face_ranges.push_back({
            static_cast<std::uint32_t>(face_index - 1),
            static_cast<std::uint32_t>(first_index),
            static_cast<std::uint32_t>(face_index_count)});
    }

    if (out_mesh.vertices.empty()) {
        return fail(CAD_ERROR_OPERATION_FAILED, "shape produced no triangles");
    }
    return CAD_OK;
}

template <typename Value, typename Values>
cad_result copy_mesh_values(
    cad_mesh handle,
    Value* output,
    std::uint32_t capacity,
    Values values) {
    std::lock_guard lock(g_meshes_mutex);
    auto* entry = lookup_mesh_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "mesh handle is invalid or stale");
    }

    const auto& source = values(*entry->mesh);
    if (source.size() > std::numeric_limits<std::uint32_t>::max()) {
        return fail(CAD_ERROR_OPERATION_FAILED, "mesh buffer exceeds 32-bit count range");
    }
    if (capacity < source.size()) {
        return fail(CAD_ERROR_BUFFER_TOO_SMALL, "mesh output buffer is too small");
    }
    if (!source.empty() && output == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "mesh output buffer must not be null");
    }

    if (!source.empty()) {
        std::copy(source.begin(), source.end(), output);
    }
    return CAD_OK;
}

template <typename Value, typename Values>
cad_result copy_mesh_bytes(
    cad_mesh handle,
    std::uint8_t* output,
    std::uint32_t* byte_capacity,
    Values values) {
    if (byte_capacity == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "byte_capacity must not be null");
    }

    std::lock_guard lock(g_meshes_mutex);
    auto* entry = lookup_mesh_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "mesh handle is invalid or stale");
    }

    const auto& source = values(*entry->mesh);
    if (source.size() > std::numeric_limits<std::uint32_t>::max() / sizeof(Value)) {
        return fail(CAD_ERROR_OPERATION_FAILED, "mesh byte buffer exceeds 32-bit range");
    }
    const auto required = static_cast<std::uint32_t>(source.size() * sizeof(Value));
    const auto capacity = *byte_capacity;
    *byte_capacity = required;
    if (capacity < required || (required != 0 && output == nullptr)) {
        return fail(CAD_ERROR_BUFFER_TOO_SMALL, "mesh byte output buffer is too small");
    }
    if (required != 0) {
        std::memcpy(output, source.data(), required);
    }
    return CAD_OK;
}

template <typename Values>
cad_result mesh_value_count(
    cad_mesh handle,
    std::uint32_t* out_count,
    Values values) {
    if (out_count == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_count must not be null");
    }

    std::lock_guard lock(g_meshes_mutex);
    auto* entry = lookup_mesh_locked(handle);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "mesh handle is invalid or stale");
    }

    const auto& source = values(*entry->mesh);
    if (source.size() > std::numeric_limits<std::uint32_t>::max()) {
        return fail(CAD_ERROR_OPERATION_FAILED, "mesh buffer exceeds 32-bit count range");
    }
    *out_count = static_cast<std::uint32_t>(source.size());
    return CAD_OK;
}

} // namespace

extern "C" CADKIT_API cad_result cad_box(
    double width,
    double depth,
    double height,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (!std::isfinite(width) || !std::isfinite(depth) || !std::isfinite(height) ||
        width <= 0.0 || depth <= 0.0 || height <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "box dimensions must be finite and positive");
    }

    try {
        auto shape = BRepPrimAPI_MakeBox(width, depth, height).Shape();
        return insert_shape(std::move(shape), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_cylinder(
    double radius,
    double height,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (!std::isfinite(radius) || !std::isfinite(height) ||
        radius <= 0.0 || height <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "cylinder dimensions must be finite and positive");
    }

    try {
        auto shape = BRepPrimAPI_MakeCylinder(radius, height).Shape();
        return insert_shape(std::move(shape), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_sphere(
    double radius,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (!std::isfinite(radius) || radius <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "sphere radius must be finite and positive");
    }

    try {
        auto shape = BRepPrimAPI_MakeSphere(radius).Shape();
        return insert_shape(std::move(shape), out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_step_import(
    const char* path,
    cad_shape* out_shape) {
    clear_error();
    return import_step_shape(path, out_shape);
}

extern "C" CADKIT_API cad_result cad_step_export(
    cad_shape shape,
    const char* path) {
    clear_error();
    return export_step_shape(shape, path);
}

extern "C" CADKIT_API cad_result cad_shape_translate(
    cad_shape shape,
    cad_vec3 delta,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (!std::isfinite(delta.x) || !std::isfinite(delta.y) || !std::isfinite(delta.z)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "translation must be finite");
    }

    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(delta.x, delta.y, delta.z));
    return make_transformed_shape(shape, transform, out_shape);
}

extern "C" CADKIT_API cad_result cad_shape_rotate(
    cad_shape shape,
    cad_vec3 axis,
    double angle,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;
    if (!std::isfinite(axis.x) || !std::isfinite(axis.y) || !std::isfinite(axis.z) ||
        !std::isfinite(angle)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "rotation axis and angle must be finite");
    }

    const auto axis_length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_length <= 1e-15) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "rotation axis must not be zero");
    }

    try {
        const gp_Dir direction(axis.x, axis.y, axis.z);
        gp_Trsf transform;
        transform.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), direction), angle);
        return make_transformed_shape(shape, transform, out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_clone(
    cad_shape shape,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    return insert_shape(std::move(source), out_shape);
}

extern "C" CADKIT_API cad_result cad_shape_extrude(
    cad_shape profile,
    cad_vec3 delta,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(delta.x) || !std::isfinite(delta.y) ||
        !std::isfinite(delta.z) ||
        (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) <= 1e-30) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "extrusion vector must be finite and non-zero");
    }
    return make_extruded_shape(profile, gp_Vec(delta.x, delta.y, delta.z), out_shape);
}

extern "C" CADKIT_API cad_result cad_shape_revolve(
    cad_shape profile,
    cad_vec3 axis_origin,
    cad_vec3 axis_direction,
    double angle,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(axis_origin.x) || !std::isfinite(axis_origin.y) ||
        !std::isfinite(axis_origin.z) || !std::isfinite(axis_direction.x) ||
        !std::isfinite(axis_direction.y) || !std::isfinite(axis_direction.z) ||
        !std::isfinite(angle)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution axis and angle must be finite");
    }
    const auto axis_length_squared = axis_direction.x * axis_direction.x +
        axis_direction.y * axis_direction.y + axis_direction.z * axis_direction.z;
    if (axis_length_squared <= 1e-30) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution axis direction must not be zero");
    }
    if (std::abs(angle) <= 1e-15) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution angle must not be zero");
    }

    try {
        const gp_Ax1 axis(
            gp_Pnt(axis_origin.x, axis_origin.y, axis_origin.z),
            gp_Dir(axis_direction.x, axis_direction.y, axis_direction.z));
        return make_revolved_shape(profile, axis, angle, out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_INVALID_ARGUMENT, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, error);
    } catch (...) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "unknown revolution argument failure");
    }
}

extern "C" CADKIT_API cad_result cad_shape_fillet(
    cad_shape shape,
    double radius,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(radius) || radius <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "fillet radius must be finite and positive");
    }
    return make_edge_finish_shape<BRepFilletAPI_MakeFillet>(
        shape, radius, out_shape, "fillet produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_chamfer(
    cad_shape shape,
    double distance,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(distance) || distance <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "chamfer distance must be finite and positive");
    }
    return make_edge_finish_shape<BRepFilletAPI_MakeChamfer>(
        shape, distance, out_shape, "chamfer produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_fillet_edges(
    cad_shape shape,
    const cad_shape_ref* edges,
    std::uint32_t edge_count,
    double radius,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(radius) || radius <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "fillet radius must be finite and positive");
    }
    return make_selected_edge_finish_shape<BRepFilletAPI_MakeFillet>(
        shape,
        edges,
        edge_count,
        radius,
        out_shape,
        "fillet produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_chamfer_edges(
    cad_shape shape,
    const cad_shape_ref* edges,
    std::uint32_t edge_count,
    double distance,
    cad_shape* out_shape) {
    clear_error();
    if (!std::isfinite(distance) || distance <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "chamfer distance must be finite and positive");
    }
    return make_selected_edge_finish_shape<BRepFilletAPI_MakeChamfer>(
        shape,
        edges,
        edge_count,
        distance,
        out_shape,
        "chamfer produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_translate_operation(
    cad_shape shape,
    cad_vec3 delta,
    cad_operation* out_operation) {
    clear_error();
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;
    if (!std::isfinite(delta.x) || !std::isfinite(delta.y) || !std::isfinite(delta.z)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "translation must be finite");
    }

    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(delta.x, delta.y, delta.z));
    return make_transform_operation(shape, transform, out_operation);
}

extern "C" CADKIT_API cad_result cad_shape_rotate_operation(
    cad_shape shape,
    cad_vec3 axis,
    double angle,
    cad_operation* out_operation) {
    clear_error();
    if (out_operation == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_operation must not be null");
    }
    *out_operation = 0;
    if (!std::isfinite(axis.x) || !std::isfinite(axis.y) || !std::isfinite(axis.z) ||
        !std::isfinite(angle)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "rotation axis and angle must be finite");
    }

    const auto axis_length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (axis_length <= 1e-15) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "rotation axis must not be zero");
    }

    try {
        const gp_Dir direction(axis.x, axis.y, axis.z);
        gp_Trsf transform;
        transform.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), direction), angle);
        return make_transform_operation(shape, transform, out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_extrude_operation(
    cad_shape profile,
    cad_vec3 delta,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(delta.x) || !std::isfinite(delta.y) ||
        !std::isfinite(delta.z) ||
        (delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) <= 1e-30) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "extrusion vector must be finite and non-zero");
    }
    return make_extrusion_operation(profile, gp_Vec(delta.x, delta.y, delta.z), out_operation);
}

extern "C" CADKIT_API cad_result cad_shape_revolve_operation(
    cad_shape profile,
    cad_vec3 axis_origin,
    cad_vec3 axis_direction,
    double angle,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(axis_origin.x) || !std::isfinite(axis_origin.y) ||
        !std::isfinite(axis_origin.z) || !std::isfinite(axis_direction.x) ||
        !std::isfinite(axis_direction.y) || !std::isfinite(axis_direction.z) ||
        !std::isfinite(angle)) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution axis and angle must be finite");
    }
    const auto axis_length_squared = axis_direction.x * axis_direction.x +
        axis_direction.y * axis_direction.y + axis_direction.z * axis_direction.z;
    if (axis_length_squared <= 1e-30) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution axis direction must not be zero");
    }
    if (std::abs(angle) <= 1e-15) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "revolution angle must not be zero");
    }

    try {
        const gp_Ax1 axis(
            gp_Pnt(axis_origin.x, axis_origin.y, axis_origin.z),
            gp_Dir(axis_direction.x, axis_direction.y, axis_direction.z));
        return make_revolution_operation(profile, axis, angle, out_operation);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_INVALID_ARGUMENT, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, error);
    } catch (...) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "unknown revolution argument failure");
    }
}

extern "C" CADKIT_API cad_result cad_shape_fillet_operation(
    cad_shape shape,
    double radius,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(radius) || radius <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "fillet radius must be finite and positive");
    }
    return make_edge_finish_operation<BRepFilletAPI_MakeFillet>(
        shape, radius, out_operation, "fillet produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_chamfer_operation(
    cad_shape shape,
    double distance,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(distance) || distance <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "chamfer distance must be finite and positive");
    }
    return make_edge_finish_operation<BRepFilletAPI_MakeChamfer>(
        shape, distance, out_operation, "chamfer produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_fillet_edges_operation(
    cad_shape shape,
    const cad_shape_ref* edges,
    std::uint32_t edge_count,
    double radius,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(radius) || radius <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "fillet radius must be finite and positive");
    }
    return make_selected_edge_finish_operation<BRepFilletAPI_MakeFillet>(
        shape,
        edges,
        edge_count,
        radius,
        out_operation,
        "fillet produced no shape");
}

extern "C" CADKIT_API cad_result cad_shape_chamfer_edges_operation(
    cad_shape shape,
    const cad_shape_ref* edges,
    std::uint32_t edge_count,
    double distance,
    cad_operation* out_operation) {
    clear_error();
    if (!std::isfinite(distance) || distance <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "chamfer distance must be finite and positive");
    }
    return make_selected_edge_finish_operation<BRepFilletAPI_MakeChamfer>(
        shape,
        edges,
        edge_count,
        distance,
        out_operation,
        "chamfer produced no shape");
}

extern "C" CADKIT_API cad_result cad_fuse(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape) {
    clear_error();
    return make_boolean_shape<BRepAlgoAPI_Fuse>(first, second, out_shape);
}

extern "C" CADKIT_API cad_result cad_cut(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape) {
    clear_error();
    return make_boolean_shape<BRepAlgoAPI_Cut>(first, second, out_shape);
}

extern "C" CADKIT_API cad_result cad_common(
    cad_shape first,
    cad_shape second,
    cad_shape* out_shape) {
    clear_error();
    return make_boolean_shape<BRepAlgoAPI_Common>(first, second, out_shape);
}

extern "C" CADKIT_API cad_result cad_fuse_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation) {
    clear_error();
    return make_boolean_operation<BRepAlgoAPI_Fuse>(first, second, out_operation);
}

extern "C" CADKIT_API cad_result cad_cut_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation) {
    clear_error();
    return make_boolean_operation<BRepAlgoAPI_Cut>(first, second, out_operation);
}

extern "C" CADKIT_API cad_result cad_common_operation(
    cad_shape first,
    cad_shape second,
    cad_operation* out_operation) {
    clear_error();
    return make_boolean_operation<BRepAlgoAPI_Common>(first, second, out_operation);
}

extern "C" CADKIT_API cad_result cad_shape_bounds(
    cad_shape shape,
    cad_bounds* out_bounds) {
    clear_error();
    if (out_bounds == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_bounds must not be null");
    }

    std::lock_guard lock(g_shapes_mutex);
    auto* entry = lookup_locked(shape);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "shape handle is invalid or stale");
    }

    try {
        Bnd_Box bounds;
        BRepBndLib::AddOptimal(*entry->shape, bounds, true, false);
        if (bounds.IsVoid()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "shape has no bounds");
        }

        double xmin = 0.0;
        double ymin = 0.0;
        double zmin = 0.0;
        double xmax = 0.0;
        double ymax = 0.0;
        double zmax = 0.0;
        bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        out_bounds->min = {xmin, ymin, zmin};
        out_bounds->max = {xmax, ymax, zmax};
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_area(
    cad_shape shape,
    double* out_area) {
    clear_error();
    return measure_shape(shape, out_area, [](const TopoDS_Shape& source, GProp_GProps& properties) {
        BRepGProp::SurfaceProperties(source, properties);
    });
}

extern "C" CADKIT_API cad_result cad_shape_volume(
    cad_shape shape,
    double* out_volume) {
    clear_error();
    return measure_shape(shape, out_volume, [](const TopoDS_Shape& source, GProp_GProps& properties) {
        BRepGProp::VolumeProperties(source, properties);
    });
}

extern "C" CADKIT_API cad_result cad_shape_kind_get(
    cad_shape shape,
    cad_shape_kind* out_kind) {
    clear_error();
    if (out_kind == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_kind must not be null");
    }
    *out_kind = CAD_SHAPE_UNKNOWN;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    *out_kind = shape_kind_from_occt(source.ShapeType());
    return CAD_OK;
}

extern "C" CADKIT_API cad_result cad_shape_subshape_count(
    cad_shape shape,
    cad_shape_kind kind,
    std::uint32_t* out_count) {
    clear_error();
    if (out_count == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_count must not be null");
    }
    *out_count = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        std::vector<TopoDS_Shape> subshapes;
        const auto collect_result = collect_subshapes(source, kind, subshapes);
        if (collect_result != CAD_OK) {
            return collect_result;
        }
        if (subshapes.size() > std::numeric_limits<std::uint32_t>::max()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "subshape count exceeds 32-bit range");
        }
        *out_count = static_cast<std::uint32_t>(subshapes.size());
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_subshape_at(
    cad_shape shape,
    cad_shape_kind kind,
    std::uint32_t index,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        std::vector<TopoDS_Shape> subshapes;
        const auto collect_result = collect_subshapes(source, kind, subshapes);
        if (collect_result != CAD_OK) {
            return collect_result;
        }
        if (index >= subshapes.size()) {
            return fail(CAD_ERROR_INVALID_ARGUMENT, "subshape index is out of range");
        }
        return insert_shape(subshapes[index], out_shape);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_subshapes(
    cad_shape shape,
    cad_shape_kind kind,
    cad_shape* output,
    std::uint32_t capacity) {
    clear_error();

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        std::vector<TopoDS_Shape> subshapes;
        const auto collect_result = collect_subshapes(source, kind, subshapes);
        if (collect_result != CAD_OK) {
            return collect_result;
        }
        if (subshapes.size() > std::numeric_limits<std::uint32_t>::max()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "subshape count exceeds 32-bit range");
        }
        if (capacity < subshapes.size()) {
            return fail(CAD_ERROR_BUFFER_TOO_SMALL, "subshape output buffer is too small");
        }
        if (!subshapes.empty() && output == nullptr) {
            return fail(CAD_ERROR_INVALID_ARGUMENT, "subshape output buffer must not be null");
        }

        std::uint32_t inserted = 0;
        for (const auto& subshape : subshapes) {
            const auto insert_result = insert_shape(subshape, &output[inserted]);
            if (insert_result != CAD_OK) {
                std::lock_guard lock(g_shapes_mutex);
                while (inserted > 0) {
                    --inserted;
                    release_shape_locked(output[inserted]);
                }
                return insert_result;
            }
            ++inserted;
        }
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_shape_is_same(
    cad_shape first,
    cad_shape second,
    std::uint8_t* out_same) {
    clear_error();
    if (out_same == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_same must not be null");
    }
    *out_same = 0;

    TopoDS_Shape first_shape;
    const auto first_result = copy_shape(first, first_shape);
    if (first_result != CAD_OK) {
        return first_result;
    }
    TopoDS_Shape second_shape;
    const auto second_result = copy_shape(second, second_shape);
    if (second_result != CAD_OK) {
        return second_result;
    }
    *out_same = first_shape.IsSame(second_shape) ? 1u : 0u;
    return CAD_OK;
}

extern "C" CADKIT_API cad_result cad_face_surface_kind(
    cad_shape face,
    cad_surface_kind* out_kind) {
    clear_error();
    if (out_kind == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_kind must not be null");
    }
    *out_kind = CAD_SURFACE_UNKNOWN;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(face, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_FACE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be a face");
    }

    try {
        TopLoc_Location location;
        const auto surface = BRep_Tool::Surface(TopoDS::Face(source), location);
        *out_kind = surface_kind_from_occt(surface);
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_face_center(
    cad_shape face,
    cad_vec3* out_center) {
    clear_error();
    if (out_center == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_center must not be null");
    }
    *out_center = {};

    TopoDS_Shape source;
    const auto copy_result = copy_shape(face, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_FACE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be a face");
    }

    try {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(source, properties);
        const auto center = properties.CentreOfMass();
        if (!std::isfinite(center.X()) || !std::isfinite(center.Y()) ||
            !std::isfinite(center.Z())) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face center is not finite");
        }
        *out_center = {center.X(), center.Y(), center.Z()};
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_face_area(
    cad_shape face,
    double* out_area) {
    clear_error();
    if (out_area == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_area must not be null");
    }
    *out_area = 0.0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(face, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_FACE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be a face");
    }

    try {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(source, properties);
        const auto area = properties.Mass();
        if (!std::isfinite(area) || area < 0.0) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face area is not finite");
        }
        *out_area = area;
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_face_normal(
    cad_shape face,
    cad_vec3* out_normal) {
    clear_error();
    if (out_normal == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_normal must not be null");
    }
    *out_normal = {};

    TopoDS_Shape source;
    const auto copy_result = copy_shape(face, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_FACE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be a face");
    }

    try {
        const auto topo_face = TopoDS::Face(source);
        TopLoc_Location location;
        const auto surface = BRep_Tool::Surface(topo_face, location);
        if (surface.IsNull()) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face has no surface");
        }

        double u_min = 0.0;
        double u_max = 0.0;
        double v_min = 0.0;
        double v_max = 0.0;
        BRepTools::UVBounds(topo_face, u_min, u_max, v_min, v_max);
        if (!std::isfinite(u_min) || !std::isfinite(u_max) ||
            !std::isfinite(v_min) || !std::isfinite(v_max)) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face parameter bounds are not finite");
        }

        gp_Pnt point;
        gp_Vec derivative_u;
        gp_Vec derivative_v;
        surface->D1(
            u_min + (u_max - u_min) * 0.5,
            v_min + (v_max - v_min) * 0.5,
            point,
            derivative_u,
            derivative_v);
        derivative_u.Transform(location.Transformation());
        derivative_v.Transform(location.Transformation());
        auto normal = derivative_u.Crossed(derivative_v);
        if (normal.SquareMagnitude() <= 1e-30) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face normal is degenerate");
        }
        normal.Normalize();
        if (source.Orientation() == TopAbs_REVERSED) {
            normal.Reverse();
        }
        if (!std::isfinite(normal.X()) || !std::isfinite(normal.Y()) ||
            !std::isfinite(normal.Z())) {
            return fail(CAD_ERROR_OPERATION_FAILED, "face normal is not finite");
        }
        *out_normal = {normal.X(), normal.Y(), normal.Z()};
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_edge_curve_kind(
    cad_shape edge,
    cad_curve_kind* out_kind) {
    clear_error();
    if (out_kind == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_kind must not be null");
    }
    *out_kind = CAD_CURVE_UNKNOWN;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(edge, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_EDGE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be an edge");
    }

    try {
        const BRepAdaptor_Curve curve(TopoDS::Edge(source));
        *out_kind = curve_kind_from_occt(curve.GetType());
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_edge_length(
    cad_shape edge,
    double* out_length) {
    clear_error();
    if (out_length == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_length must not be null");
    }
    *out_length = 0.0;

    TopoDS_Shape source;
    const auto copy_result = copy_shape(edge, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_EDGE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be an edge");
    }

    try {
        GProp_GProps properties;
        BRepGProp::LinearProperties(source, properties);
        const auto length = properties.Mass();
        if (!std::isfinite(length) || length < 0.0) {
            return fail(CAD_ERROR_OPERATION_FAILED, "edge length is not finite");
        }
        *out_length = length;
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_edge_tangent_at(
    cad_shape edge,
    double parameter,
    cad_vec3* out_tangent) {
    clear_error();
    if (out_tangent == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_tangent must not be null");
    }
    *out_tangent = {};
    if (!std::isfinite(parameter) || parameter < 0.0 || parameter > 1.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "edge parameter must be finite and in [0, 1]");
    }

    TopoDS_Shape source;
    const auto copy_result = copy_shape(edge, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_EDGE) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be an edge");
    }

    try {
        const BRepAdaptor_Curve curve(TopoDS::Edge(source));
        const auto first = curve.FirstParameter();
        const auto last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last) || last < first) {
            return fail(CAD_ERROR_OPERATION_FAILED, "edge parameter range is invalid");
        }

        const auto value = first + (last - first) * parameter;
        gp_Pnt point;
        gp_Vec tangent;
        curve.D1(value, point, tangent);
        if (tangent.SquareMagnitude() <= 1e-30) {
            return fail(CAD_ERROR_OPERATION_FAILED, "edge tangent is degenerate");
        }
        tangent.Normalize();
        if (source.Orientation() == TopAbs_REVERSED) {
            tangent.Reverse();
        }
        if (!std::isfinite(tangent.X()) || !std::isfinite(tangent.Y()) ||
            !std::isfinite(tangent.Z())) {
            return fail(CAD_ERROR_OPERATION_FAILED, "edge tangent is not finite");
        }
        *out_tangent = {tangent.X(), tangent.Y(), tangent.Z()};
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_vertex_position(
    cad_shape vertex,
    cad_vec3* out_position) {
    clear_error();
    if (out_position == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_position must not be null");
    }
    *out_position = {};

    TopoDS_Shape source;
    const auto copy_result = copy_shape(vertex, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    if (source.ShapeType() != TopAbs_VERTEX) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "shape must be a vertex");
    }

    try {
        const auto point = BRep_Tool::Pnt(TopoDS::Vertex(source));
        if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) ||
            !std::isfinite(point.Z())) {
            return fail(CAD_ERROR_OPERATION_FAILED, "vertex position is not finite");
        }
        *out_position = {point.X(), point.Y(), point.Z()};
        return CAD_OK;
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_operation_result_shape(
    cad_operation operation,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape result;
    const auto copy_result = copy_operation_result(operation, result);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    return insert_shape(std::move(result), out_shape);
}

extern "C" CADKIT_API cad_result cad_operation_history_count(
    cad_operation operation,
    cad_history_relation relation,
    std::uint32_t* out_count) {
    clear_error();
    if (out_count == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_count must not be null");
    }
    *out_count = 0;

    std::lock_guard lock(g_operations_mutex);
    auto* entry = lookup_operation_locked(operation);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "operation handle is invalid or stale");
    }
    const auto* relations = history_relations(*entry->operation, relation);
    if (relations == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "history relation is invalid");
    }
    if (relations->size() > std::numeric_limits<std::uint32_t>::max()) {
        return fail(CAD_ERROR_OPERATION_FAILED, "history count exceeds 32-bit range");
    }
    *out_count = static_cast<std::uint32_t>(relations->size());
    return CAD_OK;
}

extern "C" CADKIT_API cad_result cad_operation_history_source_at(
    cad_operation operation,
    cad_history_relation relation,
    std::uint32_t index,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape source;
    const auto copy_result = copy_operation_history_shape(
        operation,
        relation,
        index,
        false,
        source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    return insert_shape(std::move(source), out_shape);
}

extern "C" CADKIT_API cad_result cad_operation_history_target_at(
    cad_operation operation,
    cad_history_relation relation,
    std::uint32_t index,
    cad_shape* out_shape) {
    clear_error();
    if (out_shape == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_shape must not be null");
    }
    *out_shape = 0;

    TopoDS_Shape target;
    const auto copy_result = copy_operation_history_shape(
        operation,
        relation,
        index,
        true,
        target);
    if (copy_result != CAD_OK) {
        return copy_result;
    }
    return insert_shape(std::move(target), out_shape);
}

extern "C" CADKIT_API cad_result cad_shape_tessellate(
    cad_shape shape,
    const cad_mesh_options* options,
    cad_mesh* out_mesh) {
    clear_error();
    if (out_mesh == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_mesh must not be null");
    }
    *out_mesh = 0;

    const cad_mesh_options default_options{0.1, 0.5};
    const auto& effective_options = options == nullptr ? default_options : *options;
    if (!std::isfinite(effective_options.linear_deflection) ||
        !std::isfinite(effective_options.angular_deflection) ||
        effective_options.linear_deflection <= 0.0 ||
        effective_options.angular_deflection <= 0.0) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "mesh options must be finite and positive");
    }

    TopoDS_Shape source;
    const auto copy_result = copy_shape(shape, source);
    if (copy_result != CAD_OK) {
        return copy_result;
    }

    try {
        MeshData mesh;
        const auto build_result = build_mesh(source, effective_options, mesh);
        if (build_result != CAD_OK) {
            return build_result;
        }
        return insert_mesh(std::move(mesh), out_mesh);
    } catch (const Standard_Failure& error) {
        return fail_occt(CAD_ERROR_OPERATION_FAILED, error);
    } catch (const std::bad_alloc& error) {
        return fail(CAD_ERROR_OUT_OF_MEMORY, error);
    } catch (const std::exception& error) {
        return fail(CAD_ERROR_OPERATION_FAILED, error);
    } catch (...) {
        return fail(CAD_ERROR_OPERATION_FAILED, "unknown native exception");
    }
}

extern "C" CADKIT_API cad_result cad_mesh_vertex_count(
    cad_mesh mesh,
    std::uint32_t* out_count) {
    clear_error();
    return mesh_value_count(mesh, out_count, [](const MeshData& value) -> const auto& {
        return value.vertices;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_index_count(
    cad_mesh mesh,
    std::uint32_t* out_count) {
    clear_error();
    return mesh_value_count(mesh, out_count, [](const MeshData& value) -> const auto& {
        return value.indices;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_face_range_count(
    cad_mesh mesh,
    std::uint32_t* out_count) {
    clear_error();
    return mesh_value_count(mesh, out_count, [](const MeshData& value) -> const auto& {
        return value.face_ranges;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_face_range_at(
    cad_mesh mesh,
    std::uint32_t index,
    cad_mesh_face_range* out_range) {
    clear_error();
    if (out_range == nullptr) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "out_range must not be null");
    }
    *out_range = {};

    std::lock_guard lock(g_meshes_mutex);
    auto* entry = lookup_mesh_locked(mesh);
    if (entry == nullptr) {
        return fail(CAD_ERROR_INVALID_HANDLE, "mesh handle is invalid or stale");
    }
    if (index >= entry->mesh->face_ranges.size()) {
        return fail(CAD_ERROR_INVALID_ARGUMENT, "mesh face range index is out of range");
    }
    *out_range = entry->mesh->face_ranges[index];
    return CAD_OK;
}

extern "C" CADKIT_API cad_result cad_mesh_copy_face_ranges(
    cad_mesh mesh,
    cad_mesh_face_range* output,
    std::uint32_t capacity) {
    clear_error();
    return copy_mesh_values(mesh, output, capacity, [](const MeshData& value) -> const auto& {
        return value.face_ranges;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_vertices(
    cad_mesh mesh,
    cad_vec3* output,
    std::uint32_t capacity) {
    clear_error();
    return copy_mesh_values(mesh, output, capacity, [](const MeshData& value) -> const auto& {
        return value.vertices;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_normals(
    cad_mesh mesh,
    cad_vec3* output,
    std::uint32_t capacity) {
    clear_error();
    return copy_mesh_values(mesh, output, capacity, [](const MeshData& value) -> const auto& {
        return value.normals;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_indices(
    cad_mesh mesh,
    std::uint32_t* output,
    std::uint32_t capacity) {
    clear_error();
    return copy_mesh_values(mesh, output, capacity, [](const MeshData& value) -> const auto& {
        return value.indices;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_vertices_bytes(
    cad_mesh mesh,
    std::uint8_t* output,
    std::uint32_t* byte_capacity) {
    clear_error();
    return copy_mesh_bytes<cad_vec3>(mesh, output, byte_capacity, [](const MeshData& value) -> const auto& {
        return value.vertices;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_normals_bytes(
    cad_mesh mesh,
    std::uint8_t* output,
    std::uint32_t* byte_capacity) {
    clear_error();
    return copy_mesh_bytes<cad_vec3>(mesh, output, byte_capacity, [](const MeshData& value) -> const auto& {
        return value.normals;
    });
}

extern "C" CADKIT_API cad_result cad_mesh_copy_indices_bytes(
    cad_mesh mesh,
    std::uint8_t* output,
    std::uint32_t* byte_capacity) {
    clear_error();
    return copy_mesh_bytes<std::uint32_t>(mesh, output, byte_capacity, [](const MeshData& value) -> const auto& {
        return value.indices;
    });
}

extern "C" CADKIT_API void cad_shape_destroy(cad_shape shape) {
    clear_error();
    std::lock_guard lock(g_shapes_mutex);
    release_shape_locked(shape);
}

extern "C" CADKIT_API void cad_mesh_destroy(cad_mesh mesh) {
    clear_error();
    std::lock_guard lock(g_meshes_mutex);
    auto* entry = lookup_mesh_locked(mesh);
    if (entry == nullptr) {
        return;
    }

    const auto slot = static_cast<std::uint16_t>(mesh & kSlotMask);
    entry->mesh.reset();
    entry->generation = static_cast<std::uint16_t>(entry->generation + 1);
    if (entry->generation == 0) {
        entry->generation = 1;
    }
    g_free_mesh_slots.push_back(slot);
}

extern "C" CADKIT_API void cad_operation_destroy(cad_operation operation) {
    clear_error();
    std::lock_guard lock(g_operations_mutex);
    release_operation_locked(operation);
}

extern "C" CADKIT_API const char* cad_last_error(void) {
    return g_last_error.c_str();
}
