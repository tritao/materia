// Topological names for faces, edges and vertices (plans/TOPOLOGICAL_NAMING.md).
//
// Every shape carries an immutable ElementMap. Faces are named explicitly;
// edges and vertices are derived from their adjacent faces, except boundary
// and wire edges and their vertices, which keep tracked names. Names are
// canonical text: atoms are escaped ([A-Za-z0-9_.+-] kept, other bytes %XX),
// structure uses ( ) { } | , : and the weak markers # ~ @.
#pragma once

#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cadkit_naming {

// Bumped whenever a rule change would rename an element (TN-D13).
constexpr std::uint32_t kSchemeVersion = 1;

enum class ElementKind { Face, Edge, Vertex };

using ShapeIndex = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

// `text` as a name atom.
std::string atom(const std::string& text);
// A weak name depends on enumeration or geometric order (TN-D8).
bool is_weak(const std::string& name);

class ElementMap {
public:
    // Indexed like TopExp::MapShapes(shape, kind). Faces are always named;
    // an empty edge or vertex entry means "derive from the faces".
    std::vector<std::string> faces;
    std::vector<std::string> edges;
    std::vector<std::string> vertices;

    // The final names of `kind` for `shape` (the shape this map belongs to).
    // Derived names are computed once and cached.
    const std::vector<std::string>& names(const TopoDS_Shape& shape, ElementKind kind) const;

private:
    void derive(const TopoDS_Shape& shape) const;
    mutable std::once_flag derived_once_;
    mutable std::vector<std::string> derived_edges_;
    mutable std::vector<std::string> derived_vertices_;
};

using ElementMapPtr = std::shared_ptr<const ElementMap>;

// A shape and its names, as an operation's input.
struct NamedShape {
    TopoDS_Shape shape;
    ElementMapPtr names;
};

// A mutable copy of `names` (derived names are recomputed for it).
std::shared_ptr<ElementMap> editable_copy(const ElementMap& names);

// Weak index names (`face#k`), for shapes nothing has named yet.
ElementMapPtr index_names(const TopoDS_Shape& shape);

// Faces named by role: each (face, role) pair is matched by IsSame; faces
// without a role get weak index names.
ElementMapPtr role_names(const TopoDS_Shape& shape,
                         const std::vector<std::pair<TopoDS_Shape, std::string>>& roles);

// A box: faces `box.+x` ... `box.-z` from their outward normals.
ElementMapPtr box_names(const TopoDS_Shape& box);

// History of one operation, as the propagation rules read it.
// Elements an operation generated from one source element, each with its role
// (`side`, `fillet`, ...); a generated element is named `role(sources)` (rule 5).
using Generated = std::vector<std::pair<TopoDS_Shape, std::string>>;

class History {
public:
    virtual ~History() = default;
    virtual std::vector<TopoDS_Shape> modified(const TopoDS_Shape& source) = 0;
    virtual bool deleted(const TopoDS_Shape& source) = 0;
    // `slot` is the source's input position, for adapters whose roles depend on it.
    virtual Generated generated(const TopoDS_Shape& source, std::size_t slot) {
        (void)source;
        (void)slot;
        return {};
    }
};

// An adapter written at the operation's call site (TN2's per-operation adapters).
class LambdaHistory final : public History {
public:
    std::function<std::vector<TopoDS_Shape>(const TopoDS_Shape&)> on_modified;
    std::function<bool(const TopoDS_Shape&)> on_deleted;
    std::function<Generated(const TopoDS_Shape&, std::size_t)> on_generated;

    std::vector<TopoDS_Shape> modified(const TopoDS_Shape& source) override {
        return on_modified ? on_modified(source) : std::vector<TopoDS_Shape>{};
    }
    bool deleted(const TopoDS_Shape& source) override { return on_deleted ? on_deleted(source) : false; }
    Generated generated(const TopoDS_Shape& source, std::size_t slot) override {
        return on_generated ? on_generated(source, slot) : Generated{};
    }
};

// The non-null shapes of an OCCT list.
template <typename List>
std::vector<TopoDS_Shape> shapes_of(const List& list) {
    std::vector<TopoDS_Shape> result;
    for (const auto& shape : list) {
        if (!shape.IsNull()) result.push_back(shape);
    }
    return result;
}

// History of an OCCT algorithm with the BRepBuilderAPI_MakeShape interface.
template <typename Algorithm>
class AlgorithmHistory final : public History {
public:
    explicit AlgorithmHistory(Algorithm& algorithm) : algorithm_(algorithm) {}
    std::vector<TopoDS_Shape> modified(const TopoDS_Shape& source) override {
        std::vector<TopoDS_Shape> targets;
        for (const auto& target : algorithm_.Modified(source)) {
            if (!target.IsNull()) targets.push_back(target);
        }
        return targets;
    }
    bool deleted(const TopoDS_Shape& source) override { return algorithm_.IsDeleted(source); }

private:
    Algorithm& algorithm_;
};

// For algorithms without history that may copy edges (wire and face
// building): a source edge missing from the result maps to the one result
// edge on the same underlying curve.
class SharedCurveHistory final : public History {
public:
    explicit SharedCurveHistory(const TopoDS_Shape& result);
    std::vector<TopoDS_Shape> modified(const TopoDS_Shape& source) override;
    bool deleted(const TopoDS_Shape&) override { return false; }

private:
    ShapeIndex result_edges_;
};

// Names of `result`, carried from `inputs` through `history` (rules 1–7:
// unchanged and one-to-one modified elements keep their names; split pieces
// get `parent{faces bounding only this piece}`; merged elements keep the
// smallest name; generated elements get `role(sources)`; collisions between
// inputs get `@slot`; leftovers get weak index names).
ElementMapPtr propagate(const std::vector<NamedShape>& inputs, History* history, const TopoDS_Shape& result);

// `names` with the given faces named by role where they only had weak names
// (caps of lofts, which come from no single source element).
ElementMapPtr with_face_roles(const TopoDS_Shape& shape, const ElementMap& names,
                              const std::vector<std::pair<TopoDS_Shape, std::string>>& roles);

// The names of `sub`, a subshape of `parent`; its edges and vertices keep
// the parent's final names as tracked names.
ElementMapPtr restrict_to(const TopoDS_Shape& parent, const ElementMap& names, const TopoDS_Shape& sub);

// `names` with the subshapes of `kind` given the atoms in `seeds` (one per
// subshape; an empty entry keeps the current name).
ElementMapPtr seed(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind,
                   const std::vector<std::string>& seeds);

// `names` with `tag:` prefixed to every face name and tracked edge or vertex
// name that no input has (TN-D5).
ElementMapPtr stamp(const TopoDS_Shape& shape, const ElementMap& names, const std::string& tag,
                    const std::vector<NamedShape>& inputs);

// The final names of `kind`, newline-separated.
std::string joined_names(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind);

}  // namespace cadkit_naming
