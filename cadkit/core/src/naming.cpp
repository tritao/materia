#include "naming.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace cadkit_naming {
namespace {

using Adjacency = NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>;

TopAbs_ShapeEnum occt_kind(ElementKind kind) {
    switch (kind) {
    case ElementKind::Face:
        return TopAbs_FACE;
    case ElementKind::Edge:
        return TopAbs_EDGE;
    case ElementKind::Vertex:
        return TopAbs_VERTEX;
    }
    return TopAbs_SHAPE;
}

ShapeIndex index_of(const TopoDS_Shape& shape, ElementKind kind) {
    ShapeIndex index;
    TopExp::MapShapes(shape, occt_kind(kind), index);
    return index;
}

std::string join(std::vector<std::string> parts, const char* separator, bool sorted) {
    if (sorted) std::sort(parts.begin(), parts.end());
    std::string result;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i) result += separator;
        result += parts[i];
    }
    return result;
}

std::shared_ptr<ElementMap> copy_of(const ElementMap& names) {
    auto result = std::make_shared<ElementMap>();
    result->faces = names.faces;
    result->edges = names.edges;
    result->vertices = names.vertices;
    return result;
}

std::shared_ptr<ElementMap> sized_for(const TopoDS_Shape& shape) {
    auto result = std::make_shared<ElementMap>();
    result->faces.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Face).Extent()), std::string());
    result->edges.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Edge).Extent()), std::string());
    result->vertices.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Vertex).Extent()), std::string());
    return result;
}

void fill_weak_faces(ElementMap& names) {
    for (std::size_t i = 0; i < names.faces.size(); ++i) {
        if (names.faces[i].empty()) names.faces[i] = "face#" + std::to_string(i);
    }
}

// Lexicographic order of points, for the weak ordinals of otherwise identical names.
bool point_before(const gp_Pnt& a, const gp_Pnt& b) {
    constexpr double tolerance = 1e-9;
    if (std::abs(a.X() - b.X()) > tolerance) return a.X() < b.X();
    if (std::abs(a.Y() - b.Y()) > tolerance) return a.Y() < b.Y();
    return a.Z() < b.Z() - tolerance;
}

gp_Pnt edge_middle(const TopoDS_Edge& edge) {
    if (BRep_Tool::Degenerated(edge)) return BRep_Tool::Pnt(TopExp::FirstVertex(edge));
    BRepAdaptor_Curve curve(edge);
    return curve.Value((curve.FirstParameter() + curve.LastParameter()) / 2);
}

// Gives each group of equal names a weak ordinal `~k` in the order of `position`.
template <typename Position>
void number_duplicates(std::vector<std::string>& names, Position position) {
    std::map<std::string, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < names.size(); ++i) groups[names[i]].push_back(i);
    for (auto& group : groups) {
        auto& members = group.second;
        if (members.size() < 2) continue;
        std::vector<std::pair<gp_Pnt, std::size_t>> ordered;
        for (auto member : members) ordered.emplace_back(position(member), member);
        std::stable_sort(ordered.begin(), ordered.end(),
                         [](const auto& a, const auto& b) { return point_before(a.first, b.first); });
        for (std::size_t k = 0; k < ordered.size(); ++k) names[ordered[k].second] += "~" + std::to_string(k);
    }
}

std::vector<std::string> adjacent_names(const Adjacency& adjacency, const TopoDS_Shape& element,
                                        const ShapeIndex& index, const std::vector<std::string>& names) {
    std::vector<std::string> result;
    if (!adjacency.Contains(element)) return result;
    for (const auto& neighbour : adjacency.FindFromKey(element)) {
        const int position = index.FindIndex(neighbour);
        if (position > 0 && static_cast<std::size_t>(position) <= names.size()) {
            result.push_back(names[static_cast<std::size_t>(position - 1)]);
        }
    }
    return result;
}

const std::string& entry(const std::vector<std::string>& names, int position) {
    static const std::string empty;
    return position > 0 && static_cast<std::size_t>(position) <= names.size()
        ? names[static_cast<std::size_t>(position - 1)] : empty;
}

}  // namespace

std::string atom(const std::string& text) {
    if (text.empty()) throw std::invalid_argument("a name atom must not be empty");
    static const char* hex = "0123456789ABCDEF";
    std::string result;
    for (unsigned char c : text) {
        const bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
            c == '_' || c == '.' || c == '+' || c == '-';
        if (plain) {
            result += static_cast<char>(c);
        } else {
            result += '%';
            result += hex[c >> 4];
            result += hex[c & 15];
        }
    }
    return result;
}

bool is_weak(const std::string& name) {
    return name.find_first_of("#~@") != std::string::npos;
}

const std::vector<std::string>& ElementMap::names(const TopoDS_Shape& shape, ElementKind kind) const {
    if (kind == ElementKind::Face) return faces;
    std::call_once(derived_once_, [&]() { derive(shape); });
    return kind == ElementKind::Edge ? derived_edges_ : derived_vertices_;
}

void ElementMap::derive(const TopoDS_Shape& shape) const {
    const auto faceIndex = index_of(shape, ElementKind::Face);
    const auto edgeIndex = index_of(shape, ElementKind::Edge);
    const auto vertexIndex = index_of(shape, ElementKind::Vertex);
    Adjacency edgeFaces, vertexFaces, vertexEdges;
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, vertexFaces);
    TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertexEdges);

    // Edges from their faces (TN-D4); tracked names only where faces cannot tell.
    std::vector<std::string> edgeNames(static_cast<std::size_t>(edgeIndex.Extent()));
    for (int i = 1; i <= edgeIndex.Extent(); ++i) {
        const auto& edge = TopoDS::Edge(edgeIndex(i));
        auto adjacent = adjacent_names(edgeFaces, edge, faceIndex, faces);
        std::sort(adjacent.begin(), adjacent.end());
        const auto& tracked = entry(edges, i);
        std::string name;
        bool seam = false;
        if (adjacent.size() == 1 && edgeFaces.Contains(edge)) {
            seam = BRep_Tool::IsClosed(edge, TopoDS::Face(edgeFaces.FindFromKey(edge).First()));
        }
        if (BRep_Tool::Degenerated(edge) && !adjacent.empty()) {
            name = "E(" + adjacent.front() + "|degenerate)";
        } else if (seam) {
            name = "E(" + adjacent.front() + "|seam)";
        } else if (adjacent.size() >= 2) {
            name = "E(" + join(adjacent, "|", false) + ")";
        } else if (!tracked.empty()) {
            name = tracked;
        } else if (adjacent.size() == 1) {
            name = "E(" + adjacent.front() + "|edge#" + std::to_string(i - 1) + ")";
        } else {
            name = "edge#" + std::to_string(i - 1);
        }
        edgeNames[static_cast<std::size_t>(i - 1)] = name;
    }

    // Vertices from their faces; below three faces their edges join in.
    std::vector<std::string> vertexNames(static_cast<std::size_t>(vertexIndex.Extent()));
    for (int i = 1; i <= vertexIndex.Extent(); ++i) {
        const auto& vertex = vertexIndex(i);
        auto adjacent = adjacent_names(vertexFaces, vertex, faceIndex, faces);
        const auto& tracked = entry(vertices, i);
        std::string name;
        if (!tracked.empty() && adjacent.size() < 3) {
            name = tracked;
        } else if (adjacent.size() >= 3) {
            name = "V(" + join(adjacent, "|", true) + ")";
        } else {
            auto parts = adjacent;
            auto edgeParts = adjacent_names(vertexEdges, vertex, edgeIndex, edgeNames);
            parts.insert(parts.end(), edgeParts.begin(), edgeParts.end());
            name = parts.empty() ? "vertex#" + std::to_string(i - 1) : "V(" + join(parts, "|", true) + ")";
        }
        vertexNames[static_cast<std::size_t>(i - 1)] = name;
    }

    // Edges between the same faces: told apart by their vertices, then by a weak ordinal.
    std::map<std::string, int> counts;
    for (const auto& name : edgeNames) counts[name]++;
    for (int i = 1; i <= edgeIndex.Extent(); ++i) {
        auto& name = edgeNames[static_cast<std::size_t>(i - 1)];
        if (counts[name] < 2) continue;
        TopoDS_Vertex first, last;
        TopExp::Vertices(TopoDS::Edge(edgeIndex(i)), first, last);
        std::vector<std::string> ends;
        if (!first.IsNull()) ends.push_back(entry(vertexNames, vertexIndex.FindIndex(first)));
        if (!last.IsNull()) ends.push_back(entry(vertexNames, vertexIndex.FindIndex(last)));
        name += "{" + join(ends, ",", true) + "}";
    }
    number_duplicates(edgeNames, [&](std::size_t i) {
        return edge_middle(TopoDS::Edge(edgeIndex(static_cast<int>(i) + 1)));
    });
    number_duplicates(vertexNames, [&](std::size_t i) {
        return BRep_Tool::Pnt(TopoDS::Vertex(vertexIndex(static_cast<int>(i) + 1)));
    });
    derived_edges_ = std::move(edgeNames);
    derived_vertices_ = std::move(vertexNames);
}

std::shared_ptr<ElementMap> editable_copy(const ElementMap& names) {
    return copy_of(names);
}

ElementMapPtr index_names(const TopoDS_Shape& shape) {
    auto result = sized_for(shape);
    fill_weak_faces(*result);
    return result;
}

ElementMapPtr role_names(const TopoDS_Shape& shape,
                         const std::vector<std::pair<TopoDS_Shape, std::string>>& roles) {
    auto result = sized_for(shape);
    const auto faceIndex = index_of(shape, ElementKind::Face);
    for (const auto& role : roles) {
        const int position = faceIndex.FindIndex(role.first);
        if (position > 0) result->faces[static_cast<std::size_t>(position - 1)] = role.second;
    }
    fill_weak_faces(*result);
    return result;
}

ElementMapPtr box_names(const TopoDS_Shape& box) {
    auto result = sized_for(box);
    const auto faceIndex = index_of(box, ElementKind::Face);
    for (int i = 1; i <= faceIndex.Extent(); ++i) {
        const auto& face = TopoDS::Face(faceIndex(i));
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane) continue;
        auto normal = surface.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
        const double components[3] = {normal.X(), normal.Y(), normal.Z()};
        static const char* axes = "xyz";
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(std::abs(components[axis]) - 1.0) < 1e-9) {
                result->faces[static_cast<std::size_t>(i - 1)] =
                    std::string("box.") + (components[axis] > 0 ? "+" : "-") + axes[axis];
            }
        }
    }
    fill_weak_faces(*result);
    return result;
}

SharedCurveHistory::SharedCurveHistory(const TopoDS_Shape& result) {
    TopExp::MapShapes(result, TopAbs_EDGE, result_edges_);
}

std::vector<TopoDS_Shape> SharedCurveHistory::modified(const TopoDS_Shape& source) {
    std::vector<TopoDS_Shape> targets;
    if (source.ShapeType() != TopAbs_EDGE || result_edges_.Contains(source)) return targets;
    double first = 0, last = 0;
    const auto curve = BRep_Tool::Curve(TopoDS::Edge(source), first, last);
    if (curve.IsNull()) return targets;
    for (int i = 1; i <= result_edges_.Extent(); ++i) {
        double f = 0, l = 0;
        if (BRep_Tool::Curve(TopoDS::Edge(result_edges_(i)), f, l) == curve) targets.push_back(result_edges_(i));
    }
    if (targets.size() > 1) targets.clear();
    return targets;
}

ElementMapPtr propagate(const std::vector<NamedShape>& inputs, History* history, const TopoDS_Shape& result) {
    auto output = sized_for(result);
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex}) {
        const auto resultIndex = index_of(result, kind);
        // (slot, name) candidates per result element.
        std::vector<std::vector<std::pair<std::size_t, std::string>>> candidates(
            static_cast<std::size_t>(resultIndex.Extent()));
        for (std::size_t slot = 0; slot < inputs.size(); ++slot) {
            const auto& input = inputs[slot];
            const auto sourceIndex = index_of(input.shape, kind);
            const auto& names = input.names->names(input.shape, kind);
            for (int i = 1; i <= sourceIndex.Extent(); ++i) {
                const auto& source = sourceIndex(i);
                const auto& name = names[static_cast<std::size_t>(i - 1)];
                std::vector<TopoDS_Shape> targets;
                if (history != nullptr) targets = history->modified(source);
                if (targets.empty()) {
                    if (history != nullptr && history->deleted(source)) continue;
                    targets.push_back(source);  // unchanged (rule 1)
                } else if (targets.size() > 1) {
                    continue;  // splits are TN2's (rule 3); the pieces fall to rule 7 meanwhile
                }
                const int position = resultIndex.FindIndex(targets.front());
                if (position > 0) candidates[static_cast<std::size_t>(position - 1)].emplace_back(slot, name);  // rule 2
            }
        }
        std::vector<std::string> assigned(candidates.size());
        std::vector<std::size_t> slots(candidates.size(), 0);
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (candidates[i].empty()) continue;
            // Several sources: the smallest name stands for them (aliases are TN2's).
            auto best = std::min_element(candidates[i].begin(), candidates[i].end(),
                                         [](const auto& a, const auto& b) { return a.second < b.second; });
            assigned[i] = best->second;
            slots[i] = best->first;
        }
        // Rule 6: one name brought by several inputs.
        std::map<std::string, std::set<std::size_t>> origins;
        for (std::size_t i = 0; i < assigned.size(); ++i) {
            if (!assigned[i].empty()) origins[assigned[i]].insert(slots[i]);
        }
        for (std::size_t i = 0; i < assigned.size(); ++i) {
            if (!assigned[i].empty() && origins[assigned[i]].size() > 1) assigned[i] += "@" + std::to_string(slots[i]);
        }
        auto& target = kind == ElementKind::Face ? output->faces : kind == ElementKind::Edge ? output->edges : output->vertices;
        target = std::move(assigned);
    }
    fill_weak_faces(*output);  // rule 7
    return output;
}

ElementMapPtr restrict_to(const TopoDS_Shape& parent, const ElementMap& names, const TopoDS_Shape& sub) {
    auto result = sized_for(sub);
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex}) {
        const auto parentIndex = index_of(parent, kind);
        const auto subIndex = index_of(sub, kind);
        const auto& parentNames = names.names(parent, kind);
        auto& target = kind == ElementKind::Face ? result->faces : kind == ElementKind::Edge ? result->edges : result->vertices;
        for (int i = 1; i <= subIndex.Extent(); ++i) {
            target[static_cast<std::size_t>(i - 1)] = entry(parentNames, parentIndex.FindIndex(subIndex(i)));
        }
    }
    fill_weak_faces(*result);
    return result;
}

ElementMapPtr seed(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind,
                   const std::vector<std::string>& seeds) {
    auto result = copy_of(names);
    auto& target = kind == ElementKind::Face ? result->faces : kind == ElementKind::Edge ? result->edges : result->vertices;
    if (seeds.size() != static_cast<std::size_t>(index_of(shape, kind).Extent())) {
        throw std::invalid_argument("seed names must give one entry per subshape");
    }
    target.resize(seeds.size());
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        if (!seeds[i].empty()) target[i] = atom(seeds[i]);
    }
    return result;
}

ElementMapPtr stamp(const TopoDS_Shape& shape, const ElementMap& names, const std::string& tag,
                    const std::vector<NamedShape>& inputs) {
    auto result = copy_of(names);
    const auto prefix = atom(tag) + ":";
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex}) {
        std::set<std::string> known;
        for (const auto& input : inputs) {
            const auto& inputNames = input.names->names(input.shape, kind);
            known.insert(inputNames.begin(), inputNames.end());
        }
        auto& target = kind == ElementKind::Face ? result->faces : kind == ElementKind::Edge ? result->edges : result->vertices;
        for (auto& name : target) {
            if (!name.empty() && known.count(name) == 0) name = prefix + name;
        }
    }
    (void)shape;
    return result;
}

std::string joined_names(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind) {
    return join(names.names(shape, kind), "\n", false);
}

}  // namespace cadkit_naming
