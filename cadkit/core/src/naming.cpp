#include "naming.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>
#include <map>
#include <set>
#include <cstdlib>
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
    case ElementKind::Solid:
        return TopAbs_SOLID;
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
    result->solids = names.solids;
    result->face_aliases = names.face_aliases;
    result->solid_aliases = names.solid_aliases;
    return result;
}

std::shared_ptr<ElementMap> sized_for(const TopoDS_Shape& shape) {
    auto result = std::make_shared<ElementMap>();
    result->faces.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Face).Extent()), std::string());
    result->edges.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Edge).Extent()), std::string());
    result->vertices.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Vertex).Extent()), std::string());
    result->solids.assign(static_cast<std::size_t>(index_of(shape, ElementKind::Solid).Extent()), std::string());
    result->face_aliases.assign(result->faces.size(), {});
    result->solid_aliases.assign(result->solids.size(), {});
    return result;
}

void fill_weak_faces(ElementMap& names) {
    for (std::size_t i = 0; i < names.faces.size(); ++i) {
        if (names.faces[i].empty()) names.faces[i] = "face#" + std::to_string(i);
    }
    // The one body of a shape is `solid`; several unnamed ones are told apart only by index.
    for (std::size_t i = 0; i < names.solids.size(); ++i) {
        if (names.solids[i].empty()) names.solids[i] = names.solids.size() == 1 ? "solid" : "solid#" + std::to_string(i);
    }
    names.face_aliases.resize(names.faces.size());
    names.solid_aliases.resize(names.solids.size());
}

// Lexicographic order of points, for the weak ordinals of otherwise identical names.
bool point_before(const gp_Pnt& a, const gp_Pnt& b) {
    constexpr double tolerance = 1e-9;
    if (std::abs(a.X() - b.X()) > tolerance) return a.X() < b.X();
    if (std::abs(a.Y() - b.Y()) > tolerance) return a.Y() < b.Y();
    return a.Z() < b.Z() - tolerance;
}

gp_Pnt centre_of(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    if (shape.ShapeType() == TopAbs_SOLID)
        BRepGProp::VolumeProperties(shape, properties);
    else
        BRepGProp::SurfaceProperties(shape, properties);
    return properties.CentreOfMass();
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
    if (kind == ElementKind::Solid) return solids;
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

std::vector<std::string>& ElementMap::tracked(ElementKind kind) {
    switch (kind) {
    case ElementKind::Face:
        return faces;
    case ElementKind::Edge:
        return edges;
    case ElementKind::Vertex:
        return vertices;
    case ElementKind::Solid:
        return solids;
    }
    return faces;
}

const std::vector<std::vector<std::string>>& ElementMap::aliases(ElementKind kind) const {
    static const std::vector<std::vector<std::string>> none;
    return kind == ElementKind::Face ? face_aliases : kind == ElementKind::Solid ? solid_aliases : none;
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
    // Generated elements may differ in kind from their sources (an edge sweeps a face): collect them first.
    struct Birth {
        TopoDS_Shape target;
        std::string role;
        std::string source;
    };
    std::vector<Birth> births;
    if (history != nullptr) {
        for (std::size_t slot = 0; slot < inputs.size(); ++slot) {
            const auto& input = inputs[slot];
            for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex, ElementKind::Solid}) {
                const auto sourceIndex = index_of(input.shape, kind);
                const auto& names = input.names->names(input.shape, kind);
                for (int i = 1; i <= sourceIndex.Extent(); ++i) {
                    for (const auto& target : history->generated(sourceIndex(i), slot))
                        births.push_back({target.first, target.second, names[static_cast<std::size_t>(i - 1)]});
                }
            }
        }
    }
    // Where each input face went (result face positions), for solids that history does not name.
    std::vector<std::vector<std::vector<std::size_t>>> faceTargets(inputs.size());
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex, ElementKind::Solid}) {
        const auto resultIndex = index_of(result, kind);
        const auto count = static_cast<std::size_t>(resultIndex.Extent());
        // (slot, name) of the sources each result element continues (rules 1, 2, 4).
        std::vector<std::vector<std::pair<std::size_t, std::string>>> carried(count);
        // The aliases the carried sources bring along (rule 4).
        std::vector<std::set<std::string>> carriedAliases(count);
        // role -> source names, for generated elements (rule 5).
        std::vector<std::map<std::string, std::set<std::string>>> generated(count);
        // Split pieces: (parent name, slot, piece positions) (rule 3).
        struct Split {
            std::string parent;
            std::size_t slot;
            std::vector<std::size_t> pieces;
        };
        std::vector<Split> splits;
        for (std::size_t slot = 0; slot < inputs.size(); ++slot) {
            const auto& input = inputs[slot];
            const auto sourceIndex = index_of(input.shape, kind);
            const auto& names = input.names->names(input.shape, kind);
            const auto& sourceAliases = input.names->aliases(kind);
            for (int i = 1; i <= sourceIndex.Extent(); ++i) {
                const auto& source = sourceIndex(i);
                const auto& name = names[static_cast<std::size_t>(i - 1)];
                std::vector<TopoDS_Shape> targets;
                if (history != nullptr) targets = history->modified(source);
                if (targets.empty()) {
                    if (history != nullptr && history->deleted(source)) continue;
                    targets.push_back(source);  // unchanged (rule 1)
                }
                std::vector<std::size_t> found;
                for (const auto& target : targets) {
                    const int position = resultIndex.FindIndex(target);
                    if (position > 0) found.push_back(static_cast<std::size_t>(position - 1));
                }
                std::sort(found.begin(), found.end());
                found.erase(std::unique(found.begin(), found.end()), found.end());
                if (kind == ElementKind::Face) {
                    faceTargets[slot].resize(static_cast<std::size_t>(sourceIndex.Extent()));
                    faceTargets[slot][static_cast<std::size_t>(i - 1)] = found;
                }
                if (found.size() == 1) {
                    carried[found.front()].emplace_back(slot, name);  // rule 2
                    if (static_cast<std::size_t>(i - 1) < sourceAliases.size())
                        for (const auto& alias : sourceAliases[static_cast<std::size_t>(i - 1)]) carriedAliases[found.front()].insert(alias);
                } else if (found.size() > 1) {
                    splits.push_back({name, slot, found});
                }
            }
        }
        for (const auto& birth : births) {
            if (birth.target.ShapeType() != occt_kind(kind)) continue;
            const int position = resultIndex.FindIndex(birth.target);
            if (position > 0) generated[static_cast<std::size_t>(position - 1)][birth.role].insert(birth.source);
        }
        if (kind == ElementKind::Solid) {
            // A solid history does not name (a fuse builds a new one) is named after the input solids whose faces
            // it is bounded by: identity follows the boundary.
            Adjacency resultFaceSolids;
            TopExp::MapShapesAndUniqueAncestors(result, TopAbs_FACE, TopAbs_SOLID, resultFaceSolids);
            const auto resultFaces = index_of(result, ElementKind::Face);
            for (std::size_t slot = 0; slot < inputs.size(); ++slot) {
                const auto& input = inputs[slot];
                const auto inputFaces = index_of(input.shape, ElementKind::Face);
                const auto inputSolids = index_of(input.shape, ElementKind::Solid);
                const auto& solidNames = input.names->names(input.shape, ElementKind::Solid);
                Adjacency inputFaceSolids;
                TopExp::MapShapesAndUniqueAncestors(input.shape, TopAbs_FACE, TopAbs_SOLID, inputFaceSolids);
                for (std::size_t face = 0; face < faceTargets[slot].size(); ++face) {
                    const auto& source = inputFaces(static_cast<int>(face) + 1);
                    if (!inputFaceSolids.Contains(source)) continue;
                    for (const auto& owner : inputFaceSolids.FindFromKey(source)) {
                        const auto& name = entry(solidNames, inputSolids.FindIndex(owner));
                        if (name.empty()) continue;
                        for (auto target : faceTargets[slot][face]) {
                            const auto& targetFace = resultFaces(static_cast<int>(target) + 1);
                            if (!resultFaceSolids.Contains(targetFace)) continue;
                            for (const auto& body : resultFaceSolids.FindFromKey(targetFace)) {
                                const int position = resultIndex.FindIndex(body);
                                if (position > 0 && carried[static_cast<std::size_t>(position - 1)].empty())
                                    generated[static_cast<std::size_t>(position - 1)]["\x01bounded"].insert(name);
                            }
                        }
                    }
                }
            }
            for (std::size_t i = 0; i < count; ++i) {
                const auto bounded = generated[i].find("\x01bounded");
                if (bounded == generated[i].end()) continue;
                for (const auto& name : bounded->second) carried[i].emplace_back(0, name);
                generated[i].erase(bounded);
            }
        }
        std::vector<std::string> assigned(count);
        std::vector<std::size_t> slots(count, 0);
        std::vector<std::vector<std::string>> aliases(count);
        for (std::size_t i = 0; i < count; ++i) {
            if (!carried[i].empty()) {
                // Several sources merged: the smallest name stands for them, the others become aliases (rule 4).
                auto best = std::min_element(carried[i].begin(), carried[i].end(),
                                             [](const auto& a, const auto& b) { return a.second < b.second; });
                assigned[i] = best->second;
                slots[i] = best->first;
                auto others = carriedAliases[i];
                for (const auto& source : carried[i]) others.insert(source.second);
                others.erase(assigned[i]);
                aliases[i].assign(others.begin(), others.end());
            } else if (!generated[i].empty()) {
                const auto& role = *generated[i].begin();  // the smallest role
                assigned[i] = role.first + "(" + join(std::vector<std::string>(role.second.begin(), role.second.end()), ",", true) + ")";
            }
        }
        // Rule 3: pieces of a split face are told apart by the faces that bound only them.
        if (!splits.empty()) {
            Adjacency edgeFaces;
            ShapeIndex faceIndex;
            if (kind == ElementKind::Face) {
                TopExp::MapShapesAndUniqueAncestors(result, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
                faceIndex = resultIndex;
            }
            auto neighbours = [&](std::size_t piece) {
                std::set<std::string> result;
                for (int e = 1; e <= edgeFaces.Extent(); ++e) {
                    const auto& faces = edgeFaces(e);
                    bool touches = false;
                    for (const auto& face : faces) touches = touches || faceIndex.FindIndex(face) == static_cast<int>(piece + 1);
                    if (!touches) continue;
                    for (const auto& face : faces) {
                        const int other = faceIndex.FindIndex(face);
                        if (other > 0 && other != static_cast<int>(piece + 1) && !assigned[static_cast<std::size_t>(other - 1)].empty())
                            result.insert(assigned[static_cast<std::size_t>(other - 1)]);
                    }
                }
                return result;
            };
            std::vector<std::string> named(count);
            for (const auto& split : splits) {
                std::vector<std::set<std::string>> around;
                for (auto piece : split.pieces) around.push_back(kind == ElementKind::Face ? neighbours(piece) : std::set<std::string>{});
                std::set<std::string> common = around.front();
                for (const auto& set : around) {
                    std::set<std::string> kept;
                    std::set_intersection(common.begin(), common.end(), set.begin(), set.end(), std::inserter(kept, kept.begin()));
                    common = kept;
                }
                for (std::size_t k = 0; k < split.pieces.size(); ++k) {
                    std::vector<std::string> own;
                    std::set_difference(around[k].begin(), around[k].end(), common.begin(), common.end(), std::back_inserter(own));
                    const auto piece = split.pieces[k];
                    if (!assigned[piece].empty() || !named[piece].empty()) continue;  // a carried name wins
                    named[piece] = split.parent + "{" + join(own, ",", true) + "}";
                    slots[piece] = split.slot;
                }
            }
            for (std::size_t i = 0; i < count; ++i) {
                if (!named[i].empty()) assigned[i] = named[i];
            }
        }
        // Rule 6: one name brought by several inputs.
        std::map<std::string, std::set<std::size_t>> origins;
        for (std::size_t i = 0; i < count; ++i) {
            if (!assigned[i].empty()) origins[assigned[i]].insert(slots[i]);
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (!assigned[i].empty() && origins[assigned[i]].size() > 1) assigned[i] += "@" + std::to_string(slots[i]);
        }
        // Pieces still alike (symmetric splits): a weak ordinal by position.
        if (kind == ElementKind::Face || kind == ElementKind::Solid) {
            std::map<std::string, int> seen;
            for (const auto& name : assigned) {
                if (!name.empty()) seen[name]++;
            }
            std::vector<std::string> duplicates;
            for (std::size_t i = 0; i < count; ++i) duplicates.push_back(!assigned[i].empty() && seen[assigned[i]] > 1 ? assigned[i] : "");
            bool any = false;
            for (const auto& name : duplicates) any = any || !name.empty();
            if (any) {
                std::vector<std::string> numbered = assigned;
                number_duplicates(numbered, [&](std::size_t i) { return centre_of(resultIndex(static_cast<int>(i) + 1)); });
                for (std::size_t i = 0; i < count; ++i) {
                    if (!duplicates[i].empty()) assigned[i] = numbered[i];
                }
            }
        }
        output->tracked(kind) = std::move(assigned);
        if (kind == ElementKind::Face)
            output->face_aliases = std::move(aliases);
        else if (kind == ElementKind::Solid)
            output->solid_aliases = std::move(aliases);
    }
    fill_weak_faces(*output);  // rule 7
    return output;
}

ElementMapPtr with_face_roles(const TopoDS_Shape& shape, const ElementMap& names,
                              const std::vector<std::pair<TopoDS_Shape, std::string>>& roles) {
    auto result = copy_of(names);
    const auto faceIndex = index_of(shape, ElementKind::Face);
    for (const auto& role : roles) {
        const int position = role.first.IsNull() ? 0 : faceIndex.FindIndex(role.first);
        if (position > 0 && is_weak(result->faces[static_cast<std::size_t>(position - 1)]))
            result->faces[static_cast<std::size_t>(position - 1)] = role.second;
    }
    return result;
}

ElementMapPtr restrict_to(const TopoDS_Shape& parent, const ElementMap& names, const TopoDS_Shape& sub) {
    auto result = sized_for(sub);
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex, ElementKind::Solid}) {
        const auto parentIndex = index_of(parent, kind);
        const auto subIndex = index_of(sub, kind);
        const auto& parentNames = names.names(parent, kind);
        const auto& parentAliases = names.aliases(kind);
        auto& target = result->tracked(kind);
        for (int i = 1; i <= subIndex.Extent(); ++i) {
            const int position = parentIndex.FindIndex(subIndex(i));
            target[static_cast<std::size_t>(i - 1)] = entry(parentNames, position);
            if (position > 0 && static_cast<std::size_t>(position) <= parentAliases.size()) {
                auto& aliases = kind == ElementKind::Face ? result->face_aliases : result->solid_aliases;
                aliases.resize(target.size());
                aliases[static_cast<std::size_t>(i - 1)] = parentAliases[static_cast<std::size_t>(position - 1)];
            }
        }
    }
    fill_weak_faces(*result);
    return result;
}

ElementMapPtr seed(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind,
                   const std::vector<std::string>& seeds) {
    auto result = copy_of(names);
    auto& target = result->tracked(kind);
    if (seeds.size() != static_cast<std::size_t>(index_of(shape, kind).Extent())) {
        throw std::invalid_argument("seed names must give one entry per subshape");
    }
    target.resize(seeds.size());
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        if (!seeds[i].empty()) target[i] = atom(seeds[i]);
    }
    return result;
}

namespace {
std::string relative_form(const std::string& name, std::set<std::string>& items);
}  // namespace

ElementMapPtr stamp(const TopoDS_Shape& shape, const ElementMap& names, const std::string& tag,
                    const std::vector<NamedShape>& inputs) {
    auto result = copy_of(names);
    const auto prefix = atom(tag) + ":";
    for (auto kind : {ElementKind::Face, ElementKind::Edge, ElementKind::Vertex, ElementKind::Solid}) {
        // A split piece, an ordinal or a slot only divides an input's element: it keeps that identity untagged.
        std::set<std::string> known;
        for (const auto& input : inputs) {
            for (const auto& name : input.names->names(input.shape, kind)) {
                std::set<std::string> items;
                known.insert(relative_form(name, items));
            }
            for (const auto& aliases : input.names->aliases(kind))
                for (const auto& alias : aliases) {
                    std::set<std::string> items;
                    known.insert(relative_form(alias, items));
                }
        }
        auto stampIfNew = [&](std::string& name) {
            std::set<std::string> items;
            if (!name.empty() && known.count(relative_form(name, items)) == 0) name = prefix + name;
        };
        for (auto& name : result->tracked(kind)) stampIfNew(name);
        if (kind == ElementKind::Face || kind == ElementKind::Solid)
            for (auto& aliases : kind == ElementKind::Face ? result->face_aliases : result->solid_aliases)
                for (auto& alias : aliases) stampIfNew(alias);
    }
    (void)shape;
    return result;
}

namespace {

// `name` without split suffixes, ordinals and input slots; the suffixes' items go to `items`.
std::string relative_form(const std::string& name, std::set<std::string>& items) {
    std::string result;
    for (std::size_t i = 0; i < name.size(); ++i) {
        const char c = name[i];
        if (c == '{') {
            int depth = 1;
            std::string item;
            std::size_t j = i + 1;
            for (; j < name.size() && depth > 0; ++j) {
                const char d = name[j];
                if (d == '{' || d == '(') depth++;
                if (d == '}' || d == ')') depth--;
                if (depth == 0) break;
                if (depth == 1 && d == ',') {
                    items.insert(item);
                    item.clear();
                } else {
                    item += d;
                }
            }
            if (!item.empty()) items.insert(item);
            i = j;
        } else if ((c == '~' || c == '@') && i + 1 < name.size() && std::isdigit(static_cast<unsigned char>(name[i + 1]))) {
            while (i + 1 < name.size() && std::isdigit(static_cast<unsigned char>(name[i + 1]))) ++i;
        } else {
            result += c;
        }
    }
    return result;
}

}  // namespace

NameMatch match_name(const std::string& reference, const std::string& candidate) {
    if (reference == candidate) return {is_weak(reference) ? MatchGrade::Weak : MatchGrade::Exact, 1.0};
    std::set<std::string> referenceItems, candidateItems;
    if (relative_form(reference, referenceItems) != relative_form(candidate, candidateItems)) return {MatchGrade::None, 0.0};
    std::set<std::string> both, either;
    std::set_intersection(referenceItems.begin(), referenceItems.end(), candidateItems.begin(), candidateItems.end(),
                          std::inserter(both, both.begin()));
    std::set_union(referenceItems.begin(), referenceItems.end(), candidateItems.begin(), candidateItems.end(),
                   std::inserter(either, either.begin()));
    const double overlap = either.empty() ? 1.0 : static_cast<double>(both.size()) / static_cast<double>(either.size());
    return {MatchGrade::Relative, overlap};
}

std::string creator_tag(const std::string& name) {
    for (std::size_t i = 0; i < name.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(name[i]);
        if (c == ':') return name.substr(0, i);
        const bool atom = std::isalnum(c) || c == '_' || c == '.' || c == '+' || c == '-' || c == '%';
        if (!atom) return "";
    }
    return "";
}

namespace {

bool atom_char(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '.' || c == '+' || c == '-' || c == '%' || c == '#';
}

std::string unescape(const std::string& text) {
    std::string result;
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 2 < text.size()) {
            const auto hex = text.substr(i + 1, 2);
            char* end = nullptr;
            const long value = std::strtol(hex.c_str(), &end, 16);
            if (end == hex.c_str() + 2) {
                result += static_cast<char>(value);
                i += 2;
                continue;
            }
        }
        result += text[i];
    }
    return result;
}

std::string counted(const std::string& word, const std::string& index) {
    char* end = nullptr;
    const long value = std::strtol(index.c_str(), &end, 10);
    return word + " " + (end != index.c_str() && *end == 0 ? std::to_string(value + 1) : index);
}

std::string role_words(const std::string& atom) {
    static const std::map<std::string, std::string> roles = {
        {"box.+z", "top"}, {"box.-z", "bottom"}, {"box.+x", "right"}, {"box.-x", "left"}, {"box.+y", "back"},
        {"box.-y", "front"}, {"cyl.side", "side"}, {"cyl.top", "top"}, {"cyl.bottom", "bottom"}, {"sphere", "surface"},
        {"face", "face"}, {"solid", "body"}, {"circle", "circle"}, {"start", "start cap"}, {"end", "end cap"}};
    const auto known = roles.find(atom);
    if (known != roles.end()) return known->second;
    const auto hash = atom.find('#');
    if (hash != std::string::npos) return counted(atom.substr(0, hash), atom.substr(hash + 1));
    if (atom.rfind("e.", 0) == 0) return "edge " + unescape(atom.substr(2));
    if (atom.rfind("r.", 0) == 0) return "region";
    if (atom.rfind("seg.", 0) == 0) return counted("segment", atom.substr(4));
    if (atom.rfind("pt.", 0) == 0) return counted("point", atom.substr(3));
    return unescape(atom);
}

std::string tag_words(const std::string& tag) {
    if (tag == "m") return "mirror";
    if (tag.size() > 1 && tag[0] == 'i' && std::isdigit(static_cast<unsigned char>(tag[1]))) {
        std::string result = "copy ";
        std::string part;
        for (std::size_t i = 1; i <= tag.size(); ++i) {
            if (i == tag.size() || tag[i] == '.') {
                result += (result.size() > 5 ? "." : "") + std::to_string(std::stol(part) + 1);
                part.clear();
            } else {
                part += tag[i];
            }
        }
        return result;
    }
    return unescape(tag);
}

std::string joined_words(const std::vector<std::string>& parts) {
    std::string result;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i > 0) result += i + 1 == parts.size() ? " and " : ", ";
        result += parts[i];
    }
    return result;
}

// The longest "tag › " prefix every part shares, removed from them: "f3 › top" and "f3 › right" give "f3 › ".
std::string hoist(std::vector<std::string>& parts) {
    static const std::string arrow = " \u203A ";
    if (parts.empty()) return "";
    std::string common;
    for (;;) {
        const auto end = parts[0].find(arrow, common.size());
        if (end == std::string::npos) break;
        const auto candidate = parts[0].substr(0, end + arrow.size());
        bool shared = true;
        for (const auto& part : parts) shared = shared && part.compare(0, candidate.size(), candidate) == 0;
        if (!shared) break;
        common = candidate;
    }
    for (auto& part : parts) part = part.substr(common.size());
    return common;
}

// A recursive reader of the name grammar (plans/TOPOLOGICAL_NAMING.md), producing words.
struct LabelReader {
    const std::string& text;
    std::size_t at = 0;

    std::string atom() {
        const auto start = at;
        while (at < text.size() && atom_char(text[at])) ++at;
        return text.substr(start, at - start);
    }

    std::vector<std::string> list(char separator, char close) {
        std::vector<std::string> items;
        while (at < text.size() && text[at] != close) {
            items.push_back(name());
            if (at < text.size() && text[at] == separator) ++at;
        }
        if (at < text.size()) ++at;
        return items;
    }

    std::string body() {
        if (text.compare(at, 2, "E(") == 0 || text.compare(at, 2, "V(") == 0) {
            const bool edge = text[at] == 'E';
            at += 2;
            auto parts = list('|', ')');
            if (edge && parts.size() == 2 && parts[1] == "seam") return "seam of " + parts[0];
            if (edge && parts.size() == 2 && parts[1] == "degenerate") return "pole of " + parts[0];
            const auto shared = hoist(parts);
            return shared + (edge ? "edge between " : "vertex between ") + joined_words(parts);
        }
        const auto role = atom();
        if (at < text.size() && text[at] == '(') {
            ++at;
            const auto args = joined_words(list(',', ')'));
            if (role == "side") return "side from " + args;
            if (role == "lateral") return "edge from " + args;
            if (role == "start") return "start cap";
            if (role == "end") return "end cap";
            if (role == "fillet") return "fillet of " + args;
            if (role == "chamfer") return "chamfer of " + args;
            if (role == "corner") return "corner at " + args;
            if (role == "inner") return "inner " + args;
            if (role == "offset") return "offset of " + args;
            return unescape(role) + " of " + args;
        }
        return role_words(role);
    }

    std::string name() {
        std::string prefix;
        for (;;) {
            const auto start = at;
            const auto tag = atom();
            if (!tag.empty() && at < text.size() && text[at] == ':') {
                ++at;
                prefix += tag_words(tag) + " \u203A ";
                continue;
            }
            at = start;
            break;
        }
        auto words = body();
        while (at < text.size()) {
            if (text[at] == '{') {
                int depth = 0;
                do {
                    if (text[at] == '{' || text[at] == '(') depth++;
                    if (text[at] == '}' || text[at] == ')') depth--;
                    ++at;
                } while (at < text.size() && depth > 0);
                words += " (piece)";
            } else if ((text[at] == '~' || text[at] == '@') && at + 1 < text.size() &&
                       std::isdigit(static_cast<unsigned char>(text[at + 1]))) {
                const bool copy = text[at] == '@';
                ++at;
                std::string digits;
                while (at < text.size() && std::isdigit(static_cast<unsigned char>(text[at]))) digits += text[at++];
                words += copy ? " (copy " + std::to_string(std::stol(digits) + 1) + ")" : " (" + std::to_string(std::stol(digits) + 1) + ")";
            } else {
                break;
            }
        }
        return prefix + words;
    }
};

}  // namespace

std::string label(const std::string& name) {
    LabelReader reader{name};
    return reader.name();
}

std::string joined_names(const TopoDS_Shape& shape, const ElementMap& names, ElementKind kind) {
    return join(names.names(shape, kind), "\n", false);
}

}  // namespace cadkit_naming
