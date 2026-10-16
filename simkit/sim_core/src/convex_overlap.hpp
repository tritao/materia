#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <type_traits>

namespace nksim::geometry {

using Point = std::array<double, 3>;
enum class Overlap { separated, intersecting, unresolved };

inline double dot(const Point &a, const Point &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Point subtract(const Point &a, const Point &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

/** Outward support planes, derived once for a bounded hull of at most 64
 * vertices. Coplanar triangles share a plane; degenerate hulls have none. */
inline std::vector<std::array<double, 4>> hull_planes(const std::vector<double> &coordinates) {
    std::vector<Point> points;
    for (std::size_t i = 0; i < coordinates.size(); i += 3)
        points.push_back({coordinates[i], coordinates[i + 1], coordinates[i + 2]});
    std::vector<std::array<double, 4>> planes;
    for (std::size_t a = 0; a < points.size(); ++a)
        for (std::size_t b = a + 1; b < points.size(); ++b)
            for (std::size_t c = b + 1; c < points.size(); ++c) {
                const auto u = subtract(points[b], points[a]), v = subtract(points[c], points[a]);
                Point normal{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                             u[0] * v[1] - u[1] * v[0]};
                const double length = std::sqrt(dot(normal, normal));
                if (length <= 1e-14) continue;
                for (double &value : normal) value /= length;
                double offset = dot(normal, points[a]);
                bool positive = false, negative = false;
                for (const auto &point : points) {
                    const double side = dot(normal, point) - offset;
                    positive = positive || side > 1e-10;
                    negative = negative || side < -1e-10;
                }
                if (positive == negative) continue; // An interior plane or a flat hull.
                if (positive) {for (double &value : normal) value = -value; offset = -offset;}
                bool duplicate = false;
                for (const auto &plane : planes)
                    if (std::abs(plane[0] - normal[0]) < 1e-10 &&
                        std::abs(plane[1] - normal[1]) < 1e-10 &&
                        std::abs(plane[2] - normal[2]) < 1e-10 &&
                        std::abs(plane[3] - offset) < 1e-10) duplicate = true;
                if (!duplicate) planes.push_back({normal[0], normal[1], normal[2], offset});
            }
    return planes;
}

inline double ray_hull(const std::vector<std::array<double, 4>> &planes,
                        const Point &origin, const Point &direction, double limit) {
    double near = 0.0, far = limit;
    for (const auto &plane : planes) {
        const Point normal{plane[0], plane[1], plane[2]};
        const double distance = plane[3] - dot(normal, origin);
        const double approach = dot(normal, direction);
        if (std::abs(approach) < 1e-15) {
            if (distance < 0.0) return limit;
        } else {
            const double crossing = distance / approach;
            if (approach > 0.0) far = std::min(far, crossing);
            else near = std::max(near, crossing);
            if (near > far) return limit;
        }
    }
    return near;
}

// The closest point lies on one of the simplex's faces. Enumerating their
// affine projections (at most fifteen) handles degenerate edges and faces
// without choosing a special tetrahedron orientation or discarding a face.
struct Simplex {
    struct SupportPoint {
        Point difference{};
        Point first{};
        Point second{};
    };
    std::array<SupportPoint, 4> points{};
    unsigned count = 0;
};

struct ClosestWitness {
    Point difference{};
    Point first{};
    Point second{};
};

template<class Scalar = double>
inline ClosestWitness closest(Simplex &simplex) {
    using WorkingPoint = std::array<Scalar, 3>;
    auto product = [](const WorkingPoint &a, const WorkingPoint &b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    Scalar best = std::numeric_limits<Scalar>::infinity();
    ClosestWitness nearest{};
    Simplex retained;
    for (unsigned mask = 1; mask < (1u << simplex.count); ++mask) {
        std::array<unsigned, 4> indices{};
        unsigned count = 0;
        for (unsigned i = 0; i < simplex.count; ++i)
            if (mask & (1u << i)) indices[count++] = i;
        const unsigned dimensions = count - 1;
        WorkingPoint origin{};
        for (unsigned axis = 0; axis < 3; ++axis) origin[axis] = simplex.points[indices[0]].difference[axis];
        WorkingPoint edges[3]{};
        Scalar matrix[3][4]{};
        Scalar scale = 0.0;
        for (unsigned i = 0; i < dimensions; ++i)
            for (unsigned axis = 0; axis < 3; ++axis)
                edges[i][axis] = Scalar(simplex.points[indices[i + 1]].difference[axis]) - origin[axis];
        for (unsigned i = 0; i < dimensions; ++i) {
            for (unsigned j = 0; j < dimensions; ++j) {
                matrix[i][j] = product(edges[i], edges[j]);
                scale = std::max(scale, std::abs(matrix[i][j]));
            }
            matrix[i][dimensions] = -product(edges[i], origin);
        }
        constexpr Scalar pivot_tolerance = std::is_same_v<Scalar, double>
            ? Scalar(1e-14) : Scalar(64) * std::numeric_limits<Scalar>::epsilon();
        bool singular = false;
        for (unsigned column = 0; column < dimensions; ++column) {
            unsigned pivot = column;
            for (unsigned row = column + 1; row < dimensions; ++row)
                if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) pivot = row;
            if (std::abs(matrix[pivot][column]) <= scale * pivot_tolerance) {singular = true; break;}
            for (unsigned j = column; j <= dimensions; ++j)
                std::swap(matrix[column][j], matrix[pivot][j]);
            const Scalar divisor = matrix[column][column];
            for (unsigned j = column; j <= dimensions; ++j) matrix[column][j] /= divisor;
            for (unsigned row = 0; row < dimensions; ++row) if (row != column) {
                const Scalar factor = matrix[row][column];
                for (unsigned j = column; j <= dimensions; ++j)
                    matrix[row][j] -= factor * matrix[column][j];
            }
        }
        if (singular) continue; // Its lower-dimensional faces are also enumerated.
        Scalar weights[4] = {1.0, 0.0, 0.0, 0.0};
        for (unsigned i = 0; i < dimensions; ++i) {
            weights[i + 1] = matrix[i][dimensions];
            weights[0] -= weights[i + 1];
        }
        bool outside = false;
        Scalar total = 0.0;
        for (unsigned i = 0; i < count; ++i) {
            if (weights[i] < -1e-12) outside = true;
            weights[i] = std::max(Scalar(0), weights[i]);
            total += weights[i];
        }
        if (outside) continue;
        WorkingPoint difference{}, first{}, second{};
        for (unsigned i = 0; i < count; ++i)
            for (unsigned axis = 0; axis < 3; ++axis) {
                const Scalar weight = weights[i] / total;
                difference[axis] += simplex.points[indices[i]].difference[axis] * weight;
                first[axis] += simplex.points[indices[i]].first[axis] * weight;
                second[axis] += simplex.points[indices[i]].second[axis] * weight;
            }
        const Scalar squared = product(difference, difference);
        if (squared < best) {
            best = squared;
            for (unsigned axis = 0; axis < 3; ++axis) {
                nearest.difference[axis] = double(difference[axis]);
                nearest.first[axis] = double(first[axis]);
                nearest.second[axis] = double(second[axis]);
            }
            retained.count = 0;
            for (unsigned i = 0; i < count; ++i)
                if (weights[i] > 0.0) retained.points[retained.count++] = simplex.points[indices[i]];
        }
    }
    simplex = retained;
    return nearest;
}

/** GJK against a support function for the Minkowski difference. A separating
 * plane proves absence; a simplex within two nanometres proves contact. This
 * covers float-backed CAD vertices meeting at a shared boundary. Failure to
 * converge is explicit, never silently reported as absence. */
template<class Scalar, class Support>
Overlap overlap_with_witness_precision(Support support, Point &witness) {
    constexpr double tolerance = 2e-9;
    Simplex simplex;
    simplex.points[simplex.count++] = support(Point{1.0, 0.0, 0.0});
    ClosestWitness nearest{simplex.points[0].difference, simplex.points[0].first,
                          simplex.points[0].second};
    for (unsigned iteration = 0; iteration < 64; ++iteration) {
        const double squared = dot(nearest.difference, nearest.difference);
        if (squared <= tolerance * tolerance) {
            for (unsigned axis = 0; axis < 3; ++axis)
                witness[axis] = (nearest.first[axis] + nearest.second[axis]) * 0.5;
            return Overlap::intersecting;
        }
        const Point point_direction{-nearest.difference[0], -nearest.difference[1], -nearest.difference[2]};
        const auto point = support(point_direction);
        if (dot(nearest.difference, point.difference) > tolerance * std::sqrt(squared))
            return Overlap::separated;
        for (unsigned i = 0; i < simplex.count; ++i)
            if (simplex.points[i].difference == point.difference) return Overlap::unresolved;
        if (simplex.count >= 4) return Overlap::unresolved;
        simplex.points[simplex.count++] = point;
        nearest = closest<Scalar>(simplex);
    }
    return Overlap::unresolved;
}

/** Retry only unresolved contact queries with extended affine arithmetic. The
 * geometric tolerance, support function and explicit failure contract are unchanged. */
template<class Support>
Overlap overlap_with_witness(Support support, Point &witness) {
    const auto ordinary = overlap_with_witness_precision<double>(support, witness);
    return ordinary == Overlap::unresolved
        ? overlap_with_witness_precision<long double>(support, witness) : ordinary;
}

template<class Support>
Overlap overlap(Support support) {
    Point witness{};
    return overlap_with_witness([&](const Point &direction) {
        const Point point = support(direction);
        return Simplex::SupportPoint{point, point, Point{}};
    }, witness);
}

} // namespace nksim::geometry
