"""Linear three-direction compliance reduction, in micrometres per newton."""

import itertools
import math


def principal(matrix):
    """Jacobi eigensystem of the symmetric part; columns are world directions."""
    a = [[(matrix[i][j] + matrix[j][i]) / 2 for j in range(3)] for i in range(3)]
    vectors = [[float(i == j) for j in range(3)] for i in range(3)]
    for _ in range(60):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) < 1e-14 * max(abs(a[i][i]) for i in range(3)):
            break
        angle = .5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        rotation = [[float(i == j) for j in range(3)] for i in range(3)]
        rotation[p][p] = rotation[q][q] = c
        rotation[p][q], rotation[q][p] = s, -s
        a = [[sum(rotation[k][i] * a[k][l] * rotation[l][j] for k in range(3) for l in range(3)) for j in range(3)] for i in range(3)]
        vectors = [[sum(vectors[i][k] * rotation[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    result = []
    for j in range(3):
        direction = [vectors[i][j] for i in range(3)]
        sign = 1 if direction[max(range(3), key=lambda i: abs(direction[i]))] >= 0 else -1
        result.append({"complianceUmPerN": a[j][j], "worldDirection": [sign * x for x in direction]})
    return sorted(result, key=lambda x: x["complianceUmPerN"], reverse=True)


def reduce(matrix, envelope_n):
    scale = max(abs(matrix[i][i]) for i in range(3))
    symmetry = max(abs(matrix[i][j] - matrix[j][i]) for i in range(3) for j in range(3)) / scale
    corners = []
    for signs in itertools.product((-1, 1), repeat=3):
        force = [envelope_n * s for s in signs]
        displacement = [sum(matrix[i][j] * force[j] for j in range(3)) for i in range(3)]
        corners.append({"forceN": force, "relativeDisplacementUm": displacement, "normUm": math.sqrt(sum(x*x for x in displacement))})
    modes = principal(matrix)
    if symmetry > 1e-3 or any(m["complianceUmPerN"] <= 0 for m in modes):
        raise ValueError(f"Compliance symmetry/positive definiteness gate failed: {symmetry}")
    return {"complianceUmPerN": matrix, "symmetryRelativeError": symmetry, "principalCompliances": modes,
            "loadBoxCorners": corners, "worstLoadBoxCorner": max(corners, key=lambda x: x["normUm"])}
