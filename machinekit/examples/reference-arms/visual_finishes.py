"""Visual-only normals and paint clipping; no changes to collision or inertial data."""
import numpy as np
import trimesh


def smooth_stl(mesh, crease_degrees=40):
    """Angle-weighted corner normals across coincident STL vertices, retaining creases."""
    triangles = mesh.triangles
    face_normals = mesh.face_normals
    weights = np.zeros((len(triangles), 3))
    for corner in range(3):
        a = triangles[:, (corner + 1) % 3] - triangles[:, corner]
        b = triangles[:, (corner + 2) % 3] - triangles[:, corner]
        weights[:, corner] = np.arctan2(np.linalg.norm(np.cross(a, b), axis=1), np.einsum('ij,ij->i', a, b))
    points = triangles.reshape(-1, 3)
    _, groups = np.unique(np.round(points, 10), axis=0, return_inverse=True)
    order = np.argsort(groups, kind='stable')
    cuts = np.r_[0, np.flatnonzero(np.diff(groups[order])) + 1, len(order)]
    normal = np.repeat(face_normals, 3, axis=0)
    result = np.zeros_like(normal)
    cosine = np.cos(np.deg2rad(crease_degrees))
    for start, end in zip(cuts[:-1], cuts[1:]):
        ids = order[start:end]
        neighbors = normal[ids]
        shared = neighbors @ neighbors.T >= cosine
        sums = shared @ (neighbors * weights.reshape(-1)[ids, None])
        lengths = np.linalg.norm(sums, axis=1)
        result[ids] = sums / np.maximum(lengths[:, None], 1e-30)
    return trimesh.Trimesh(points.copy(), np.arange(len(points)).reshape(-1, 3),
                           vertex_normals=result, process=False)


def clip_polygon(polygon, normal, offset):
    """Split position+normal vertices at a plane, interpolating the shading normals."""
    distance = polygon[:, :3] @ normal - offset
    if np.max(distance) <= 1e-12:
        return polygon, None
    if np.min(distance) >= -1e-12:
        return None, polygon
    inside, outside = [], []
    for index in range(len(polygon)):
        a, b = polygon[index - 1], polygon[index]
        da, db = distance[index - 1], distance[index]
        if (da < 0) != (db < 0):
            point = a + (b - a) * (da / (da - db))
            inside.append(point); outside.append(point)
        (inside if db <= 0 else outside).append(b)
    return (np.array(inside) if len(inside) >= 3 else None,
            np.array(outside) if len(outside) >= 3 else None)


def mesh_ring_planes(mesh, ring):
    """Use the existing polygonal cover rim, including its source tessellation."""
    axis = ring['axis']
    radial_axes = [i for i in range(3) if i != axis]
    vertices = mesh.vertices
    radial = vertices[:, radial_axes] - np.array(ring['center'])[radial_axes]
    match = ((np.abs(np.linalg.norm(radial, axis=1)-ring['radius'][0]) < 1e-7) &
             (vertices[:, axis] >= ring['axial'][0]) & (vertices[:, axis] <= ring['axial'][1]))
    points = np.unique(np.round(radial[match], 8), axis=0)
    if len(points) < 8:
        raise ValueError('Appearance cover rim does not match the pinned source mesh')
    points = points[np.argsort(np.arctan2(points[:, 1], points[:, 0]))]
    planes = []
    for a, b in zip(points, np.roll(points, -1, axis=0)):
        edge = b-a
        normal = np.zeros(3); normal[radial_axes] = [edge[1], -edge[0]]
        normal /= np.linalg.norm(normal)
        offset = normal[radial_axes] @ a + normal @ ring['center']
        planes.append((normal, offset))
    return planes


def ring_parts(polygon, ring):
    """Partition a triangle by an annular slab with 128-plane circular boundaries."""
    axis = ring['axis']
    radial_axes = [i for i in range(3) if i != axis]
    radial = polygon[:, radial_axes] - np.array(ring['center'])[radial_axes]
    low, high = radial.min(0), radial.max(0)
    nearest = np.maximum(np.maximum(low, -high), 0)
    minimum = np.linalg.norm(nearest)
    maximum = np.linalg.norm(radial, axis=1).max()
    inner, outer = ring['radius']
    inner_planes = ring.get("_innerPlanes")
    inner_minimum = inner if inner_planes is None else min(offset-normal @ ring["center"] for normal, offset in inner_planes)
    axial = polygon[:, axis]
    if axial.max() < ring['axial'][0] or axial.min() > ring['axial'][1] or minimum > outer or maximum < inner_minimum:
        return [polygon], []
    if axial.min() >= ring['axial'][0] and axial.max() <= ring['axial'][1] and minimum >= inner and maximum <= outer:
        return [], [polygon]
    white, red = [], []
    remaining = polygon
    planes = []
    for sign, bound in [(1, ring['axial'][1]), (-1, -ring['axial'][0])]:
        normal = np.zeros(3); normal[axis] = sign
        planes.append((normal, bound))
    # Circumscribed polygons: maximum radial discrepancy <18 µm at these cover radii.
    directions = np.column_stack((np.cos(np.arange(128)*2*np.pi/128), np.sin(np.arange(128)*2*np.pi/128)))
    for direction in directions:
        normal = np.zeros(3); normal[radial_axes] = direction
        planes.append((normal, outer + normal @ ring['center']))
    for normal, offset in planes:
        remaining, outside = clip_polygon(remaining, normal, offset)
        if outside is not None: white.append(outside)
        if remaining is None: return white, red
    # The complement of the inner disk forms the red annulus.
    if inner_planes is None:
        inner_planes = []
        for direction in directions:
            normal = np.zeros(3); normal[radial_axes] = direction
            inner_planes.append((normal, inner + normal @ ring['center']))
    for normal, offset in inner_planes:
        remaining, outside = clip_polygon(remaining, normal, offset)
        if outside is not None: red.append(outside)
        if remaining is None: return white, red
    white.append(remaining)
    return white, red


def paint_regions(link, regions, profile):
    painted = []
    for mesh, _, _, _ in regions:
        triangles = np.concatenate((mesh.triangles, mesh.vertex_normals[mesh.faces]), axis=2)
        buckets = [[], [], []]
        rings = [dict(ring) for ring in profile['rings'].get(link, [])]
        for ring in rings:
            if ring.get('innerAtMeshEdge'):
                ring['_innerPlanes'] = mesh_ring_planes(mesh, ring)
        for triangle in triangles:
            white = [triangle]
            for ring in rings:
                pending = []
                for polygon in white:
                    rest, red = ring_parts(polygon, ring)
                    pending.extend(rest); buckets[1].extend(red)
                white = pending
            flange = profile['flange']
            for polygon in white:
                if link == flange['link'] and flange.get('wholeLink'):
                    buckets[2].append(polygon)
                elif link == flange['link']:
                    normal = np.eye(3)[flange['axis']]
                    body, metal = clip_polygon(polygon, normal, flange['minimum'])
                    if body is not None: buckets[0].append(body)
                    if metal is not None: buckets[2].append(metal)
                else:
                    buckets[0].append(polygon)
        for polygons, finish in zip(buckets, ['white', 'red', 'silver']):
            if not polygons: continue
            triangles = np.array([polygon[[0, i, i+1]] for polygon in polygons for i in range(1, len(polygon)-1)])
            points = triangles[:, :, :3]
            area = np.linalg.norm(np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0]), axis=1)
            triangles = triangles[area > 1e-18]
            if not len(triangles): continue
            vertices = triangles[:, :, :3].reshape(-1, 3)
            normals = triangles[:, :, 3:].reshape(-1, 3)
            normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-30)
            part = trimesh.Trimesh(vertices, np.arange(len(vertices)).reshape(-1, 3), vertex_normals=normals, process=False)
            material = profile[finish]
            painted.append((part, material['color'], material['metallic'], material['roughness']))
    return painted
