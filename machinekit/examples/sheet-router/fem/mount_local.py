"""Built-geometry local section/connection model, N/mm/rad.

This is an elastic section screen, not a solid/contact stress qualification.
Loads act at the actual pitch point. No fitted end-anchor rotational stiffness
or radial-only preload projection is used. Materials/contact remain assumptions.
"""
import math
from functools import lru_cache
import numpy as np
from mount_frame import Frame, Section, rectangle
from mount_compliance import section

E = 210000.
G = E / 2.6


def rotation(p):
    x, y, z, w = (p['q'+k] for k in 'xyzw')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def position(p):
    return np.array([p[k] for k in 'xyz'], float)


def into(frame, source, point):
    return rotation(frame).T @ (position(source)+rotation(source)@np.asarray(point)-position(frame))


def union_section(stock, voids):
    """Union of rectangular stock minus conservative rectangular bore slices.

    The maximum slices at distinct axial stations are superposed deliberately:
    a weak prismatic surrogate, not a prediction of local hole stress.
    """
    rectangles = stock+voids
    xs = sorted({r[3]+sign*r[1]/2 for r in rectangles for sign in (-1, 1)})
    zs = sorted({r[4]+sign*r[2]/2 for r in rectangles for sign in (-1, 1)})
    cells = []
    for a, b in zip(xs, xs[1:]):
        for c, d in zip(zs, zs[1:]):
            x, z = (a+b)/2, (c+d)/2
            def inside(r): return abs(x-r[3]) < r[1]/2+1e-9 and abs(z-r[4]) < r[2]/2+1e-9
            if any(inside(r) for r in stock) and not any(inside(r) for r in voids):
                cells.append((1., b-a, d-c, x, z))
    area, centre, covariance = section(cells)
    eigen, axes = np.linalg.eigh(covariance)
    # Independent free-warping rectangles give a weak torsion surrogate; the
    # joined angle's additional warping restraint is not credited.
    torsion = sum(rectangle(r[1], r[2]).torsion for r in cells)
    return (Section(area, eigen[1], eigen[0], torsion, area*5/6, area*5/6),
            centre, np.array([axes[0, 0], 0., axes[1, 0]]), cells)


def receiver_section(values):
    width, thickness, flange, height = (values[k] for k in ('width', 'thickness', 'flangeThickness', 'flangeWidth'))
    stock = [(1., width, thickness, 0., thickness/2),
             (1., flange, height, -width/2-flange/2, height/2)]
    voids = []
    for hole in values['holes']['holes']:
        direction = rotation(hole['frame'])[:, 2]
        if abs(direction[1]) > 1e-7:
            raise ValueError('Unexpected longitudinal receiver drilling')
        diameter = max(hole['diameter'], (hole.get('thread') or {}).get('majorDiameter', 0.))
        depth = hole['depth']
        x, z = hole['frame']['x'], hole['frame']['z']
        if abs(direction[2]) > .999999:
            voids.append((-1., diameter, depth, x, z+direction[2]*depth/2))
        elif abs(direction[0]) > .999999:
            voids.append((-1., depth, diameter, x+direction[0]*depth/2, z))
        else:
            raise ValueError('Receiver bore must follow a declared principal axis')
        if hole.get('counterbore'):
            raise ValueError('Add explicit counterbore slice before using this receiver')
    return union_section(stock, voids)


def spring_oriented(model, a, b, stiffness, basis, group):
    k = np.asarray(stiffness, float)
    translated = basis @ np.diag(k[:3]) @ basis.T
    rotated = basis @ np.diag(k[3:]) @ basis.T
    for matrix in (translated, rotated):
        if np.max(np.abs(matrix-np.diag(np.diag(matrix)))) > 1e-7*max(1., np.max(np.abs(matrix))):
            raise ValueError('This fixture requires principal-axis connector springs')
    model.spring(a, b, np.r_[np.diag(translated), np.diag(rotated)], group)


def beam(model, name, a, b, cross_section, normal, group, mesh=10.):
    a, b = np.asarray(a, float), np.asarray(b, float)
    count = max(1, math.ceil(np.linalg.norm(b-a)/mesh))
    nodes = []
    for i in range(count+1):
        node = name+':'+str(i)
        model.node(node, a+(b-a)*i/count)
        nodes.append(node)
    for left, right in zip(nodes, nodes[1:]):
        model.beam(left, right, cross_section, normal, group)
    return nodes[0], nodes[-1]


def port(model, owner, name, point):
    model.node(name, point)
    model.rigid(owner, name)
    return name


@lru_cache(maxsize=1024)
def host_normal_flexibility(width, depth, wall, pad, strip, load_x, removed_webs):
    """Local RHS distortion with the mean section's six rigid motions removed.

    A cassette window removes its entire affected web strip, rather than
    crediting an uncut section. Longitudinal load spreading is not credited.
    Pad and top wall share normal displacement without composite-offset credit.
    All local distortion remains free; no arbitrary corner is clamped. The
    removed-web case is a weaker material subset of the intact section model.
    """
    hx, hz = (width-wall)/2, (depth-wall)/2
    if not -hx < load_x < hx:
        raise ValueError('Host load point needs an explicit corner detail')
    web = rectangle(strip,wall)
    top = Section(strip*(wall+pad),strip*(wall**3+pad**3)/12,
                  (wall+pad)*strip**3/12,
                  web.torsion+(rectangle(strip,pad).torsion if pad else 0),
                  strip*(wall+pad)*5/6,strip*(wall+pad)*5/6)
    model = Frame()
    for name, point in [('bottom-left',(-hx,0,-hz)),('bottom-right',(hx,0,-hz)),
                        ('top-left',(-hx,0,hz)),('top-right',(hx,0,hz)),('load',(load_x,0,hz))]:
        model.node(name,point)
    for a,b in [('top-left','load'),('load','top-right')]:
        model.beam(a,b,top,(0,1,0),'host top wall and unbonded pad')
    model.beam('bottom-left','bottom-right',web,(0,1,0),'host bottom wall')
    for side,a,b in [(-1,'bottom-left','top-left'),(1,'bottom-right','top-right')]:
        if side not in removed_webs:
            model.beam(a,b,web,(0,1,0),'host remaining webs')
    return model.solve({'load':[0,0,1.,0,0,0]},
                       reference=['bottom-left','bottom-right','top-left','top-right'])['work']


def seat_model(data, prefix, point_y, mesh=10.):
    rows = {r['id']: r for r in data['mountInputs']}
    receiver = rows[prefix+'Receiver']
    frame = receiver['pose']
    values = receiver['parameters']
    cross_section, centre, normal, cells = receiver_section(values)
    supports = [rows[prefix+f'Cleat{i}'] for i in range(1, 6)]
    local = lambda row, point: into(frame, row['pose'], point)
    stations = sorted({-values['length']/2, values['length']/2, point_y,
                       *(float(local(r, [0, 0, r['parameters']['height']-10])[1]) for r in supports)})
    model = Frame()
    station_nodes = {}
    previous = None
    for i, y in enumerate(stations):
        name = 'receiver-station-'+str(i)
        model.node(name, (centre[0], y, centre[1]))
        station_nodes[y] = name
        if previous is not None:
            a, b = beam(model, 'receiver-span-'+str(i), model.points[previous], model.points[name],
                        cross_section, normal, 'receiver net bending/torsion', mesh)
            model.rigid(previous, a)
            model.rigid(name, b)
        previous = name
    for index, row in enumerate(supports):
        name = 'cleat'+str(index+1)
        height = row['parameters']['height']
        basis = rotation(frame).T @ rotation(row['pose'])
        p = lambda xyz: local(row, xyz)
        # Debits match the actual 12.5 mm recesses and 6.6 mm clearances.
        # Their maximum slices are superposed, without gross-section credit.
        web_width, web_wall, base_width = 64-2*12.5-2*6.6, 11-2.6, 64-2*13.
        bottom, top = beam(model, name+'-web', p([5.5, 0, 13]), p([5.5, 0, height-10]),
                           rectangle(web_width, web_wall), basis[:, 1], 'cleat web/prying', mesh)
        toe, heel = beam(model, name+'-base', p([5.5, 0, 6.5]), p([20, 0, 6.5]),
                         rectangle(base_width, 5.), basis[:, 1], 'cleat counterbore floor', mesh)
        model.rigid(bottom, toe)
        station_y = float(p([0, 0, height-10])[1])
        owner = station_nodes[station_y]
        for j, y in enumerate((-14., 14.)):
            first = port(model, top, name+f'-web-head{j}', p([10, y, height-10]))
            second = port(model, owner, name+f'-receiver-thread{j}', p([0, y, height-10]))
            axial = 1/(10/(E*20.1)+1/(E*6*web_wall)+1/(E*6*10))
            shear = 1/(10/(G*20.1)+1/(E*6*web_wall)+1/(E*6*10))
            spring_oriented(model, first, second, [axial, shear, shear, 0, 0, 0], basis,
                            'web bolts/thread bearing')
        # Closed, preloaded web-face contact supplies the real roll couple.
        # No invented rotational spring. Opening must be checked against bolt
        # preload before this tangent stiffness is admitted to a final model.
        for j, z in enumerate((13., height)):
            first = port(model, top, name+f'-web-contact{j}', p([0, 0, z]))
            second = port(model, owner, name+f'-receiver-contact{j}', p([0, 0, z]))
            spring_oriented(model, first, second, [E*web_width*(height-13)/2/web_wall, 0, 0, 0, 0, 0], basis,
                            'preloaded web-face contact')
        # Host local wall/pad compliance is separate from global tube bending.
        host_id = 'beam' if prefix.startswith('rackMountX') else 'frameRight' if prefix.startswith('rackMountYRight') else 'frameLeft'
        tube = rows[host_id]
        t = tube['parameters']['wall']
        pad_id = 'railPadYRight' if host_id == 'frameRight' else 'railPadYLeft'
        pad = rows[pad_id]['parameters']['height'] if host_id != 'beam' else 0.
        tube_frame = tube['pose']
        def host_flexibility(at):
            world = position(frame)+rotation(frame)@at
            local_point = rotation(tube_frame).T@(world-position(tube_frame))
            tx = local_point[0]
            span = tube['parameters']['width']
            strip = 12+2*t-6.6
            removed = set()
            for hole in tube['parameters']['holes']['holes']:
                if not hole['id'].endswith('-cassette-entry'):
                    continue
                axis = rotation(hole['frame'])[:,2]
                if abs(axis[0]) < .999:
                    raise ValueError('Host cassette window needs an explicit section model')
                if abs(local_point[2]-hole['frame']['z']) <= hole['diameter']/2+strip/2:
                    removed.add(-1 if hole['frame']['x'] < 0 else 1)
            return host_normal_flexibility(span,tube['parameters']['depth'],t,pad,strip,
                                           round(float(tx),9),tuple(sorted(removed)))
        for j, y in enumerate((-18., 18.)):
            at = p([20, y, 6.5])
            bolt = port(model, heel, name+f'-base-bolt{j}', at)
            # Actual source bolt station to the nearest vertical RHS web.
            wall_c = host_flexibility(at)
            grip = 5+4+pad+t+3.2
            shear = 1/(grip/(G*20.1)+2/(E*6*t))
            normal_k = 1/(grip/(E*20.1)+wall_c)
            spring_oriented(model, bolt, None, [shear, shear, normal_k, 0, 0, 0], basis,
                            'base bolts/host pad and wall')
        # The 4 mm bedding is not credited in shear. Four normal contact
        # patches retain finite base roll support under the assumed preload.
        area = (28*64-2*math.pi*3.3**2)/4
        for j, (x, y) in enumerate(( (x, y) for x in (2., 26.) for y in (-24., 24.))):
            at = p([x, y, 0])
            contact = port(model, heel, name+f'-bedding{j}', at)
            stiffness = 1/(4/(4000*area)+host_flexibility(at))
            spring_oriented(model, contact, None, [0, 0, stiffness, 0, 0, 0], basis,
                            'preloaded normal bedding/host strips')
    pitch = port(model, station_nodes[point_y], 'rack-pitch', (9.5, point_y, 18+3.2+19/2))
    return model, pitch, dict(netAreaMm2=cross_section.area, netTorsionMm4=cross_section.torsion,
                             sectionCentreXZMm=centre.tolist(), weakSectionRectangles=cells)
