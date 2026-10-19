"""CalculiX model of a bonded elastic frame carrying ideal rigid axis assemblies."""

import math
import re

from frame_mesh import xyz
from ccx_deck import rigid_equations, run_solver, node_lines, element_lines, equation_lines
from packet import digest, elastic_input
from compliance import reduce as reduce_compliance


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def solve_small(matrix, rhs):
    rows = [list(a) + [b] for a, b in zip(matrix, rhs)]
    for i in range(len(rhs)):
        pivot = max(range(i, len(rhs)), key=lambda j: abs(rows[j][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        value = rows[i][i]
        if abs(value) < 1e-12:
            raise ValueError("Load-transfer region cannot support a complete wrench")
        rows[i] = [v / value for v in rows[i]]
        for j in range(len(rhs)):
            if i != j:
                factor = rows[j][i]
                rows[j] = [a - factor * b for a, b in zip(rows[j], rows[i])]
    return [r[-1] for r in rows]


def distribute_wrench(nodes, ids, point, force):
    """Least-norm distributed loads conserving the force and moment about a region centroid."""
    ids = sorted(ids)
    centre = tuple(sum(nodes[n][i] for n in ids) / len(ids) for i in range(3))
    scale = max(math.dist(nodes[n], centre) for n in ids)
    if scale < 1e-6:
        raise ValueError("A load-transfer region must have finite extent")
    lever = tuple(point[i] - centre[i] for i in range(3))
    target = list(force) + [v / scale for v in cross(lever, force)]
    gram = [[0.0] * 6 for _ in range(6)]
    columns = []
    for n in ids:
        r = tuple((nodes[n][i] - centre[i]) / scale for i in range(3))
        for i in range(3):
            unit = tuple(float(i == j) for j in range(3))
            column = list(unit) + list(cross(r, unit))
            columns.append((n, i + 1, column))
            for a in range(6):
                for b in range(6):
                    gram[a][b] += column[a] * column[b]
    factors = solve_small(gram, target)
    return [(n, dof, sum(a * b for a, b in zip(column, factors))) for n, dof, column in columns]


def descendants(parts, member, root):
    seen = set()
    while member is not None:
        if member == root:
            return True
        if member in seen or member not in parts:
            raise ValueError("Invalid component parent graph; re-export the FEM packet")
        seen.add(member)
        member = parts[member]["parentId"]
    return False


def write_deck(mesh, packet, config, output, support_name="soft-three-point", ground_offsets=None, stiffen_group=None, gravity_only=False, unit_axis=None):
    support = config["supportConfigurations"][support_name]
    constraints = support["constraints"]
    spring_k = support["verticalSpringNPerMm"]
    ground_offsets = ground_offsets or {}
    nodes = dict(mesh["nodes"])
    parts = {p["id"]: p for p in packet["parts"]}
    patches = {p["name"]: p for p in mesh["patches"]}
    tool_point = xyz(packet["datums"]["toolTipMm"])
    table_point = xyz(packet["datums"]["tableTopMm"])
    if tool_point[2] < table_point[2] - 1e-6:
        raise ValueError("Cutting point is below the table")
    next_node = max(nodes) + 1
    references = {}
    for name, point in [("head", tool_point), ("table", tool_point)] + [("foot-" + s["id"], xyz(s["floorMm"])) for s in packet["supports"]]:
        references[name] = {"translation": next_node, "rotation": next_node + 1, "pointMm": point}
        nodes[next_node] = point
        nodes[next_node + 1] = point
        next_node += 2
    grounds = {}
    if spring_k:
        for name in constraints:
            point = references["foot-" + name]["pointMm"]
            grounds[name] = next_node
            nodes[next_node] = point
            next_node += 1
    equations, dependent = [], set()
    for patch in mesh["patches"]:
        if patch["kind"] == "support":
            continue
        head = patch["kind"] in ("guide-z", "drive-z")
        body = references["head" if head else "table"]
        dofs = {"guide-z": (1, 2), "drive-z": (3,), "guide-y": (1, 3), "drive-y": (2,)}[patch["kind"]]
        for equation in rigid_equations(nodes, patch["nodes"], body["translation"], body["rotation"], tool_point, dofs):
            key = equation[0][:2]
            if key in dependent:
                raise ValueError("Overlapping kinematic constraints on a frame node")
            dependent.add(key)
            equations.append(equation)
    gravity = {}
    def add(n, dof, value):
        gravity[n, dof] = gravity.get((n, dof), 0.0) + value
    def on_reference(ref, point, force):
        for i, value in enumerate(force):
            add(ref["translation"], i + 1, value)
        moment = cross(tuple(point[i] - ref["pointMm"][i] for i in range(3)), force)
        for i, value in enumerate(moment):
            add(ref["rotation"], i + 1, value)
    allocations, meshed_mass, lumped_mass = [], 0.0, 0.0
    g = config["gravityMPerS2"]
    for part in packet["parts"]:
        member, mass, centre = part["id"], part["massKg"], xyz(part["centreOfMassMm"])
        if member in mesh["elements"]:
            meshed_mass += mass
            continue
        force = (0, 0, -mass * g)
        if descendants(parts, member, "zAxis/blockLeftLower"):
            target = "head"
        elif descendants(parts, member, "xyStage/blockY0"):
            target = "table"
        elif member.startswith("frame/levelingFoot"):
            target = "foot-" + member.removeprefix("frame/levelingFoot")
        else:
            target = part["parentId"]
            seen = set()
            while target not in mesh["elements"]:
                if target is None or target in seen or target not in parts:
                    raise ValueError(f"No represented attachment for component {member}")
                seen.add(target)
                target = parts[target]["parentId"]
        if target in references:
            on_reference(references[target], centre, force)
        else:
            for n, dof, value in distribute_wrench(nodes, mesh["bodyNodes"][target], centre, force):
                add(n, dof, value)
        lumped_mass += mass
        allocations.append({"member": member, "massKg": mass, "target": target})
    payload_point = xyz(packet["datums"]["workpieceReferenceMm"])
    on_reference(references["table"], payload_point, (0, 0, -config["payloadKg"] * g))
    if not math.isclose(meshed_mass + lumped_mass, packet["totalComponentMassKg"], rel_tol=1e-9):
        raise ValueError("Component mass budget does not close")
    # CalculiX numerical input fields are limited to 20 characters; .12g stays inside that limit.
    lines = ["*HEADING", "Bonded mineral-cast frame; rigid axes; assumed properties; mm N tonne s"] + node_lines(nodes)
    sets, groups = {}, {}
    def energy_group(member):
        if member == "frame/base/casting": return "BASE_CASTING"
        if member == "frame/column/casting": return "COLUMN_CASTING"
        if member.startswith(("frame/base/column", "frame/column/foot")): return "CLAMP_PADS"
        return "INSERTS"
    for index, (member, elements) in enumerate(mesh["elements"].items()):
        label = f"BODY{index}"
        sets[member] = label
        lines += element_lines(elements,label)
        group = energy_group(member)
        groups.setdefault(group, []).append(label)
        material = "MAT_" + parts[member]["materialId"].upper().replace("-", "_")
        if stiffen_group == group: material += "_STIFF"
        lines += [f"*SOLID SECTION, ELSET={label}, MATERIAL={material}"]
    for key, value in config["materials"].items():
        prop = elastic_input(value["youngsPa"], value["poissonRatio"], value["densityKgPerM3"])
        for suffix, factor in (("", 1), ("_STIFF", 1.01)):
            label = "MAT_" + key.upper().replace("-", "_") + suffix
            lines += [f"*MATERIAL, NAME={label}", "*ELASTIC", f"{factor*prop['youngsMPa']:.12g}, {prop['poissonRatio']:.12g}",
                      "*DENSITY", f"{prop['densityTonnePerMm3']:.12g}"]
    material_checks=[]
    for member in mesh["elements"]:
        material = config["materials"][parts[member]["materialId"]]
        expected_mass = parts[member]["volumeMm3"] * 1e-9 * material["densityKgPerM3"]
        material_checks.append({"member":member,"materialId":parts[member]["materialId"],"densityKgPerM3":material["densityKgPerM3"],"packetMassKg":parts[member]["massKg"],"densityVolumeMassKg":expected_mass})
        if not math.isclose(parts[member]["massKg"], expected_mass, rel_tol=1e-7):
            raise ValueError(f"Mesh material density differs from packet mass: {member}")
    for group, labels in groups.items():
        lines += [f"*ELSET,ELSET={group}"]
        lines += [", ".join(labels[i:i+12]) for i in range(0,len(labels),12)]
    lines += equation_lines(equations)
    support_nodes = []
    for name, dofs in constraints.items():
        ref = references["foot-" + name]
        patch = patches["foot-" + name]
        label = "FOOT_" + name.upper()
        lines += [f"*NSET,NSET={label}"]
        for start in range(0, len(patch["nodes"]), 12):
            lines.append(", ".join(map(str, patch["nodes"][start:start + 12])))
        lines += [f"*RIGID BODY,NSET={label},REF NODE={ref['translation']},ROT NODE={ref['rotation']}"]
        support_nodes += [ref["translation"], ref["rotation"]]
    if spring_k:
        first_element = max(e for rows in mesh["elements"].values() for e, _ in rows) + 1
        lines += ["*ELEMENT,TYPE=SPRING2,ELSET=SUPPORT_SPRINGS"]
        for i, name in enumerate(constraints):
            lines.append(f"{first_element+i}, {references['foot-'+name]['translation']}, {grounds[name]}")
        lines += ["*SPRING,ELSET=SUPPORT_SPRINGS", "3, 3", f"{spring_k:.6f}"]
        groups["SUPPORT_SPRINGS"] = []
        support_nodes += list(grounds.values())
    lines += ["*NSET,NSET=OBS", ", ".join(str(references[b][k]) for b in ("head", "table") for k in ("translation", "rotation")),
              "*NSET,NSET=SUPPORTS", ", ".join(map(str, support_nodes)), "*BOUNDARY"]
    for name, dofs in constraints.items():
        for dof in dofs:
            lines.append(f"{references['foot-' + name]['translation']}, {dof}, {dof}, 0")
    for n in grounds.values():
        lines.append(f"{n}, 1, 3, 0")
    cases = [("gravity", None, 0), ("x", 1, 1), ("y", 2, 1), ("z", 3, 1),
             ("x-half", 1, .5), ("x-negative", 1, -1)]
    if gravity_only:
        cases = cases[:1]
    if unit_axis is not None:
        cases = [(unit_axis,"xyz".index(unit_axis)+1,1)]
    for name, axis, load in cases:
        loads = dict(gravity) if axis is None else {}
        if axis is None and spring_k:
            for station, ground in grounds.items():
                preload = spring_k*ground_offsets.get(station,0)
                n=references["foot-"+station]["translation"]
                loads[n,3]=loads.get((n,3),0)+preload
                loads[ground,3]=loads.get((ground,3),0)-preload
        if axis:
            for body, sign in (("head", 1), ("table", -1)):
                loads[references[body]["translation"], axis] = sign * load
        lines += ["*STEP", f"** {name}", "*STATIC"]
        if grounds:
            lines += ["*BOUNDARY"]
            for station, n in grounds.items():
                offset = 0  # Leveling is represented by equivalent initial spring force pairs.
                lines.append(f"{n}, 3, 3, {offset:.12g}")
        lines += ["*DLOAD,OP=NEW"]
        if axis is None:
            lines += [f"{label}, GRAV, {g * 1000:.12g}, 0, 0, -1" for label in sets.values()]
        lines += ["*CLOAD,OP=NEW"]
        lines += [f"{n}, {dof}, {value:.12g}" for (n, dof), value in sorted(loads.items()) if abs(value) > 1e-14]
        lines += ["*NODE PRINT,NSET=OBS", "U", "*NODE PRINT,NSET=SUPPORTS", "RF", "*NODE PRINT,NSET=SUPPORTS", "U"]
        for group in groups:
            lines += [f"*EL PRINT,ELSET={group},TOTALS=ONLY", "ELSE"]
        lines += ["*NODE FILE", "U", "*END STEP"]
    path = output / "frame.inp"
    path.write_text("\n".join(lines) + "\n")
    return {"materialMassChecks":material_checks,"supportConfiguration":support_name,"supportConstraints":constraints,"springNPerMm":spring_k,"grounds":grounds,
            "groundOffsetsMm":ground_offsets,"gravityReferenceLoads":{str(n)+":"+str(d):v for (n,d),v in gravity.items() if n in support_nodes},
            "energyGroups":list(groups),"references": references, "cases": [c[0] for c in cases], "inputSha256": digest(path),
            "massBudgetKg": {"meshed": meshed_mass, "lumpedComponents": lumped_mass, "payload": config["payloadKg"],
                             "total": meshed_mass + lumped_mass + config["payloadKg"]},
            "gravityAllocations": allocations, "cutPointMm": tool_point,
            "rigidFixtureHeightMm": tool_point[2] - table_point[2], "equations": len(equations)}


def read_results(text, expected_steps):
    tables, active, values = {}, None, None
    pattern = re.compile(r"\s*(displacements|forces)\s+\(.*?\) for set (\S+) and time\s+(\S+)")
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            kind, group, time = match.groups()
            step = int(round(float(time.replace("D", "E"))))
            active = step, kind, group
            if active in tables:
                raise ValueError("Duplicate result table/increment; expected one linear result per step")
            values = tables[active] = {}
        elif active is not None:
            row = line.split()
            if len(row) == 4 and row[0].isdigit():
                node = int(row[0])
                vector = tuple(float(v.replace("D", "E")) for v in row[1:])
                if node in values or not all(math.isfinite(v) for v in vector):
                    raise ValueError("Invalid/duplicate nodal result")
                values[node] = vector
            elif values:
                active = None
    wanted = {(step, kind, group) for step in range(1, expected_steps + 1)
              for kind, group in (("displacements", "OBS"), ("forces", "SUPPORTS"), ("displacements", "SUPPORTS"))}
    if set(tables) != wanted or any(not v for v in tables.values()):
        raise ValueError("Incomplete or unexpected static result tables")
    return tables


def read_energies(text, expected_steps, groups):
    result = {}
    pattern = re.compile(r"total internal energy.*?set (\S+) and time\s+(\S+)", re.I)
    active = None
    for line in text.splitlines():
        match = pattern.search(line)
        if match:
            group, time = match.groups()
            active = (int(round(float(time.replace("D", "E")))), group)
        elif active and line.strip():
            value = float(line.split()[0].replace("D", "E"))
            if not math.isfinite(value) or value < 0 or active in result:
                raise ValueError("Invalid/duplicate element energy total")
            result[active] = value
            active = None
    wanted = {(step, group) for step in range(1, expected_steps+1) for group in groups}
    if set(result) != wanted:
        raise ValueError(f"Incomplete strain energy results: {wanted-set(result)}")
    return result


def assess(packet, config, model, tables, energies=None):
    refs, results = model["references"], []
    gravity_force, gravity_moment = [0.0] * 3, [0.0] * 3
    for mass, point in [(p["massKg"], xyz(p["centreOfMassMm"])) for p in packet["parts"]] + [
            (config["payloadKg"], xyz(packet["datums"]["workpieceReferenceMm"]))]:
        force = (0, 0, -mass * config["gravityMPerS2"])
        moment = cross(point, force)
        for i in range(3):
            gravity_force[i] += force[i]
            gravity_moment[i] += moment[i]
    length_scale = max(math.dist(refs["head"]["pointMm"], r["pointMm"]) for r in refs.values())
    for step, name in enumerate(model["cases"], 1):
        u, rf = tables[step, "displacements", "OBS"], tables[step, "forces", "SUPPORTS"]
        is_gravity = name == "gravity"
        force_scale = abs(gravity_force[2]) if is_gravity else 1
        resultant = list(gravity_force) if is_gravity else [0.0]*3
        moment = list(gravity_moment) if is_gravity else [0.0]*3
        supports = {}
        for station, dofs in model["supportConstraints"].items():
            ref = refs["foot-" + station]
            force = [0.0]*3
            for dof in dofs:
                correction = model["gravityReferenceLoads"].get(str(ref["translation"])+":"+str(dof), 0) if is_gravity else 0
                force[dof-1] = rf[ref["translation"]][dof-1] - correction
            if model["springNPerMm"]:
                offset = model["groundOffsetsMm"].get(station,0) if is_gravity else 0
                foot_z = tables[step,"displacements","SUPPORTS"][ref["translation"]][2]
                force[2] = model["springNPerMm"]*(offset-foot_z)
            if is_gravity and force[2] < -1e-4*force_scale:
                raise ValueError(f"Gravity requires a tensile foot reaction: {station}")
            supports[station] = force
            m = cross(ref["pointMm"], force)
            for i in range(3):
                resultant[i] += force[i]
                moment[i] += m[i]
        force_error = math.sqrt(sum(v*v for v in resultant))/force_scale
        moment_error = math.sqrt(sum(v*v for v in moment))/(force_scale*length_scale)
        if max(force_error, moment_error) > config["equilibriumRelativeTolerance"]:
            raise ValueError(f"Force/moment balance failed for {name}: {force_error}, {moment_error}")
        relative = [u[refs["head"]["translation"]][i]-u[refs["table"]["translation"]][i] for i in range(3)]
        item = {"case":name,"relativeDisplacementMm":relative,"relativeDisplacementUm":[1000*x for x in relative],
                "supportReactionsN":supports,"forceRelativeError":force_error,"momentRelativeError":moment_error}
        if is_gravity and model["springNPerMm"]:
            foot_heights={station:tables[step,"displacements","SUPPORTS"][refs["foot-"+station]["translation"]][2] for station in model["supportConstraints"]}
            if max(abs(v) for v in foot_heights.values())>1e-6:
                raise ValueError(f"Four-spring leveling height check failed: {foot_heights}")
            item["leveledFootHeightsMm"]=foot_heights
        if energies is not None and name in "xyz":
            groups = {group:energies[step,group] for group in model["energyGroups"]}
            if model["springNPerMm"]:
                analytical = sum(f[2]**2/(2*model["springNPerMm"]) for f in supports.values())
                if abs(groups["SUPPORT_SPRINGS"]/analytical-1)>1e-3:
                    raise ValueError("Spring element energy disagrees with reaction squared / 2k")
            total = sum(groups.values())
            expected = .5*relative["xyz".index(name)]
            error = abs(total/expected-1)
            if error > 1e-3:
                raise ValueError(f"Strain energy/external work mismatch: {name}: {error}")
            item.update(strainEnergyNmm=groups,complianceShares={k:v/total for k,v in groups.items()},energyWorkRelativeError=error)
        results.append(item)
    by_name = {r["case"]:r for r in results}
    positive, half, negative = (by_name[n]["relativeDisplacementUm"] for n in ("x","x-half","x-negative"))
    linearity = max(math.dist(positive,[2*x for x in half]),math.dist(positive,[-x for x in negative]))/math.sqrt(sum(x*x for x in positive))
    if linearity > 1e-3:
        raise ValueError(f"Linearity/sign symmetry check failed: {linearity}")
    matrix = [[by_name[axis]["relativeDisplacementUm"][i] for axis in "xyz"] for i in range(3)]
    return {"cases":results,"linearityRelativeError":linearity,**reduce_compliance(matrix,config["cuttingForceN"])}
