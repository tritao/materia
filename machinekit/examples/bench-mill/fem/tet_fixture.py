"""Known-answer C3D10 and rigid-equation fixtures using the frame's real deck path."""

from pathlib import Path

from ccx_deck import (element_lines, equation_lines, node_lines, nset_lines, number,
                      rigid_equations, run_solver, tet10_order)
from packet import digest
from solver_fixtures import read_table


def verify(gmsh, ccx, output):
    gmsh.initialize()
    try:
        gmsh.model.add("C3D10-verification")
        gmsh.model.occ.addBox(0, 0, 0, 100, 10, 10)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 8)
        gmsh.option.setNumber("Mesh.ElementOrder", 2)
        gmsh.model.mesh.generate(3)
        tags, raw, _ = gmsh.model.mesh.getNodes()
        nodes = {int(n): tuple(float(v) for v in raw[3*i:3*i+3]) for i, n in enumerate(tags)}
        types, element_ids, connectivity = gmsh.model.mesh.getElements(3)
        if list(types) != [11]:
            raise ValueError("C3D10 fixture did not generate quadratic tetrahedra")
        order = tet10_order(gmsh)
        elements = [(int(n), [int(connectivity[0][10*i+j]) for j in order]) for i, n in enumerate(element_ids[0])]
    finally:
        gmsh.finalize()
    fixed = sorted(n for n, p in nodes.items() if abs(p[0]) < 1e-7)
    tip = sorted(n for n, p in nodes.items() if abs(p[0]-100) < 1e-7)
    reference, rotation = max(nodes)+1, max(nodes)+2
    root, root_rotation = reference+2, reference+3
    nodes[root] = nodes[root_rotation] = (0, 5, 5)
    results = {}
    for name in ("tension", "rigid-equations"):
        origin = (100, 5, 5) if name == "tension" else (120, 3, 7)
        nodes[reference] = nodes[rotation] = origin
        lines = ["*HEADING", "Gmsh C3D10 and frame coupling verification"] + node_lines(nodes)
        lines += element_lines(elements, "BAR")
        lines += ["*MATERIAL,NAME=REFERENCE", "*ELASTIC", "210000,0", "*SOLID SECTION,ELSET=BAR,MATERIAL=REFERENCE"]
        lines += nset_lines("FIXED", fixed) + nset_lines("SUPPORT", [root,root_rotation])
        observed = [reference] if name == "tension" else tip
        lines += nset_lines("OBS", observed)
        lines += equation_lines(rigid_equations(nodes,tip,reference,rotation,origin,(1,2,3)))
        lines += [f"*RIGID BODY,NSET=FIXED,REF NODE={root},ROT NODE={root_rotation}", "*BOUNDARY",
                  f"{root},1,3", f"{root_rotation},1,3"]
        translation, angles = (.01,-.02,.03), (1e-4,-2e-4,3e-4)
        if name == "rigid-equations":
            for node, values in ((reference,translation),(rotation,angles)):
                lines += [f"{node},{d},{d},{number(value)}" for d,value in enumerate(values,1)]
        lines += ["*STEP", "*STATIC"]
        if name == "tension":
            # RF at a constrained reference includes its applied point load.
            lines += ["*CLOAD", f"{reference},1,1000", f"{root},1,10"]
        lines += ["*NODE PRINT,NSET=OBS", "U", "*NODE PRINT,NSET=SUPPORT", "RF", "*END STEP"]
        folder = (Path(output)/name).resolve()
        folder.mkdir(parents=True,exist_ok=True)
        deck = folder/"frame.inp"
        deck.write_text("\n".join(lines)+"\n")
        run_solver(ccx,folder,120)
        text = (folder/"frame.dat").read_text()
        u,rf = read_table(text,"U"),read_table(text,"RF")
        result = {"elements":len(elements),"inputSha256":digest(deck),"outputSha256":digest(folder/"frame.dat")}
        if name == "tension":
            expected = 1000*100/(210000*100)
            error = abs(u[reference][0]-expected)/expected
            # Physical support reaction = reported RF minus the known CLOAD at the reference.
            reaction = rf[root][0]-10
            reaction_error = abs(reaction+1010)/1010
            if error>.01 or reaction_error>1e-5:
                raise ValueError(f"C3D10 tension/reference reaction fixture failed: {error}, {reaction_error}")
            result.update(displacementMm=u[reference][0],analyticalMm=expected,relativeError=error,
                          reactionN=reaction,reactionRelativeError=reaction_error)
        else:
            worst = 0.0
            for node in tip:
                r = tuple(nodes[node][i]-origin[i] for i in range(3))
                expected = (translation[0]+angles[1]*r[2]-angles[2]*r[1],
                            translation[1]+angles[2]*r[0]-angles[0]*r[2],
                            translation[2]+angles[0]*r[1]-angles[1]*r[0])
                worst = max(worst,max(abs(u[node][i]-expected[i]) for i in range(3)))
            if worst>1e-7:
                raise ValueError(f"Prescribed rigid motion fixture failed: {worst} mm")
            result.update(maximumDisplacementErrorMm=worst,translationMm=translation,rotationRad=angles)
        results[name]=result
    return results
