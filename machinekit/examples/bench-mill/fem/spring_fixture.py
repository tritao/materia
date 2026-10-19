"""Known spring/precompression answer for the four-foot leveling formulation."""

from ccx_deck import node_lines, run_solver
from packet import digest
from solver_fixtures import read_table


def verify(ccx, output):
    results=[]
    # Compressive 1000 N preload offsets 1000 N self-weight at a loaded height of zero.
    for name, gravity, preload in (("unleveled",1000,0),("leveled",1000,1000),("unit-increment",1,0)):
        folder=output/name
        folder.mkdir(parents=True,exist_ok=True)
        lines=node_lines({1:(0,0,0),2:(0,0,0)})+[
            "*ELEMENT,TYPE=SPRING2,ELSET=SUPPORT", "1,1,2", "*SPRING,ELSET=SUPPORT", "3,3", "100000.",
            "*NSET,NSET=OBS", "1,2", "*BOUNDARY", "1,1,2,0", "2,1,3,0", "*STEP", "*STATIC", "*CLOAD",
            f"1,3,{preload-gravity:.12g}", f"2,3,{-preload:.12g}", "*NODE PRINT,NSET=OBS", "U", "*END STEP"]
        (folder/'frame.inp').write_text('\n'.join(lines)+'\n')
        run_solver(ccx,folder,60)
        u=read_table((folder/'frame.dat').read_text(),'U')
        expected=(preload-gravity)/100000
        reaction=preload+100000*(u[2][2]-u[1][2])
        if abs(u[1][2]-expected)>1e-10 or abs(reaction-gravity)>1e-6:
            raise ValueError(f"Support leveling fixture failed: {name}")
        results.append({'case':name,'displacementMm':u[1][2],'expectedMm':expected,'physicalReactionN':reaction,
                        'inputSha256':digest(folder/'frame.inp'),'resultSha256':digest(folder/'frame.dat')})
    return {'stiffnessNPerMm':100000,'method':'Initial spring force represented by equal/opposite nodal forces; ground DOFs remain zero',
            'cases':results,'passed':True}
