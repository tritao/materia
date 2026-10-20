"""Summarise converged SR5 runs and recover eccentric Y branch loads from springs."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from study import tables, norm, sub


def modal_motion(text,tip,bed):
    rows=[]
    # Six static tables precede six modal tables in the authored deck.
    for block in re.split(r'displacements \(vx,vy,vz\)[^\n]*\n',text)[7:]:
        values={}
        for line in block.splitlines():
            v=line.split()
            if len(v)==4 and v[0].isdigit():values[int(v[0])]=tuple(float(x.replace('D','E')) for x in v[1:])
            elif values:break
        if values:rows.append(norm(sub(values[tip],values[bed]))/max(norm(v) for v in values.values()))
    if len(rows)!=6:raise ValueError('Missing six modal observation tables')
    return rows


def spring_loads(deck,text):
    tab=tables(text);roots={}
    for name,ref,rot in re.findall(r'\*RIGID BODY,NSET=R([^,]+),REF NODE=(\d+),ROT NODE=(\d+)',deck):roots[name]=(int(ref),int(rot))
    springrows=[]
    pattern=r'\*ELEMENT,TYPE=SPRING2,ELSET=S\d+\n\d+,(\d+),(\d+)\n\*SPRING,ELSET=S\d+\n(\d+),\d+\n([^\n]+)'
    for n,m,dof,k in re.findall(pattern,deck):springrows.append((int(n),int(m),int(dof)-1,float(k)))
    result=[]
    for step in range(1,7):
        u=tab[(step,'U')];row={}
        for side in ['Left','Right']:
            force=0
            for end in ['Front','Rear']:
                ref=roots['y'+side+end][0]
                force+=sum(-k*(u[n][d]-u[m][d]) for n,m,d,k in springrows if n==ref and d==1)
            row[side]=force
        result.append(row)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);args=parser.parse_args();folder=args.folder
    data=json.loads((folder/'results.json').read_text());fine=[r for r in data['results'] if r['meshMm']==50]
    if len(fine)!=18:raise ValueError('Expected both bed lengths, nine poses and two mesh levels')
    summary=[];yloads=[]
    for length in [2600,3150]:
        packet=json.loads((folder/f'y{length}'/'manifest.json').read_text())
        if hashlib.sha256((folder/f'y{length}'/'manifest.json').read_bytes()).hexdigest()!=data['identity']['packetSha256'][str(length)]:raise ValueError('Packet changed after solve')
        rows=[r for r in fine if r['workY']==length];cases=[];relative_modes=[]
        reciprocity=[];foot_margins=[];rigid_fractions=[]
        for i,row in enumerate(rows):
            output=folder/f'solve-y{length}-p{i}-h50';deck=(output/'frame.inp').read_text();text=(output/'frame.dat').read_text()
            first=deck.split('*CLOAD,OP=NEW\n')[1];tip=int(first.split(',')[0]);bed=int(next(s for s in first.splitlines() if s.endswith(',1,-300')).split(',')[0])
            ratios=modal_motion(text,tip,bed);relative_modes.append(dict(pose=row['pose'],frequenciesHz=row['firstModesHz'],relativeToolMotionToMaxObservedMotion=ratios))
            forces=spring_loads(deck,text)
            c=row['cases'];matrix=[[c[j]['toolRelativeMm'][k]/300 for j in range(3)] for k in range(3)]
            error=max(abs(matrix[j][k]-matrix[k][j])/math.sqrt(matrix[j][j]*matrix[k][k]) for j in range(3) for k in range(3))
            if error>.001:raise ValueError('Compliance reciprocity failed')
            reciprocity.append(error)
            for foot in range(6):
                margin=c[3]['supportReactionsN'][str(foot)][2]-math.sqrt(sum(c[j]['supportReactionsN'][str(foot)][2]**2 for j in range(3)))-sum(abs(c[j]['supportReactionsN'][str(foot)][2]) for j in [4,5])
                if margin<=0:raise ValueError('Linear foundation would lift a foot under the combined envelope')
                foot_margins.append(margin)
            for case in c[:3]:
                budget=case['budgetUmPerN'];fraction=(budget.get('rigid',0)+budget.get('coupling',0))/sum(budget.values())
                if fraction>.001:raise ValueError('Artificial rigid members materially affect compliance')
                rigid_fractions.append(fraction)
            for side in ['Left','Right']:
                # A 300 N force of arbitrary direction: norm of the three unit-load responses.
                cut=math.sqrt(sum(forces[j][side]**2 for j in range(3)))
                effective_mass=abs(forces[5][side])/3
                yloads.append(dict(workY=length,pose=row['pose'],side=side,cuttingForceN=cut,effectiveMassKg=effective_mass,
                    cutXYZReactionsN=[forces[j][side] for j in range(3)],accelerationReactionN=forces[5][side],
                    reservedPeakForceN=1.5*(cut+3*effective_mass+15)))
        for axis in range(3):
            worst=max(rows,key=lambda r:r['cases'][axis]['complianceUmPerN'][axis]);case=worst['cases'][axis]
            compliance=case['complianceUmPerN'][axis]
            cases.append(dict(axis='XYZ'[axis],pose=worst['pose'],complianceUmPerN=compliance,stiffnessNPerUm=1/compliance,
                toolRelativeMmAt300N=case['toolRelativeMm'],budgetUmPerN=case['budgetUmPerN']))
        summary.append(dict(workY=length,maxReciprocityError=max(reciprocity),minimumCompressiveFootMarginN=min(foot_margins),maxArtificialRigidComplianceFraction=max(rigid_fractions),worstCutting=cases,firstModeRangeHz=[min(r['firstModesHz'][0] for r in rows),max(r['firstModesHz'][0] for r in rows)],
            modalMotion=relative_modes,largestGravityFrameTwistMm=max(abs(r['cases'][3]['frameTwistMm']) for r in rows),
            maximumForceBalanceRelativeError=max(c['supportForceRelativeError'] for r in rows for c in r['cases']),
            maximumEnergyRelativeError=max(c['energyRelativeError'] or 0 for r in rows for c in r['cases'])))
    # Material/section comparison, with a deliberately optimistic perfectly bonded fill bound.
    b,h,t=100,200,6.3;inside=(b-2*t)*(h-2*t);steel_area=b*h-inside
    ix=(b*h**3-(b-2*t)*(h-2*t)**3)/12;iy=(h*b**3-(h-2*t)*(b-2*t)**3)/12
    fill=dict(steelKgPerM=steel_area*.00785,fillKgPerM=inside*.0024,
        perfectlyBonded30GPaBendingGainStrong=1+30/210*((b-2*t)*(h-2*t)**3/12)/ix,
        perfectlyBonded30GPaBendingGainWeak=1+30/210*((h-2*t)*(b-2*t)**3/12)/iy,
        unbondedStiffnessGain=1,dampingQuantified=False)
    result=dict(status='SR5 screen complete; SR-D3 stiffness and first-mode targets fail; fabrication blocked pending structural redesign',
        basis='Linear B32R RHS and plate-strip model, finite 6-DOF joints, CAD mass/inertia; full-Z; opposing force at a rigid link to nearest bed stock',
        target=dict(xyStiffnessNPerUm=2,firstGantryModeHz=40),variants=summary,fillComparison=fill,
        meshConvergence=data['meshConvergence'],identity=dict(**data['identity'],resultsSha256=hashlib.sha256((folder/'results.json').read_bytes()).hexdigest(),reportSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
        decisions=dict(beam='Retain steel RHS 200x100x6.3 as baseline; same-section aluminium has 0.329 times bending rigidity. Beam enlargement alone cannot fix the dominant plate and guide terms.',
            sidePlates='Reject the 12 mm aluminium beam feet for final design; 20 mm steel is the next candidate (about 14 times strip bending rigidity), subject to SR6 geometry and drive recheck.',
            xCarriage='Reject the open 12 mm X carriage plate. Use a closed box bridge and spaced X rails/blocks; thickness alone does not resolve the guide roll stiffness.',
            zGuides='Replace the single Z block with spaced blocks/rails; obtain supplier rotational stiffness before release.',
            fill='Keep tubes empty. Fill adds substantial moving/fixed mass and cannot fix guide/plate compliance; damping has not been quantified.'),
        limitations=['Assumed guide rotation 10 MN mm/rad per block, translation 100 kN/mm; not supplier qualification.',
            'Bolts are linear springs, weld regions are rigid offsets. No contact, preload separation, wall distortion, stress, buckling or fatigue qualification.',
            'All non-stock part masses are grouped rigidly by carrier; entire routed chains move with their carriers as conservative upper bounds.',
            'Weldment stock densities are scaled to conserve the CAD union mass at overlapping members.',
            'Bed, spindle mount, spindle bearings and tool flexibility excluded; nearest-member bed reaction excludes the table diaphragm and distributed bed attachment.',
            'Large predicted deflections invalidate precision use of linear results; these values reject the baseline rather than predict real cutting accuracy.',
            '20 mm feet, a closed bridge and spaced guides are design candidates; this study does not claim those changes meet SR-D3 or fit the CAD assembly.'])
    (folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(folder/'y-loads.json').write_text(json.dumps(yloads,indent=2)+'\n')
    for row in summary:
        print('Y='+str(row['workY'])+' first modes='+str(row['firstModeRangeHz']))
        for c in row['worstCutting']:print(c['axis'],f'{c["stiffnessNPerUm"]:.6f} N/um',f'{c["complianceUmPerN"]:.3f} um/N',sorted(c['budgetUmPerN'].items(),key=lambda kv:-kv[1])[:4])
    worst=max(yloads,key=lambda r:r['reservedPeakForceN']);print('Worst reserved Y branch:',json.dumps(worst))

if __name__=='__main__':main()
