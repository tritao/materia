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
    if len(rows) not in (6,20):raise ValueError('Missing modal observation tables')
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
            references=[roots['YdriveMoving'+side][0]] if 'YdriveMoving'+side in roots else [roots['y'+side+end][0] for end in ['Front','Rear']]
            for ref in references:
                force+=sum(-k*(u[n][d]-u[m][d]) for n,m,d,k in springrows if n==ref and d==1)
            row[side]=force
        result.append(row)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);args=parser.parse_args();folder=args.folder
    data=json.loads((folder/'results.json').read_text());fine=[r for r in data['results'] if r['meshMm']==50]
    if len(fine)!=18:raise ValueError('Expected both bed lengths, nine poses and two mesh levels')
    # Gantry-dominant modes can lie above the first six global frame modes.
    # Qualify all twenty reported eigenvalues before using that classification.
    modal_convergence=[]
    for row in fine:
        coarse=next(r for r in data['results'] if r['meshMm']==100 and r['workY']==row['workY'] and r['pose']==row['pose'])
        if len(row['firstModesHz'])!=20 or len(coarse['firstModesHz'])!=20:raise ValueError('Expected twenty modes in updated report')
        error=max(abs(a/b-1) for a,b in zip(coarse['firstModesHz'],row['firstModesHz']))
        if error>.05:raise ValueError('Twenty-mode mesh convergence exceeds 5%')
        modal_convergence.append(dict(workY=row['workY'],pose=row['pose'],allTwentyModesRelativeChange=error))
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
        dominant=[next((f for f,m in zip(r['firstModesHz'],r.get('modalParticipation',[])) if m['gantryFraction']>=.5),None) for r in rows]
        summary.append(dict(workY=length,firstGantryDominantHz=dominant,gantryModeCriterion='At least 50% of estimated modal kinetic energy in moving gantry members; native stock uses Simpson translational mass quadrature',maxReciprocityError=max(reciprocity),minimumCompressiveFootMarginN=min(foot_margins),maxArtificialRigidComplianceFraction=max(rigid_fractions),worstCutting=cases,firstModeRangeHz=[min(r['firstModesHz'][0] for r in rows),max(r['firstModesHz'][0] for r in rows)],
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
    updated=bool(packet.get('guideLayout'))
    passing=all(c['stiffnessNPerUm']>=2 for row in summary for c in row['worstCutting'][:2]) and all(f is not None and f>=40 for row in summary for f in row['firstGantryDominantHz'])
    result=dict(status=('Updated guide screen passes reduced-model targets; supplier and fabrication qualification remains' if passing else 'Structural screen complete; SR-D3 targets still require work; no fabrication release'),
        basis='Linear B32R RHS and plate-strip model, finite 6-DOF joints, CAD mass/inertia; full-Z; opposing force at a rigid link to nearest bed stock',
        target=dict(xyStiffnessNPerUm=2,firstGantryModeHz=40),variants=summary,fillComparison=fill,
        meshConvergence=data['meshConvergence'],twentyModeConvergence=modal_convergence,identity=dict(**data['identity'],resultsSha256=hashlib.sha256((folder/'results.json').read_bytes()).hexdigest(),reportSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
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
    if updated:
        result['limitations']=[text for text in result['limitations'] if 'nearest-member bed reaction' not in text]
        result['limitations'].append('Rigid table is supported at six bed-stock seats through translational mounting springs; table elasticity still requires qualification. Cabinet devices and blower are independently floor-supported.')
        result['limitations'].append('Other fixed masses attach at nearest stock on a fixed 50 mm analysis grid; distributed mass shape remains approximate. Global frame modes and gantry-dominant modes are reported separately.')
        result['basis']='Spatial per-block translation springs and plate grillages, actual CAD connectors, moving Z rails; no assumed guide rotational spring'
        result['decisions'].update(beam='Retain steel RHS 200x100x6.3, 200 mm side vertical.',
            xCarriage='25 mm flat aluminium plate supported on four X blocks; no closed box selected.',
            zGuides='Four fixed Z blocks on X carriage, two rails on moving plate; 200 mm travel.',
            sidePlates='20 mm steel gantry feet after the guide-only trial identified their bending contribution.')
        yaxis=next(a for a in packet['axes'] if a['axis']=='y')
        stiffness=yaxis['compliance']['stiffnessNPerM'];hinge=yaxis['compliance']['hingeMPerN'];mass=yaxis['movingMassKg']
        result['yDriveSensitivity']=dict(workY=packet['workY'],movingMassKg=mass,combinedStiffnessNPerM=2*stiffness,hingeComplianceFraction=hinge*stiffness,idealDriveOnlyHz=math.sqrt(2*stiffness/mass)/(2*math.pi),halfHingeStiffnessHz=math.sqrt(2/(1/stiffness+hinge)/mass)/(2*math.pi),tenPercentHeavierHz=math.sqrt(2*stiffness/(1.1*mass))/(2*math.pi))
        result['decisions']['racks']='Use supplier feed-force rating and standard 19 mm rack candidate; retain 20 mm conservative CAD envelope pending exact supplier geometry. No 25 mm upsizing.'
        result['limitations'][0]='Per-block radial stiffness: HG20C ZA 460 N/um, HG25C ZA 539 N/um. Lateral stiffness remains assumed at 100 N/um; no guide rotational spring.'
        result['limitations']=[t for t in result['limitations'] if not t.startswith('20 mm feet, a closed bridge') and not t.startswith('Large predicted deflections')]
        result['limitations'].append('Direct spindle saddle is represented by a rigid body on four translational mounting springs; clamp, bearing and tool flexibility remain excluded.')
        result['limitations'].append('Moving-rail reinforcement uses 60% gross rectangular section and discrete assumed mounting stiffness; plate grillage omits Poisson coupling. Retain these as sensitivity inputs.')
    (folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(folder/'y-loads.json').write_text(json.dumps(yloads,indent=2)+'\n')
    for row in summary:
        print('Y='+str(row['workY'])+' first modes='+str(row['firstModeRangeHz']))
        for c in row['worstCutting']:print(c['axis'],f'{c["stiffnessNPerUm"]:.6f} N/um',f'{c["complianceUmPerN"]:.3f} um/N',sorted(c['budgetUmPerN'].items(),key=lambda kv:-kv[1])[:4])
    worst=max(yloads,key=lambda r:r['reservedPeakForceN']);print('Worst reserved Y branch:',json.dumps(worst))

if __name__=='__main__':main()
