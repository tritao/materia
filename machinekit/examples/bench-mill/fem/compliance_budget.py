"""Catalogue/assumed lumped springs in series with upper-bound frame compliance."""

import math

from frame_solver import cross, solve_small
from frame_mesh import xyz
from compliance import reduce as reduce_compliance


def network(springs, point, active=range(6)):
    """Rigid body spring network; each scalar spring stores (group, point, axis, N/mm)."""
    origin=tuple(sum(s[1][i] for s in springs)/len(springs) for i in range(3))
    rows=[]
    for group,location,direction,k in springs:
        lever=tuple(location[i]-origin[i] for i in range(3))
        rows.append((group,list(direction)+list(cross(lever,direction)),k))
    stiffness=[[sum(k*a[i]*a[j] for _,a,k in rows) for j in active] for i in active]
    solutions=[]
    for axis in range(3):
        force=[float(i==axis) for i in range(3)]
        wrench=force+list(cross(tuple(point[i]-origin[i] for i in range(3)),force))
        if any(abs(wrench[i])>1e-10 for i in set(range(6))-set(active)):
            raise ValueError("Unsupported moment in lumped network")
        small=solve_small(stiffness,[wrench[i] for i in active])
        solution=[0.0]*6
        for i,v in zip(active,small): solution[i]=v
        solutions.append(solution)
    groups={}
    for group,row,k in rows:
        strains=[sum(a*b for a,b in zip(row,q)) for q in solutions]
        matrix=groups.setdefault(group,[[0.0]*3 for _ in range(3)])
        for i in range(3):
            for j in range(3): matrix[i][j]+=1000*k*strains[i]*strains[j]
    return groups,solutions,rows


def tool_case_packet(packet, inputs, case_id):
    """Change analysis contact only; physical CAD/mass records remain unchanged."""
    import copy
    case = next(c for c in inputs['toolCases'] if c['id'] == case_id)
    result = copy.deepcopy(packet)
    holder = next(p for p in packet['parts'] if p['id'] == 'spindle/holder')
    gauge = holder['boundsMm']['minZ'] + inputs['toolHolder']['neckLengthMm']
    point = result['datums']['toolTipMm']
    point['z'] = gauge - case['holderLengthMm'] - case['stickOutMm']
    result['analysisToolCase'] = case
    result['analysisContactNote'] = 'Rigid workpiece extends to the selected contact; CAD tool masses unchanged in this static-only comparison.'
    return result


def budget(packet, study, inputs, solution):
    for name,value in inputs.items():
        if isinstance(value,dict) and not any('source' in k.lower() for k in value):
            raise ValueError(f"Budget input has no source: {name}")
    groups={}
    details={}
    parts={p['id']:p for p in packet['parts']}
    point=xyz(packet['datums']['toolTipMm'])
    def add(group,matrix):
        target=groups.setdefault(group,[[0.0]*3 for _ in range(3)])
        for i in range(3):
            for j in range(3): target[i][j]+=matrix[i][j]
    # FE element energies give the exact directional diagonal decomposition.
    # Off-diagonals remain in the frame matrix, rather than inventing group cross-energies.
    frame=solution['complianceUmPerN']
    add('frame-fem',frame)
    for axis in 'xyz':
        index='xyz'.index(axis)
        case=next(c for c in solution['cases'] if c['case']==axis)
        details[axis]={group:frame[index][index]*share for group,share in case['complianceShares'].items()}
    guide=inputs['guides'];drive=inputs['drives'];belt=inputs['beltX']
    unit=[[float(i==j) for i in range(3)] for j in range(3)]
    for axis in 'xyz':
        i='xyz'.index(axis)
        prefix='zAxis/' if axis=='z' else 'xyStage/'
        nut=parts[prefix+('nut' if axis=='z' else 'nut'+axis.upper())]
        bearing=parts[prefix+('fixedBearing' if axis=='z' else 'fixedBearing'+axis.upper())]
        nut_point=xyz(nut['centreOfMassMm'])
        length=abs(nut_point[i]-xyz(bearing['centreOfMassMm'])[i])
        screw=parts[prefix+("screw" if axis=="z" else "screw"+axis.upper())]
        shaft_e=study["materials"][screw["materialId"]]["youngsPa"]*1e-6
        shaft=length/(shaft_e*math.pi*drive["shaftRootDiameterMm"]**2/4)
        axial={'screw-'+axis:shaft,'nut-'+axis:1/(1000*drive['nutNPerUm']),
               'support-bearing-'+axis:1/(1000*drive['supportBearingNPerUm'])}
        if axis=='x':
            motor=xyz(parts['xyStage/motorPulleyX']['centreOfMassMm'])
            pulley=xyz(parts['xyStage/screwPulleyX']['centreOfMassMm'])
            span=math.dist(motor,pulley)
            radius=belt['teeth']*belt['pitchMm']/(2*math.pi)
            torsion=2*belt['axialRigidityN']*radius**2/span
            reflected=torsion*(2*math.pi/drive['leadMm'])**2
            axial['belt-x']=1/reflected
            details['beltReflection']={'spanMm':span,'pitchRadiusMm':radius,'torsionalNmmPerRad':torsion,'reflectedAxialNPerMm':reflected}
        k=1/sum(axial.values())
        springs=[]
        for block in packet['guides']:
            if block['axis']==axis:
                for j in range(3):
                    if j!=i: springs.append(('guides-'+axis,xyz(block['topMm']),unit[j],guide['blockRadialNPerUm']*1000))
        springs.append(('drive-'+axis,nut_point,unit[i],k))
        components,q,rows=network(springs,point)
        add('guides-'+axis,components['guides-'+axis])
        _,row,_=rows[-1]
        loads=[k*sum(a*b for a,b in zip(row,v)) for v in q]
        for group,c in axial.items(): add(group,[[1000*c*loads[a]*loads[b] for b in range(3)] for a in range(3)])
        details['drive-'+axis]={'fixedToNutLengthMm':length,'axialSeriesNPerMm':k}
    clamp=inputs['columnClamp'];springs=[]
    for part in packet['parts']:
        if part['id'].startswith('frame/base/column'):
            for i in range(3):
                k=clamp['normalNPerUmPerPad'] if i==2 else clamp['tangentialNPerUmPerPad']
                springs.append(('column-clamp-contact',xyz(part['centreOfMassMm']),unit[i],k*1000))
    for group,matrix in network(springs,point)[0].items(): add(group,matrix)
    bearing=inputs['spindleBearings']
    gauge=parts['spindle/holder']['boundsMm']['minZ']+inputs['toolHolder']['neckLengthMm']
    springs=[]
    for offset in bearing['stationOffsetsAboveGaugeMm']:
        for i in (0,1): springs.append(('spindle-bearings',(point[0],point[1],gauge+offset),unit[i],bearing['radialPairNPerUm']*500))
    springs.append(('spindle-bearings',(point[0],point[1],gauge+bearing['stationOffsetsAboveGaugeMm'][0]),unit[2],bearing['axialPairNPerUm']*1000))
    for group,matrix in network(springs,point,range(5))[0].items(): add(group,matrix)
    holder=inputs['toolHolder'];tool=inputs['tool'];case=packet.get('analysisToolCase')
    holder_e=study['materials'][parts['spindle/holder']['materialId']]['youngsPa']*1e-6
    tool_e=study['materials'][parts['spindle/tool']['materialId']]['youngsPa']*1e-6
    bounds=parts['spindle/tool']['boundsMm']
    stick=bounds['maxZ']-bounds['minZ'];diameter=(bounds['maxX']-bounds['minX'])
    if case:
        stick,diameter=case['stickOutMm'],case['diameterMm']
        tool_e=inputs['toolMaterials'][case['material']]['youngsPa']*1e-6
    area=math.pi*diameter**2/4 if diameter else 1
    fraction=case['inertiaFraction'] if case else tool['inertiaFraction']
    inertia=math.pi*diameter**4/64*fraction if diameter else 1
    tool_bend=1000*stick**3/(3*tool_e*inertia)
    tool_axial=1000*stick/(tool_e*area)
    add('tool',[[tool_bend,0,0],[0,tool_bend,0],[0,0,tool_axial]])
    length=case['holderLengthMm'] if case else holder['neckLengthMm'];bore=diameter if case else holder['boreDiameterMm'];inertia=math.pi*(holder['outerDiameterMm']**4-bore**4)/64
    area=math.pi*(holder['outerDiameterMm']**2-bore**2)/4
    holder_bend=1000*((length+stick)**3-stick**3)/(3*holder_e*inertia)
    contact=1/holder['taperTranslationNPerUm']+1000*(length+stick)**2/holder['taperRotationNmmPerRad']
    axial=1000*length/(holder_e*area)+1/holder['taperTranslationNPerUm']
    if not case or case['id']!='spindle-nose':
        add('tool-holder',[[holder_bend+contact,0,0],[0,holder_bend+contact,0],[0,0,axial]])
    total=[[sum(matrix[i][j] for matrix in groups.values()) for j in range(3)] for i in range(3)]
    shares={axis:{group:matrix[i][i]/total[i][i] for group,matrix in groups.items()} for i,axis in enumerate('xyz')}
    casting_shares={axis:sum(details[axis][g] for g in ('BASE_CASTING','COLUMN_CASTING'))/total[i][i] for i,axis in enumerate('xyz')}
    machine=[[total[i][j]-groups['tool'][i][j] for j in range(3)] for i in range(3)]
    machine_shares={axis:sum(details[axis][g] for g in ('BASE_CASTING','COLUMN_CASTING'))/machine[i][i] for i,axis in enumerate('xyz')}
    return {'castingMachineLoopShares':machine_shares,'machineLoopExcludingTool':reduce_compliance(machine,study['cuttingForceN']),'status':inputs['status'],'inputData':inputs,'componentComplianceUmPerN':groups,'directionalShares':shares,
            'frameEnergyDiagonalUmPerN':details,'castingLoopShares':casting_shares,
            'geometryEligible':any(v>=study['frameShareTarget']['maximumFraction'] for v in machine_shares.values()) if case and case['decisionCase'] else None,
            'toolCase':case,'decisionBasis':'Machine loop at spindle nose and short test bar; job-dependent cutters are context only',
            'frameShareTarget':study['frameShareTarget'],'toolStickOutMm':stick,'toolDiameterMm':diameter,
            'supportAccounting':'Support springs and foot-floor deformation remain inside frame-fem and are split by FE energy; no duplicate series spring.',
            **reduce_compliance(total,study['cuttingForceN'])}
