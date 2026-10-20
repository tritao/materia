"""Produce the reproducible modal, mass and sensitivity tables for the router review."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from modes import GROUPS


LABELS={'frame':'frame','bed':'table','gantry':'beam','footLeft':'left foot','footRight':'right foot','x':'X carriage','z':'Z ram','zrails':'Z rails','spindle':'spindle'}


def shape(mode):
    names=[]
    for key in ('frame','bed','gantry','x','z','spindle'):
        b=mode['bodies'][key]
        if math.sqrt(sum(v*v for v in b['rmsXYZNormalized']))>.12:names.append(LABELS[key]+' '+b['dominantDirection'])
    beam=mode['bodies']['gantry'];rotation=beam['rotationRadPerMmMaxMotion'];axis=max(range(3),key=lambda i:abs(rotation[i]))
    return ', '.join(names)+f"; beam R{'XYZ'[axis]} {1000*rotation[axis]:+.3f} mrad/mm, bend residual {beam['rigidFitResidualNormalized']:.3f}"


def energy_table(rows):
    text=['| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |',
        '|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for m in rows:
        cells=' | '.join(f"{100*m['strainEnergyFractions'][g]:.1f}" for g in GROUPS)
        text.append(f"| {m['mode']} | {m['frequencyHz']:.3f} | {100*m['gantryFraction']:.1f} | {shape(m)} | {cells} |")
    return '\n'.join(text)


def main():
    p=argparse.ArgumentParser();p.add_argument('--packets',required=True,type=Path);p.add_argument('--extended',required=True,type=Path);p.add_argument('--sensitivities',required=True,type=Path);p.add_argument('--baseline',required=True,type=Path);a=p.parse_args()
    summary=json.loads((a.packets/'summary.json').read_text());study=json.loads((a.packets/'results.json').read_text());extended=json.loads(a.extended.read_text());sens=json.loads(a.sensitivities.read_text())
    text=['## Anchored modal diagnosis (2026-10-20)','',
        'Full 100/50 mm, nine-pose, two-bed rerun of the f47ed13f4 CAD. Six anchored supports each use 50,000 N/mm in XYZ, with no rotational clamp. Foundation values are assumptions, not anchor/slab qualification. All masses and the original 428 mm tool lever are retained.', '',
        '| Bed Y | Worst X N/µm | Worst Y N/µm | Lowest global Hz | Lowest gantry Hz |',
        '|---|---:|---:|---:|---:|']
    for v in summary['variants']:
        text.append(f"| {v['workY']} | {v['worstCutting'][0]['stiffnessNPerUm']:.3f} | {v['worstCutting'][1]['stiffnessNPerUm']:.3f} | {v['lowestGlobalMode']['frequencyHz']:.3f} ({v['lowestGlobalMode']['dominantDirection']}) | {v['lowestGantryDominantMode']['frequencyHz']:.3f} ({v['lowestGantryDominantMode']['dominantDirection']}) |")
    text+=['','Energy columns below are percentages of **modal strain energy**: native CalculiX ELSE totals plus ½kΔu² for every spring. KE is moving-mass kinetic-energy share; ≥50% defines the reported gantry class. Y guides credit block springs only; Y rail bending is not resolved. X includes plate, guide and drive springs; Z includes ram, ribs, rails, guide/screw and saddle joints. Table and artificial rigid offsets are shown so each row accounts for 100%.', '',
        'Shapes use global XYZ. Each body’s direction is its RMS displacement direction. Beam rotation is a least-squares rigid-body fit, normalized to 1 mm maximum physical-node motion; bend residual is the remaining normalized RMS displacement. Eigenvector sign is arbitrary. These observations distinguish translation, rotation and beam bending without pretending the entire gantry is rigid.','']
    profiles=[]
    for v in summary['variants']:
        length=v['workY'];critical=v['lowestGlobalMode']['pose'];globalrow=next(r for r in study['results'] if r['meshMm']==50 and r['workY']==length and r['pose']==critical)
        more=next(r for r in extended['rows'] if r['workY']==length);fine=more['results'][1]
        text += [f"### {length} mm bed: first six global modes",'',f"Pose X={critical['x']:g}, Y={critical['y']:g}, full Z. Lowest frame-dominant mode is {v['lowestFrameDominantMode']['frequencyHz']:.3f} Hz. These are the first six modes at the lowest-global-frequency pose, not six copies of one branch at different poses.",'',energy_table(globalrow['modalParticipation'][:6]),'',f'### {length} mm bed: first six gantry-dominant modes','',f"Pose X={fine['pose']['x']:g}, Y={fine['pose']['y']:g}, full Z, chosen for the lowest gantry frequency. Forty modes were extracted on both meshes to resolve six gantry-class modes; all forty frequencies converge within {100*more['allFortyModesRelativeChange']:.3f}%.",'',energy_table(fine['firstSixGantry']),'']
        profiles.append(dict(workY=length,globalPose=critical,globalModes=globalrow['modalParticipation'][:6],gantryPose=fine['pose'],gantryModes=fine['firstSixGantry']))
    text+=['### Single-pose sensitivities','',f"{sens['criterion']}. Bed {sens['workY']:g}, X={sens['pose']['x']:g}, Y={sens['pose']['y']:g}, full Z, 50 mm mesh. These are analytical property/layout variants at fixed mounting interfaces, not collision-checked replacement CAD. Y-spacing includes longer steel feet and rails; HGR30 includes catalogue mass increments and ZA radial rigidity. Lateral guide stiffness remains assumed. Changing plate thickness requires reworking the actual mounting stack.",'',
        '| Variant (ranked by tracked gantry frequency gain) | ΔX N/µm | ΔY N/µm | Δfirst gantry Hz | Δtracked branch Hz | Motion-signature MAC | Δmoving Y kg |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in sorted(sens['rows'][1:],key=lambda r:-r['deltaTrackedGantryHz']):
        text.append(f"| {r['name']} | {r['deltaXYStiffnessNPerUm'][0]:+.3f} | {r['deltaXYStiffnessNPerUm'][1]:+.3f} | {r['deltaFirstGantryHz']:+.3f} | {r['deltaTrackedGantryHz']:+.3f} | {r['bodyMotionSignatureMAC']:.3f} | {r['deltaMovingMassKg']:+.3f} |")
    text += ['', 'Tracked-branch MAC compares normalized body translations and rotations (200 mm rotation scale), not a consistent-mass matrix MAC. Both the classified first mode and matched branch are reported so threshold swaps are visible. Weld ×0.1 changes translation springs from 10¹⁰ to 10⁹ N/mm and rotation from 10¹³ to 10¹² N·mm/rad; these are quasi-rigid numerical connectors, not qualified weld flexibility. Free supports keep six vertical springs but only X at foot 0 and Y at feet 0/3; friction/no-slip is assumed.', '',
        'HGR30 uses [HIWIN HGH25CA/HGH30CA](https://www.hiwin.com/wp-content/uploads/Linear_Guideway-E.pdf): radial ZA 539→618 N/µm, block mass increment 0.51→0.88 kg and rail increment 3.21→4.47 kg/m. Incremental mass is added to generic CAD masses; rail bending and supplier mounting-height changes are not resolved.', '', '### Added moving mass','', '| Member | Previous kg | Revised kg | Added kg |','|---|---:|---:|---:|']
    old=json.loads((a.baseline/'y3150/manifest.json').read_text());new=json.loads((a.packets/'y3150/manifest.json').read_text())
    before={r['id']:r['kg'] for r in next(x for x in old['axes'] if x['axis']=='y')['movingMembers']};after={r['id']:r['kg'] for r in next(x for x in new['axes'] if x['axis']=='y')['movingMembers']}
    differences=[dict(id=k,beforeKg=before.get(k,0),afterKg=after.get(k,0),deltaKg=after.get(k,0)-before.get(k,0)) for k in before.keys()|after.keys() if abs(after.get(k,0)-before.get(k,0))>1e-6]
    for r in sorted(differences,key=lambda r:-r['deltaKg']):text.append(f"| {r['id']} | {r['beforeKg']:.3f} | {r['afterKg']:.3f} | {r['deltaKg']:+.3f} |")
    text.append(f"| **Total moving Y** | **{sum(before.values()):.3f}** | **{sum(after.values()):.3f}** | **{sum(after.values())-sum(before.values()):+.3f}** |")
    text+=['','Fixed base braces are not part of moving Y mass. The beam mass also increases slightly because thicker feet change the generated span. Part-level inertia and material mass are changed in each sensitivity, not just a modal denominator.', '',
        'Artifacts: `build/sheet-router-modal/anchored/{results,summary}.json`, `extended/extended-modes.json`, `sensitivities/sensitivities.json`, and `diagnosis.json`. The full-run solve-time sources, packet hashes and solver hash remain with the results. Extended/sensitivity runs record their source hashes separately. No fabrication release.','']
    output=dict(variants=summary['variants'],profiles=profiles,sensitivities=sens,massDifferences=differences,baselineManifestSha256=hashlib.sha256((a.baseline/'y3150/manifest.json').read_bytes()).hexdigest())
    (a.packets.parent/'diagnosis.json').write_text(json.dumps(output,indent=2)+'\n')
    print('\n'.join(text))


if __name__=='__main__':main()
