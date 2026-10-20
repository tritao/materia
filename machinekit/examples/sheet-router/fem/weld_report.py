"""Vector throat stresses from recovered prepared-joint forces and moments."""
import argparse,itertools,json,math
from pathlib import Path
import numpy as np
from qualify_welds import contacts,read_stitches,layout_signature
from weld_effective import effective_stitches
from weld_model import with_foot_welds

STATIC_ALLOWABLE=360/(math.sqrt(3)*.8*1.25)  # S235 lower-bound material basis.
FATIGUE_CYCLES=100000000
# EN 1993-1-9:2005 Figure 7.1: m=3 to 5 million, then m=5
# to the 100-million-cycle cut-off. Keep the variable-amplitude cut-off
# rather than crediting the higher constant-amplitude endurance limit.
FATIGUE_ALLOWABLE=36*(2000000/5000000)**(1/3)*(5000000/FATIGUE_CYCLES)**(1/5)/1.15


def static_stress(fields,layout):
    # Concurrent per-axis 300 N cutting and 3 m/s² XYZ accelerations.
    variable=np.concatenate((fields[:3],fields[4:]),axis=0)
    factors=np.array(list(itertools.product((-1.,1.),repeat=len(variable))))
    combinations=np.einsum('sv,vcw->scw',factors,variable)
    design=1.35*fields[3][None,:,:]+1.5*combinations
    results=[]
    for i,c in enumerate(layout):
        traction=design[:,i,:]@c['transform'].T
        norms=np.linalg.norm(traction.reshape(len(factors),-1,3),axis=2);s,p=np.unravel_index(np.argmax(norms),norms.shape)
        changing=(combinations[:,i,:]@c['transform'].T).reshape(len(factors),-1,3)
        # A conservative component interval encloses every reversal pair.
        ranges=np.max(changing,axis=0)-np.min(changing,axis=0)
        # Fatigue uses the throat traction range, not the static von-Mises
        # multiplier. Applying FAT36 to its full vector magnitude also bounds
        # the less restrictive FAT80 longitudinal-shear interaction.
        fatigue=float(np.max(np.linalg.norm(ranges,axis=1)))
        results.append(dict(group=c['group'],pieces=c['pieces'],staticDesignMPa=float(norms[s,p]),
            detail='RHS sidewall-to-foot fillets with full-size tabbed ends' if c['group'] in ('GFL','GFR') else 'fillets at prepared RHS cope/contact; flush butt edges excluded',
            legMm=c['legMm'],throatMm=c['legMm']/math.sqrt(2),effectiveLengthMm=c['lengthMm'],
            fatigueCategoryMPa=36,longitudinalShearCategoryMPa=80,
            staticAllowableMPa=STATIC_ALLOWABLE,staticPass=bool(norms[s,p]<=STATIC_ALLOWABLE),
            staticControllingLoadSigns=factors[s].tolist(),staticWrenchAtControllingCase=design[s,i,:].tolist(),
            fatigueQuasiStaticRangeMPa=fatigue,fatigueAllowableMPa=FATIGUE_ALLOWABLE,fatigueQuasiStaticPass=fatigue<=FATIGUE_ALLOWABLE))
    return results


def main():
    p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);p.add_argument('--fabrication',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--packets',type=Path,required=True);a=p.parse_args()
    records=[]
    for bed in (2600,3150):
        native=json.loads((a.fabrication/f'y{bed}/native.json').read_text());physical=read_stitches(a.fabrication/f'y{bed}');packet=json.loads((a.packets/f'y{bed}/manifest.json').read_text())
        for path in sorted(a.results.glob(f'y{bed}-*/reactions.json')):
            result=json.loads(path.read_text());joined,stitches=with_foot_welds(packet,native,physical,result['pose']);stitches,_=effective_stitches(joined,stitches);layout=contacts(joined,stitches)
            if [(c['group'],list(c['pieces'])) for c in layout]!=[(c['group'],c['pieces']) for c in result['contactOrder']]:raise ValueError('Reaction/contact ordering mismatch: '+str(path))
            if result.get('jointLayoutSha256')!=layout_signature(layout):raise ValueError('Reaction/weld detail signature mismatch: '+str(path))
            rows=static_stress(np.array(result['staticWrenches']),layout)
            records.append(dict(workY=bed,pose=result['pose'],path=str(path),contacts=rows))
    a.output.write_text(json.dumps(dict(status='Static and quasi-static fatigue envelope; dynamic qualification pending',basis=dict(fuMPa=360,betaW=.8,gammaM2=1.25,gravityFactor=1.35,variableFactor=1.5,
        fatigueCategoryMPa=36,longitudinalShearCategoryMPa=80,fatigueCycles=FATIGUE_CYCLES,fatigueGammaMf=1.15,legMm='5 nominal; 6 at middle post/stringer; 8 at beam feet; see each contact',fatigueRange='traction-vector interval diameter against FAT36; conservative for the FAT36 normal/transverse and FAT80 longitudinal shear interaction; EN 1993-1-9:2005 Figure 7.1 m=3 to 5 million then m=5 to the 100-million-cycle cut-off'),poses=records),indent=2)+'\n')
    for bed in (2600,3150):
        rows=[c for r in records if r['workY']==bed for c in r['contacts']]
        if rows:print(bed,'poses',sum(r['workY']==bed for r in records),'static failed',sorted({r['group'] for r in rows if not r['staticPass']}),'fatigue preliminary failed',sorted({r['group'] for r in rows if not r['fatigueQuasiStaticPass']}),'max MPa',max(r['staticDesignMPa'] for r in rows))

if __name__=='__main__':main()
