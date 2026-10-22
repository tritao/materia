"""Collate brace-study evidence without turning an incomplete screen into release."""
import argparse,csv,json
from pathlib import Path


def write_csv(path, rows):
    if not rows:return
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n");writer.writeheader();writer.writerows(rows)


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    modes=[];welds=[];dynamic_welds=[];summary=[]
    for bed in (2600,3150):
        for variant in sorted((a.root/'macro-serial'/f'y{bed}').iterdir()):
            if not variant.is_dir():continue
            cases=[json.loads(f.read_text()) for folder in (variant,a.root/'macro-fine'/f'y{bed}'/variant.name) for f in folder.glob('*/summary.json')]
            if not cases:continue
            changes=json.loads((variant/'changes.json').read_text());row=dict(bedMm=bed,variant=variant.name,macroPoses=len({(c['pose']['x'],c['pose']['y']) for c in cases}),macroCases=len(cases),fixedMassSavedKg=changes['fixedMassSavedKg'],movingMassSavedKg=changes['movingMassSavedKg'],weldLengthSavedM=changes['weldLengthSavedMm']/1000)
            for i,axis in enumerate('XY'):
                worst=min(cases,key=lambda c:c['xyStiffnessNPerUm'][i]);row['min'+axis+'NPerUm']=worst['xyStiffnessNPerUm'][i];row['pose'+axis]=str(worst['pose'])
            worst=min(cases,key=lambda c:c['modes'][0]['frequencyHz']);row['minModeHz']=worst['modes'][0]['frequencyHz'];row['modePose']=str(worst['pose'])
            for case in cases:
                for i,m in enumerate(case['modes'][:6],1):
                    bodies=m['bodies'];shape='; '.join(k+': translation '+v['dominantDirection']+', rotation '+ 'XYZ'[max(range(3),key=lambda i:abs(v['rotationRadPerMmMaxMotion'][i]))] for k,v in bodies.items() if k in ('frame','gantry','x','z'))
                    modes.append(dict(bedMm=bed,variant=variant.name,pose=str(case['pose']),meshMm=case['meshMm'],mode=i,frequencyHz=m['frequencyHz'],shape=shape,gantryKineticFraction=m['gantryKineticFraction'],bodyDiagnostics=json.dumps(bodies,separators=(',',':')),**{'energy_'+k:v for k,v in m['strainEnergyShare'].items()}))
            prepared=a.root/'prepared-serial'/f'y{bed}'/variant.name
            native=[json.loads(f.read_text()) for folder in (prepared,a.root/'prepared-fine'/f'y{bed}'/variant.name,a.root/'prepared-corners'/f'y{bed}'/variant.name,a.root/'prepared-extra'/f'y{bed}'/variant.name) for f in folder.glob('*/summary.json')]
            row['preparedCases']=len(native);row['preparedPoses']=len({(c['pose']['x'],c['pose']['y']) for c in native})
            for c in native:
                for w in c['weldStatic']:welds.append(dict(bedMm=bed,variant=variant.name,pose=str(c['pose']),meshMm=c['meshMm'],**w))
            if native:
                row['weldGroupCount']=len({w['group'] for c in native for w in c['weldStatic']});row['contactPairCount']=len({(w['group'],tuple(w['pieces'])) for c in native for w in c['weldStatic']})
                row['quasiStaticFatigueMaxMPa']=max(c['maxFatigueMPa'] for c in native);row['quasiStaticFatiguePass']=all(c['fatiguePass'] and c['staticPass'] for c in native)
            motion=[]
            for directory in ('motion-coarse','motion-dense'):
                for f in (a.root/directory).glob(f'y{bed}-{variant.name}-x*.json'):
                    m=json.loads(f.read_text())
                    if m['variant']==variant.name:motion.append(m)
            row['motionScreens']=len(motion)
            for metric in ('peakContourErrorMm','movingHandoverPeakMm','residualAfterStopMm'):
                values=[(h['worst'][metric]['value'],m,h) for m in motion for h in m['histories']]
                if values:
                    value,m,h=max(values,key=lambda x:x[0]);row[metric]=value;row[metric+'Case']=str((m['pose'],h['history'],h['worst'][metric]))
            for f in (a.root/'weld-motion').glob(f'y{bed}-{variant.name}-x*.json'):
                d=json.loads(f.read_text())
                if d['variant']!=variant.name:continue
                for h in d.get('fatigue',[]):
                    for c in h['contacts']:dynamic_welds.append(dict(bedMm=bed,variant=variant.name,pose=str(d['pose']),meshMm=d['meshMm'],halfStep=d['signature']['half'],dt=next(v['dt'] for v in d['histories'] if v['history']==h['history']),history=h['history'],frequencyScale=h['frequencyScale'],dampingRatio=h['dampingRatio'],**c))
            selected=[w for w in dynamic_welds if w['bedMm']==bed and w['variant']==variant.name]
            if selected:
                row['dynamicFatigueMaxMPa']=max(w['fatigueRangeMPa'] for w in selected);row['dynamicFatiguePass']=all(w['fatiguePass'] and w['staticPass'] for w in selected)
            row['release']='NOT RELEASED: rack-support joint rerun and explicit geometry approval required'
            summary.append(row)
    write_csv(a.output/'variants.csv',summary);write_csv(a.output/'lowest-six-modes.csv',modes)
    # Nested per-group fields stay JSON as well as CSV; never omit a failing joint.
    write_csv(a.output/'weld-groups.csv',welds);write_csv(a.output/'weld-dynamic-groups.csv',dynamic_welds)
    condensed=a.output/'condensed';condensed.mkdir(exist_ok=True)
    write_csv(condensed/'variants.csv',summary)
    governing=[]
    for row in summary:
        candidates=[m for m in modes if m['bedMm']==row['bedMm'] and m['variant']==row['variant']]
        first=min((m for m in candidates if m['mode']==1),key=lambda m:m['frequencyHz'])
        governing.extend({k:v for k,v in m.items() if k!='bodyDiagnostics'} for m in candidates if m['pose']==first['pose'] and m['meshMm']==first['meshMm'])
    write_csv(condensed/'governing-lowest-six.csv',governing)
    groups={}
    for w in welds:
        key=(w['bedMm'],w['variant'],w['group'],tuple(w['pieces']))
        row=groups.setdefault(key,dict(bedMm=w['bedMm'],variant=w['variant'],group=w['group'],pieces=str(w['pieces']),detail=w['detail'],legMm=w['legMm'],throatMm=w['throatMm'],effectiveLengthMm=w['effectiveLengthMm'],fatigueCategoryMPa=w['fatigueCategoryMPa'],fatigueAllowableMPa=w['fatigueAllowableMPa'],staticAllowableMPa=w['staticAllowableMPa'],maxQuasiStaticRangeMPa=-1,maxStaticDesignMPa=-1,staticPass=True,quasiStaticPass=True))
        if w['fatigueQuasiStaticRangeMPa']>row['maxQuasiStaticRangeMPa']:row.update(maxQuasiStaticRangeMPa=w['fatigueQuasiStaticRangeMPa'],quasiStaticPose=w['pose'],quasiStaticMeshMm=w['meshMm'])
        if w['staticDesignMPa']>row['maxStaticDesignMPa']:row.update(maxStaticDesignMPa=w['staticDesignMPa'],staticPose=w['pose'],staticMeshMm=w['meshMm'])
        row['staticPass'] &= w['staticPass'];row['quasiStaticPass'] &= w['fatigueQuasiStaticPass']
    for w in dynamic_welds:
        key=(w['bedMm'],w['variant'],w['group'],tuple(w['pieces']))
        if key not in groups:raise ValueError('Dynamic joint missing from static qualification')
        row=groups[key]
        if w['fatigueRangeMPa']>row.get('maxDynamicRangeMPa',-1):row.update(maxDynamicRangeMPa=w['fatigueRangeMPa'],dynamicPose=w['pose'],dynamicMeshMm=w['meshMm'],dynamicHalfStep=w['halfStep'],dynamicDt=w['dt'],dynamicHistory=w['history'],dynamicFrequencyScale=w['frequencyScale'],dynamicDampingRatio=w['dampingRatio'])
        row['dynamicPass']=row.get('dynamicPass',True) and w['fatiguePass'] and w['staticPass']
    write_csv(condensed/'weld-group-envelopes.csv',list(groups.values()))
    (a.output/'results.json').write_text(json.dumps(dict(variants=summary,modes=modes,weldGroups=welds,dynamicWeldGroups=dynamic_welds),indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
