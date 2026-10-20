"""Publish reaction-based SR6 weld results; never sign a fabrication release."""
import argparse,csv,json,math
from pathlib import Path
from weld_report import FATIGUE_ALLOWABLE,FATIGUE_CYCLES,STATIC_ALLOWABLE
from weld_model import with_foot_welds


def main():
    p=argparse.ArgumentParser()
    for name in ('static','coarse','fabrication','dynamic','output','previous','packets'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();static=json.loads(a.static.read_text());coarse=json.loads(a.coarse.read_text())
    if static['basis']['fatigueCycles']!=100000000:raise ValueError('Duty is not the approved 100 million reversals')
    a.output.mkdir(parents=True,exist_ok=True);any_fail=False
    for bed in (2600,3150):
        folder=a.fabrication/f'y{bed}';details=json.loads((folder/'weld-detail.json').read_text())
        native=json.loads((folder/'native.json').read_text());dynamic=json.loads((a.dynamic/f'dynamic-y{bed}.json').read_text())
        if native['seamDiagnostics'] or native['serviceAccess']['status']!='service access checks pass':raise ValueError('Prepared geometry/service packet does not pass')
        dynamic_fine=json.loads((a.dynamic/f'dynamic-y{bed}-dt250us.json').read_text())
        poses=[r for r in static['poses'] if r['workY']==bed]
        if len(poses)!=9 or len({tuple(r['pose'][k] for k in ('x','y','z')) for r in poses})!=9:raise ValueError('Incomplete static pose envelope')
        coarse_poses=[r for r in coarse['poses'] if r['workY']==bed]
        if len(coarse_poses)<3:raise ValueError('Missing controlling/modal/middle coarse mesh checks')
        poses+=coarse_poses
        if len(dynamic['cases'])!=12 or not all(c['toolMotionBudgetPass'] for c in dynamic['cases']):raise ValueError('Incomplete/failed dynamic envelope')
        if len(dynamic_fine['cases'])!=12 or not all(c['toolMotionBudgetPass'] for c in dynamic_fine['cases']):raise ValueError('Incomplete/failed fine-time-step dynamic envelope')
        dynamic_cases=dynamic['cases']+dynamic_fine['cases']
        groups=[dict(id=r['id'],stocks=r['stocks']) for r in details['groups']]
        groups += [dict(id=g,stocks=['beam','beamFoot'+side]) for g,side in (('GFL','Left'),('GFR','Right'))]
        rows=[]
        for group in groups:
            id=group['id'];q=[c for r in poses for c in r['contacts'] if c['group']==id]
            d=[r for c in dynamic_cases for r in c['contacts'] if r['group']==id]
            if bool(q)!=bool(d):raise ValueError('Dynamic/static group coverage mismatch: '+id)
            if not q:
                rows.append(dict(**group,status='NOT CREDITED: no run reaches max(30 mm, 6 throats)',staticDesignMPa=None,fatigueRangeMPa=None))
                continue
            stress=max(max(r['staticDesignMPa'] for r in q),max(r['staticPlusDynamicDesignMPa'] for r in d))
            fatigue=max(max(r['fatigueQuasiStaticRangeMPa'] for r in q),max(r['fatigueRangeMPa'] for r in d))
            fatigue_pose=max(poses,key=lambda p:max((c['fatigueQuasiStaticRangeMPa'] for c in p['contacts'] if c['group']==id),default=0))
            dynamic_governs=max(r['fatigueRangeMPa'] for r in d)>max(r['fatigueQuasiStaticRangeMPa'] for r in q)
            legs=sorted({r['legMm'] for r in q})
            rows.append(dict(**group,status='PASS' if stress<=STATIC_ALLOWABLE and fatigue<=FATIGUE_ALLOWABLE else 'FAIL',
                detail=q[0]['detail'],legMm=legs,throatMm=[v/math.sqrt(2) for v in legs],
                minimumEffectiveLengthPerContactMm=min(r['effectiveLengthMm'] for r in q),
                staticDesignMPa=stress,staticAllowableMPa=STATIC_ALLOWABLE,fatigueRangeMPa=fatigue,
                fatigueAllowableMPa=FATIGUE_ALLOWABLE,fatigueCategoryMPa=36,longitudinalShearCategoryMPa=80,
                fatigueControllingSource='dynamic' if dynamic_governs else fatigue_pose['path'],
                fatigueControllingPose=dynamic['pose'] if dynamic_governs else fatigue_pose['pose'],
                staticControllingPose=max(poses,key=lambda p:max((c['staticDesignMPa'] for c in p['contacts'] if c['group']==id),default=0))['pose']))
        failures=[r['id'] for r in rows if r['status']=='FAIL']
        any_fail=any_fail or bool(failures)
        previous=json.loads((a.previous/f'y{bed}.json').read_text());by_stocks={tuple(sorted(r['stocks'])):r for r in rows}
        review=[]
        for old in previous['groups']:
            if old['id'] not in previous['failedGroups']:continue
            current=by_stocks.get(tuple(sorted(old['stocks'])))
            review.append(dict(previousId=old['id'],stocks=old['stocks'],currentId=current['id'] if current else None,
                resolution=current['status'] if current else 'REMOVED: deliberate non-weld cope clearance; connected alternative load path checked'))
        result=dict(workY=bed,status='reaction qualification PASS' if not failures else 'reaction qualification FAIL',
            fabricationReleased=False,fatigueCycles=FATIGUE_CYCLES,
            staticPoses=9,staticMeshesMm=[50,100],coarseCheckPoses=[r['pose'] for r in coarse_poses],dynamicPose=dynamic['pose'],dynamicModeCounts=[20,40],dynamicTimeStepsS=[.0005,.00025],dampingRatios=[.005,.01],
            basis=static['basis'],loadEnvelope='Concurrent signed 300 N XYZ cuts and 3 m/s² XYZ inertia; gravity; G=1.35 Q=1.5 for strength, unfactored full reversals for fatigue. Dynamic XY contour and rapid transitions at governing modal pose.',
            model='68 separate prepared stocks, exact rigid-offset MPCs and actual spatial weld springs; no old proximity ties. Isolated short runs and flush butt edges carry no credited weld load.',
            detailCorrections=['1 mm non-weld cope relief removes grazing crossmember/post, plan-brace/post and end-brace/top-rail contacts',
                'Post/stringer joints use full accessible runs; middle post01/post11 joints have 6 mm legs; 2600-bed middle joints require full-size tabbed ends',
                'Beam feet: two 92 mm fillets per foot, 8 mm legs, full-size run-on/off tab ends; no inaccessible all-around credit'],
            coverage=dict(geometryFindings=native['seamDiagnostics'],preparedPieces=len(native['members']),groups=len(rows),creditedGroups=sum(r['status']=='PASS' for r in rows),uncreditedGroups=sum(r['status'].startswith('NOT CREDITED') for r in rows)),
            failedGroups=failures,previousOpenGroupReview=review,groups=rows,
            limitations=['Reduced elastic member/joint model; excludes RHS wall distortion and local weld-toe notch stress',
                'FAT36 full traction-vector cut-off is a conservative weld-throat screen, not a certified hot-spot/parent-metal fatigue assessment',
                'WPS/access, full-size tab end witnesses, material identity, relief, epoxy application and shop datum-transfer capability require recorded acceptance; no simulated sign-off'])
        (a.output/f'y{bed}-weld-qualification.json').write_text(json.dumps(result,indent=2)+'\n')
        release=json.loads((folder/'shop-release.json').read_text())
        release['digitalChecks'].pop('weldStrengthScreen',None)
        release['digitalChecks']['weldReactionQualification']=not failures
        release['pending']=[
            'Qualified WPS/access and full-size tabbed-end acceptance; parent-metal/toe detail review',
            'Steel identification and heat-treater cycle/payload confirmation, or approved VSR procedure',
            'Epoxy supplier approval of 2–6 mm layer and sustained rail contact pressure/creep, plus witness pour',
            'Shop confirms whole welded beam-and-feet unit travel/support and datum-transfer tolerances',
            'Shop-release review signed against the exact STEP/seam/service/qualification packet hashes']
        release['reactionQualification']=f'weld-qualification.json';release['released']=False
        (a.output/f'y{bed}-shop-release.json').write_text(json.dumps(release,indent=2)+'\n')
        with (a.output/f'y{bed}-weld-qualification.csv').open('w',newline='') as f:
            fields=['group','stocks','status','leg_mm','throat_mm','fatigue_category_mpa','fatigue_cycles','strength_mpa','strength_limit_mpa','range_mpa','cutoff_mpa']
            w=csv.writer(f,lineterminator='\n');w.writerow(fields)
            for r in rows:w.writerow([r['id'],' / '.join(r['stocks']),r['status'],r.get('legMm',[None])[0],r.get('throatMm',[None])[0],r.get('fatigueCategoryMPa'),FATIGUE_CYCLES,r['staticDesignMPa'],STATIC_ALLOWABLE,r['fatigueRangeMPa'],FATIGUE_ALLOWABLE])
        packet=json.loads((a.packets/f'y{bed}/manifest.json').read_text());beam=next(b for b in packet['beams'] if b['id']=='beam')
        _,lines=with_foot_welds(packet,native,[],dict(y=bed/2));datum=[beam['startMm'][0],beam['startMm'][1],min(p['poseMm']['z'] for p in packet['parts'] if p['id'] in ('beamFootLeft','beamFootRight'))]
        with (a.output/f'y{bed}-gantry-weld-stitches.csv').open('w',newline='') as f:
            w=csv.writer(f,lineterminator='\n');w.writerow(['group','seam','leg_mm','throat_mm','length_mm','start_x','start_y','start_z','end_x','end_y','end_z','full_size_ends','instruction'])
            for line in lines:w.writerow([line['group'],line['seam'],line['legMm'],line['legMm']/math.sqrt(2),line['lengthMm'],*[line['a'][i]-datum[i] for i in range(3)],*[line['b'][i]-datum[i] for i in range(3)],True,'Sacrificial run-on/off tabs; verify full-size finished ends; alternate left-front/right-rear/left-rear/right-front; cool per WPS'])
        print(bed,result['status'],result['coverage'],'failures',failures,flush=True)
    if any_fail:raise ValueError('One or more weld groups fail; no fabrication release')


if __name__=='__main__':main()
