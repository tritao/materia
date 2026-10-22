"""Offline brace removal on qualified SR6 packets; never edits the frozen CAD."""
import argparse,copy,gzip,hashlib,json,math,re,shutil,sys,subprocess
from pathlib import Path
import numpy as np
import qualify_welds as qw
from weld_model import WeldModel,with_foot_welds
from weld_effective import effective_stitches
from weld_report import static_stress,FATIGUE_ALLOWABLE,STATIC_ALLOWABLE
from study import make_model,project
from modes import energy,displacements,body_motion

NAMES=['baseline','no-plan','single-a','single-b','no-plan-single-a','no-plan-single-b','no-plan-front-open','single-balanced','no-plan-single-balanced']
class BraceModel(WeldModel):
    latest=None
    def __init__(self,*a,**kw):super().__init__(*a,**kw);BraceModel.latest=self
    def deck(self,loads,modal=True):
        text=super().deck(loads,modal)
        # The legacy deck enumerates gravity groups explicitly. Splitting braces
        # for diagnostics must preserve their entire distributed gravity load.
        extra=''.join('\nG'+g+',GRAV,9810,0,0,-1' for g in self.groups if g.startswith('brace_'))
        return text.replace('Gframe,GRAV,9810,0,0,-1','Gframe,GRAV,9810,0,0,-1'+extra)
    def beam(self,*a,**kw):
        first=len(self.lines);super().beam(*a,**kw)
        for row in self.lines[first:]:
            if row['group']=='braces':
                row['group']='brace_'+next(k for k in ('side','end','plan') if k+'Brace' in row['space'])

def variant(packet,native,stitches,name):
    if name not in NAMES:raise ValueError('Unknown brace variant: '+name)
    packet,native,stitches=copy.deepcopy((packet,native,stitches))
    remove=set();stocks={m['sourceStock'] for m in native['members']}
    for stock in stocks:
        if name.startswith('no-plan') and stock.startswith('planBrace'):remove.add(stock)
        if 'single-' in name and stock.startswith(('sideBrace','endBrace')):
            slope=('a' if stock[-2]=='0' else 'b') if name.endswith('balanced') else name[-1]
            if stock[-1]!=slope:remove.add(stock)
        if name=='no-plan-front-open' and stock.startswith('endBrace0'):remove.add(stock)
    old_mass=sum(m['volumeMm3'] for m in native['members'])*7850e-9
    removed_ids={m['id'] for m in native['members'] if m['sourceStock'] in remove}
    native['members']=[m for m in native['members'] if m['id'] not in removed_ids]
    native['seams']=[s for s in native['seams'] if not {s['a']['member'],s['b']['member']}&removed_ids]
    seams={s['id'] for s in native['seams']};stitches=[s for s in stitches if s['seam'] in seams]
    # A single diagonal is a new continuous stock, not the two old coped halves.
    merged=[]
    if 'single-' in name:
        preparations={p['stock']:p for p in native['preparations']}
        for stock in sorted(stocks-remove):
            if not stock.startswith(('sideBrace','endBrace')):continue
            partners=set(preparations[stock]['copeTo'])&remove
            if not partners:continue  # Keep all copes around retained posts/frame members.
            members=[m for m in native['members'] if m['sourceStock']==stock]
            b=next(b for b in packet['beams'] if b['id']=='base/'+stock);direction=np.array(b['endMm'])-b['startMm'];direction/=np.linalg.norm(direction)
            members.sort(key=lambda m:np.dot(m['startMm'],direction));clusters=[]
            for member in members:
                if not clusters:clusters.append([member]);continue
                previous=clusters[-1][-1];mid=[(x+y)/2 for x,y in zip(previous['endMm'],member['startMm'])]
                crossing=False
                for partner in partners:
                    other=next(b for b in packet['beams'] if b['id']=='base/'+partner)
                    if math.dist(mid,project(mid,other['startMm'],other['endMm']))<=math.hypot(other['widthMm'],other['heightMm'])/2+1e-5:crossing=True
                if crossing:clusters[-1].append(member)
                else:clusters.append([member])
            for cluster in clusters:
                if len(cluster)<2:continue
                start,end=cluster[0]['startMm'],cluster[-1]['endMm'];length=math.dist(start,end)
                area=b['widthMm']*b['heightMm']-(b['widthMm']-2*b['wallMm'])*(b['heightMm']-2*b['wallMm'])
                restored=max(0,length-sum(math.dist(m['startMm'],m['endMm']) for m in cluster))*area
                new=copy.deepcopy(cluster[0]);new.update(startMm=start,endMm=end,centrelineLengthMm=length,stockLengthMm=length,volumeMm3=sum(m['volumeMm3'] for m in cluster)+restored)
                ids={m['id'] for m in cluster};native['members']=[m for m in native['members'] if m['id'] not in ids]+[new]
                for seam in native['seams']:
                    for side in ('a','b'):
                        if seam[side]['member'] in ids:seam[side]['member']=new['id']
                merged.append(dict(stock=stock,restoredVolumeMm3=restored,notes='continuous across removed partner only; all copes and seams at retained posts/frame stay'))
    packet['beams']=[b for b in packet['beams'] if b['id'].removeprefix('base/') not in remove]
    new_mass=sum(m['volumeMm3'] for m in native['members'])*7850e-9
    saved=old_mass-new_mass;packet['totalMassKg']-=saved
    base=next(p for p in packet['parts'] if p['id']=='base');base['massKg']-=saved
    packet['floorSupport']='anchored'
    return packet,native,stitches,dict(removedStocks=sorted(remove),continuousDiagonals=merged,fixedMassSavedKg=saved,movingMassSavedKg=0.,weldLengthSavedMm=None)

def diagnostics(model,path,result):
    text=(path/'frame.dat').read_text();blocks=re.split(r'E I G E N V A L U E\s+N U M B E R\s+\d+',text)[1:];rows=[]
    masses=list(model.mass_loads)
    for _,conn,row in model.elements:
        rho=row['material'][1]
        if not rho:continue
        _,w,h,*walls=row['sec'];area=w*h-(w-2*walls[0])*(h-2*walls[0]) if walls else w*h
        mass=area*math.dist(model.nodes[conn[0]],model.nodes[conn[2]])*rho
        for node,weight in zip(conn,(1/6,4/6,1/6)):masses.append((node,mass*weight,'y' if row['group']=='gantry' else 'frame'))
    for block,mode in zip(blocks,result['modes']):
        u=displacements(block);budget=energy(block,model.groups)
        for n,m,d,k,g in model.spring_elements:budget[g]=budget.get(g,0)+.5*k*(u[n][d]-u[m][d])**2
        total=sum(budget.values());shares={g:v/total for g,v in budget.items()}
        # Include native stock spaces so frame sway cannot disappear behind a rigid-body label.
        bodies=body_motion(model,u)
        kinetic={}
        for node,mass,carrier in masses:kinetic[carrier]=kinetic.get(carrier,0)+mass*sum(v*v for v in u[node])
        gantry=sum(kinetic.get(c,0) for c in ('x','y','z'))/sum(kinetic.values())
        rows.append(dict(frequencyHz=mode['frequencyHz'],strainEnergyShare=shares,bodies=bodies,
            accelerationResidueXY=np.outer(mode['toolRelativeEigenvectorXY'],mode['gammaXY']).tolist(),
            gantryKineticFraction=gantry,gammaXY=mode['gammaXY'],toolRelativeEigenvectorXY=mode['toolRelativeEigenvectorXY']))
    return rows

def disk():
    free=shutil.disk_usage(Path.cwd()).free
    if free<8_000_000_000:raise RuntimeError('STOP: root free space below 8 GB: '+str(free))

def checked_case(packet,joined,effective,pose,h,ccx,folder,count):
    result=qw.run_case(packet,joined,effective,pose,h,ccx,folder,count)
    if len(result['modes'])!=count:raise ValueError('Incomplete requested modal basis')
    model=BraceModel.latest;layout=qw.contacts(joined,effective);stress=static_stress(np.asarray(result['staticWrenches']),layout)
    C=np.asarray([r['tipRelativeMm'] for r in result['cases'][:3]])/300
    reciprocal=float(np.max(np.abs(C-C.T))/max(np.max(np.abs(np.diag(C))),1e-15))
    if min(np.diag(C))<=0 or reciprocal>.003:raise ValueError('Non-positive/non-reciprocal native compliance: '+str(C))
    text=(folder/'frame.dat').read_text();tab=qw.tables(text);energy_errors=[]
    for i in range(3):
        u=tab[(i+1,'U')];block=re.split(r'S T E P\s+\d+',text)[i+1];total=sum(energy(block,model.groups).values())
        total+=sum(.5*k*(u[n][d]-u[m][d])**2 for n,m,d,k,_ in model.spring_elements)
        expected=.5*300*result['cases'][i]['tipRelativeMm'][i];error=abs(total/expected-1)
        if error>.01:raise ValueError('Native cut work/energy mismatch: '+str(error))
        energy_errors.append(error)
    modes=diagnostics(model,folder,result) if count else []
    return result,model,stress,modes,reciprocal,energy_errors

def main():
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ccx',type=Path,required=True);p.add_argument('--names',nargs='+',default=NAMES);p.add_argument('--beds',nargs='+',type=int,default=[2600,3150]);p.add_argument('--mesh',type=int,default=50);p.add_argument('--poses',nargs='+',help='Optional x:y pose subset in mm');p.add_argument('--static-only',action='store_true');p.add_argument('--modal-all-poses',action='store_true');p.add_argument('--all-poses',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    qw.WeldModel=BraceModel;native_solver=qw.run_solver
    qw.run_solver=lambda ccx,folder,timeout:native_solver(ccx,folder,timeout,threads=1)
    index=[]
    for bed in a.beds:
        base=json.loads((a.inputs/f'packets/y{bed}/manifest.json').read_text());native=json.loads((a.inputs/f'fabrication/y{bed}/native.json').read_text());physical=qw.read_stitches(a.inputs/f'fabrication/y{bed}')
        for name in a.names:
            packet,cut,stitches,changes=variant(base,native,physical,name)
            changes['weldLengthSavedMm']=sum(s['lengthMm'] for s in physical)-sum(s['lengthMm'] for s in stitches)
            root=a.output/f'y{bed}'/name;root.mkdir(parents=True,exist_ok=True)
            for file,value in [('packet.json',packet),('native.json',cut),('stitches.json',stitches),('changes.json',changes)]: (root/file).write_text(json.dumps(value,indent=2)+'\n')
            selected={tuple(map(float,value.split(':'))) for value in a.poses} if a.poses else {(x,y) for x in (0,800,1600) for y in (0,bed/2,bed)}
            poses=[q for q in base['poses'] if a.all_poses or (q['x'],q['y']) in selected]
            records=[]
            for pose in poses:
                disk();folder=root/f'x{pose["x"]:g}-y{pose["y"]:g}-h{a.mesh}'
                joined,lines=with_foot_welds(packet,cut,stitches,pose);effective,rejected=effective_stitches(joined,lines)
                count=0 if a.static_only else (40 if a.modal_all_poses or pose['x']==800 else 0)
                if (folder/'frame.dat.gz').exists() and not (folder/'frame.dat').exists():
                    with gzip.open(folder/'frame.dat.gz','rb') as src,(folder/'frame.dat').open('wb') as dst:shutil.copyfileobj(src,dst)
                attempts=[]
                for h in dict.fromkeys([a.mesh,75,50]):
                    folder=root/f'x{pose["x"]:g}-y{pose["y"]:g}-h{h}'
                    if (folder/'numerical-failure.json').exists():
                        attempts.append(json.loads((folder/'numerical-failure.json').read_text()));continue
                    try:
                        if (folder/'frame.dat.gz').exists() and not (folder/'frame.dat').exists():
                            with gzip.open(folder/'frame.dat.gz','rb') as src,(folder/'frame.dat').open('wb') as dst:shutil.copyfileobj(src,dst)
                        result,model,stress,modes,reciprocal,energy_errors=checked_case(packet,joined,effective,pose,h,a.ccx.resolve(),folder,count)
                        break
                    except (ValueError,subprocess.TimeoutExpired) as error:
                        folder.mkdir(parents=True,exist_ok=True);failure=dict(meshMm=h,error=str(error),disposition='Numerical solve rejected; no design acceptance from this case')
                        (folder/'numerical-failure.json').write_text(json.dumps(failure,indent=2)+'\n');attempts.append(failure)
                        print('NUMERICAL RETRY',bed,name,pose['x'],pose['y'],h,error,flush=True)
                else:raise ValueError('No numerically valid mesh for '+str((bed,name,pose)))
                detail=dict(workY=bed,variant=name,pose=result['pose'],meshMm=h,numericalAttempts=attempts,xyStiffnessNPerUm=[.3/result['cases'][i]['tipRelativeMm'][i] for i in (0,1)],
                    weldStatic=stress,modes=modes,staticAccelerationResponseMmPerMPerS2=(np.asarray([r['tipRelativeMm'][:2] for r in result['cases'][4:6]]).T/3).tolist(),
                    maxFatigueMPa=max(c['fatigueQuasiStaticRangeMPa'] for c in stress),staticPass=all(c['staticPass'] for c in stress),fatiguePass=all(c['fatigueQuasiStaticPass'] for c in stress),
                    cuttingReciprocityRelativeError=reciprocal,cutWorkEnergyRelativeErrors=energy_errors,nativeChecks=result['cases'],solverDeckSha256=result['solverDeckSha256'])
                (folder/'summary.json').write_text(json.dumps(detail,separators=(',',':'))+'\n');records.append(dict(path=str(folder),**{k:detail[k] for k in ('pose','xyStiffnessNPerUm','maxFatigueMPa','fatiguePass','staticPass')}))
                print(bed,name,pose['x'],pose['y'],detail['xyStiffnessNPerUm'],'fatigue',detail['maxFatigueMPa'],'modes',len(modes),flush=True)
                # Keep decks/reactions/summaries; bulky native visualization is not evidence.
                for file in folder.iterdir():
                    if file.suffix in ('.frd','.12d','.cvg','.sta') or file.name=='spooles.out':
                        if file.is_symlink():raise RuntimeError('No symlink cleanup')
                        file.unlink()
                if (folder/'frame.dat').exists():
                    with (folder/'frame.dat').open('rb') as src,gzip.open(folder/'frame.dat.gz','wb',compresslevel=3) as dst:shutil.copyfileobj(src,dst)
                    (folder/'frame.dat').unlink()
                disk()
            row=dict(workY=bed,variant=name,changes=changes,cases=records);index.append(row);(a.output/f'index-y{bed}.json').write_text(json.dumps(index,indent=2)+'\n')
if __name__=='__main__':main()
