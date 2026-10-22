"""Frozen-layout macro-model screen, paired with the prepared-joint weld study.
Diagnostic brace energy sets overlap Gbraces ONLY for output, never for loading.
"""
import argparse,gzip,json,math,re,shutil
from pathlib import Path
import numpy as np
import qualify_welds  # sets up the retained Gmsh environment
import study
from study import Model
from modes import displacements,energy
from brace_study import variant,diagnostics,disk,NAMES

class Macro(Model):
    latest=None
    def __init__(self,h,packet):
        super().__init__(h);Macro.latest=self
        self.braces={tuple(b['startMm']+b['endMm']):'brace_'+next(k for k in ('side','end','plan') if k+'Brace' in b['id']) for b in packet['beams'] if 'Brace' in b['id']}
    def deck(self,loads,modal=True):
        text=super().deck(loads,modal);sets={}
        for eid,conn,row in self.elements:
            if row['group']=='braces':sets.setdefault(self.braces[tuple(row['a']+row['b'])],[]).append(eid)
        extra=[]
        for group,ids in sets.items():extra+=['*ELSET,ELSET=G'+group]+[','.join(map(str,ids[i:i+12])) for i in range(0,len(ids),12)]
        text=text.replace('*STEP\n','\n'.join(extra)+'\n*STEP\n',1)
        prints='\n'.join(line for group in sets for line in ['*EL PRINT,ELSET=G'+group+',TOTALS=ONLY','ELSE'])
        self.energy_sets=sets
        return text.replace('*END STEP',prints+'\n*END STEP')

def main():
    p=argparse.ArgumentParser();p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--ccx',type=Path,required=True);p.add_argument('--names',nargs='+',default=NAMES);p.add_argument('--beds',nargs='+',type=int,default=[2600,3150]);p.add_argument('--mesh',type=int,default=100);p.add_argument('--poses',nargs='+',help='Optional x:y pose subset in mm');p.add_argument('--modal-all-poses',action='store_true');p.add_argument('--all-poses',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    original=study.make_model;solver=study.run_solver
    study.make_model=lambda packet,pose,h,model=None:original(packet,pose,h,model if model is not None else Macro(h,packet))
    study.run_solver=lambda ccx,folder,timeout:solver(ccx,folder,timeout,threads=1)
    index=[]
    for bed in a.beds:
        base=json.loads((a.inputs/f'packets/y{bed}/manifest.json').read_text());native=json.loads((a.inputs/f'fabrication/y{bed}/native.json').read_text());physical=qualify_welds.read_stitches(a.inputs/f'fabrication/y{bed}')
        for name in a.names:
            packet,native_,stitches,changes=variant(base,native,physical,name);packet['modeCount']=40
            changes['weldLengthSavedMm']=sum(s['lengthMm'] for s in physical)-sum(s['lengthMm'] for s in stitches)
            root=a.output/f'y{bed}'/name;root.mkdir(parents=True,exist_ok=True);(root/'packet.json').write_text(json.dumps(packet)+'\n');(root/'changes.json').write_text(json.dumps(changes)+'\n')
            selected={tuple(map(float,value.split(':'))) for value in a.poses} if a.poses else {(x,y) for x in (0,800,1600) for y in (0,bed/2,bed)};records=[]
            for pose in base['poses']:
                if not a.all_poses and (pose['x'],pose['y']) not in selected:continue
                disk();folder=root/f'x{pose["x"]:g}-y{pose["y"]:g}-h{a.mesh}'
                if (folder/'summary.json').exists():
                    detail=json.loads((folder/'summary.json').read_text())
                else:
                    result=study.solve(packet,pose,a.mesh,a.ccx.resolve(),folder);model=Macro.latest;text=(folder/'frame.dat').read_text()
                    if len(result['firstModesHz'])!=40:raise ValueError('Incomplete requested modal basis')
                    blocks=re.split(r'E I G E N V A L U E\s+N U M B E R\s+\d+',text)[1:];basis=[]
                    masses=list(model.mass_loads)
                    for _,conn,row in model.elements:
                        rho=row['material'][1]
                        if not rho:continue
                        _,w,h,*walls=row['sec'];area=w*h-(w-2*walls[0])*(h-2*walls[0]) if walls else w*h;mass=area*math.dist(model.nodes[conn[0]],model.nodes[conn[2]])*rho
                        for n,weight in zip(conn,(1/6,4/6,1/6)):masses.append((n,mass*weight,'y' if row['group']=='gantry' else 'frame'))
                    for hz,block in zip(result['firstModesHz'],blocks):
                        u=displacements(block);mass=sum(m*sum(v*v for v in u[n]) for n,m,_ in masses)
                        gamma=[sum(m*u[n][i] for n,m,c in masses if c in (('x','z') if i==0 else ('x','y','z')))/mass for i in range(2)]
                        output=[u[model.query_nodes['tip']][i]-u[model.query_nodes['bed']][i] for i in range(2)]
                        basis.append(dict(frequencyHz=hz,gammaXY=gamma,toolRelativeEigenvectorXY=output))
                    modes=diagnostics(model,folder,dict(modes=basis))
                    for block,mode in zip(blocks,modes):
                        components=energy(block,model.energy_sets);budget=energy(block,model.groups)
                        u=displacements(block)
                        for n,m,d,k,g in model.spring_elements:budget[g]=budget.get(g,0)+.5*k*(u[n][d]-u[m][d])**2
                        total=sum(budget.values());old=mode['strainEnergyShare'].pop('braces',0)
                        if abs(sum(components.values())/total-old)>1e-5:raise ValueError('Brace subset energy does not sum to parent')
                        mode['strainEnergyShare'].update({k:v/total for k,v in components.items()})
                    C=np.asarray([c['toolRelativeMm'] for c in result['cases'][:3]])/300
                    reciprocal=float(np.max(np.abs(C-C.T))/np.max(np.abs(np.diag(C))))
                    if reciprocal>.003:raise ValueError('Macro cutting reciprocity failed: '+str(reciprocal))
                    detail=dict(cuttingReciprocityRelativeError=reciprocal,workY=bed,variant=name,pose={k:pose[k] for k in ('x','y','z')},meshMm=a.mesh,
                        xyStiffnessNPerUm=[1/result['cases'][i]['complianceUmPerN'][i] for i in (0,1)],modes=modes,
                        staticAccelerationResponseMmPerMPerS2=(np.asarray([c['toolRelativeMm'][:2] for c in result['cases'][4:6]]).T/3).tolist(),
                        nativeChecks=result['cases'],model='frozen merged-joint macro model; prepared-contact weld qualification is separate')
                    (folder/'summary.json').write_text(json.dumps(detail,separators=(',',':'))+'\n')
                    for file in list(folder.iterdir()):
                        if file.name in ['frame.frd.gz','frame.12d.gz','frame.cvg','frame.sta','spooles.out']:file.unlink()
                    with (folder/'frame.dat').open('rb') as src,gzip.open(folder/'frame.dat.gz','wb',compresslevel=3) as dst:shutil.copyfileobj(src,dst)
                    (folder/'frame.dat').unlink()
                records.append(dict(path=str(folder),pose=detail['pose'],xyStiffnessNPerUm=detail['xyStiffnessNPerUm']));print(bed,name,pose['x'],pose['y'],detail['xyStiffnessNPerUm'],'f',detail['modes'][0]['frequencyHz'],flush=True)
            index.append(dict(workY=bed,variant=name,changes=changes,cases=records));(a.output/f'index-y{bed}.json').write_text(json.dumps(index,indent=2)+'\n')
if __name__=='__main__':main()
