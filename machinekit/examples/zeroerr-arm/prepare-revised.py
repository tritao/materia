#!/usr/bin/env python3
"""Prepare the pinned 80T wrist revision; run the baseline preparation first."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import urllib.request
import zipfile
import numpy as np

HERE = Path(__file__).resolve().parent

def digest(data): return hashlib.sha256(data).hexdigest()

def main():
    layout = json.loads((HERE/'revised-layout.json').read_text())
    baseline = json.loads((HERE/'reference-layout.json').read_text())
    revision = layout['revision']
    source = revision['additionalSource']
    archive = HERE/'build/zeroerr-10kg.zip'
    if not archive.exists():
        local = HERE.parent/'cobot-arm/build/references/zeroerr/10kg.zip'
        if local.exists(): shutil.copyfile(local, archive)
        else: urllib.request.urlretrieve(source['url'], archive)
    if digest(archive.read_bytes()) != source['sha256']: raise ValueError('10 kg archive changed')
    cache = HERE/'build/host/native/cadkit/cmake-ninja/CMakeCache.txt'
    prefix = Path(re.search(r'^CADKIT_OCCT_DIR:PATH=(.+)$',cache.read_text(),re.M).group(1))
    vendor = HERE/'build/revised/vendor';vendor.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('prepare_reference',HERE/'prepare-reference.py')
    prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)
    original_assembly=(HERE/'build/vendor/assembly.STEP').read_bytes()
    if digest(original_assembly)!=baseline['source']['assemblySha256']:raise ValueError('5 kg assembly changed')
    prep.verify_placements(original_assembly,baseline['occurrences'])
    with zipfile.ZipFile(archive) as z:
        assembly=z.read(source['assembly'])
        if digest(assembly)!=source['assemblySha256']:raise ValueError('10 kg assembly changed')
        prep.verify_placements(assembly,revision['originalPlacements'])
        for a in layout['assets']:
            data=z.read(a['file']) if a['id'] in ('asset14','asset15','asset16') else (HERE/'build/vendor'/f"{a['id']}.STEP").read_bytes()
            if digest(data)!=a['sha256']:raise ValueError('Component changed: '+a['id'])
            (vendor/f"{a['id']}.STEP").write_bytes(data)
    # Independently verify the explicit rigid alignment and shortening translations.
    def m4(v):return np.vstack((np.array(v).reshape(3,4),[0,0,0,1]))
    original={o['sourceOccurrence']:o for o in revision['originalPlacements']}
    near={o['id']:o for o in baseline['occurrences']}
    transform=m4(near['joint3']['matrix'])@np.linalg.inv(m4(original[664436]['matrix']))
    np.testing.assert_allclose(transform[:3].ravel(),revision['sourceToArmMatrix'],atol=1e-8)
    # The common elbow chain must coincide, not merely have similarly named parts.
    for name, source_id in [('elbowOutputAdapter',664437),('elbowHousingAdapter',664438),
                            ('elbowHousing',664439),('forearm',664440)]:
        np.testing.assert_allclose((transform@m4(original[source_id]['matrix']))[:3].ravel(),
                                   near[name]['matrix'],atol=1e-8)
    for o in layout['occurrences']:
        if o.get('sourcePackage')=='10kg':
            expected=transform@m4(original[o['sourceOccurrence']]['matrix'])
            if o['id'] not in ('forearm','cableBracket4'):expected[2,3]-=189
            np.testing.assert_allclose(expected[:3].ravel(),o['matrix'],atol=1e-8)
        else:np.testing.assert_allclose(o['matrix'],near[o['id']]['matrix'],atol=1e-8)
    libraries=['TKDESTEP','TKXSBase','TKShHealing','TKBO','TKBool','TKPrim','TKOffset','TKGeomAlgo','TKTopAlgo','TKBRep','TKGeomBase','TKG3d','TKG2d','TKMath','TKernel']
    def compile(name):
        out=HERE/'build'/name
        subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-I',str(prefix/'include/opencascade'),str(HERE/(name+'.cpp')),'-L',str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),*['-l'+l for l in libraries],'-o',str(out)],check=True)
        return out
    repair=compile('repair-reference');shorten=compile('shorten-reference')
    repairs=json.loads((HERE/'build/vendor/repairs.json').read_text())
    for r in repairs:
        key=f"{r['asset']}-solid{r['solid']}"
        for suffix in ('.STEP','-rotor.STEP','-stator.STEP'):
            p=HERE/'build/vendor'/(key+suffix)
            if p.exists():shutil.copyfile(p,vendor/p.name)
    subprocess.run([str(repair),str(vendor/'asset15.STEP'),str(vendor/'asset15'),str(vendor/'asset15-repair.jsonl'),'partition80'],check=True,stdout=subprocess.DEVNULL)
    for line in (vendor/'asset15-repair.jsonl').read_text().splitlines():
        r=json.loads(line);a=next(a for a in layout['assets'] if a['id']=='asset15');r.update(asset='asset15',sourceSha256=a['sha256'])
        key=f"asset15-solid{r['solid']}";r['sha256']=digest((vendor/(key+'.STEP')).read_bytes())
        if 'partitionZ' in r:
            for side in ('rotor','stator'):r[side+'Sha256']=digest((vendor/(key+'-'+side+'.STEP')).read_bytes())
        repairs.append(r)
    (vendor/'repairs.json').write_text(json.dumps(repairs,indent=2)+'\n')
    subprocess.run([str(shorten),str(vendor/'asset14.STEP'),str(vendor/'asset14-shortened.STEP'),str(vendor/'shortening.json')],check=True,stdout=subprocess.DEVNULL)
    r=json.loads((vendor/'shortening.json').read_text());a=next(a for a in layout['assets'] if a['id']=='asset14')
    r.update(asset='asset14',sourceSha256=a['sha256'],sha256=digest((vendor/'asset14-shortened.STEP').read_bytes()))
    (vendor/'derived-links.json').write_text(json.dumps([r],indent=2)+'\n')
    print('Prepared pinned 80T wrist chain and shortened forearm; interfaces retained, hardware uncommissioned')

if __name__=='__main__':main()
