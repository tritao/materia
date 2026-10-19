"""Hash-verified exact mesh snapshots make solver-deck resume reproducible."""
import json
from pathlib import Path
import shutil
import hashlib
from packet import digest
from frame_mesh import build_mesh


def cached_mesh(gmsh,packet,manifest,config,size,output,cache_directory=None):
    cache_directory=cache_directory or Path(__file__).resolve().parents[1]/'build/fem/mesh-cache'
    identity={'manifestSha256':digest(manifest),'config':config,'sizeMm':size,'gmshVersion':gmsh.__version__,
        'gmshLibrarySha256':digest(Path(gmsh.libpath)),
        'meshSources':{n:digest(Path(__file__).with_name(n)) for n in ('frame_mesh.py','ccx_deck.py','mesh_resolution.py','mesh_cache.py')}}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    folder=cache_directory/key
    meta=folder/'identity.json';data=folder/'mesh.json';msh=folder/'frame.msh'
    if meta.exists():
        saved=json.loads(meta.read_text())
        if saved['identity']!=identity or saved['dataSha256']!=digest(data) or saved['mshSha256']!=digest(msh):
            raise ValueError('Corrupted or mismatched mesh cache: '+str(folder))
        mesh=json.loads(data.read_text());mesh['nodes']={int(n):tuple(p) for n,p in mesh['nodes'].items()}
        mesh['bodyNodes']={member:set(ids) for member,ids in mesh['bodyNodes'].items()}
        output.mkdir(parents=True,exist_ok=True);shutil.copyfile(msh,output/'frame.msh')
        print('Reusing exact validated mesh '+key[:16],flush=True)
        return mesh
    mesh=build_mesh(gmsh,packet,manifest,config,size,output)
    mesh['cachedGmshWarnings']=[line for line in gmsh.logger.get() if 'Warning' in line or 'Error' in line]
    folder.mkdir(parents=True,exist_ok=True)
    data.write_text(json.dumps(mesh,default=lambda v:sorted(v) if isinstance(v,set) else v,separators=(',',':'))+'\n')
    shutil.copyfile(output/'frame.msh',msh)
    # Metadata last: incomplete caches cannot qualify for reuse.
    meta.write_text(json.dumps({'identity':identity,'dataSha256':digest(data),'mshSha256':digest(msh)},indent=2)+'\n')
    return mesh
