#!/usr/bin/env python3
"""Render the exported actual scene triangles; no invented connector detail."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation

root=Path(__file__).resolve().parent/'build/expert'
rows=json.loads((root/'render.json').read_text())
meshes=[]
for row in rows:
    v=np.fromfile(root/(row['id']+'.vertices'),dtype='<f8').reshape(-1,3)
    ix=np.fromfile(root/(row['id']+'.indices'),dtype='<i4').reshape(-1,3)
    f=row['pose'];v=Rotation.from_quat([f['qx'],f['qy'],f['qz'],f['qw']]).apply(v)+[f['x'],f['y'],f['z']]
    meshes.append((v[ix],row['color']))
fig=plt.figure(figsize=(13,6))
ax=fig.add_subplot(121,projection='3d',computed_zorder=True)
light=np.array([.3,-.5,.8]);light/=np.linalg.norm(light)
for triangles,color in meshes:
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
    shades=.55+.45*np.abs(normals@light)
    ax.add_collection3d(Poly3DCollection(triangles,facecolor=np.asarray(color)[None,:]*shades[:,None],edgecolor='none',rasterized=True))
ax.set(xlim=(-220,260),ylim=(-110,225),zlim=(-25,255),xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
ax.set_box_aspect((480,335,280));ax.view_init(elev=25,azim=-55)
from matplotlib.collections import PolyCollection
ax2=fig.add_subplot(122)
triangles=np.concatenate([m[0] for m in meshes]);colors=np.concatenate([np.tile(m[1],(len(m[0]),1)) for m in meshes])
order=np.argsort(triangles[:,:,2].mean(axis=1))
ax2.add_collection(PolyCollection(triangles[order,:,:2],facecolors=colors[order],edgecolors='none',rasterized=True))
ax2.set(xlim=(-220,275),ylim=(-100,220),xlabel='X (mm)',ylabel='Y (mm)',title='Top view')
ax2.set_aspect('equal');ax2.grid(alpha=.15)
fig.suptitle('Dual PGC + Fairino Expert — provisional routing')
fig.text(.5,.015,'Supplier harness dimensions, cable ratings and wrist mating position remain unverified.',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(root/'expert-layout.png',dpi=160)
print(root/'expert-layout.png')
