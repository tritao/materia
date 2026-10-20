"""Spatial block supports and plate grillages for the front guide arrangement."""
from study import add, sub, point, project


def front_carriage(model,packet,pose,a,b,axes):
    dx=pose['x']-800;dy=pose['y']-packet['workY']/2
    plates={p['id']:p for p in packet['plates']}
    blocks=pose['blocks']
    def shifted_plate(name):
        p=plates[name];centre=point(p['poseMm']);centre=add(centre,(dx,dy,0));centre=add(centre,(0,-p['thicknessMm']/2,0))
        return p,centre
    def grill(name,space,contacts,group):
        p,c=shifted_plate(name)
        def clamp(q):return (max(c[0]-p['widthMm']/2,min(c[0]+p['widthMm']/2,q[0])),c[1],max(c[2]-p['depthMm']/2,min(c[2]+p['depthMm']/2,q[2])))
        mapped=[clamp(q) for q in contacts]
        xs=sorted(set([c[0]-p['widthMm']/2,c[0],c[0]+p['widthMm']/2]+[q[0] for q in mapped]));zs=sorted(set([c[2]-p['depthMm']/2,c[2],c[2]+p['depthMm']/2]+[q[2] for q in mapped]))
        xs=sorted(set(round(v,6) for v in xs));zs=sorted(set(round(v,6) for v in zs))
        def tributary(values,i):return ((values[i+1]-values[i]) if i==0 else (values[i]-values[i-1]) if i==len(values)-1 else (values[i+1]-values[i-1]))/2
        for i,x in enumerate(xs):
            for z,w in zip(zs,zs[1:]):model.beam((x,c[1],z),(x,c[1],w),('RECT',tributary(xs,i),p['thicknessMm']),(1,0,0),(p['youngsMPa'],0),group,space)
        for i,z in enumerate(zs):
            for x,y in zip(xs,xs[1:]):model.beam((x,c[1],z),(y,c[1],z),('RECT',p['thicknessMm'],tributary(zs,i)),(0,1,0),(p['youngsMPa'],0),group,space)
        for q in contacts:model.rigid(clamp(q),q,space)
        return c
    xb=[r for r in blocks if r['axis']=='x'];zb=[r for r in blocks if r['axis']=='z']
    drive={d['id']:d['pointMm'] for d in pose['drives']}
    xp,_=shifted_plate('xCarriage');zp,zc=shifted_plate('zCarriage')
    top_bridge=packet.get('analysisTopXBridge')
    def mount_contact(block):
        q=tuple(block['railPointMm'])
        return (q[0],xp['poseMm']['y']-xp['thicknessMm']/2,top_bridge['midZMm']) if block.get('topMounted') else q
    xcontacts=[mount_contact(r) for r in xb]+[tuple(r['railPointMm']) for r in zb]+[tuple(drive['pinionX']),tuple(drive['screwZNut'])]
    zcontacts=[tuple(r['railPointMm']) for r in zb]+[tuple(drive['screwZNut'])]
    saddle=pose.get('spindleMountsMm')
    if saddle:zcontacts += [tuple(q) for q in saddle]
    else:
        shelf=plates['zFlangeShelf'];shelfcentre=add(point(shelf['poseMm']),(dx,dy,shelf['thicknessMm']/2))
        shelfback=(shelfcentre[0],zc[1],shelfcentre[2]);zcontacts.append(shelfback)
    ribdepth=zp.get('ribDepthMm',0);ribthick=zp.get('ribThicknessMm',0);ribwidth=zp.get('ribWidthMm',ribthick)
    ribxs=[zc[0]+side*(zp['widthMm']-ribwidth)/2 for side in (-1,1)] if ribdepth else []
    if ribdepth:
        riblevels=sorted(set([zc[2]-zp['depthMm']/2,zc[2]+zp['depthMm']/2]+[q[2] for q in zcontacts]))
        for x in ribxs:
            for z in riblevels:zcontacts.append((x,zc[1],z))
    xcentre=grill('xCarriage','x',xcontacts,'Xplate');zcentre=grill('zCarriage','z',zcontacts,'Zplate')
    # Integral side ribs share the plate material and mass from the CAD solid.
    # Cross-section offsets transmit bending; no extra guide rotational spring.
    if ribdepth:
        y=zp['poseMm']['y']-zp['thicknessMm']-ribdepth/2
        for x in ribxs:
            for z,w in zip(riblevels,riblevels[1:]):model.beam((x,y,z),(x,y,w),(('BOX',ribwidth,ribdepth,*([ribthick]*4)) if ribwidth>ribthick else ('RECT',ribthick,ribdepth)),(1,0,0),(zp['youngsMPa'],0),'Zribs','z')
            for z in riblevels:model.rigid((x,zc[1],z),(x,y,z),'z')
    for i,block in enumerate(xb):
        q=tuple(block['railPointMm']);support=project(q,a,b);model.rigid(support,q,'gantry')
        if block.get('topMounted'):
            anchor=mount_contact(block);end=(q[0],q[1],top_bridge['midZMm'])
            model.beam(anchor,end,('RECT',xp['widthMm']/2,top_bridge['thicknessMm']),(1,0,0),(xp['youngsMPa'],0),'Xplate','x')
            model.rigid(end,q,'x')
        fixed=model.root('Xfixed'+str(i),q,'gantry');moving=model.root('Xmoving'+str(i),q,'x')
        model.spring(moving,fixed,block.get('translationNPerMmXYZ',[0,block['radialNPerMm'],block['lateralNPerMm']])+[0,0,0],'Xblocks')
    q=tuple(drive['pinionX']);model.rigid(project(q,a,b),q,'gantry')
    fixed=model.root('XdriveFixed',q,'gantry');moving=model.root('XdriveMoving',q,'x')
    model.spring(moving,fixed,[axes['x']['compliance']['stiffnessNPerM']/1000,0,0,0,0,0],'Xdrive')
    for i,block in enumerate(zb):
        q=tuple(block['railPointMm']);fixed=model.root('Zfixed'+str(i),q,'x');moving=model.root('Zmoving'+str(i),q,'z')
        model.spring(moving,fixed,[block['lateralNPerMm'],block['radialNPerMm'],0,0,0,0],'Zblocks')
    q=tuple(drive['screwZNut']);fixed=model.root('ZdriveFixed',q,'x');moving=model.root('ZdriveMoving',q,'z')
    model.spring(moving,fixed,[0,0,axes['z']['screw']['stiffnessNPerM']/1000,0,0,0],'Zscrew')
    # Rails on the moving plate reinforce its overhang. Credit a reduced section,
    # excluding 40% of gross rail width for grooves/section uncertainty.
    for x in [zc[0]-packet['guideLayout']['zRailSpacing']/2,zc[0]+packet['guideLayout']['zRailSpacing']/2]:
        y=zp['poseMm']['y']+7.5;lo=zc[2]-zp['depthMm']/2;hi=zc[2]+zp['depthMm']/2
        levels=sorted(set([lo,hi]+[q[2] for q in zcontacts]+[lo+(hi-lo)*i/12 for i in range(13)]))
        for z,w in zip(levels,levels[1:]):model.beam((x,y,z),(x,y,w),('RECT',12,15),(1,0,0),(210000,0),'movingRails','zrails')
        for i,z in enumerate(levels):
            # Each rail bolt ties to the nearest plate grid line at the same height.
            # Add the corresponding full-width plate strip to avoid a rigid vertical rib.
            if z not in [q[2] for q in zcontacts] and z not in (lo,hi):continue
            model.rigid((x,y,z),(x,zc[1],z),'zrails')
            fixed=model.root('railPlate'+str(x)+str(i),(x,zc[1],z),'z')
            moving=model.root('railSteel'+str(x)+str(i),(x,zc[1],z),'zrails')
            model.spring(moving,fixed,[1e6]*3+[1e9]*3,'railBolts')
    tip=tuple(pose['toolTipMm'])
    if saddle:
        centre=(tip[0],tip[1],sum(q[2] for q in saddle)/len(saddle))
        for i,q in enumerate(saddle):
            q=tuple(q);model.rigid(centre,q,'spindle')
            moving=model.root('saddleMoving'+str(i),q,'spindle');fixed=model.root('saddlePlate'+str(i),q,'z')
            model.spring(moving,fixed,[1e6]*3+[0]*3,'saddleBolts')
        model.rigid(centre,tip,'spindle');model.tip_space='spindle'
        model.spindle_root=model.root('SpindleMass',centre,'spindle')
    else:
        model.beam(shelfback,shelfcentre,('RECT',shelf['widthMm'],shelf['thicknessMm']),(1,0,0),(shelf['youngsMPa'],0),'flangeShelf','z')
        model.rigid(shelfcentre,tip,'z')
    return model.root('Xmass',xcentre,'x'),model.root('Zmass',zcentre,'z')
