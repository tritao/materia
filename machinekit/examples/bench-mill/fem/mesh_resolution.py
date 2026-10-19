"""CAD-derived normal section probes for minimum casting wall/rib/roof resolution."""

import math


def sections(packet):
    layout = packet["frameLayout"]
    w, roof, h = (layout[k] for k in ("wall","roof","baseHeight"))
    front, column_front = layout["front"],layout["columnFront"]
    pocket_front = front+(160 if layout["footPockets"] else w)
    pocket_length = (column_front-w-pocket_front-w)/2
    y = pocket_front+pocket_length*.43
    parts = {p["id"]:p for p in packet["parts"]}
    probes=[]
    def add(name, member, start, end):
        probes.append({"name":name,"member":member,"startMm":start,"endMm":end,"thicknessMm":math.dist(start,end)})
    # Outer underside channel has its top lowered by the guide-shoulder relief.
    beam_outer=(layout["yRailSpacing"]+layout["seatWidth"])/2
    x=(beam_outer+layout["baseWidth"]/2-w)/2
    add("base-roof","frame/base/casting",[x,y,h-roof],[x,y,h-20])
    for sign in (-1,1):
        inner=sign*(layout["yRailSpacing"]-layout["seatWidth"])/2
        add("base-longitudinal-rib-"+str(sign),"frame/base/casting",[inner,y,100],[inner+sign*layout["seatWidth"],y,100])
        outer=sign*layout["baseWidth"]/2
        add("base-side-wall-"+str(sign),"frame/base/casting",[outer,y,100],[outer-sign*w,y,100])
    rib_y=pocket_front+pocket_length
    add("base-cross-rib","frame/base/casting",[3.17,rib_y,100],[3.17,rib_y+w,100])
    z=layout["railPlane"]+layout["columnHeight"]*.35
    cy=layout["columnCentreY"]+layout["columnDepth"]*.3
    for sign in (-1,1):
        outside=sign*layout["columnWidth"]/2
        add("column-side-wall-"+str(sign),"frame/column/casting",[outside,cy,z],[outside-sign*w,cy,z])
    add("column-drive-web","frame/column/casting",[3.17,layout["zChannelBack"],z],[3.17,layout["zChannelBack"]+w,z])
    chamber=(layout["columnHeight"]-2*roof-w)/2
    rib_z=layout["railPlane"]+roof+chamber
    add("column-cross-rib","frame/column/casting",[3.17,cy,rib_z],[3.17,cy,rib_z+w])
    top=layout["railPlane"]+layout["columnHeight"]
    add("column-roof","frame/column/casting",[3.17,cy,top-roof],[3.17,cy,top])
    if layout["footPockets"]:
        for support in packet["supports"]:
            part=parts["frame/base/foot"+support["id"]]
            x,y=(part["centreOfMassMm"][k] for k in "xy")
            bottom=part["boundsMm"]["minZ"]
            # Offset beyond the 20 mm square stem opening, underneath the steel seat.
            add("foot-floor-"+support["id"],"frame/base/casting",[x+.38*w,y+.071*w,0],[x+.38*w,y+.071*w,bottom])
    return probes


def partitions(packet):
    """Full member-bounding planes cap tetrahedra at each minimum section's mid-depth."""
    parts=packet["parts"]
    result={}
    for section in sections(packet):
        member=section["member"].rsplit("/",1)[0]+"/"
        a,b=section["startMm"],section["endMm"]
        axis=max(range(3),key=lambda i:abs(a[i]-b[i]))
        coordinate=(a[axis]+b[axis])/2
        key=(member,axis,coordinate)
        if key in result:
            result[key]["features"].append(section["name"])
            continue
        owners=[p for p in parts if p["id"].startswith(member)]
        low=[min(p["boundsMm"]["min"+k] for p in owners) for k in "XYZ"]
        high=[max(p["boundsMm"]["max"+k] for p in owners) for k in "XYZ"]
        tangents=[i for i in range(3) if i!=axis]
        corners=[]
        for first,second in ((0,0),(1,0),(1,1),(0,1)):
            point=[0.0]*3;point[axis]=coordinate
            for direction,index in zip((first,second),tangents):
                # End at the shared base/column plane to avoid tiny partial cuts
                # in the neighboring member; extend beyond lateral outside faces.
                margin=0 if index==2 else 1
                point[index]=high[index]+margin if direction else low[index]-margin
            corners.append(point)
        result[key]={"memberPrefix":member,"normalAxis":"xyz"[axis],"coordinateMm":coordinate,
                     "cornersMm":corners,"features":[section["name"]]}
    return list(result.values())


def measure(gmsh, packet, elements):
    owner={tag:member for member,rows in elements.items() for tag,_ in rows}
    result=[]
    for probe in sections(packet):
        a,b=probe["startMm"],probe["endMm"]
        hits=[]
        samples=401
        for i in range(samples):
            t=(i+.5)/samples
            point=[a[j]+t*(b[j]-a[j]) for j in range(3)]
            tag,*_=gmsh.model.mesh.getElementByCoordinates(*point,dim=3,strict=True)
            tag=int(tag)
            if owner.get(tag)!=probe["member"]:
                raise ValueError(f"Thickness probe leaves its named casting: {probe['name']}")
            hits.append(tag)
        lengths=[]
        count=0;previous=None
        for tag in hits:
            if tag!=previous:
                if count: lengths.append(count)
                count=0;previous=tag
            count+=1
        lengths.append(count)
        longest=max(lengths)*probe["thicknessMm"]/samples
        # Chord length prevents counting two diagonal slivers as two resolved layers.
        resolved=len(set(hits))>=2 and longest<=.5*probe["thicknessMm"]+probe["thicknessMm"]/samples
        result.append({**probe,"quadraticElementsCrossed":len(set(hits)),"maximumChordMm":longest,"resolved":resolved})
    return {"allSectionsResolved":all(p["resolved"] for p in result),"probes":result,
            "method":"401 interior samples per CAD-derived normal section; >=2 quadratic tetrahedra and no sampled chord longer than half thickness",
            "scope":"Named minimum foot floors, relieved base roof, transverse ribs, column side walls and roof. Local insert-edge singularities are not stress-qualified."}
