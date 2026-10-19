#!/usr/bin/env python3
"""Read the five simple R12 extrusion sections, retaining exact lines, bulge arcs and circles."""
import hashlib
import io
import json
import math
from pathlib import Path
import zipfile

from vallder import DEFAULT_OUTPUT, write_json

SECTIONS = {5250: (30,30),5263:(40,40),5279:(40,80),5295:(80,80),5308:(180,20)}


def profile_family(dimensions):
    width,height=dimensions
    if (width,height)==(30,30):
        system,pitch="TSLOT-30-SLOT8",30
    elif (width,height) in ((40,40),(40,80),(80,80)):
        system,pitch="TSLOT-40-SLOT8",40
    else:
        return {"id":f"SLOTTED-PLATE-{width}x{height}","kind":"slotted_plate","system_id":None,
                "dimensions_mm":[width,height],"interface_status":"section_specific"}
    def centres(span):
        return [-span/2+pitch/2+i*pitch for i in range(span//pitch)]
    xs,ys=centres(width),centres(height)
    return {"id":f"{system}-{width}x{height}","kind":"tslot","system_id":system,
            "dimensions_mm":[width,height],"grid_pitch_mm":pitch,"nominal_slot_mm":8,
            "slot_positions_mm":[ys,xs,ys,xs],"bore_centres_mm":[[x,y] for x in xs for y in ys],
            "interface_status":"candidate_not_cross_vendor_qualified"}


def area(vertices):
    result = 0.0
    for a,b in zip(vertices,vertices[1:]+vertices[:1]):
        result += (a[0]*b[1]-b[0]*a[1])/2
        bulge = a[2]
        if bulge:
            chord = math.hypot(b[0]-a[0],b[1]-a[1])
            radius = chord*(1+bulge*bulge)/(4*abs(bulge))
            theta = 4*math.atan(bulge)
            result += radius*radius*(theta-math.sin(theta))/2
    return result


def reverse(vertices):
    return [[vertices[i][0],vertices[i][1],-vertices[(i-1)%len(vertices)][2]] for i in reversed(range(len(vertices)))]


def parse(data):
    lines = data.decode("latin1").splitlines()
    if len(lines)%2:
        raise ValueError("Odd DXF group-code count")
    pairs = [(int(lines[i].strip()),lines[i+1].strip()) for i in range(0,len(lines),2)]
    entities, current, active = [], None, False
    for code,value in pairs:
        if code==2 and value=="ENTITIES":
            active=True
            continue
        if not active:
            continue
        if code==0:
            if current:
                entities.append(current)
            if value=="ENDSEC":
                break
            current={"kind":value,"tags":{}}
        elif current:
            current["tags"][code]=value
    loops,circles,vertices=[],[],None
    elevation=None
    for entity in entities:
        kind,tags=entity["kind"],entity["tags"]
        if any(abs(float(tags.get(code,default))-default)>1e-9 for code,default in ((210,0),(220,0),(230,1))):
            raise ValueError("Unsupported DXF extrusion normal")
        if kind=="POLYLINE":
            if vertices is not None or not int(tags.get(70,"0"))&1:
                raise ValueError("Expected closed, unnested DXF polylines")
            vertices=[]
        elif kind=="VERTEX" and vertices is not None:
            z=float(tags.get(30,"0"))
            if elevation is None:
                elevation=z
            if abs(z-elevation)>1e-6:
                raise ValueError("Section vertex is not planar")
            vertices.append([float(tags[10]),float(tags[20]),float(tags.get(42,"0"))])
        elif kind=="SEQEND" and vertices is not None:
            if len(vertices)>2 and math.hypot(vertices[0][0]-vertices[-1][0],vertices[0][1]-vertices[-1][1])<1e-8:
                vertices.pop()
            if len(vertices)<3 or any(math.hypot(a[0]-b[0],a[1]-b[1])<1e-9 for a,b in zip(vertices,vertices[1:]+vertices[:1])):
                raise ValueError("Degenerate DXF polyline")
            loops.append(vertices)
            vertices=None
        elif kind=="CIRCLE":
            if elevation is not None and abs(float(tags.get(30,"0"))-elevation)>1e-6:
                raise ValueError("Section circle is not in the polyline plane")
            circles.append({"x":float(tags[10]),"y":float(tags[20]),"radius":float(tags[40])})
        else:
            raise ValueError("Unsupported section entity: "+kind)
    if vertices is not None or not loops:
        raise ValueError("Incomplete section loops")
    loops.sort(key=lambda v:abs(area(v)),reverse=True)
    if area(loops[0])<0:
        loops[0]=reverse(loops[0])
    for i in range(1,len(loops)):
        if area(loops[i])>0:
            loops[i]=reverse(loops[i])
    # Source vertex extents define the section centering; native validation checks true arc bounds.
    x=[v[0] for loop in loops for v in loop]; y=[v[1] for loop in loops for v in loop]
    cx,cy=(min(x)+max(x))/2,(min(y)+max(y))/2
    for loop in loops:
        for v in loop:
            v[0]-=cx;v[1]-=cy
    for circle in circles:
        circle["x"]-=cx;circle["y"]-=cy
    section_area=sum(area(loop) for loop in loops)-sum(math.pi*c["radius"]**2 for c in circles)
    if section_area<=0:
        raise ValueError("Section area is not positive")
    return {"loops":loops,"circles":circles,"area_mm2":section_area,
            "source_elevation_mm":elevation,
            "source_origin_offset_mm":[cx,cy],"vertex_extents_mm":[max(x)-min(x),max(y)-min(y)]}


def main(output):
    catalog=json.loads((output/"catalog.json").read_text())
    inventory=json.loads((output/"cad/inventory.json").read_text())
    profiles=[]
    families,offers=[],[]
    for product_id,dimensions in SECTIONS.items():
        product=next(p for p in catalog["products"] if p["supplier_product_id"]==product_id)
        sources=[d for d in inventory["downloads"] if any(r["product_id"]==product_id for r in d["references"])]
        candidates=[d for d in sources if d.get("format")=="zip" and len(d.get("members",[]))==1 and
                    d["members"][0]["format"]=="dxf"]
        if len(candidates)!=1:
            raise ValueError("Ambiguous DXF section source")
        download=candidates[0]
        data=(output/"cad/downloads"/download["cache_file"]).read_bytes()
        if hashlib.sha256(data).hexdigest()!=download["sha256"]:
            raise ValueError("DXF source archive hash mismatch")
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            name=download["members"][0]["name"]
            section=archive.read(name)
        parsed=parse(section)
        parsed["rotation_degrees"]=0
        if any(abs(a-b)>0.02 for a,b in zip(parsed["vertex_extents_mm"],dimensions)) and all(
                abs(a-b)<0.02 for a,b in zip(reversed(parsed["vertex_extents_mm"]),dimensions)):
            for loop in parsed["loops"]:
                for v in loop:
                    v[0],v[1]=v[1],-v[0]
            for circle in parsed["circles"]:
                circle["x"],circle["y"]=circle["y"],-circle["x"]
            parsed["vertex_extents_mm"].reverse()
            parsed["rotation_degrees"]=-90
        if any(abs(a-b)>0.02 for a,b in zip(parsed["vertex_extents_mm"],dimensions)):
            raise ValueError("DXF section size differs from product designation")
        family=profile_family(dimensions)
        families.append(family)
        source_sha256=hashlib.sha256(section).hexdigest()
        offers.append({"supplier_id":"vallder","sku":product["sku"],"product_id":product_id,
                       "family_id":family["id"],"source_section_sha256":source_sha256,
                       "url":product["url"],"price_basis":"per_metre","price":product["price"],
                       "availability":product["availability"],"source":product["source"],
                       "cut_length_quote_required":True,"compatibility_verified":False})
        profiles.append({"supplier_id":"vallder","product_id":product_id,"sku":product["sku"],
            "family_id":family["id"],"system_id":family["system_id"],"profile_kind":family["kind"],
            "name":product["name"],"source_url":download["url"],"source_sha256":hashlib.sha256(section).hexdigest(),
            "archive_sha256":download["sha256"],"archive_member":name,"dimensions_mm":dimensions,
            "required_inputs":["length"],"compatibility_verified":False,
            "units_evidence":"DXF extents agree with supplier product dimensions in mm",**parsed})
    write_json(output/"profiles.json",{"schema_version":1,"profiles":profiles})
    write_json(output/"profile-families.json",{"schema_version":1,"families":families,"supplier_offers":offers})
    print(f"Prepared {len(profiles)} length-parametric DXF sections")


if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT)
    main(parser.parse_args().output)
