#!/usr/bin/env python3
"""Rectangular direct-mount tray proposal, separate from the accepted enclosure.
Casting-local XY, mm. Design proposal, not CAM or fabrication release.
"""
import hashlib
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def rectangle(r, clockwise=False):
    x0,y0,x1,y1=r
    loop=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    return loop[::-1] if clockwise else loop

def offset_rectangle(r, distance):
    x0,y0,x1,y1=r
    return (x0-distance,y0-distance,x1+distance,y1+distance)

def main():
    layout_path=ROOT/'build/enclosure/layout.json'
    layout=json.loads(layout_path.read_text()); b=layout['input']['plinth']
    width=b['maxY']-b['minY']+50; depth=b['maxX']-b['minX']+50
    if abs(width-930)>.001 or abs(depth-800)>.001:
        raise ValueError('Casting changed; review rectangular tray before regeneration')
    outer_w,outer_d=layout['dimensionsMm'][1]-4,layout['dimensionsMm'][0]-4
    outer=(-outer_w/2,-outer_d/2,outer_w/2,outer_d/2)
    # Long-side fixings permit a genuinely rectangular opening: no anchor tongues or motor notch.
    hole=(-440.,-290.,465.,290.)
    points=[(x,side*315.) for side in (-1,1) for x in (-380.,-190.,0.,190.,380.)]
    outer_loop=rectangle(outer); inner_loop=rectangle(hole,True)
    inner_tape=rectangle(offset_rectangle(hole,7.5),True)
    outer_tape=rectangle(offset_rectangle(outer,-12.5))
    min_edge=min(min(width/2-abs(x),depth/2-abs(y)) for x,y in points)
    min_ligament=min(abs(y)-290 for x,y in points)
    if min_edge<80 or min_ligament-8-15<0:
        raise ValueError('Socket edge or screw-to-gasket clearance failed')
    if len(outer_loop)!=4 or len(inner_loop)!=4: raise ValueError('Tray must have two rectangles')
    length=lambda loop:sum(math.dist(a,b) for a,b in zip(loop,loop[1:]+loop[:1]))
    tape_length=length(inner_tape)+length(outer_tape)
    source=json.loads((ROOT/'tray-anchor-sourcing.json').read_text())
    proposal={
      'status':'rectangular design proposal; separate from accepted scene; not fabrication release',
      'revision':'rectangular-long-side-fixings-v2',
      'inputLayoutSha256':hashlib.sha256(layout_path.read_bytes()).hexdigest(),
      'coordinateFrame':'Casting-local XY, mm; heights relative to casting top',
      'tray':{'pieces':1,'construction':'compact-laminate-13, coolant/edge treatment unqualified',
        'blankMm':[outer_w,outer_d,13],'openingBoundsMm':hole,'openingSizeMm':[905,580],
        'outlineLoopsMm':[outer_loop,inner_loop],'installation':'Install before upper machine assembly; assembled-machine removal unverified.',
        'rearMotorCover':'No notch or separate cover in this proposal. Rectangular opening extends to casting rear edge; complete-machine clearance still to check.'},
      'sockets':{'count':10,'thread':'M6','candidateLengthMm':40,'mouthZMm':0,'positionsMm':points,
        'minimumOutsideEdgeMm':min_edge,'supplierSelection':None,
        'reference':'PFEIFER 05.250.064.040; M6 x 40; insertion dimension a=25, thread dimension e=6 mm',
        'retentionGeometry':'Nominal cylindrical socket and crimped/waved tail; outside diameter and tail dimensions assumed, not vendor CAD.',
        'spacingAndInternalEdgeQualification':False},
      'casting':{'localRebatesXYMm':[],'rebateDepthMm':0,'backing':'Reserve solid casting around anchors; current 4 mm ledges are superseded by top-level support in this proposal.',
        'seatChange':'Seat at casting top, rather than a recessed floor. Tray underside +2 clears existing lifting-insert tops at 0.',
        'rearSealSupportRequired':True},
      'screwSeatProposal':{'screw':'ISO 4762 M6 x 25; head diameter 10, height 6 mm nominal',
        'counterboreDiameterMm':16,'counterboreDepthMm':7,'remainingLaminateMm':6,'sleeveClearanceHoleMm':10.5,
        'sleeve':{'outerDiameterMm':10,'innerDiameterMm':6.6,'lengthMm':8},
        'washer':{'outerDiameterMm':14,'innerDiameterMm':6.6,'thicknessMm':1,'note':'Material and sealing detail unselected'},
        'nominalSocketInsertionMm':16,'catalogueThreadDimensionMm':6,'catalogueInsertionDimensionMm':25,
        'compressionLoadPath':'Head -> washer -> removable sleeve -> flush socket face; supported gasket carries sheet',
        'minimumHeadRecessToSealBandMm':min_ligament-8-15,
        'note':'Insertion depth is not guaranteed threaded engagement. Supplier thread and screw compatibility still to confirm.'},
      'gasket':dict(source['gasketRequirement'],innerCentrelineMm=inner_tape,outerCentrelineMm=outer_tape,
        netLengthMm=tape_length,suggestedPurchaseLengthM=math.ceil(tape_length*1.15/1000),
        quantity=math.ceil(tape_length*1.15/1000),rearTransitionSupported=False,
        note='Two rectangular loops. Rear inner return lies beyond casting and requires an underside support tied to frame; no raised-cover transition.'),
      'stackZMm':{'castingTop':0,'socketMouthAndSeat':0,'trayUnderside':2,'trayTop':15,
        'sleeveTopAndWasherUnderside':8,'screwHeadUnderside':9,'screwTip':-16},
      'checks':{'singleConnectedSheet':True,'oneRectangularOpening':True,'drySideMainScrewSeats':True,
        'minimumSocketAxisToOpeningMm':min_ligament,'fullMachineTravel':False,'trayRemoval':False,'leakTightness':False},
      'bomCandidates':[{'partNumber':source['requirement']['partNumber'],'quantity':10,'status':'unselected'},
        {'partNumber':'HOBBER-TRAY-GASKET-TAPE-CANDIDATE','quantity':math.ceil(tape_length*1.15/1000),'unit':'m','status':'grade unselected'},
        {'partNumber':'HOBBER-TRAY-COMPRESSION-SLEEVE-PROPOSAL','quantity':10,'status':'dimensions provisional'}]}
    count=len(points)
    def item(number,description,quantity,material):
        return dict(partNumber=number,description=description,quantity=quantity,material=material)
    items=[
        item('HOBBER-TRAY-SHEET-RECT-V2','One-piece routed compact-laminate tray, rectangular opening, 13 mm',1,'compact laminate'),
        item(source['requirement']['partNumber'],'M6 x 40 cast-in fixing socket, retained end; supplier selection pending',count,'metal; grade depends on selected socket'),
        item('HOBBER-TRAY-SCREW-ISO4762-M6X25','ISO 4762 M6 x 25 socket head screw; material class unselected',count,'stainless steel, class unselected'),
        item('HOBBER-TRAY-WASHER-14X6.6X1','Load-spreading washer OD14 / ID6.6 / thickness1 mm, provisional',count,'metal; grade unselected'),
        item('HOBBER-TRAY-COMPRESSION-SLEEVE-PROPOSAL','Unthreaded M6 compression spacer OD10 x length8 mm; bore/tolerance review pending',count,'stainless steel'),
        item('HOBBER-TRAY-GASKET-TAPE-CANDIDATE','Single-sided adhesive closed-cell tape 15 x 3 mm; quantity in metres',math.ceil(tape_length*1.15/1000),None)]
    notes={
      items[0]['partNumber']:dict(unit='piece',blankMm=[outer_w,outer_d,13],cutoutMm=[905,580],status='design proposal; machining and coolant qualification pending'),
      items[1]['partNumber']:dict(unit='piece',status='unselected candidate',note='10 required, not 10 supplier packs. Concrete rating not a mineral-cast rating.'),
      items[2]['partNumber']:dict(unit='piece',status='size proposed; grade and supplier unselected',note='25 mm under-head length. 16 mm insertion is not a claim of 16 mm threaded engagement.'),
      items[3]['partNumber']:dict(unit='piece',status='provisional dimensions; stiffness and sealing review pending',note='No standard washer SKU claimed for these dimensions.'),
      items[4]['partNumber']:dict(unit='piece',status='Ettinger candidate, not yet selected',modelBoreMm=6.6,candidateBoreMm=6.4,lengthToleranceMm=None),
      items[5]['partNumber']:dict(unit='m',status='rubber grade/adhesive/supplier unselected',netLengthM=tape_length/1000,purchaseAllowanceFraction=.15,note='Rounded purchase length, not metres plus a second allowance. Roll lengths/minimum order unknown.')}
    bom=dict(schemaVersion=1,scope='Complete known tray-proposal BOM only; replacement design, not additive to the accepted enclosure BOM.',
        revision=proposal['revision'],status='design proposal; not purchasing release',items=items,lineNotes=notes,
        sourcingFile='tray-anchor-sourcing.json',
        openItems=[
          dict(description='Rear inner-gasket support and its fasteners',quantity=None,status='required; geometry/quantity not yet designed'),
          dict(description='Gasket corner/joint sealant and cut-edge treatment',quantity=None,status='material and amount depend on selected gasket/coolant'),
          dict(description='Temporary M6 socket-positioning bolts and washers for mould',quantity=None,status='tooling, not permanent installed BOM; length depends on mould thickness; may reuse compatible hardware')],
        excluded='Existing six-piece tray, old clamps and frame hardware are not added here. Independent frame-support BOM remains with the accepted enclosure.')
    (ROOT/'tray-bom.json').write_text(json.dumps(bom,indent=2)+'\n')
    proposal['bomCandidates']=[dict(partNumber=i['partNumber'],quantity=i['quantity'],unit=notes[i['partNumber']]['unit'],status=notes[i['partNumber']]['status']) for i in items]
    (ROOT/'tray-mounting-proposal.json').write_text(json.dumps(proposal,indent=2)+'\n')
    def path(loop):return 'M '+' L '.join(f'{x:.3f},{y:.3f}' for x,y in loop)+' Z'
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="1350" viewBox="0 0 1280 1350">',
      '<rect width="1280" height="1350" fill="#fafbfc"/><style>text{font-family:sans-serif;fill:#243746}.note{font-size:18px}</style>',
      '<text x="50" y="45" font-size="28">Rectangular tray / rectangular opening / direct M6 mounting</text>',
      '<text x="50" y="78" class="note">Proposal v2 — independent of the accepted enclosure scene.</text>',
      '<g transform="translate(640,600) scale(1,-1)">',
      f'<path d="{path(outer_loop)} {path(inner_loop)}" fill="#dee8ee" fill-rule="evenodd" stroke="#38556a" stroke-width="2"/>']
    for tape in (inner_tape,outer_tape):svg.append(f'<path d="{path(tape)}" fill="none" stroke="#449e79" stroke-width="15" opacity="0.7"/>')
    for x,y in points:svg.append(f'<circle cx="{x}" cy="{y}" r="8" fill="#fff" stroke="#324c60"/><circle cx="{x}" cy="{y}" r="5.25" fill="#536d82"/>')
    svg+=['</g>',f'<text x="50" y="1130" class="note">Blank {outer_w:.2f} × {outer_d:.2f} × 13 mm · opening 905 × 580 mm · 10 M6 fixings</text>',
      f'<text x="50" y="1160" class="note">Green: two rectangular 15 × 3 mm tape loops, installed at 2 mm · net {tape_length/1000:.2f} m</text>',
      '<text x="50" y="1190" class="note">Seat at casting top 0 / tray underside +2 / tray top +15 mm. No inward tabs or motor notch.</text>',
      '<text x="50" y="1220" class="note">Socket: M6 × 40, reference a=25 / e=6 mm. Outside diameter and crimp shape remain nominal.</text>',
      '<text x="50" y="1250" class="note">OPEN: rear gasket support; complete-machine clearance; anchor geometry and retention qualification.</text>',
      '<text x="50" y="1280" class="note">M6 × 25 insertion is 16 mm; this does not imply 16 mm of threaded engagement.</text>',
      '<text x="50" y="1310" class="note">Design proposal only: actual coolant grade, bearing and seal compression still to verify.</text></svg>']
    (ROOT/'tray-mounting-proposal.svg').write_text('\n'.join(svg)+'\n')
    print(json.dumps({'positions':10,'openingMm':[905,580],'outsideEdgeMm':min_edge,'headToSealGapMm':min_ligament-23,
        'tapeNetM':tape_length/1000,'purchaseM':math.ceil(tape_length*1.15/1000)},indent=2))

if __name__=='__main__':main()
