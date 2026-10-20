import machinekit.hobbing.FoundationLayout;
import machinekit.structural.CastingFeatures as C;
import machinekit.structural.CastingFeatures.CastingBox;
import machinekit.structural.CastingCells;
import materia.assembly.AssemblyFrames as F;

/** Candidate placement, not an anchor qualification or an alteration to the working machine. */
class HobberTrayAnchorStudy {
  static function intersects(a:Array<Float>,b:Array<Float>):Bool {
    for(i in 0...3)if(Math.min(a[i+3],b[i+3])-Math.max(a[i],b[i])<1e-6)return false;
    return true;
  }
  static function edge(x:Float,y:Float,d:Array<Float>):Float return Math.min(d[0]/2-Math.abs(x),d[1]/2-Math.abs(y));
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FoundationLayout.hx")&&
        !sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FlexsplineHobber.hx")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find hobber worktree";root=parent;
    }
    var out=root+"/machinekit/examples/flexspline-hobber/build/tfixx-anchor";
    if(!sys.FileSystem.exists(out+"/cad-check.json"))throw "Run inspect-tfixx-reference.py on the manufacturer's DXF first";
    var reference:Dynamic=haxe.Json.parse(sys.io.File.getContent(out+"/cad-check.json"));
    if(reference.measuredBodyLengthMm!=50||reference.measuredSleeveDiameterMm!=13.5||reference.currentCatalogueOrder!="1000200636")throw "Anchor reference changed";
    var d=FoundationLayout.dimensions("bed"),z=d[2]-machinekit.enclosure.TraySeat.DEPTH;
    var points:Array<{id:String,x:Float,y:Float,edgeMm:Float}> = [];
    function add(x:Float,y:Float):Void points.push({id:"trayAnchor"+points.length,x:x,y:y,edgeMm:edge(x,y,d)});
    // A 5 mm placement allowance beyond the catalogue's 50 mm concrete edge minimum.
    for(side in [-1,1])for(x in [-365.0,0.0,365.0])add(x,side*(d[1]/2-55));
    // The front work well requires these positions to move sideways as well as inward.
    for(y in [-230.0,230.0])add(-d[0]/2+55,y);
    for(y in [-150.0,150.0])add(d[0]/2-55,y);
    var bosses:Array<CastingBox> = [],tabs:Array<CastingBox> = [];
    for(p in points) {
      bosses.push(C.box(p.id+"-backing",70,70,70,p.x,p.y,z-70));
      // Local extensions of the 25 mm rebate; do not cut the rail seats with a wider continuous rebate.
      if(Math.abs(p.y)>300)tabs.push(C.box(p.id+"-seat",60,80,4,p.x,p.y<0?-d[1]/2+40:d[1]/2-40,z));
      else tabs.push(C.box(p.id+"-seat",80,60,4,p.x<0?-d[0]/2+40:d[0]/2-40,p.y,z));
    }
    var cores=C.decode(FoundationLayout.bodyValues("bed").token("features"));
    var lightening=[for(c in cores)if(StringTools.startsWith(c.id,"core-"))c];
    var service=[for(c in cores)if(!StringTools.startsWith(c.id,"core-"))c];
    var remaining=CastingCells.outsideBoxes(lightening,[for(b in bosses)CastingCells.bounds(b)]).concat(service);
    var inserts:Array<CastingBox> = [for(r in FoundationLayout.inserts("bed"))if(!StringTools.startsWith(r.id,"trayClampSeat"))
      {id:r.id,width:r.width,depth:r.depth,height:r.height,pose:r.pose}];
    var minEdge=1e30,minSpacing=1e30,minServiceEdge=1e30;
    for(i in 0...points.length) {
      var p=points[i];minEdge=Math.min(minEdge,p.edgeMm);
      // Conservative 24 mm square envelope includes the source drawing's wide clip projection.
      var anchor=C.box(p.id,24,24,50,p.x,p.y,z-50),ab=CastingCells.bounds(anchor);
      for(c in remaining)if(intersects(ab,CastingCells.bounds(c)))throw 'Candidate ${p.id} intersects void ${c.id}';
      for(c in service) {
        var b=CastingCells.bounds(c);
        if(Math.min(ab[5],b[5])-Math.max(ab[2],b[2])<1e-6)continue;
        var dx=Math.max(0,Math.max(b[0]-p.x,p.x-b[3])),dy=Math.max(0,Math.max(b[1]-p.y,p.y-b[4]));
        minServiceEdge=Math.min(minServiceEdge,Math.sqrt(dx*dx+dy*dy));
      }
      for(r in inserts)if(intersects(ab,CastingCells.bounds(r)))throw 'Candidate ${p.id} intersects existing insert ${r.id}';
      for(r in inserts)if(intersects(CastingCells.bounds(tabs[i]),CastingCells.bounds(r)))throw 'Local seat ${p.id} cuts insert ${r.id}';
      for(r in inserts)if(intersects(CastingCells.bounds(bosses[i]),CastingCells.bounds(r)))throw 'Backing ${p.id} conflicts with insert ${r.id}';
      for(j in i+1...points.length)minSpacing=Math.min(minSpacing,Math.sqrt(Math.pow(p.x-points[j].x,2)+Math.pow(p.y-points[j].y,2)));
    }
    if(minEdge<50||minSpacing<100||minServiceEdge<50)throw "Candidate fails catalogue layout screening";
    var wideRebateConflicts:Array<String> = [];
    var wideFront=C.box("wide-front-ledge",80,d[1],4,-d[0]/2+40,0,z);
    for(c in service)if(c.id=="work-well"&&intersects(CastingCells.bounds(wideFront),CastingCells.bounds(c)))wideRebateConflicts.push(c.id);
    if(wideRebateConflicts.indexOf("work-well")<0)throw "Expected front work-well interruption changed";
    var wideRear=C.box("wide-rear-ledge",80,d[1],4,d[0]/2-40,0,z),rearInsertConflicts=[];
    for(r in inserts)if(intersects(CastingCells.bounds(wideRear),CastingCells.bounds(r)))rearInsertConflicts.push(r.id);
    var oldPositions=[for(r in FoundationLayout.trayClampSeats()) {id:r.id,x:r.pose.x,y:r.pose.y,edgeMm:edge(r.pose.x,r.pose.y,d)}];
    var report={status:"Candidate placement screened; not integrated or released",coordinateFrame:"Casting-local XY, Z from casting sole, mm",
      reference:reference,castingSizeMm:d,anchors:points,count:points.length,socketFaceZMm:z,
      bodyEnvelopeMm:[24,24,50],envelopeNote:"Conservative square screening volume; supplied DXF is a 2D side view, not a 3D solid or thread model",
      catalogueConcreteScreen:{minimumEdgeMm:50,minimumSpacingMm:100,actualMinimumEdgeMm:minEdge,actualMinimumSpacingMm:minSpacing,actualMinimumServiceVoidEdgeMm:minServiceEdge},
      oldClampPositions:oldPositions,localRebates:tabs,reservedSolidBosses:bosses,uniformRebateReview:{widthMm:80,frontServiceInterruptions:wideRebateConflicts,rearInsertConflicts:rearInsertConflicts},
      integrationRequired:["Replace old tray clamp seats; these anchors do not fit the current 25 mm ledge",
        "Add the local 80 mm-deep plan tabs and reserve the proposed backing before cutting lightening cores",
        "Route the single-piece tray opening and gasket around the new local tabs",
        "Select recessed sealed M10 fastening details or a specified thread adapter; check the 13 mm panel bearing thickness",
        "Retain the rear motor cover separately; its old nearby clamp points do not meet these anchor spacing/edge screens",
        "Confirm current part/drawing equivalence and clip detail, epoxy-mineral substrate suitability and actual capacities with supplier"],
      qualification:{geometryScreen:true,concreteRatingsApplicableToEpoxy:false,anchorageStrength:false,leakTightness:false,fullMachineTravel:false}};
    sys.io.File.saveContent(out+"/placement.json",haxe.Json.stringify(report,null,"  "));
    sys.io.File.saveContent(out+"/placement.svg",drawing(d,points,tabs,bosses,inserts,service));
    Sys.println('T-FIXX placement passed: ${points.length} candidate positions, minimum edge $minEdge mm, minimum spacing $minSpacing mm; local tabs clear retained inserts and reserved anchors clear voids. Uniform front rebate intersects the work well; use local tabs.');
  }
  static function drawing(d:Array<Float>,points:Array<{id:String,x:Float,y:Float,edgeMm:Float}>,tabs:Array<CastingBox>,bosses:Array<CastingBox>,inserts:Array<CastingBox>,voids:Array<CastingBox>):String {
    var s=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="1250" viewBox="0 0 1200 1250"><rect width="1200" height="1250" fill="#fafbfc"/><style>text{font-family:sans-serif;fill:#263746}.small{font-size:17px}.label{font-size:22px}</style>',
      '<text x="70" y="50" font-size="28">T-FIXX M10 × 50 A4 — candidate placement</text>',
      '<text x="70" y="83" class="small">Casting-local plan. Geometric screening only; epoxy anchorage is unqualified.</text>',
      '<g transform="translate(600,535) scale(1,-1)">'];
    function rect(b:Array<Float>,fill:String,stroke:String,opacity:Float=1):Void s.push('<rect x="${b[0]}" y="${b[1]}" width="${b[3]-b[0]}" height="${b[4]-b[1]}" fill="$fill" stroke="$stroke" opacity="$opacity"/>');
    rect([-d[0]/2,-d[1]/2,0,d[0]/2,d[1]/2,1],"#dce3e8","#455d70");
    for(c in voids)if(c.id=="work-well"||c.id=="X-drive-channel")rect(CastingCells.bounds(c),"#acb9c3","#8b9daa");
    for(r in inserts)if(r.id=="railLeft"||r.id=="railRight"||StringTools.startsWith(r.id,"lift"))rect(CastingCells.bounds(r),"#7296b9","#355c81");
    for(c in FoundationLayout.trayLedges())rect(CastingCells.bounds(c),"#f5dfaf","#c4a46d",0.8);
    for(b in bosses)rect(CastingCells.bounds(b),"none","#489b68");
    for(t in tabs)rect(CastingCells.bounds(t),"#eab36b","#b67a2f",0.65);
    for(p in points)s.push('<circle cx="${p.x}" cy="${p.y}" r="12" fill="#f7fff9" stroke="#196239" stroke-width="2"/><circle cx="${p.x}" cy="${p.y}" r="5" fill="#257447"/>');
    s.push('</g><text x="70" y="985" class="label">10 positions · 55 mm minimum exterior edge distance · ≥100 mm spacing</text>');
    s.push('<text x="70" y="1020" class="small">Green: anchor screening envelopes / reserved backing · orange: required local tray-seat tabs</text>');
    s.push('<text x="70" y="1050" class="small">Blue: retained rail / lifting inserts · grey: work well and rear drive channel · beige: existing 25 mm rebate</text>');
    s.push('<text x="70" y="1100" class="label">Source check: 50 mm body / 13.5 mm sleeve agree with the 2026 catalogue.</text>');
    s.push('<text x="70" y="1135" class="small">Legacy CAD clip: 1.5 mm; current nominal clip: 2 mm. Check SKU and seating with supplier.</text>');
    s.push('<text x="70" y="1170" class="small">Current six-piece tray / clamps remain unchanged. One-piece tray needs tongues, gasket and screw-seat details.</text>');
    s.push('<text x="70" y="1205" class="small">An 80 mm-wide front rebate reaches the work well. Local tabs preserve that opening.</text></svg>');
    return s.join("\n");
  }
}
