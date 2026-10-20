import FoundationStudyGeometry.FoundationStudyRegion;
import FoundationStudyGeometry as Geometry;
import materia.casting.BoxAlgebra;
import materia.casting.CastingFeature.CastingFeatureKind;
import machinekit.hobbing.FoundationLayout;
import machinekit.enclosure.PanelCatalogue;
import materia.assembly.AssemblyFrames as F;

private typedef SeatingCase = {
  var id:String;
  var depthMm:Float;
  var panelTopAboveCastingMm:Float;
  var generalCoreRoofDatumClearanceMm:Float;
  var ledges:Array<FoundationStudyRegion>;
  var serviceInterruptions:Array<{ledge:String,feature:String,boundsMm:Array<Float>}>;
  var existingMountInsertConflicts:Array<String>;
}

/** Casting-local interface study. Does not alter the machine or release fabrication dimensions. */
class HobberBaseInterfaceStudy {

  static inline final CONSTRUCTION=machinekit.enclosure.TraySeat.CONSTRUCTION;
  static inline final LEDGE=machinekit.enclosure.TraySeat.WIDTH;
  static inline final SHALLOW_DEPTH=machinekit.enclosure.TraySeat.DEPTH;
  static inline final GASKET=machinekit.enclosure.TraySeat.GASKET; // Assumed compressed thickness, not a selected gasket's free thickness.
  static inline final GASKET_WIDTH=12.0;
  static function overlap(a:FoundationStudyRegion,b:FoundationStudyRegion):Null<Array<Float>> {
    var x=Geometry.bounds(a),y=Geometry.bounds(b),lo=[for(i in 0...3)Math.max(x[i],y[i])],hi=[for(i in 0...3)Math.min(x[i+3],y[i+3])];
    for(i in 0...3)if(hi[i]-lo[i]<=1e-7)return null;
    return lo.concat(hi);
  }
  static function seating(id:String,depth:Float,sheet:Float,d:Array<Float>,cores:Array<FoundationStudyRegion>):SeatingCase {
    var w=d[0],l=d[1],h=d[2];
    if(depth<=0||depth>=h||LEDGE<=GASKET_WIDTH||2*LEDGE>=Math.min(w,l))throw "Invalid interface study stack";
    var ledges=[
      Geometry.region("front-ledge",LEDGE,l,depth,-w/2+LEDGE/2,0,h-depth),
      Geometry.region("rear-ledge",LEDGE,l,depth,w/2-LEDGE/2,0,h-depth),
      Geometry.region("left-ledge",w-2*LEDGE,LEDGE,depth,0,-l/2+LEDGE/2,h-depth),
      Geometry.region("right-ledge",w-2*LEDGE,LEDGE,depth,0,l/2-LEDGE/2,h-depth)
    ];
    var interruptions:Array<{ledge:String,feature:String,boundsMm:Array<Float>}> = [];
    for(ledge in ledges)for(core in cores){var common=overlap(ledge,core);if(common!=null)interruptions.push({ledge:ledge.id,feature:core.id,boundsMm:common});}
    var conflicts:Array<String> = [];
    for(row in FoundationLayout.inserts("bed")) {
      var insert:FoundationStudyRegion=Geometry.insertRegion(row);
      for(ledge in ledges)if(overlap(ledge,insert)!=null)conflicts.push(ledge.id+" / "+row.id);
    }
    if(conflicts.length>0)throw "Proposed ledge crosses existing mounting inserts: "+conflicts.join(", ");
    if(interruptions.length!=1||interruptions[0].feature!="X-drive-channel"||interruptions[0].ledge!="rear-ledge")
      throw "Casting service openings changed: review the perimeter seal route";
    var roof=Math.POSITIVE_INFINITY;
    for(core in cores)if(core.kind==Lightening)roof=Math.min(roof,h-Geometry.bounds(core)[5]-depth);
    if(roof<=0)throw "Seat level reaches the general lightening-core roof datum";
    return {id:id,depthMm:depth,panelTopAboveCastingMm:sheet+GASKET-depth,generalCoreRoofDatumClearanceMm:roof,
      ledges:ledges,serviceInterruptions:interruptions,existingMountInsertConflicts:conflicts};
  }
  public static function main():Void {
    var panel=PanelCatalogue.require(CONSTRUCTION),sheet=panel.thicknessMm();panel.validate();
    if(sheet!=13||Std.string(panel.fabrication)!="RouterCut"||panel.qualifications.coolantRecord!=null||panel.qualifications.heatLimitC!=null)
      throw "Review the preliminary 13 mm router-cut material policy";
    var d=FoundationLayout.dimensions("bed"),cores=Geometry.designRegions();
    // Compare the seats with the pre-rebate machine voids, excluding the now-authored tray features.
    cores=[for(core in cores)if(!StringTools.startsWith(core.id,"tray-"))core];
    var raised=seating("raised",SHALLOW_DEPTH,sheet,d,cores),flush=seating("flush",sheet+GASKET,sheet,d,cores);
    if(raised.panelTopAboveCastingMm!=11||flush.panelTopAboveCastingMm!=0||raised.depthMm>=flush.depthMm)
      throw "Seat stack does not match the intended raised/flush comparison";
    var report={status:"Concept study; dimensions assumed, not released for fabrication",coordinateFrame:"casting-local millimetres",
      castingSizeMm:d,selectedCase:raised.id,selectionReason:"Retain the shallow casting ledge; accommodate panel thickness above it. Machine clearance and drainage remain to check.",
      proposal:{constructionId:panel.id,panelThicknessMm:sheet,panelFabrication:Std.string(panel.fabrication),
        ledgeWidthMm:LEDGE,compressedGasketMm:GASKET,gasketWidthMm:GASKET_WIDTH,
        ledgeDepthMm:raised.depthMm,panelTopAboveCastingMm:raised.panelTopAboveCastingMm,
        fastening:"Local threaded inserts with reserved solid bosses; size, edge cover and pitch require design",
        frameSupport:"Dedicated mounting pads carry frame and door loads; panel requires perimeter and intermediate support"},
      cases:[raised,flush],
      roofMetric:"Seat level minus the highest general lightening-core roof. A conservative datum comparison, not remaining wall thickness or structural qualification.",
      requiredNextDetails:["Check the raised panel and supports against the full moving-machine envelope",
        "Design inner upstand / sealed collar and drainage so the raised panel cannot trap coolant on the casting",
        "Removable sealed collar/bridge at the rear X-drive opening, preserving motor access",
        "Support spacing and panel deflection under coolant, chip and maintenance loads",
        "Select chemical-grade laminate and qualify actual coolant, cleaners, chip heat and cut edges",
        "Local anchor bosses, edge cover, clamp loads, gasket selection, compression stops and sealed corners",
        "Dedicated frame mounting pads and differential-expansion allowance"],
      checks:{existingMountsClear:true,noGeneralCoreIntersection:true,rearServiceBreakDetected:true,stackHeightsConsistent:true,
        structuralQualification:false,leakQualification:false,thermalQualification:false}};
    var root=Sys.getCwd();while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FlexsplineHobber.hx")){
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot locate hobber worktree";root=parent;
    }
    var out=root+"/machinekit/examples/flexspline-hobber/build/base-interface";
    sys.FileSystem.createDirectory(haxe.io.Path.directory(out));sys.FileSystem.createDirectory(out);
    sys.io.File.saveContent(out+"/interface.json",haxe.Json.stringify(report,null,"  "));
    sys.io.File.saveContent(out+"/interface.svg",drawing(d,sheet,cores,raised,flush));
    Sys.println('Base interface study passed: ${sheet} mm laminate; raised seat ${raised.depthMm} mm / panel +${raised.panelTopAboveCastingMm} mm; flush seat ${flush.depthMm} mm. Both clear existing inserts; rear drive opening requires a sealed bridge.');
  }
  static function drawing(d:Array<Float>,sheet:Float,cores:Array<FoundationStudyRegion>,raised:SeatingCase,flush:SeatingCase):String {
    var w=d[0],l=d[1],h=d[2],s=Math.min(610/w,400/l),ox=65.0,oy=150.0;
    function px(x:Float):Float return ox+(x+w/2)*s;
    function py(y:Float):Float return oy+(y+l/2)*s;
    var svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1200" viewBox="0 0 1600 1200">',
      '<style>text{font:18px "DejaVu Sans",sans-serif;fill:#243746}.title{font-size:28px;font-weight:700}.small{font-size:15px}.label{font-weight:700}.note{fill:#9a4122}</style>',
      '<rect width="1600" height="1200" fill="#fafcfe"/>',
      '<text x="55" y="48" class="title">Hobber base — 13 mm router-cut compact laminate</text>',
      '<text x="55" y="80" class="note">DESIGN STUDY · assumed dimensions · casting and enclosure CAD remain unchanged</text>',
      '<text x="65" y="122" class="label">CASTING PLAN / PROPOSED PERIMETER SEAT</text>',
      '<rect x="$ox" y="$oy" width="${w*s}" height="${l*s}" fill="#e2e7eb" stroke="#617384" stroke-width="2"/>'];
    for(ledge in raised.ledges){var b=Geometry.bounds(ledge);svg.push('<rect x="${px(b[0])}" y="${py(b[1])}" width="${(b[3]-b[0])*s}" height="${(b[4]-b[1])*s}" fill="#f7d99a" stroke="#b97c20"/>');}
    for(core in cores){var b=Geometry.bounds(core);if(b[5]>h-raised.depthMm&&b[2]<h){
      var x0=Math.max(-w/2,b[0]),x1=Math.min(w/2,b[3]),y0=Math.max(-l/2,b[1]),y1=Math.min(l/2,b[4]);
      svg.push('<rect x="${px(x0)}" y="${py(y0)}" width="${(x1-x0)*s}" height="${(y1-y0)*s}" fill="#c7dfed" stroke="#51768c"/>');
    }}
    var opening=raised.serviceInterruptions[0].boundsMm;
    svg.push('<rect x="${px(opening[0])}" y="${py(opening[1])}" width="${(opening[3]-opening[0])*s}" height="${(opening[4]-opening[1])*s}" fill="#f5b5a7" stroke="#b83225" stroke-width="3"/>');
    svg.push('<text x="$ox" y="${oy+l*s+30}">X front ← ${w} mm → X rear · depth ${l} mm</text>');
    var lines=["Selected study: 25 mm-wide ledge, 4 mm deep.","13 mm panel + 2 mm compressed gasket: top +11 mm.","Flush alternative needs a 15 mm-deep ledge.","Both options clear existing mounting inserts.","Rear drive opening interrupts both seat options.","Red area: removable sealed service bridge required.","Frame / door loads use dedicated mounting pads.","Panel spans, drainage and material grade still to qualify."];
    for(i in 0...lines.length)svg.push('<text x="665" y="${165+i*39}"${i==4?' class="note"':""}>${lines[i]}</text>');
    svg.push('<text x="65" y="635" class="label">EDGE SECTIONS — same scale, away from service openings</text>');
    function section(c:SeatingCase,x:Float):Void {
      var by=785.0,k=5.0,bottom=-c.depthMm;
      function sx(v:Float):Float return x+v*k;
      function sy(v:Float):Float return by-v*k;
      svg.push('<text x="${x-250}" y="680" class="label">${c.id=="raised"?"A — RAISED / SELECTED":"B — FLUSH / ALTERNATIVE"}</text>');
      svg.push('<path d="M ${sx(0)} ${sy(-40)} L ${sx(55)} ${sy(-40)} L ${sx(55)} ${sy(0)} L ${sx(25)} ${sy(0)} L ${sx(25)} ${sy(bottom)} L ${sx(0)} ${sy(bottom)} Z" fill="#d8dfe5" stroke="#617384" stroke-width="2"/>');
      svg.push('<rect x="${sx(-45)}" y="${sy(c.panelTopAboveCastingMm)}" width="${69*k}" height="${sheet*k}" fill="#6489a4" stroke="#385970"/>');
      svg.push('<rect x="${sx(12)}" y="${sy(bottom+GASKET)}" width="${GASKET_WIDTH*k}" height="${GASKET*k}" fill="#4f9479"/>');
      svg.push('<path d="M ${sx(-48)} $by H ${sx(59)}" stroke="#788795" stroke-dasharray="5 5"/>');
      svg.push('<text x="${x-250}" y="1020" class="small">Seat depth ${c.depthMm} mm · panel top +${c.panelTopAboveCastingMm} mm</text>');
      svg.push('<text x="${x-250}" y="1050" class="small">General-core roof datum clearance: ${c.generalCoreRoofDatumClearanceMm} mm*</text>');
    }
    section(raised,340);section(flush,1120);
    svg.push('<text x="65" y="1100" class="small">Blue: compact laminate · green: compressed gasket · grey: casting · dashed: casting top datum.</text>');
    svg.push('<text x="65" y="1130" class="small">*Datum clearance is not remaining wall thickness or a strength calculation. Raised inner edge needs a sealed drainage detail.</text>');
    svg.push('<text x="65" y="1160" class="small">Supports, clamp strips, local anchor bosses, thermal movement and coolant / chip-heat suitability remain design work.</text></svg>');
    return svg.join("\n");
  }
}
