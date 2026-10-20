import machinekit.hobbing.FoundationLayout;
import machinekit.structural.CastingFeatures as C;
import machinekit.structural.CastingCells;

/** Independent M6 tray proposal: placement screening, no supplier geometry or strength claim. */
class HobberTrayMountingStudy {
  static function intersects(a:Array<Float>,b:Array<Float>):Bool {
    for(i in 0...3)if(Math.min(a[i+3],b[i+3])-Math.max(a[i],b[i])<1e-6)return false;
    return true;
  }
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/tray-mounting-proposal.json")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find proposal";root=parent;
    }
    var folder=root+"/machinekit/examples/flexspline-hobber",path=folder+"/tray-mounting-proposal.json";
    var proposal:Dynamic=haxe.Json.parse(sys.io.File.getContent(path));
    var points:Array<Array<Float>> = proposal.sockets.positionsMm,tabs:Array<Array<Float>> = proposal.casting.localRebatesXYMm;
    var z=FoundationLayout.dimensions("bed")[2]+proposal.sockets.mouthZMm;
    var cores=C.decode(FoundationLayout.bodyValues("bed").token("voids"));
    var bosses=[for(p in points)[p[0]-25,p[1]-25,z-65,p[0]+25,p[1]+25,z]];
    var lightening=[for(c in cores)if(StringTools.startsWith(c.id,"core-"))c];
    var service=[for(c in cores)if(!StringTools.startsWith(c.id,"core-"))c];
    var remaining=CastingCells.outsideBoxes(lightening,bosses).concat(service);
    var conflicts:Array<String> = [];
    for(i in 0...points.length) {
      var p=points[i];
      // 24 mm square deliberately bounds an unknown M6 retention detail; not nominal vendor CAD.
      var envelope=[p[0]-12,p[1]-12,z-40,p[0]+12,p[1]+12,z];
      for(c in remaining)if(intersects(envelope,CastingCells.bounds(c)))conflicts.push('Anchor $i / void ${c.id}');

      for(r in FoundationLayout.inserts("bed"))if(!StringTools.startsWith(r.id,"trayClampSeat")) {
        var insert=CastingCells.bounds({id:r.id,width:r.width,depth:r.depth,height:r.height,pose:r.pose});
        if(intersects(envelope,insert))conflicts.push('Anchor $i / retained insert ${r.id}');
        for(t in tabs)if(intersects([t[0],t[1],z,t[2],t[3],z+4],insert))conflicts.push('Rebate / retained insert ${r.id}');
      }
    }
    var report={status:conflicts.length==0?"Proposed envelopes and rebates clear; full integration pending":"Conflicts require redesign",conflicts:conflicts,
      revision:proposal.revision,anchorCount:points.length,screeningEnvelopeMm:[24,24,40],reservedBackingMm:[50,50,65],socketFaceCastingLocalZMm:z,
      actualSupplierRetentionGeometry:false,fullMachineTravel:false,rearCoverFastening:false,epoxyAnchorageQualified:false,
      note:"Old tray-clamp seats excluded because replacement is proposed. Only general lightening cells are trimmed around proposed backing; service voids remain unchanged."};
    sys.io.File.saveContent(folder+"/tray-mounting-screen.json",haxe.Json.stringify(report,null,"  "));
    if(conflicts.length>0)throw conflicts.join("\n");
    Sys.println('M6 tray proposal: ${points.length} anchor envelopes and local rebates clear retained inserts and service voids with reserved backing; no supplier or full-machine qualification.');
  }
}
