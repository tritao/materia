import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import machinekit.standard.FlatWasher;
import machinekit.tooling.SocketFixtureCad;
import machinekit.tooling.SocketCoupon;
import machinekit.tooling.ToolingPanel;
import machinekit.tooling.SweptRemoval;
import materia.tooling.casting.SocketFixturePlanner;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

/** Two-socket manufacturing review through existing panel/CAD/removal tooling. */
class HobberSocketFixture {
  static var assertions=0;
  static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
  static function rejects(action:Void->Void):Bool {try action()catch(_:Dynamic)return true;return false;}
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/HobberSocketFixture.hx")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find hobber worktree";root=parent;
    }
    var out=root+"/machinekit/examples/flexspline-hobber/build/socket-fixture";sys.FileSystem.createDirectory(out);
    var washer=FlatWasher.metric("M6"),p=SocketFixturePlanner.coupon(null,18,30,washer.thickness);
    check(p.sites.length==2,"Coupon needs two sockets");
    check(rejects(()->SocketFixturePlanner.coupon(null,18,20,washer.thickness)),"Reject positioning screw shorter than thread engagement");
    check(rejects(()->SocketFixturePlanner.coupon(null,18,50,washer.thickness)),"Reject screw bottoming in blind cavity");
    check(p.positionToleranceMm==null,"Unqualified fixture must not invent an accuracy tolerance");
    check(!ToolingReadiness.evaluate(p.validation).ready,"Geometric success must not qualify the manufacturing process");
    var panelPlan=SocketFixtureCad.panels(p);PanelAssemblyPlan.validate(panelPlan);
    for(panel in [p.plates.fixed,p.plates.moving])check(PanelCodec.encode(PanelCodec.decode(PanelCodec.encode(panel)))==PanelCodec.encode(panel),"Panel record roundtrip");
    var assembly=SocketFixtureCad.assembly(p,true),parts=assembly.components(),poses=assembly.solvedPoses(),cache=new PosedParts();
    check(assembly.validate().length==0,"Fixture assembly validates");
    var rebuilt=MachineAssembly.decode(assembly.encode());check(rebuilt.components().length==parts.length,"Fixture assembly recipes roundtrip");
    var exactPairs=0;
    try {
      // Check all initial contacts, including standard screws, washers, nuts, pins and actual crimped sockets.
      for(i in 0...parts.length)for(j in i+1...parts.length) {
        var a=parts[i],b=parts[j],pa=poses.get(a.id),pb=poses.get(b.id);
        if(PosedParts.apart(cache.box(a.component,pa),cache.box(b.component,pb),0))continue;
        exactPairs++;check(cache.overlap(a.component,pa,b.component,pb)<0.001,'Initial overlap ${a.id}/${b.id}');
      }
      // All socket mouths sit exactly on the common form face; bores point toward the accessible screws.
      for(s in p.sites) {
        var row=[for(r in parts)if(r.id==s.id)r][0],mouth=F.transformPoint(poses.get(s.id),0,0,0);
        check(Math.abs(mouth.z-p.mouthZ)<1e-7&&Math.abs(mouth.x-s.x)+Math.abs(mouth.y-s.y)<1e-7,"Socket mouth datum");
        check(s.insertionMm>=p.socket.threadLength&&s.insertionMm<p.socket.boreDepth,"Screw fits available insertion cavity");
        var mouthSealWidth=(s.seal.footprintOutsideDiameter-s.seal.clearanceDiameter)/2;
        check(mouthSealWidth>0&&!s.seal.qualified,"Explicit but unqualified mouth sealing land");
      }
      // The full exterior is excluded from casting material, preserving the protected blind cavities.
      var coupon=[for(r in parts)if(r.id=="cured-coupon")r][0];
      var castShape=cache.posed(coupon.component,poses.get(coupon.id)),excludedVolume=0.0;
      for(s in p.sites) {
        var outer=SocketFixtureCad.socket(p.socket).geometry(Envelope),at=PosedParts.moved(outer,s.mouth);outer.close();
        check(SweptRemoval.intersectionVolume(castShape,at)<0.001,"Casting does not fill protected blind socket volume");
        excludedVolume+=at.volume();at.close();
      }
      check(Math.abs(castShape.volume()+excludedVolume-p.cavity.width*p.cavity.depth*p.cavity.height)<0.01,"Exact exterior subtraction, no rectangular air pocket around crimped tail");
      // Driver approaches from outside/below through the lower plate access openings.
      for(s in p.sites) {
        var driver=Part.cylinderSpan(2.5,-45,p.plateThickness-p.washerThickness-6.1,s.x,s.y);
        for(id in [p.plates.fixed.id,p.plates.moving.id,"cured-coupon"]){var r=[for(r in parts)if(r.id==id)r][0],body=cache.posed(r.component,poses.get(id));check(SweptRemoval.intersectionVolume(driver,body)<0.001,"Driver approach blocked by "+id);body.close();}
        driver.close();
      }
      // Sweep only removable planar/cylindrical tooling. The crimped sockets remain stationary in casting.
      var plateRow=[for(r in parts)if(r.id==p.plates.moving.id)r][0],plate=cache.posed(plateRow.component,poses.get(plateRow.id));
      var sweep=SweptRemoval.sweep(plate,new Vector(0,0,-150));plate.close();
      check(SweptRemoval.intersectionVolume(sweep,castShape)<0.001,"Locating plate must withdraw clear of cured casting");
      var beforeReleaseVolume=0.0;
      for(s in p.sites) {
        var r=[for(r in parts)if(r.id==s.id)r][0],body=cache.posed(r.component,poses.get(r.id));
        check(SweptRemoval.intersectionVolume(sweep,body)<0.001,"Locating plate trapped on socket");body.close();
        var screw=[for(r in parts)if(r.id==s.id+"-positioning")r][0],held=cache.posed(screw.component,poses.get(screw.id));
        beforeReleaseVolume+=SweptRemoval.intersectionVolume(sweep,held);held.close();
      }
      check(beforeReleaseVolume>1,"Negative control: retained screw heads must block plate withdrawal");sweep.close();
      // After unscrewing, each screw/washer can translate through the bed access hole.
      var looseRemoved=0;
      for(r in parts)if(r.id.indexOf("positioning")>=0) {
        var body=cache.posed(r.component,poses.get(r.id)),travel=SweptRemoval.sweep(body,new Vector(0,0,-80));body.close();
        for(id in [p.plates.fixed.id,p.plates.moving.id,"cured-coupon","socket0","socket1"]) {
          var obstacle=[for(x in parts)if(x.id==id)x][0],other=cache.posed(obstacle.component,poses.get(id));
          check(SweptRemoval.intersectionVolume(travel,other)<0.001,"Released positioning hardware cannot withdraw: "+r.id+" / "+id);other.close();
        }
        travel.close();looseRemoved++;
      }
      check(looseRemoved==4,"Two positioning screws and washers checked");
      // Walls withdraw outward, rather than sweeping the embedded crimped tails.
      for(id in ["south-wall","north-wall","west-wall","east-wall"]) {
        var r=[for(r in parts)if(r.id==id)r][0],body=cache.posed(r.component,poses.get(id));
        var delta=id=="south-wall"?new Vector(0,-100,0):id=="north-wall"?new Vector(0,100,0):id=="west-wall"?new Vector(-100,0,0):new Vector(100,0,0);
        var travel=SweptRemoval.sweep(body,delta);body.close();
        check(SweptRemoval.intersectionVolume(travel,castShape)<0.001,"Wall trapped on casting");travel.close();
      }
      castShape.close();cache.close();
      var datumBom=new Bom(),permanentBom=new Bom();
      for(r in parts)if(r.id=="socket0"||r.id=="socket1")permanentBom.addComponent(r.component);else if(r.id!="cured-coupon")datumBom.addComponent(r.component);
      var mix=SocketCoupon.fromPlan(p).massProperties().mass;
      var steps:Array<{id:String,kind:String,members:Array<String>,after:Array<String>,status:String}> = [
        {id:"cure",kind:"process",members:["cured-coupon"],after:[],status:"physical evidence pending"},
        {id:"release-positioning",kind:"unscrew-and-withdraw",members:["socket0-positioning","socket0-positioning-washer","socket1-positioning","socket1-positioning-washer"],after:["cure"],status:"translation clearance checked after thread release; unscrewing semantic"},
        {id:"release-plate-clamps",kind:"unfasten",members:["plate-clamp-0","plate-clamp-1"],after:["release-positioning"],status:"manual release; external wall clamps not yet selected"},
        {id:"remove-walls",kind:"withdraw-outward",members:["south-wall","north-wall","west-wall","east-wall"],after:["release-plate-clamps"],status:"wall/casting sweeps checked; closure hardware pending"},
        {id:"separate-bed",kind:"remove-locators-and-bed",members:["locator-0","locator-1",p.plates.fixed.id],after:["remove-walls"],status:"panel separation and pin withdrawal remain manual review"},
        {id:"withdraw-locating-plate",kind:"withdraw-minus-Z",members:[p.plates.moving.id],after:["separate-bed"],status:"continuous sweep against cured casting and sockets checked"}
      ];
      var report={status:"Geometry review; not fabrication release",assertions:assertions,initialExactPairs:exactPairs,
        sampleCastingSizeMm:[400,240,70],nominalMixMassKg:mix,screwInsertionMm:p.sites[0].insertionMm,
        threadDimensionMm:p.socket.threadLength,locatorAccuracyMm:p.positionToleranceMm,releaseBeforeWithdrawalProved:beforeReleaseVolume>1,
        checks:{actualExteriorSubtraction:true,protectedBlindBores:true,driverAccess:true,releasedPositioningHardwareSweep:true,plateSweep:true,wallCastingSweeps:true,assemblyRoundtrip:true},
        validation:p.validation,fabrication:ToolingReadiness.evaluate(p.validation),
        limits:"Nominal OD/crimp, sealing, locator tolerance, wall closure, material strength, pressure, compaction, release and handling require physical qualification. Separate coupon, not the full hobber mould."};
      sys.io.File.saveContent(out+"/fixture-plan.json",haxe.Json.stringify(p,null,"  "));
      sys.io.File.saveContent(out+"/demolding-steps.json",haxe.Json.stringify(steps,null,"  "));
      sys.io.File.saveContent(out+"/bom.json",haxe.Json.stringify({permanentSockets:permanentBom.lines(),mixKg:mix,reusableTooling:datumBom.lines(),unselected:["Mouth seal/release compound","Wall closure hardware","Qualified stock and locating fits"]},null,"  "));
      sys.io.File.saveContent(out+"/fixture.assembly.json",assembly.encode());
      for(panel in [p.plates.fixed,p.plates.moving]) {
        var drawing=machinekit.tooling.PanelDrawing.drawing(panel);
        sys.io.File.saveContent(out+"/"+panel.id+".panel.json",PanelCodec.encode(panel));
        sys.io.File.saveContent(out+"/"+panel.id+".svg",cadkit.drawing.Svg.write(drawing));
        sys.io.File.saveContent(out+"/"+panel.id+".dxf",cadkit.drawing.Dxf.write(drawing));
      }
      for(mode in ["assembled","exploded"]) {
        var scene=AssemblyPreview.scene(SocketFixtureCad.assembly(p,true,mode=="exploded"),"blind-socket-fixture-"+mode);
        scene.recipeDiagnostics=["REVIEW ONLY: two-socket coupon. Nominal socket exterior; seal system, wall closure, process and locator accuracy unqualified."];
        var bytes=SceneArtifact.encode(scene);check(SceneArtifact.decode(bytes).parts.length==scene.parts.length,"Scene roundtrip");
        sys.io.File.saveBytes(out+"/socket-fixture-"+mode+".mtrg",bytes);
      }
      report.assertions=assertions;
      sys.io.File.saveContent(out+"/review.json",haxe.Json.stringify(report,null,"  "));
      Sys.println('Socket fixture passed $assertions assertions and $exactPairs initial exact pairs; mould panels, scenes, BOM and release sequence exported.');
    }catch(error:Dynamic){cache.close();throw error;}
  }
}
