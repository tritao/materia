import cadkit.modeling.*;
import machinekit.motion.*;
import machinekit.component.*;
import machinekit.assembly.*;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

class SfuFamilyChecks {
 static var checks=0;
 static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
 static function rejects(fn:Void->Void,label:String):Void {var failed=false;try fn()catch(_:Dynamic)failed=true;check(failed,label);}
 static function bom(axis:MachineAssembly):String {
  var lines=axis.billOfMaterials().lines();lines.sort((a,b)->Reflect.compare(a.partNumber,b.partNumber));return haxe.Json.stringify(lines);
 }
 static function main():Void {
  var rows:Array<Dynamic> = [],assembly=new MachineAssembly(),registry=MachineKitComponents.standard();
  rejects(()->new SfuNut("SFU2005"),"Require explicit root");
  rejects(()->new DsgNutHousing("DSG20H"),"Require explicit thread depths");
  rejects(()->new VallderScrewSupport("FK17"),"Quarantine FK17 flange discrepancy");
  rejects(()->new VallderScrewSupport("FF17"),"Quarantine FF17 flange discrepancy");
  for(model in SfuFamilyData.nutModels()) {
   var s=SfuFamilyData.nut(model),root=s.diameter-4,nut=new SfuNut(model,root),housingModel="DSG"+Std.int(s.diameter)+"H",housing=new DsgNutHousing(housingModel,10,10);
   var body=nut.geometry(),holder=housing.geometry();
   check(body.valid() && holder.valid(),"Valid SFU/DSG "+model);
   var posed=body.placed(new Location(new Plane(new Vector(0,housing.spec.shaftHeight,-s.offset-s.thickness),Vector.Y(),Vector.Z())));
   var common=holder.intersect(posed),overlap=common.volume();
   check(overlap<0.001,"Nut/housing seats "+model+": "+overlap);common.close();
   var points=nut.boltPattern(),other=housing.flangePattern();
   for(i in 0...points.length)check(Math.abs(-points[i].y-other[i].x)+Math.abs(points[i].x-other[i].y)<1e-6,"Rotated SFU/DSG bolt axes "+model);
   for(c in housing.connectors())if(StringTools.startsWith(c.name,"baseBolt")) {
    var probe=Part.cylinderAlongY(1,0.1,9.9,c.frame.x,c.frame.z),hit=holder.intersect(probe);
    check(hit.volume()<0.001,"DSG base thread direction");hit.close();probe.close();
   }
   for(p in other) {
    var probe=Part.cylinderSpan(1,0.1,9.9,p.x,housing.spec.shaftHeight+p.y),hit=holder.intersect(probe);
    check(hit.volume()<0.001,"DSG flange thread direction");hit.close();probe.close();
   }
   var shaft=new LeadScrew(new LeadScrewThread(Ball,s.diameter,s.pitch,1,RightHand,root),200),shaftPart=shaft.geometry();
   var shifted=shaftPart.translated(new Vector(0,housing.spec.shaftHeight,-50)),touch=holder.intersect(shifted);
   check(touch.volume()<0.001,"Screw clears housing");touch.close();shifted.close();shaftPart.close();
   var offset=(s.diameter-16)*60+(s.pitch==10 ? 140 : 0);
   assembly.addComponent(model+"-housing",housing,F.translation(offset,0,0));
   // Assembly pose is a quarter-turn about Z and the flange seat at Z=0.
   assembly.addComponent(model+"-nut",nut,{x:offset,y:housing.spec.shaftHeight,z:-s.offset-s.thickness,qx:0,qy:0,qz:Math.sqrt(0.5),qw:Math.sqrt(0.5)});
   rows.push({model:model,housing:housingModel,overlapMm3:overlap,assumedRootDiameter:root,assumedThreadDepth:10});
   posed.close();body.close();holder.close();
  }
  for(model in SfuDrawingScrew.models()) {
   var large=StringTools.startsWith(model,"SFU25"),screw=new SfuDrawingScrew(model,500,large ? 21 : 16),p=screw.geometry();
   check(p.valid() && p.solidCount()==1,"Drawing screw solid "+model);
   check(screw.axialComplianceBetween(0,500)>0 && Math.abs(screw.axialStiffness()*screw.axialComplianceBetween(0,500)-1)<1e-9,"Piecewise shaft compliance "+model);
   check(screw.fixedEnd==(large ? 76 : 60) && screw.journalDiameter==(large ? 17 : 15),"Drawing screw ends "+model);
   for(prefix in ["BK","BF"]) {
    var support=new VallderScrewSupport(prefix+(large ? "17" : "15")),body=support.geometry();
    check(support.spec.bore==screw.journalDiameter,"Journal/support diameter "+model);
    var journal=Part.cylinderSpan(screw.journalDiameter/2-0.03,-0.5,0.5),common=body.intersect(journal);
    check(common.volume()<0.001,"Journal/support centre "+model);common.close();journal.close();body.close();
   }
   assembly.addComponent(model+"-screw",screw,F.translation((large ? 800 : 650)+(model.indexOf("10")>0 ? 75 : 0),0,0));p.close();
  }
  assembly.validate();var encoded=assembly.encode(),loaded=MachineAssembly.decode(encoded,registry);
  check(loaded.components().length==assembly.components().length && bom(loaded)==bom(assembly),"SFU family recipes and engineering BOM survive reload");
  var output=Sys.getEnv("MATERIA_ROOT")+"/build/suppliers/vallder/";
  sys.io.File.saveContent(output+"sfu-family-checks.json",haxe.Json.stringify({checks:checks,models:rows,limits:["Explicit root and blind thread depths are modelling assumptions","Support centre probes do not qualify axial bearing seating/preload"]},null,"  "));
  sys.io.File.saveContent(output+"sfu-family.assembly.json",encoded);
  sys.io.File.saveBytes(output+"sfu-family.mtrg",SceneArtifact.encode(AssemblyPreview.scene(assembly,"SFU and DSG family")));
  trace(checks+" SFU/DSG geometry, journal, screw, pattern and persistence checks passed");
 }
}
