import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.motion.*;
import machinekit.component.ComponentRegistry;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

class HgFamilyChecks {
 static var checks=0;
 static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
 static function main():Void {
  var rows:Array<Dynamic> = [],assembly=new MachineAssembly();
  for(size in [15,20,25,30,35,45]) {
   var model="HGR"+size+"R",rail=new HgRail(model,237),part=rail.geometry();
   check(part.solidCount()==1 && part.volume()>0,"Solid rail "+model);
   var reference=new Part(HgBlock.load(model)),lo=HgReferenceData.low(model),hi=HgReferenceData.high(model);
   // Reference holes are symmetric; regenerate at exactly the reference positions for equivalence.
   var referenceLength=hi[2]-lo[2],shortRail=new HgRail(model,referenceLength);
   shortRail.holePositions.resize(0);
   var pitch=shortRail.spec.railHolePitch,first=referenceLength/2-Math.floor(referenceLength/2/pitch)*pitch;
   for(i in 0...Std.int(referenceLength/pitch)+1)if(first+i*pitch<referenceLength)shortRail.holePositions.push(first+i*pitch);
   var generated=shortRail.geometry(),translated=reference.translated(new Vector(0,0,-lo[2]));reference.close();
   var missing=translated.subtract(generated),extra=generated.subtract(translated),difference=missing.volume()+extra.volume();
   // HGR20 supplier STEP d6 vs table d6.1: documented discrepancy, not geometry scaling.
   var expected=size==20 ? shortRail.holePositions.length*Math.PI*(6.1*6.1-6*6)/4*(17.5-8.5) : 0.0;
   check(Math.abs(difference-expected)<0.05,"Rail source equivalence "+model+": "+difference);
   missing.close();extra.close();translated.close();generated.close();
   for(z in rail.holePositions) {
    var d=HgRail.drilling(model),tool=Part.cylinderAlongY(d[0]/2-0.02,-rail.spec.railHeight-1,1,0,z),common=part.intersect(tool);
    check(common.volume()<0.001,"Rail drill axes "+model);common.close();tool.close();
   }
   assembly.addComponent(model,rail,F.translation((size-15)*15,0,0));
   for(prefix in ["HGH","HGW"]) {
    var name=prefix+size+(prefix=="HGH" ? "CA" : "CC"),block=new HgBlock(name),body=block.geometry();
    check(body.volume()>0,"Detailed block "+name);
    var position=prefix=="HGH" ? 60.0 : 160.0,posed=body.translated(new Vector(0,0,position)),common=part.intersect(posed),overlap=common.volume();
    check(overlap<0.01,"Matching running section "+name+": "+overlap);common.close();posed.close();
    for(c in block.connectors())if(StringTools.startsWith(c.name,"mount")) {
     var top=block.spec.blockHeight-block.spec.railHeight;
     // Probe the centre of each tapped hole, with positive material control alongside it.
     var bore=Part.cylinderAlongY(0.8,top-2,top+1,c.frame.x,c.frame.z),hit=body.intersect(bore);
     check(hit.volume()<0.001,"Mount axis "+name+" "+c.name);hit.close();bore.close();
     var wall=Part.cylinderAlongY(1,top-2,top-0.5,c.frame.x+(c.frame.x>0 ? -8 : 8),c.frame.z),solid=body.intersect(wall);
     check(solid.volume()>0.1,"Mount material control "+name);solid.close();wall.close();
    }
    check(prefix!="HGW" || block.mountThreadDepth()==null,"Unknown HGW thread depth");
    assembly.addComponent(name,block,F.translation((size-15)*15,0,position));
    rows.push({model:name,rail:model,overlapMm3:overlap,threadDepth:block.mountThreadDepth()});body.close();
   }
   rows.push({model:model,referenceDifferenceMm3:difference});part.close();
  }
  var output=Sys.getEnv("MATERIA_ROOT")+"/build/suppliers/vallder/hg-reference/";
  sys.io.File.saveContent(output+"family-checks.json",haxe.Json.stringify({checks:checks,models:rows},null,"  "));
  sys.io.File.saveBytes(output+"family.mtrg",SceneArtifact.encode(AssemblyPreview.scene(assembly,"HG reference family")));
  trace(checks+" HG family geometry, bore and interface checks passed");
 }
}
