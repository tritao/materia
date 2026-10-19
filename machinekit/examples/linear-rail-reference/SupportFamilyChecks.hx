import cadkit.modeling.*;
import machinekit.motion.*;
import machinekit.component.*;
import machinekit.assembly.RbkMotorMount;
class SupportFamilyChecks {
 static function main():Void {
  var rows:Array<Dynamic> = [];
  for(model in VallderSupportData.models().concat(["FK17","FF17"])) {
   var s=VallderSupportData.get(model),path=RbkMotorMount.assetDirectory()+"/cad/assets/"+s.sha256+".step";
   if(HiwinHgh15ca.fileDigest(path)!=s.sha256)throw "Unreviewed support: "+model;
   var part=(model=="FK17" || model=="FF17") ? new Part(cadkit.Shape.importStep(path)) : new VallderScrewSupport(model).geometry(),holes:Array<Dynamic> = [];
   var positions:Array<Vector> = [];
   if(s.base) {
    for(x in [-s.pitch/2,s.pitch/2])for(z in (s.longitudinalPitch==0 ? [0.0] : [-s.longitudinalPitch/2,s.longitudinalPitch/2]))positions.push(new Vector(x,0,z));
   } else {
    var offset=s.pitch/Math.sqrt(8);
    for(x in [-offset,offset])for(y in [-offset,offset])positions.push(new Vector(x,y,0));
   }
   for(p in positions) {
    var tool=s.base ? Part.cylinderAlongY(s.clearance/2-0.03,-s.shaftHeight-1,s.height-s.shaftHeight+1,p.x,p.z) : Part.cylinderSpan(s.clearance/2-0.03,1,s.height-1,p.x,p.y);
    var hit=part.intersect(tool),volume=hit.volume();hit.close();tool.close();
    var wall=s.base ? Part.cylinderAlongY(0.6,-s.shaftHeight+0.5,-s.shaftHeight+2,p.x+s.counterbore/2+1,p.z) : Part.cylinderSpan(0.6,1,s.height-1,p.x+4,p.y);
    var control=part.intersect(wall),material=control.volume();control.close();wall.close();
    holes.push({position:[p.x,p.y,p.z],overlapMm3:volume,controlMm3:material,pass:volume<0.001 && material>0.01});
   }
   var probe=Part.cylinderSpan(s.bore/2-0.03,-0.5,0.5),common=part.intersect(probe),shaft=common.volume();common.close();probe.close();
   var failed=false;
   for(h in holes)if(h.pass!=true)failed=true;
   if(failed!=(model=="FK17" || model=="FF17") || shaft>0.001)throw "Unexpected support probe result: "+model;
   rows.push({model:model,sha256:s.sha256,boreProbeMm3:shaft,holes:holes});part.close();
  }
  sys.io.File.saveContent(RbkMotorMount.assetDirectory()+"/support-bore-checks.json",haxe.Json.stringify(rows,null,"  "));
  trace("Support bore probe report written");
 }
}
