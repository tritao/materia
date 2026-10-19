import CadKit;
import cadkit.Shape;
import machinekit.motion.*;
class Sfu20SeatAudit {
 static function main():Void {
  var rows:Array<Dynamic> = [];
  for(model in ["BK15","BF15"]) {
   var component=new VallderScrewSupport(model),part=component.geometry(),s=part.shape,solids:Array<Dynamic> = [];
   for(i in 0...s.subshapeCount(CadKit.ShapeKind.Solid)) {
    var solid=s.subshape(CadKit.ShapeKind.Solid,i),b=solid.bounds();
    solids.push({index:i,volume:solid.volume(),min:[b.get_min().get_x(),b.get_min().get_y(),b.get_min().get_z()],max:[b.get_max().get_x(),b.get_max().get_y(),b.get_max().get_z()]});solid.close();
   }
   var expected=model=="BK15" ? [{index:11,lo:-7.5,hi:1.5},{index:7,lo:1.5,hi:10.5},{index:25,lo:-14.0,hi:-7.5},{index:26,lo:10.5,hi:20.0}] : [{index:7,lo:-4.5,hi:4.5}];
   for(e in expected) {
    var solid=s.subshape(CadKit.ShapeKind.Solid,e.index),b=solid.bounds();
    if(Math.abs(b.get_min().get_z()-e.lo)>1e-6 || Math.abs(b.get_max().get_z()-e.hi)>1e-6)throw "Support seating changed: "+model;
    solid.close();
   }
   rows.push({model:model,sha256:component.spec.sha256,solids:solids});part.close();
  }
  sys.io.File.saveContent(Sys.getEnv("MATERIA_ROOT")+"/build/suppliers/vallder/sfu20-seat-audit.json",haxe.Json.stringify(rows,null,"  "));
 }
}
