import machinekit.milling.*;
import machinekit.assembly.*;
import machinekit.component.MachineKitComponents;
import cadkit.modeling.*;
import materia.assembly.AssemblyFrames as F;

class MillWorkholdingChecks {
 static var checks=0;
 static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
 static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<1e-6,message+": "+a+" / "+b);
 public static function main():Void {
  var vise=new Rke92Vise();vise.validate();
  var model=new AssemblyModel("mm");vise.addTo(model,"");var state=new AssemblyState(model.definition("vise"));
  materia.assembly.AssemblyDefinitionCodec.validateState(model.definition("vise"),state.record());
  check(true,"initial coupled state can be loaded before any FK evaluation");
  near(state.worldPose("fixedPlate").y,80,"published fixed face station");
  near(state.worldPose("movingPlate").y,140,"default 60 mm clamped opening");
  near(vise.seat.z,106,"stock sits on 26 mm parallels over 80 mm bed");
  var stock=Part.box(60,60,20).translated(new Vector(0,110,106));
  var cache=new PosedParts();
  for(row in vise.components()) {
   var geometry=cache.local(row.component);
   check(geometry.valid()&&geometry.solidCount()==1,"valid single solid: "+row.id);
   if(row.id=="body"||row.id=="fixedPlate"||row.id=="movingPlate"||StringTools.startsWith(row.id,"parallel")||row.id=="stop") {
    var posed=cache.posed(row.component,state.worldPose(row.id)),hit=posed.intersect(stock);
    check(hit.volume()<1e-5,"seated stock has no fixture penetration: "+row.id);hit.close();posed.close();
   }
  }
  for(gap in [0.0,60,138]) {
   state.setJoint("opening",gap);state.forwardKinematics();
   near(state.worldPose("movingPlate").y,80+gap,"jaw travel follows actual opening");
   near(state.worldPose("fixedPlate").y,80,"fixed jaw remains fixed");
   near(state.joint("screw-turn"),gap*2*Math.PI/4,"nominal screw follows jaw travel");
  }
  var restored=MachineAssembly.decode(vise.encode(),MachineKitComponents.standard());restored.validate();
  check(restored.components().length==vise.components().length,"vise recipes survive assembly roundtrip");
  var zpp=new ZppWorkholding(60,60,false);zpp.validate();
  var zm=new AssemblyModel("mm");zpp.addTo(zm,"");var zs=new AssemblyState(zm.definition("zpp"));
  materia.assembly.AssemblyDefinitionCodec.validateState(zm.definition("zpp"),zs.record());
  var zstock=Part.box(60,60,20).translated(new Vector(0,0,zpp.seat.z));
  for(row in zpp.components()) {
   var g=cache.local(row.component);check(g.valid()&&g.solidCount()>0,"ZPP valid supplier/fabricated geometry: "+row.id);
   if(["body","plate","vise","positive-jaw","negative-jaw","screw","parallel-1","parallel1"].indexOf(row.id)>=0) {
    var posed=cache.posed(row.component,zs.worldPose(row.id)),hit=posed.intersect(zstock);
    check(hit.volume()<1e-4,"ZPP blank seats without penetration: "+row.id);hit.close();posed.close();
   }
  }
  for(id in ["pullStud-26--26","pullStud26-26"]) {
   var member=[for(row in zpp.components())if(row.id==id)row.component][0];
   var stud=cache.posed(member,zs.worldPose(id));
   for(other in ["plate","vise"]) {
    var c=[for(row in zpp.components())if(row.id==other)row.component][0];
    var shape=cache.posed(c,zs.worldPose(other)),hit=stud.intersect(shape);
    check(hit.volume()<1e-3,"ZPP pull stud seats without body/plate penetration: "+id+" / "+other+": "+hit.volume());
    shape.close();hit.close();
   }
   stud.close();
  }
  for(gap in [10.0,60,90]) {
   zs.setJoint("half-opening",gap/2);zs.forwardKinematics();
   near(zs.worldPose("positive-jaw").x,(gap-59)/2,"right jaw follows half opening");
   near(zs.worldPose("negative-jaw").x,-(gap-59)/2,"left jaw centers the blank");
  }
  var zr=MachineAssembly.decode(zpp.encode(),MachineKitComponents.standard());zr.validate();
  check(zr.components().length==zpp.components().length,"ZPP persistent recipes rebuild supplier parts");
  zstock.close();
  var airVise=new ZppWorkholding();airVise.validate();
  var am=new AssemblyModel("mm");airVise.addTo(am,"");var ast=new AssemblyState(am.definition("air-vise"));
  materia.assembly.AssemblyDefinitionCodec.validateState(am.definition("air-vise"),ast.record());
  near(airVise.seat.z,176,"pneumatic stock seat derives from actual 126 mm jaw tops");
  var airStock=Part.box(60,60,20).translated(new Vector(0,0,airVise.seat.z));
  for(row in airVise.components()) {
   var g=cache.local(row.component);check(g.valid()&&g.solidCount()>0,"valid AR155-I/TS96 geometry: "+row.id);
   if(["body","plate","vise","positive-jaw","negative-jaw","positive-soft-jaw","negative-soft-jaw","positive-hard-jaw","negative-hard-jaw"].indexOf(row.id)>=0) {
    var posed=cache.posed(row.component,ast.worldPose(row.id)),hit=posed.intersect(airStock);
    check(hit.volume()<0.01,"pneumatic stock has no fixture penetration: "+row.id+": "+hit.volume());hit.close();posed.close();
   }
  }
  for(half in [30.0,32]) {
   ast.setJoint("half-opening",half);ast.forwardKinematics();
   near(ast.worldPose("positive-jaw").x,half-29.75,"pneumatic right carrier follows release clearance");
   near(ast.worldPose("negative-jaw").x,29.75-half,"pneumatic opposite carrier follows release clearance");
  }
  for(id in ["pullStud-48--48","pullStud48-48"]) {
   var member=[for(row in airVise.components())if(row.id==id)row.component][0];
   var stud=cache.posed(member,ast.worldPose(id));
   for(other in ["plate","vise"]) {
    var c=[for(row in airVise.components())if(row.id==other)row.component][0];
    var shape=cache.posed(c,ast.worldPose(other)),hit=stud.intersect(shape);
    check(hit.volume()<0.001,"TS96 stud seats without penetration: "+id+" / "+other+": "+hit.volume());shape.close();hit.close();
   }
   stud.close();
  }
  var ar=MachineAssembly.decode(airVise.encode(),MachineKitComponents.standard());ar.validate();
  check(ar.components().length==airVise.components().length,"pneumatic supplier assembly survives recipe roundtrip");
  check(airVise.port("airA").portName=="airA"&&airVise.port("airB").portName=="airB","pneumatic service interfaces exposed");
  airStock.close();
  stock.close();cache.close();
  Sys.println("Workholding: "+checks+" seating, geometry, jaw travel and persistence checks passed");
 }
}
