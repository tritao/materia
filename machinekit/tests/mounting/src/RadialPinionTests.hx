import machinekit.transmission.*;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.AssemblyState;

/** Verify physical ownership and the radial displacement of the carried drive,
 * rather than merely checking the drawn lever's orientation. */
class RadialPinionTests {
 static var assertions=0;
 static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
 public static function main():Void {
  for(bed in [2600.0,3150.0]) {
   var machine=new SheetRouter(new SheetRouterSpec(bed));machine.check().throwIfErrors();
   var definition=machine.definition(),state=new AssemblyState(definition);
   for(suffix in ["YLeft","YRight","X"]) {
    var id="pinion"+suffix,carrierId=id+"Preload",anchorId=id+"Anchor",motorId="motor"+suffix;
    var carrier:RadialPinionCarrier=cast machine.component(carrierId);
    check(Std.isOfType(carrier,RadialPinionCarrier),"physical lever owns the preload allowance");
    check(carrier.motor==machine.component(motorId),"lever carries the actual selected motor");
    var motorJoint=[for(j in definition.joints)if(j.child==motorId)j];
    var pivotJoint=[for(j in definition.joints)if(j.id==id+"-preload-pivot")j];
    check(pivotJoint.length==1 && pivotJoint[0].passive==true,"preload pivot is passive, not position-held");
    check(motorJoint.length==1 && motorJoint[0].parent==carrierId,"motor has one support path through lever");
    var datum=F.compose(machine.zeroPose(carrierId),carrier.connector("pinion").frame);
    var actual=F.compose(machine.zeroPose(id),F.translation(0,0,RackGeometry.pinion(machine.component(id)).width/2));
    check(Math.abs(datum.x-actual.x)+Math.abs(datum.y-actual.y)+Math.abs(datum.z-actual.z)<1e-6,"pinion datum is at axial centre");
    var rackPose=machine.zeroPose("rack"+suffix),teethUp=F.transformVector(rackPose,0,1,0);
    check(Math.abs(teethUp.x)+Math.abs(teethUp.y)+Math.abs(teethUp.z-1)<1e-6,"rack teeth face up on both mirrored drives");
    check(Math.abs(actual.z-rackPose.z-RackGeometry.pinion(machine.component(id)).diameter/2)<1e-6,"rack pitch line stays below the pinion");
    var maximum=RadialPinionAssembly.upperAngle();
    var guideOffset=56*(1-Math.cos(maximum))+13*Math.sin(maximum)+3*Math.tan(maximum);
    check(guideOffset<0.25-0.05,"whole-stroke cup guide bound retains 0.05 mm fit allowance");
    PosedParts.scope(cache->{
     for(member in machine.components())if(StringTools.startsWith(member.id,id)) {
      var solid:cadkit.modeling.Part;
      try solid=cache.local(member.component)catch(e:Dynamic){throw "Pivot part "+member.id+": "+Std.string(e);}
      check(solid.valid(),"valid physical part "+member.id);
      if(member.id==carrierId||member.id==anchorId)check(solid.solidCount()==1,"connected support "+member.id);
     }
     for(angle in [RadialPinionAssembly.lowerAngle(),0.0,RadialPinionAssembly.upperAngle()]) {
      state.setJoint(id+"-preload-pivot",angle);state.forwardKinematics();
      var dz=state.worldPose(id).z-machine.zeroPose(id).z;
      check(Math.abs(dz-80*Math.sin(angle))<1e-6,"pivot moves pinion radially "+suffix);
      var motorDz=state.worldPose(motorId).z-machine.zeroPose(motorId).z;
      check(Math.abs(motorDz-dz)<1e-6,"motor follows pinion on the same lever");
      check(cache.volume(machine,state,"pivot",carrierId,anchorId)<0.001,"lever clears fixed anchor through stroke");
      for(stop in ["UpperStop","LowerStop"])check(cache.volume(machine,state,"pivot",carrierId,id+stop)<0.001,"stroke stop does not penetrate lever");
      for(thrust in ["PivotThrustFront","PivotThrustRear"])check(cache.volume(machine,state,"pivot",carrierId,id+thrust)<0.001,"lever clears its thrust shims");
      check(cache.volume(machine,state,"pivot",id+"SpringCup",id+"SpringAdjuster")<0.001,"adjuster guide clears the tilted spring cup");
      check(cache.volume(machine,state,"pivot",id+"PivotBush",id+"PivotPin")<0.001,"journal does not penetrate its retained pin");
      var delta=-56*Math.sin(angle)+13*(1-Math.cos(angle));
      var edge=12*Math.abs(Math.sin(angle));
      check(12+delta-edge>=11.5 && 12+delta+edge<=12.1375,"disc pack remains preloaded inside its assumed range including cup tilt");
     }
    });
    state.setJoint(id+"-preload-pivot",0);state.forwardKinematics();
   }
   Sys.println('RadialPinionTests bed '+bed+': ownership, centre datum, solids and stroke pass');Sys.stdout().flush();
  }
  Sys.println('RadialPinionTests: '+assertions+' assertions pass');
 }
}
