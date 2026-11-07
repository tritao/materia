import machinekit.motion.*;
import machinekit.assembly.*;
import materia.assembly.AssemblyFrames as F;

class ScrewLinearModuleChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  public static function main():Void {
    for(size in [{profile:"HGH15CA",stroke:20.0,spacing:150.0,blocks:80.0},
      {profile:"HGH15CA",stroke:80.0,spacing:150.0,blocks:100.0},
      {profile:"HGH25CA",stroke:200.0,spacing:180.0,blocks:100.0}]) {
      var spec=new ScrewLinearModuleSpec(size.profile,size.stroke,size.spacing,size.blocks),axis=new ScrewLinearModule(spec);
      check(axis.validate().length==0,"Module assembly validation");
      var fasteners=axis.deriveFasteners();
      check(!fasteners.diagnostics.hasErrors(),"Joint-derived module hardware");
      check([for(method in axis.alignment())method].length==fasteners.joints.length,"Every module joint declares alignment");
      for(step in axis.assemblySteps())check(step.parts.length>0&&step.tools.length>0&&step.measurement.length>0,"Assembly step declares parts tools and measurement");
      axis.checkJointLoads(axis.loads({peak:1000.0,momentary:2000.0}));
      for(finding in fasteners.diagnostics.items)Sys.println(finding.code+" "+finding.message);
      Sys.println('Module ${size.profile} stroke ${size.stroke}: screw ${spec.screwLength}, rail ${spec.railLength}, risers ${spec.riserWidth} x ${spec.riserHeight}, carriage plane ${spec.carriageBottom()}, ${fasteners.count()} screws / ${fasteners.dowelCount()} pins.');
      var report=AssemblyIntegrity.check(axis,{strict:true,runningGap:2.0,fittedBefore:axis.fittingOrder(),processContacts:axis.stopContacts()});
      for(finding in report.errors())Sys.println(finding.code+" "+finding.message);
      check(report.errors().length==0,"Module integrity/clearance/fastener access");
      var cache=new PosedParts(), carriage=axis.component("carriage");
      for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop")) {
        var coordinate=(StringTools.startsWith(role,"stopFront")?-1:1)*(size.stroke/2+spec.overtravel);
        var moving=cache.posed(carriage,F.translation(coordinate,0,0)), fixed=cache.posed(axis.component(role),F.identity());
        check(moving.shape.distance(fixed.shape)<0.001,"Hard stop contact at overtravel: "+role);
        var common=moving.intersect(fixed);check(common.volume()<0.001,"Stop has no penetration: "+role);
        common.close();moving.close();fixed.close();
      }
      cache.close();
      var rebuilt=MachineAssembly.decode(axis.encode());
      check(rebuilt.components().length==axis.components().length,"Module round trip");
      var model=new cadkit.modeling.AssemblyModel("mm");axis.addTo(model,"");var state=model.initialState();
      for(x in [-size.stroke/2,size.stroke/2]){state.setJoint("travel",x);state.forwardKinematics();check(Math.abs(state.worldPose("carriage").x-x)<1e-7,"Carriage follows module travel");}
      Sys.println(axis.stockAcceptance());
    }
    Sys.println('ScrewLinearModuleChecks: $assertions assertions passed');
  }
}
