import machinekit.motion.*;
import machinekit.assembly.*;
import materia.assembly.AssemblyFrames as F;

class ScrewLinearModuleChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  // ASSUMED AF3 key: D3.5, 8 mm exposed leg, 90 mm reach along travel.
  static function scaleAccess(axis:ScrewLinearModule):Void {
    var derived=axis.derivedFasteners();if(derived==null)throw "Missing scale hardware";
    var poses=axis.solvedPoses(),cache=new PosedParts(),count=0;
    for(joint in derived.joints)if(joint.clampedMember=="scaleHead") {
      var host=axis.component(joint.receivingMember),face=host.fasteningFaces().filter(f->f.id==joint.receivingFace)[0];
      var frame=F.compose(poses.get(joint.receivingMember),face.frame);
      for(volume in joint.volumes)if(volume.access)for(targetIndex=>hole in joint.hostHoles) {
        var raw=cadkit.modeling.Part.cylinderSpan(1.75,volume.to-8,volume.to,hole.x,hole.y),stub=PosedParts.moved(raw,frame);raw.close();
        var elbow=F.transformPoint(frame,hole.x,hole.y,volume.to-8);
        var handle=cadkit.modeling.Part.cylinderAlong(1.75,new cadkit.modeling.Vector(elbow.x,elbow.y,elbow.z),new cadkit.modeling.Vector(-1,0,0),90);
        for(tool in [stub,handle]) {
          var box=PosedParts.boxOf(tool);
          for(other in axis.components()) {
            var pose=poses.get(other.id);if(PosedParts.apart(box,cache.box(other.component,pose),0))continue;
            var part=cache.posed(other.component,pose),common=part.intersect(tool),overlap=common.volume();common.close();part.close();
            check(overlap<0.001,"Scale head key clears "+other.id);
          }
          for(otherJoint in derived.joints) {
            var receiving=axis.component(otherJoint.receivingMember),otherFace=receiving.fasteningFaces().filter(f->f.id==otherJoint.receivingFace)[0];
            var otherFrame=F.compose(poses.get(otherJoint.receivingMember),otherFace.frame);
            for(physical in otherJoint.volumes)if(["head","head washer","nut","nut washer"].contains(physical.name))for(index=>atHole in otherJoint.hostHoles) {
              if(otherJoint.id==joint.id&&index==targetIndex)continue;
              var cylinder=cadkit.modeling.Part.cylinderSpan(physical.diameter/2,physical.from,physical.to,atHole.x,atHole.y),hardware=PosedParts.moved(cylinder,otherFrame);cylinder.close();
              if(!PosedParts.apart(box,PosedParts.boxOf(hardware),0)) {
                var common=tool.intersect(hardware),overlap=common.volume();common.close();check(overlap<0.001,"Head key clears hardware "+otherJoint.id);
              }
              hardware.close();
            }
          }
        }
        stub.close();handle.close();count++;
      }
    }
    check(count==2,"Two serviceable scale head screws");
    var port=F.compose(axis.spec.scalePose(),axis.spec.scaleHousing.connector("air").frame);
    var direction=F.transformVector(port,0,0,1);
    var air=cadkit.modeling.Part.cylinderAlong(7,new cadkit.modeling.Vector(port.x,port.y,port.z),new cadkit.modeling.Vector(direction.x,direction.y,direction.z),35),airBox=PosedParts.boxOf(air);
    for(other in axis.components())if(other.id!="scaleHousing") {
      var pose=poses.get(other.id);if(PosedParts.apart(airBox,cache.box(other.component,pose),0))continue;
      var part=cache.posed(other.component,pose),common=part.intersect(air),overlap=common.volume();common.close();part.close();
      check(overlap<0.001,"ASSUMED D14 air-fitting tool clears "+other.id);
    }
    air.close();cache.close();
  }
  public static function main():Void {
    var requested=Sys.args(),selected=0;
    for(size in [{profile:"HGH15CA",stroke:20.0,spacing:150.0,blocks:80.0},
      {profile:"HGH15CA",stroke:30.0,spacing:150.0,blocks:80.0}]) if(requested.length==0||requested.contains("stroke="+machinekit.component.Dimension.format(size.stroke))) {
      selected++;
      var spec=size.stroke==30?machinekit.hobbing.HobberYLayout.moduleSpec():new ScrewLinearModuleSpec(size.profile,size.stroke,size.spacing,size.blocks);
      var host=new ScrewLinearModulePart(spec.values().setToken("role","carriage")),axis=new ScrewLinearModule(spec,null,host);
      check(axis.component("carriage")==host,"Containing machine supplies its carriage interface");
      check(axis.validate().length==0,"Module assembly validation");
      var fasteners=axis.deriveFasteners();
      check(!fasteners.diagnostics.hasErrors(),"Joint-derived module hardware");
      check([for(method in axis.alignment())method].length==fasteners.joints.length,"Every module joint declares alignment");
      for(step in axis.assemblySteps()) {
        check(step.parts.length>0&&step.tools.length>0&&step.measurement.length>0,"Assembly step declares parts tools and measurement");
        for(id in step.parts)check(axis.component(id)!=null,"Assembly step references a modelled part: "+id);
      }
      axis.checkJointLoads(axis.loads({peak:1000.0,momentary:2000.0}));
      var normalFactor=1e30,crashFactor=1e30,scaleFactor=1e30;
      for(slip in fasteners.slips) {
        normalFactor=Math.min(normalFactor,slip.peakFactor);
        var momentary=slip.momentaryFactor;if(momentary!=null)crashFactor=Math.min(crashFactor,momentary);
        if(StringTools.startsWith(slip.label,"Scale "))scaleFactor=Math.min(scaleFactor,slip.peakFactor);
      }
      Sys.println('Joint slip minimum factors normal $normalFactor / momentary $crashFactor; scale-only normal $scaleFactor (20/40 N ASSUMED).');
      for(finding in fasteners.diagnostics.items)Sys.println(finding.code+" "+finding.message);
      Sys.println('Module ${size.profile} stroke ${size.stroke}: screw ${spec.screwLength}, rail ${spec.railLength}, walls ${spec.wallWidth} x ${spec.wallHeight}, carriage plane ${spec.carriageBottom()}, ${fasteners.count()} screws / ${fasteners.dowelCount()} pins.');
      var report=AssemblyIntegrity.check(axis,{strict:true,runningGap:2.0,fittedBefore:axis.fittingOrder(),processContacts:axis.stopContacts()});
      for(finding in report.diagnostics.items)Sys.println(finding.code+" "+finding.message);
      check([for(finding in report.diagnostics.items)if(finding.code=="fasteners.edge-support")finding].length==0,"Made module parts pass edge support");
      check(report.errors().length==0,"Module integrity/clearance/fastener access");
      scaleAccess(axis);
      if(size.stroke<=20) {
        var rotated=new AxisBuilder();rotated.include("module",axis,F.fromRotationMatrix(0,0,0,[0.0,-1,0,1,0,0,0,0,1]));
        var removal=AssemblyIntegrity.service(rotated,{serviceProcedures:["module/motor"]});
        check(removal.errors().length==0,"Rotated module motor withdraws along its included axis");
      }
      var fit=axis.scaleAdjustment(),limits=spec.scaleHousing.series.mounting;
      check(fit.normalFit>=fit.normalStack,"Head fitting covers ASSUMED normal stack");
      check(fit.gapAdjustment>=fit.gapStack,"Head gap setting covers ASSUMED lateral stack");
      check(axis.hasPort("position-feedback"),"Sealed scale BiSS feedback exposed");
      check(spec.scaleHousing.measuringLength>=size.stroke+2*spec.overtravel,"Scale window covers overtravel");
      var lengths=SealedScaleData.series("JFT-40").lengths;
      check([for(length in lengths)if(length<spec.scaleHousing.measuringLength&&length>=size.stroke+2*spec.overtravel)length].length==0,"Smallest standard measuring length");
      Sys.println('Scale ML ${spec.scaleHousing.measuringLength}, housing ${spec.scaleHousing.overallLength()}, outside wall ${spec.wallWidth} x ${spec.wallHeight}, head normal stack ${fit.normalStack} / fit +/-${fit.normalFit}, gap stack ${fit.gapStack} / adjustment +/-${fit.gapAdjustment} mm; stack assumptions unqualified.');
      var cache=new PosedParts(), carriage=axis.component("nutCarrier");
      for(role in ["stopFront","stopRear"]) {
        var coordinate=(StringTools.startsWith(role,"stopFront")?-1:1)*(size.stroke/2+spec.overtravel);
        var moving=cache.posed(carriage,F.translation(coordinate,0,0)), fixed=cache.posed(axis.component(role),F.identity());
        check(moving.shape.distance(fixed.shape)<0.001,"Hard stop contact at overtravel: "+role);
        var common=moving.intersect(fixed);check(common.volume()<0.001,"Stop has no penetration: "+role);
        common.close();moving.close();fixed.close();
      }
      check(axis.components().filter(m->StringTools.startsWith(m.id,"wall")).length==2,"Two structural side walls");
      for(id in ["wallLeft","wallRight"]) {
        var wall=axis.component(id).geometry(),bounds=PosedParts.boxOf(wall);wall.close();
        check(Math.abs(bounds.minX-spec.baseMin)<0.001&&Math.abs(bounds.maxX-spec.baseMax)<0.001,"Walls span full body length");
      }
      for(member in axis.components())check(!StringTools.startsWith(member.id,"riser")&&!StringTools.startsWith(member.id,"lip")&&!StringTools.startsWith(member.id,"retainer"),"No replaced v1 parts");
      cache.close();
      var rebuilt=MachineAssembly.decode(axis.encode());
      check(rebuilt.components().length==axis.components().length,"Module round trip");
      var savedPinEnds=0;for(member in rebuilt.components())for(face in member.component.pinnedFaces())savedPinEnds+=face.holes.length;
      check(savedPinEnds==2*fasteners.dowelCount(),"Both ends of every match-pin survive snapshot and persistence");
      var model=new cadkit.modeling.AssemblyModel("mm");axis.addTo(model,"");var state=model.initialState();
      var line=state.worldConnector("scaleHousing","axis"),mount=state.worldConnector("carriage","interface");
      Sys.println('Scale Abbe from carriage interface: ${Math.abs(line.y-mount.y)} across / ${Math.abs(line.z-mount.z)} normal mm; cable R${SealedScaleData.CABLE_BEND_RADIUS}.');
      var overDefinition=model.definition("physical-overtravel");
      for(joint in overDefinition.joints)if(joint.id=="travel"){joint.limits.lower=-size.stroke/2-spec.overtravel;joint.limits.upper=size.stroke/2+spec.overtravel;}
      var overState=new cadkit.modeling.AssemblyState(overDefinition);
      for(endpoint in [-size.stroke/2-spec.overtravel,size.stroke/2+spec.overtravel]) {
        overState.setJoint("travel",endpoint);overState.forwardKinematics();
        var reading=overState.worldPose("scaleHousing").x-overState.worldPose("scaleHead").x;
        check(reading>=spec.scaleHousing.startOffset()-1e-7&&reading<=spec.scaleHousing.startOffset()+spec.scaleHousing.measuringLength+1e-7,"Actual head stays inside reading window through overtravel");
      }
      for(x in [-size.stroke/2,size.stroke/2]){state.setJoint("travel",x);state.forwardKinematics();check(Math.abs(state.worldPose("carriage").x-x)<1e-7,"Carriage follows module travel");}
      axis.place("test-driver",new MotorDriver("GENERIC-400V-SERVO-AMP",new machinekit.power.PhaseRmsCurrent(5),1,new machinekit.power.DcVoltage(565)),F.translation(1000,1000,1000));
      axis.place("test-supply",new machinekit.power.ThreePhaseRectifier(new machinekit.power.AcMains(ThreePhase,new machinekit.power.AcRmsVoltage(400),50),new machinekit.power.DcCurrent(40),1,240,120,100,15000),F.translation(1500,1000,1000));
      axis.exposePort("test-mains","test-supply","mains");
      axis.connectPorts("test-power","test-supply","power1","test-driver","power");
      axis.addMotor("test-drive","screw-turn","motor","test-driver",1);
      var controlled=new cadkit.modeling.AssemblyModel("mm");axis.addTo(controlled,"");var definition=controlled.definition("dual-loop");
      check([for(encoder in definition.encoders)if(encoder.id=="position"&&encoder.joint=="travel")encoder].length==1,"Independent load position encoder");
      check([for(encoder in definition.encoders)if(encoder.id=="test-drive.encoder"&&encoder.joint=="screw-turn")encoder].length==1,"Native motor velocity feedback survives load scale");
      Sys.println(axis.stockAcceptance());
    }
    check(selected>0,"Requested module size exists");
    Sys.println('ScrewLinearModuleChecks: $assertions assertions passed');
  }
}
