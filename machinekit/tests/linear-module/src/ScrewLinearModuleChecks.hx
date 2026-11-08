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
      {profile:"HGH15CA",stroke:80.0,spacing:150.0,blocks:100.0},
      {profile:"HGH25CA",stroke:200.0,spacing:180.0,blocks:100.0}]) if(requested.length==0||requested.contains("stroke="+machinekit.component.Dimension.format(size.stroke))) {
      selected++;
      var spec=new ScrewLinearModuleSpec(size.profile,size.stroke,size.spacing,size.blocks);
      var host=new ScrewLinearModulePart(spec.values().setToken("role","carriage")),axis=new ScrewLinearModule(spec,null,host);
      check(axis.component("carriage")==host,"Containing machine supplies its carriage interface");
      check(axis.validate().length==0,"Module assembly validation");
      for(requirement in axis.serviceRemovals())switch requirement {
        case ServiceRemoval("screw",_,_,off,_):
          var guards=size.stroke<=20?ScrewLinearModuleCover.ROLES.filter(id->!StringTools.startsWith(id,"mount")):ScrewLinearModuleBellows.members(spec);
          check(guards.length>0&&guards.filter(id->!off.contains(id)).length==0,"Remove guards before withdrawing the screw and ball nut");
        case _:
      }
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
      Sys.println('Module ${size.profile} stroke ${size.stroke}: screw ${spec.screwLength}, rail ${spec.railLength}, risers ${spec.riserWidth} x ${spec.riserHeight}, carriage plane ${spec.carriageBottom()}, ${fasteners.count()} screws / ${fasteners.dowelCount()} pins.');
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
      Sys.println('Scale ML ${spec.scaleHousing.measuringLength}, housing ${spec.scaleHousing.overallLength()}, seat 40 x ${spec.scaleSeatThickness()}, head normal stack ${fit.normalStack} / fit +/-${fit.normalFit}, gap stack ${fit.gapStack} / adjustment +/-${fit.gapAdjustment} mm; stack assumptions unqualified.');
      var cache=new PosedParts(), carriage=axis.component("carriage");
      for(role in ScrewLinearModulePart.ROLES)if(StringTools.startsWith(role,"stop")) {
        var coordinate=(StringTools.startsWith(role,"stopFront")?-1:1)*(size.stroke/2+spec.overtravel);
        var moving=cache.posed(carriage,F.translation(coordinate,0,0)), fixed=cache.posed(axis.component(role),F.identity());
        check(moving.shape.distance(fixed.shape)<0.001,"Hard stop contact at overtravel: "+role);
        var common=moving.intersect(fixed);check(common.volume()<0.001,"Stop has no penetration: "+role);
        common.close();moving.close();fixed.close();
      }
      if(size.stroke<=20) {
        check(ScrewLinearModuleCover.openingHalf(spec)>4,"Short cover leaves a payload aperture");
        for(endpoint in [-size.stroke/2-spec.overtravel,size.stroke/2+spec.overtravel]) {
          var plate=cache.posed(carriage,F.translation(endpoint,0,0));
          for(side in ["Front","Rear"]) {
            var lip=cache.posed(axis.component("lip"+side),F.identity()),common=plate.intersect(lip);
            check(common.volume()<0.001,"Wiper has no rigid penetration at overtravel");check(plate.shape.distance(lip.shape)<0.001,"Wiper stays on the carriage at overtravel");common.close();lip.close();
          }
          plate.close();
        }
        Sys.println('Short guard: exposed payload aperture ${2*(ScrewLinearModuleCover.openingHalf(spec)-4)} x ${2*(ScrewLinearModuleCover.openingWidthHalf(spec)-4)} mm; overlap10, roof gap6; clip/end sealing unqualified.');
      } else {
        var plan=axis.coverPlan();if(plan==null)throw "Long module needs bellows sizing";
        check(plan.compressed<=plan.space,"Bellows fits declared folded space");check(plan.extended>=size.stroke+2*spec.overtravel,"Bellows spans overtravel");
        Sys.println('Long guard: ${plan.folds} folds, compressed ${plan.compressed} / space ${plan.space}, extended ${plan.extended}, outer ${plan.outerWidth()} x ${plan.outerHeight()}; catalogue row and retention ASSUMED.');
      }
      var guardIds=size.stroke<=20?ScrewLinearModuleCover.ROLES.filter(id->!StringTools.startsWith(id,"mount")):ScrewLinearModuleBellows.members(spec);
      var guardMass=0.0,rigidMovingGuardMass=0.0;
      for(member in axis.components())if(guardIds.contains(member.id)) {
        var mass=member.component.massProperties().mass;guardMass+=mass;
        if(StringTools.startsWith(member.id,"mountMoving")||StringTools.endsWith(member.id,"-moving"))rigidMovingGuardMass+=mass;
      }
      Sys.println('Guard geometry mass $guardMass kg, rigid moving guards $rigidMovingGuardMass kg; bought PU/lamella density and geometry are ASSUMED, hardware excluded.');
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
        var reading=overState.worldPose("scaleHead").x-overState.worldPose("scaleHousing").x;
        if(size.stroke<=20)for(role in ["coverFront","coverRear"]) {
          var headBody=new PosedParts();
          var headShape=headBody.posed(spec.scaleHead,overState.worldPose("scaleHead")),guard=headBody.posed(axis.component(role),overState.worldPose(role));
          var common=headShape.intersect(guard);check(common.volume()<0.001&&headShape.shape.distance(guard.shape)>=ScrewLinearModuleSpec.GAP-1e-6,"Outward cable keeps running gap through overtravel");
          common.close();headShape.close();guard.close();headBody.close();
        }
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
