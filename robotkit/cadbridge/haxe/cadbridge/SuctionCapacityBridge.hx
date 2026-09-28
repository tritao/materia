package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorFrames;
import materia.assembly.AssemblyFrames;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.SuctionGrip;

/** Turn a connected MachineKit cup into a RobotKit holding-force check. */
class SuctionCapacityBridge {
  public static function toGrip(effector:EndEffector, cupInstanceId:String,
      minimumCupVacuumKpa:Float, frictionCoefficient:Float, safetyFactor:Float,
      ?state:AssemblyState):SuctionGrip {
    if (effector == null || cupInstanceId == null || cupInstanceId.length == 0)
      throw "Suction bridge requires an end effector and cup instance";
    effector.validate();
    var cup:Null<SuctionCup> = null;
    for (member in effector.components()) if (member.id == cupInstanceId) {
      if (!Std.isOfType(member.component, SuctionCup))
        throw 'Member "$cupInstanceId" is not a suction cup';
      cup = cast member.component;
    }
    if (cup == null) throw 'Unknown suction cup "$cupInstanceId"';
    if (cup.effectiveAreaMm2 == null)
      throw 'Suction cup "$cupInstanceId" has no effective sealed area';
    var chain = effector.upstreamChain(cupInstanceId, "vacuum");
    for (member in effector.components()) if (Std.isOfType(member.component, VacuumGenerator) &&
        chain.indexOf('${member.id}/vacuum') >= 0) {
      var generator:VacuumGenerator = cast member.component;
      if (generator.ratedVacuumKpa == null)
        throw 'Vacuum generator "${member.id}" has no pressure rating';
      if (minimumCupVacuumKpa > generator.ratedVacuumKpa + 1e-9)
        throw 'Cup vacuum exceeds generator "${member.id}" rating';
    }
    var solved = effector.solve(state);
    var memberPose = solved.poses.get(cupInstanceId);
    if (memberPose == null) throw 'Missing solved pose for "$cupInstanceId"';
    var mountTContact = AssemblyFrames.compose(AssemblyFrames.inverse(solved.mountWorld),
      AssemblyFrames.compose(memberPose, effector.memberConnectorFrame(cupInstanceId, "contact")));
    var converted = EndEffectorFrames.toRobotFrame(mountTContact);
    var flangeTCup = new Transform3(new Vec3(converted.position.x,
      converted.position.y, converted.position.z), new Quat(converted.quaternion.x,
      converted.quaternion.y, converted.quaternion.z, converted.quaternion.w));
    var effectiveArea:Float = cast cup.effectiveAreaMm2;
    return new SuctionGrip(flangeTCup, effectiveArea * 1e-6,
      minimumCupVacuumKpa, frictionCoefficient, safetyFactor, cup.ratedMomentNm);
  }
}
