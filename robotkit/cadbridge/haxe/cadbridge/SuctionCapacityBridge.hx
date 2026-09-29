package cadbridge;

import cadkit.modeling.AssemblyState;
import machinekit.component.ComponentCapability;
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
    var found = false;
    var suction = false;
    var effectiveArea:Null<Float> = null;
    var ratedMoment:Null<Float> = null;
    var vacuumPort = "vacuum";
    var contactConnector = "contact";
    for (member in effector.components()) if (member.id == cupInstanceId) {
      found = true;
      for (capability in member.component.capabilities()) switch capability {
        case Suction(area, moment, port, contact):
          suction = true;
          effectiveArea = area;
          ratedMoment = moment;
          vacuumPort = port;
          contactConnector = contact;
        case _:
      }
    }
    if (!found) throw 'Unknown suction cup "$cupInstanceId"';
    if (!suction) throw 'Member "$cupInstanceId" is not a suction cup';
    if (effectiveArea == null) throw 'Suction cup "$cupInstanceId" has no effective sealed area';
    var chain = effector.upstreamChain(cupInstanceId, vacuumPort);
    for (member in effector.components()) for (capability in member.component.capabilities())
      switch capability {
        case VacuumSource(rating, outputPort):
          if (chain.indexOf('${member.id}/$outputPort') >= 0) {
            if (rating == null) throw 'Vacuum generator "${member.id}" has no pressure rating';
            if (minimumCupVacuumKpa > rating + 1e-9)
              throw 'Cup vacuum exceeds generator "${member.id}" rating';
          }
        case _:
      }
    var solved = effector.solve(state);
    var memberPose = solved.poses.get(cupInstanceId);
    if (memberPose == null) throw 'Missing solved pose for "$cupInstanceId"';
    var mountTContact = AssemblyFrames.compose(AssemblyFrames.inverse(solved.mountWorld),
      AssemblyFrames.compose(memberPose, effector.memberConnectorFrame(cupInstanceId, contactConnector)));
    var converted = EndEffectorFrames.toRobotFrame(new machinekit.robotics.ConnectorFrame(mountTContact));
    var flangeTCup = new Transform3(new Vec3(converted.position.x,
      converted.position.y, converted.position.z), new Quat(converted.quaternion.x,
      converted.quaternion.y, converted.quaternion.z, converted.quaternion.w));
    return new SuctionGrip(flangeTCup, (cast effectiveArea : Float) * 1e-6,
      minimumCupVacuumKpa, frictionCoefficient, safetyFactor, ratedMoment);
  }
}
