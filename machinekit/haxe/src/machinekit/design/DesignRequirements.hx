package machinekit.design;

import haxeon.wire.JsonWire;
import machinekit.design.DesignRequirement;

/** Copies portable data and remaps every reference when nesting an assembly. */
class DesignRequirements {
  public static function copy(requirement:DesignRequirement, map:String -> String, ?frame:materia.assembly.AssemblyRecord.AssemblyFrame):DesignRequirement {
    var r:DesignRequirement = JsonWire.decode(JsonWire.encode(requirement));
    return switch (r) {
      case FastenerJoint(id, screw, target, through):
        target.member = map(target.member);
        for (ref in through) ref.member = map(ref.member);
        FastenerJoint(map(id), map(screw), target, through);
      case AccessPath(id, anchor, envelopes, pose, obstacles):
        anchor.member = map(anchor.member);
        for (j in pose) j.joint = map(j.joint);
        var selection = switch (obstacles) {
          case WholeAssembly:
            WholeAssembly;
          case InstalledMembers(m):
            InstalledMembers(m.map(map));
        };
        AccessPath(map(id), anchor, envelopes, pose, selection);
      case InterfaceAlignment(id, relationship):
        InterfaceAlignment(
          map(id),
          materia.casting.InterfaceRelationship.InterfaceRelationships.remapMembers(relationship, map)
        );
      case Coaxial(id, first, second, tolerance):
        first.member = map(first.member);
        second.member = map(second.member);
        Coaxial(map(id), first, second, tolerance);
      case FoundationConnection(id, joint):
        FoundationConnection(map(id), materia.casting.FoundationJoint.FoundationJoints.remap(joint, map));
      case ServiceRemoval(id, members, steps, prerequisites, pose):
        for (j in pose) j.joint = map(j.joint);
        if(frame!=null)for(step in steps) {
          var v=materia.assembly.AssemblyFrames.transformVector(frame,step.direction[0],step.direction[1],step.direction[2]);step.direction=[v.x,v.y,v.z];
        }
        ServiceRemoval(map(id), members.map(map), steps, prerequisites.map(map), pose);
      case GuardedChannel(id, channel, covers, drain):
        for (d in drain) d.drain = map(d.drain);
        GuardedChannel(map(id), map(channel), covers.map(map), drain);
    };
  }
  public static function id(r:DesignRequirement):String return switch (r) {
    case FastenerJoint(id,
      _, _, _), AccessPath(id, _, _, _, _), InterfaceAlignment(id, _), FoundationConnection(id, _), Coaxial(
        id,
        _,
        _,
        _
      ), ServiceRemoval(id, _, _, _, _), GuardedChannel(id, _, _, _):
      id;
  };
}
