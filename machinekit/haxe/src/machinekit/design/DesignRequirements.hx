package machinekit.design;

import haxeon.wire.JsonWire;
import machinekit.design.DesignRequirement;

/** Copies portable data and remaps every reference when nesting an assembly. */
class DesignRequirements {
	public static function copy(requirement:DesignRequirement,map:String->String):DesignRequirement {
		var r:DesignRequirement=JsonWire.decode(JsonWire.encode(requirement));
		return switch(r) {
			case FastenerJoint(id,screw,target,through):
				target.member=map(target.member);for(ref in through)ref.member=map(ref.member);
				FastenerJoint(map(id),map(screw),target,through);
			case AccessPath(id,anchor,envelopes,pose,obstacles):
				anchor.member=map(anchor.member);for(j in pose)j.joint=map(j.joint);
				var selection=switch(obstacles) {case WholeAssembly:WholeAssembly;case InstalledMembers(m):InstalledMembers(m.map(map));};
				AccessPath(map(id),anchor,envelopes,pose,selection);
			case InterfaceAlignment(id,relationship):
				InterfaceAlignment(map(id),materia.casting.InterfaceRelationship.InterfaceRelationships.remapMembers(relationship,map));
			case FoundationConnection(id,joint):
				FoundationConnection(map(id),materia.casting.FoundationJoint.FoundationJoints.remap(joint,map));
		};
	}
	public static function id(r:DesignRequirement):String return switch(r) {
		case FastenerJoint(id,_,_,_),AccessPath(id,_,_,_,_),InterfaceAlignment(id,_),FoundationConnection(id,_): id;
	};
}
