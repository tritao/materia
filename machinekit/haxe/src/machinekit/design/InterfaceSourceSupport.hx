package machinekit.design;

import machinekit.component.MachineComponent;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.KinematicJoint;

/** One reference resolver for design diagnostics and detached tooling snapshots. */
class InterfaceSourceSupport {
	public static function precision(members:Map<String,MachineComponent>,partId:String,interfaceId:String,
			definitionId:String):materia.casting.InterfaceRequirement {
		var component=members.get(partId);
		if (component==null) throw "Unknown relationship component: "+partId;
		var interfaces=DesignInterfaces.of(component);
		if (interfaces==null) throw "Relationship component has no design interfaces: "+partId;
		var matches=interfaces.precisionRequirements().filter(requirement -> requirement.id==interfaceId);
		if (matches.length!=1) throw "Unknown relationship precision interface: "+partId+"/"+interfaceId;
		if (matches[0].relationshipIds.indexOf(definitionId)<0)
			throw "Source interface omits participating relationship: "+partId+"/"+interfaceId+": "+definitionId;
		return matches[0];
	}

	public static function axis(definition:AssemblyDefinition,id:String):KinematicJoint {
		var joints=definition.joints.filter(joint -> joint.id==id);
		if (joints.length!=1) throw "Unknown relationship machine axis: "+id;
		var kind=joints[0].type;
		if (kind!=materia.assembly.AssemblyDefinition.AssemblyJointType.Revolute
			&& kind!=materia.assembly.AssemblyDefinition.AssemblyJointType.Continuous
			&& kind!=materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic)
			throw "Relationship axis needs a single directed joint: "+id;
		return joints[0];
	}
}
