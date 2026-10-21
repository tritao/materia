package machinekit.design;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Portable shapes in the declared interface frame; lengths in mm. */
@:wire enum DesignVolume {
	@:id(1) BoxVolume(width:Float,depth:Float,height:Float);
	@:id(2) CylinderVolume(radius:Float,start:Float,end:Float);
	@:id(3) RingVolume(radius:Float,bore:Float,start:Float,end:Float);
	@:id(4) SocketVolume(radius:Float,start:Float,end:Float,mouthRadius:Float,mouthEnd:Float);
}
@:wire typedef DesignEnvelope = {
	@:id(1) var id:String;
	@:id(2) var frame:AssemblyFrame;
	@:id(3) var volume:DesignVolume;
}
@:wire typedef DesignReference = {
	@:id(1) var member:String;
	@:id(2) var feature:String;
}
typedef DesignJointValue = materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
/** InstalledMembers describes a bench/assembly phase. WholeAssembly includes
 * surrounding parts added by the containing machine, without hardcoded lists.
 */
@:wire enum DesignObstacles {
	@:id(1) WholeAssembly;
	@:id(2) InstalledMembers(members:Array<String>);
}
@:wire enum DesignRequirement {
	@:id(1) FastenerJoint(id:String,screw:String,target:DesignReference,through:Array<DesignReference>);
	@:id(2) AccessPath(id:String,anchor:DesignReference,envelopes:Array<DesignEnvelope>,pose:Array<DesignJointValue>,obstacles:DesignObstacles);
	/** id identifies this assembly occurrence; relationship.id identifies the
	 * design relationship referenced by the source interfaces, across occurrences.
	 */
	@:id(3) InterfaceAlignment(id:String,relationship:materia.casting.InterfaceRelationship);
}
