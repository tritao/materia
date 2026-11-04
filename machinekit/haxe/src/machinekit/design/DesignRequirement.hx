package machinekit.design;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Portable shapes in the declared interface frame; lengths in mm. */
@:wire enum DesignVolume {
  @:id(1) BoxVolume(width:Float, depth:Float, height:Float);
  @:id(2) CylinderVolume(radius:Float, start:Float, end:Float);
  @:id(3) RingVolume(radius:Float, bore:Float, start:Float, end:Float);
  @:id(4) SocketVolume(radius:Float, start:Float, end:Float, mouthRadius:Float, mouthEnd:Float);
}
@:wire typedef DesignEnvelope = {
  @:id(1) var id:String;
  @:id(2) var frame:AssemblyFrame;
  @:id(3) var volume:DesignVolume;
}
typedef DesignReference = materia.assembly.AssemblyFeatureReference;
typedef DesignJointValue = materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
/** InstalledMembers describes a bench/assembly phase. WholeAssembly includes
 * surrounding parts added by the containing machine, without hardcoded lists.
 */
@:wire enum DesignObstacles {
  @:id(1) WholeAssembly;
  @:id(2) InstalledMembers(members:Array<String>);
}
/** One straight leg of a service removal: a world direction at the rest pose (it is normalised) and the distance, mm. */
@:wire typedef ServiceStep = {
  @:id(1) var direction:Array<Float>;
  @:id(2) var distance:Float;
}
/**
 * Where the chips and coolant of a guarded channel leave it: the channel floor falls from `high` to `low` (world points
 * at rest, mm) and a drain bore of `diameter` runs down from `low` to height `outletZ`, cut in member `drain`.
 */
@:wire typedef ChannelDrain = {
  @:id(1) var high:Array<Float>;
  @:id(2) var low:Array<Float>;
  @:id(3) var drain:String;
  @:id(4) var diameter:Float;
  @:id(5) var outletZ:Float;
}
@:wire enum DesignRequirement {
  @:id(1) FastenerJoint(id:String, screw:String, target:DesignReference, through:Array<DesignReference>);
  @:id(2) AccessPath(
    id:String,
    anchor:DesignReference,
    envelopes:Array<DesignEnvelope>,
    pose:Array<DesignJointValue>,
    obstacles:DesignObstacles
  );
  /** id identifies this assembly occurrence; relationship.id identifies the
	 * design relationship referenced by the source interfaces, across occurrences.
	 */
  @:id(3) InterfaceAlignment(id:String, relationship:materia.casting.InterfaceRelationship);
  @:id(4) FoundationConnection(id:String, joint:materia.casting.FoundationJoint);
  /** Nominal hole/connector axes; physical metrology is a separate requirement. */
  @:id(5) Coaxial(id:String, first:DesignReference, second:DesignReference, toleranceMm:Float);
  /**
	 * The service removal of `members` (a part with whatever travels with it): they move along `steps` in turn, at joint
	 * values `pose`, with `prerequisites` already taken away (hatches, covers, the motor before the screw). The screws
	 * that hold the removed and the prerequisite members are taken away too. `AssemblyIntegrity` sweeps it against the
	 * rest and reports `integrity.service.<id>`.
	 */
  @:id(6) ServiceRemoval(
    id:String,
    members:Array<String>,
    steps:Array<ServiceStep>,
    prerequisites:Array<String>,
    pose:Array<DesignJointValue>
  );
  /**
	 * A channel (a trench, a cavity in member `channel`) that way covers protect. The chips and coolant that pass the covers
	 * must be able to leave it: `drain` is absent when the design states no way out, which `AssemblyIntegrity` reports
	 * (`integrity.drain.<id>`).
	 */
  @:id(7) GuardedChannel(id:String, channel:String, covers:Array<String>, drain:Array<ChannelDrain>);
}
