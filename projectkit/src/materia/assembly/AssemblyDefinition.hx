package materia.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyRecord.AssemblyConnector;

enum abstract AssemblyJointType(String) from String to String {
	var Fixed = "fixed";
	var Revolute = "revolute";
	var Continuous = "continuous";
	var Prismatic = "prismatic";
}

enum abstract AssemblyJointRole(String) from String to String {
	var Tree = "tree";
	var Closure = "closure";
}

@:wire typedef AssemblyVector = {
	@:id(1) var x:Float;
	@:id(2) var y:Float;
	@:id(3) var z:Float;
}

/** Missing limits are unbounded. Velocity and effort are optional metadata. */
@:wire typedef AssemblyJointLimits = {
	@:id(1) var lower:Null<Float>;
	@:id(2) var upper:Null<Float>;
	@:id(3) var velocity:Null<Float>;
	@:id(4) var effort:Null<Float>;
	/**
	 * How far the joint can travel past `lower` and `upper` before it meets its
	 * end stop, in the joint's units: a machine's limit switch sits beyond its
	 * soft limit. Motion stays within the limits; a simulation faults only past
	 * the overtravel. Absent, a simulation picks a small default.
	 */
	@:id(5) @:optional var overtravel:Null<Float>;
	/** Largest acceleration the joint's drive can give, in the joint's units per second squared. */
	@:id(6) @:optional var acceleration:Null<Float>;
}

/** Connectors belong to a reusable component definition, not an occurrence. */
@:wire typedef AssemblyComponentDefinition = {
	@:id(1) var id:String;
	@:id(2) var connectors:Array<AssemblyConnector>;
}

/** An occurrence references shared component data and has a local initial pose. */
@:wire typedef AssemblyComponentOccurrence = {
	@:id(1) var id:String;
	@:id(2) var definition:String;
	@:id(3) var initialPose:AssemblyFrame;
	/** References an entry in AssemblyDefinition.assemblies. */
	@:id(4) @:optional var assembly:String;
}

/** A persistent kinematic edge or closure. The axis is unit length in the parent connector frame. */
@:wire typedef KinematicJoint = {
	@:id(1) var id:String;
	@:id(2) var type:AssemblyJointType;
	@:id(3) var role:AssemblyJointRole;
	@:id(4) var parent:String;
	@:id(5) var parentConnector:String;
	@:id(6) var child:String;
	@:id(7) var childConnector:String;
	@:id(8) var axis:AssemblyVector;
	@:id(9) var limits:AssemblyJointLimits;
	@:id(10) var defaultValue:Float;
	/** Maximum closure position residual in the assembly length unit. */
	@:id(11) @:optional var closureTolerance:Float;
}

/** Target coordinate = source coordinate × ratio + offset. */
@:wire typedef AssemblyJointCoupling = {
	@:id(1) var id:String;
	@:id(2) var source:String;
	@:id(3) var target:String;
	@:id(4) var ratio:Float;
	@:id(5) var offset:Float;
}

/** A connector exported from a member of an assembly definition. */
@:wire typedef AssemblyExposedConnector = {
	@:id(1) var name:String;
	@:id(2) var occurrence:String;
	@:id(3) var connector:String;
}

/** Reusable nested assembly. Occurrences may reference another entry in the root table. */
@:wire typedef AssemblySubdefinition = {
	@:id(1) var id:String;
	@:id(2) var definitions:Array<AssemblyComponentDefinition>;
	@:id(3) var occurrences:Array<AssemblyComponentOccurrence>;
	@:id(4) var joints:Array<KinematicJoint>;
	@:id(5) @:optional var couplings:Array<AssemblyJointCoupling>;
	@:id(6) @:optional var exposedConnectors:Array<AssemblyExposedConnector>;
}

/**
 * Versioned assembly design data. `assemblies` is a reusable definition table;
 * an occurrence with `assembly` set names a table entry and exposes only its
 * declared connectors. Flattening prefixes nested occurrence and joint IDs
 * with their occurrence path. Runtime coordinates live in AssemblyStateRecord.
 */
@:wire typedef AssemblyDefinition = {
	@:id(1) var schemaVersion:Int;
	@:id(2) var id:String;
	@:id(3) @:optional var lengthUnit:String;
	@:id(4) var definitions:Array<AssemblyComponentDefinition>;
	@:id(5) var occurrences:Array<AssemblyComponentOccurrence>;
	@:id(6) var joints:Array<KinematicJoint>;
	@:id(7) @:optional var couplings:Array<AssemblyJointCoupling>;
	@:id(8) @:optional var assemblies:Array<AssemblySubdefinition>;
	@:id(9) @:optional var exposedConnectors:Array<AssemblyExposedConnector>;
}

@:wire typedef AssemblyJointCoordinate = {
	@:id(1) var joint:String;
	@:id(2) var value:Float;
}

@:wire typedef AssemblyRootPose = {
	@:id(1) var occurrence:String;
	@:id(2) var pose:AssemblyFrame;
}

/** A saved configuration, separate from the persistent assembly definition. */
@:wire typedef AssemblyStateRecord = {
	@:id(1) var schemaVersion:Int;
	@:id(2) var definition:String;
	@:id(3) var jointCoordinates:Array<AssemblyJointCoordinate>;
	@:id(4) var rootPoses:Array<AssemblyRootPose>;
}
