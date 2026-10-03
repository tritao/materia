package materia.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyRecord.AssemblyConnector;

enum abstract AssemblyJointType(String) from String to String {
	var Fixed = "fixed";
	var Revolute = "revolute";
	var Continuous = "continuous";
	var Prismatic = "prismatic";
	/** Closures only: a shared point. */
	var Spherical = "spherical";
	/** Closures only: a shared axis line, free to slide and turn along it. */
	var Cylindrical = "cylindrical";
	/** Closures only: the child connector's origin on the parent's plane (normal = axis), normals parallel. */
	var Planar = "planar";
}

enum abstract AssemblyJointRole(String) from String to String {
	var Tree = "tree";
	var Closure = "closure";
}

/**
	How a mate places its second connector frame B against its first, A, with
	a = A's axis and b = B's axis (the mate's `axis`, in each connector frame)
	and d = B's origin − A's origin. Mates position parts; joints move them.
*/
enum abstract AssemblyMateKind(String) from String to String {
	/** B's origin on A's origin. */
	var Coincident = "coincident";
	/** B's origin on A's axis line, axes parallel. */
	var Coaxial = "coaxial";
	/** B's origin on A's plane (normal a), offset by `value` along a; normals parallel. */
	var Planar = "planar";
	/** Axes parallel (either direction). */
	var Parallel = "parallel";
	/** Axes perpendicular. */
	var Perpendicular = "perpendicular";
	/** Origins `value` apart (length unit, positive; zero is `Coincident`). */
	var Distance = "distance";
	/** `value` radians between the axes, in [0, π] (0 and π hold the axes aligned or opposed). */
	var Angle = "angle";
	/** B's frame on A's frame. */
	var Lock = "lock";
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
	/** Mates never move it (see `AssemblyMateSolver`). */
	@:id(5) @:optional var grounded:Bool;
}

/** A placement relation between two occurrence connectors (see `AssemblyMateKind`). */
@:wire typedef AssemblyMate = {
	@:id(1) var id:String;
	@:id(2) var kind:AssemblyMateKind;
	@:id(3) var first:String;
	@:id(4) var firstConnector:String;
	@:id(5) var second:String;
	@:id(6) var secondConnector:String;
	/** Unit axis, expressed in each connector frame. */
	@:id(7) var axis:AssemblyVector;
	/** Planar offset or distance (length unit), or angle (radians). */
	@:id(8) @:optional var value:Float;
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
	/**
		An input of the mechanism (a motor, a cylinder): its coordinate is set, never solved. The other movable tree
		joints on a closure loop are dependent and follow it (see `AssemblyState.dependentJoints`). Only movable tree
		joints that are not coupling targets can be driven.
	*/
	@:id(12) @:optional var driven:Bool;
}

/**
 * Target coordinate = source coordinate × ratio + offset. A target with several couplings is the sum of
 * their terms: target = Σ (ratioᵢ × sourceᵢ + offsetᵢ), as a CoreXY motor follows both axes. Each coupling
 * is one term, with its own efficiency, stiffness, backlash and drag. Every (source, target) pair is
 * unique and the terms never form a cycle.
 */
@:wire typedef AssemblyJointCoupling = {
	@:id(1) var id:String;
	@:id(2) var source:String;
	@:id(3) var target:String;
	@:id(4) var ratio:Float;
	@:id(5) var offset:Float;
	/** Power delivered over power put in, between the joints, such as a lead screw's 0.4; 1 when absent. */
	@:id(6) @:optional var efficiency:Null<Float>;
	/** Force at the source per unit of source travel (N per assembly unit; N m/rad for a turning source); absent is rigid. */
	@:id(7) @:optional var stiffness:Null<Float>;
	/** Lost motion on reversal, in the source's units. */
	@:id(8) @:optional var backlash:Null<Float>;
	/** Constant resisting effort the coupling adds at the target while it moves (N m for a turning target). */
	@:id(9) @:optional var drag:Null<Float>;
}

/**
 * A motor driving a joint directly: its usable torque (N m) or force (N), its usable speed in the
 * joint's units per second, and the inertia of its rotor (kg m²), which turns with the joint.
 */
@:wire typedef AssemblyActuator = {
	@:id(1) var id:String;
	@:id(2) var joint:String;
	@:id(3) var maxEffort:Float;
	@:id(4) var maxRate:Float;
	@:id(5) @:optional var rotorInertia:Null<Float>;
	/** Full steps in a turn of a stepper motor's rotor; absent for other motors. */
	@:id(6) @:optional var fullStepsPerRevolution:Null<Float>;
	/**
	 * What drives the joint: "stepper" or "servo". With "stepper", `holdingTorque` and `torqueSpeed`
	 * describe its pull-out curve; with "servo", the rated and peak torque and speed do. Absent, a
	 * bare effort and rate (a stepper with only `fullStepsPerRevolution` is still understood).
	 */
	@:id(7) @:optional var drive:Null<String>;
	/** Alternating speed (the joint's units per second) and torque (N m), points of the torque-speed curve. */
	@:id(8) @:optional var torqueSpeed:Array<Float>;
	@:id(9) @:optional var holdingTorque:Null<Float>;
	@:id(10) @:optional var ratedTorque:Null<Float>;
	@:id(11) @:optional var peakTorque:Null<Float>;
	@:id(12) @:optional var ratedSpeed:Null<Float>;
	@:id(13) @:optional var maxSpeed:Null<Float>;
	@:id(14) @:optional var encoderCounts:Null<Float>;
	/** Default servo gains in the actuator's units: effort per unit of position and velocity error. */
	@:id(15) @:optional var servoStiffness:Null<Float>;
	@:id(16) @:optional var servoDamping:Null<Float>;
	/** The id of the `AssemblyEncoder` that reads this motor, which a servo's feedback comes from; absent for none. */
	@:id(17) @:optional var encoder:Null<String>;
	/**
	 * A gearbox between the motor and the joint: the motor turns `gearRatio` times for one turn (or one unit of
	 * travel) of the joint, and the joint gets `gearEfficiency` of its power. `maxEffort`, `maxRate`, the
	 * torque-speed curve and the rotor are then the motor's own, before the gearbox. Absent, the motor drives the
	 * joint directly.
	 */
	@:id(18) @:optional var gearRatio:Null<Float>;
	@:id(19) @:optional var gearEfficiency:Null<Float>;
}

/**
 * An encoder on a joint: it reads the joint's travel in counts. Which joint it sits on says what it
 * sees. On a motor's own joint it is motor-side and sees the rotor (lost steps, a servo's following
 * error, but not backlash or belt stretch); on a joint the load moves through a drive it is load-side
 * and sees where the load is.
 */
@:wire typedef AssemblyEncoder = {
	@:id(1) var id:String;
	@:id(2) var joint:String;
	/** "incremental" (quadrature counts from where it was powered up) or "absolute" (the position itself). */
	@:id(3) var kind:String;
	/** Counts per revolution on a turning joint, per millimetre on a sliding joint. */
	@:id(4) var counts:Float;
	/** An index pulse once a revolution on a turning joint, at the reference mark on a sliding one. */
	@:id(5) @:optional var index:Null<Bool>;
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
	@:id(7) @:optional var mates:Array<AssemblyMate>;
	@:id(8) @:optional var actuators:Array<AssemblyActuator>;
	@:id(9) @:optional var encoders:Array<AssemblyEncoder>;
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
	@:id(10) @:optional var mates:Array<AssemblyMate>;
	@:id(11) @:optional var actuators:Array<AssemblyActuator>;
	@:id(12) @:optional var encoders:Array<AssemblyEncoder>;
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
