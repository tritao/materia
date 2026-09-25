package materia.project;

import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.AssemblyRecord.AssemblyConnector;

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

typedef AssemblyVector = {
	var x:Float;
	var y:Float;
	var z:Float;
}

/** Missing limits are unbounded. Velocity and effort are optional metadata. */
typedef AssemblyJointLimits = {
	var lower:Null<Float>;
	var upper:Null<Float>;
	var velocity:Null<Float>;
	var effort:Null<Float>;
}

/** Connectors belong to a reusable component definition, not an occurrence. */
typedef AssemblyComponentDefinition = {
	var id:String;
	var connectors:Array<AssemblyConnector>;
}

/** An occurrence references shared component data and has a local initial pose. */
typedef AssemblyComponentOccurrence = {
	var id:String;
	var definition:String;
	var initialPose:AssemblyFrame;
}

/** A persistent kinematic edge or closure. The axis is unit length in the parent connector frame. */
typedef KinematicJoint = {
	var id:String;
	var type:AssemblyJointType;
	var role:AssemblyJointRole;
	var parent:String;
	var parentConnector:String;
	var child:String;
	var childConnector:String;
	var axis:AssemblyVector;
	var limits:AssemblyJointLimits;
	var defaultValue:Float;
}

/** Versioned assembly design data. Runtime joint coordinates live in AssemblyStateRecord. */
typedef AssemblyDefinition = {
	var schemaVersion:Int;
	var id:String;
	var definitions:Array<AssemblyComponentDefinition>;
	var occurrences:Array<AssemblyComponentOccurrence>;
	var joints:Array<KinematicJoint>;
}

typedef AssemblyJointCoordinate = {
	var joint:String;
	var value:Float;
}

typedef AssemblyRootPose = {
	var occurrence:String;
	var pose:AssemblyFrame;
}

/** A saved configuration, separate from the persistent assembly definition. */
typedef AssemblyStateRecord = {
	var schemaVersion:Int;
	var definition:String;
	var jointCoordinates:Array<AssemblyJointCoordinate>;
	var rootPoses:Array<AssemblyRootPose>;
}
