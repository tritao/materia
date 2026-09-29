package materia.assembly;

/** Rigid frame in the artifact's length unit; quaternion order is x, y, z, w. */
@:wire typedef AssemblyFrame = {
	@:id(1) var x:Float;
	@:id(2) var y:Float;
	@:id(3) var z:Float;
	@:id(4) var qx:Float;
	@:id(5) var qy:Float;
	@:id(6) var qz:Float;
	@:id(7) var qw:Float;
}

@:wire typedef AssemblyConnector = {
	@:id(1) var name:String;
	@:id(2) var frame:AssemblyFrame;
}

typedef AssemblyInstance = {
	var id:String;
	var pose:AssemblyFrame;
	var connectors:Array<AssemblyConnector>;
}

/** Legacy assembly snapshot record. New kinematic definitions use AssemblyDefinition. */
typedef AssemblyJoint = {
	var id:String;
	var kind:String;
	var parent:String;
	var parentConnector:String;
	var child:String;
	var childConnector:String;
	var value:Float;
}

typedef AssemblyRecord = {
	var instances:Array<AssemblyInstance>;
	var joints:Array<AssemblyJoint>;
}
