package materia.project;

/** Rigid frame in the artifact's length unit; quaternion order is x, y, z, w. */
typedef AssemblyFrame = {
	var x:Float;
	var y:Float;
	var z:Float;
	var qx:Float;
	var qy:Float;
	var qz:Float;
	var qw:Float;
}

typedef AssemblyConnector = {
	var name:String;
	var frame:AssemblyFrame;
}

typedef AssemblyInstance = {
	var id:String;
	var pose:AssemblyFrame;
	var connectors:Array<AssemblyConnector>;
}

/** Joint frames coincide at value zero; the joint axis is local Y. */
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
