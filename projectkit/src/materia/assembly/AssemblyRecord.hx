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
	/**
		Set when the frame is derived from the component's geometry by the application that owns it (CadKit's
		geometric connectors: a face or edge to find again). Opaque here: `frame` is the last frame it resolved
		to, and stays valid for every consumer that does not re-derive it.
	*/
	@:id(3) @:optional var reference:String;
}
