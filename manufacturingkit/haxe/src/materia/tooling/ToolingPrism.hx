package materia.tooling;

import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef ToolingPoint2 = {var x:Float; var y:Float;}

/** A convex CCW profile extruded along local +Z from zero to height, in mm.
 * An envelope is the union of these prisms; overlap is permitted.
 */
typedef ToolingPrism = {
	var id:String;
	var profile:Array<ToolingPoint2>;
	var height:Float;
	var pose:AssemblyFrame;
}
