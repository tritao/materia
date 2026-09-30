import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Closure solving on small planar mechanisms, in millimetres. */
class AssemblyLoopSmoke {
	public static function run():Void {
		var fourBar = new AssemblyState(fourBarDefinition(2200, null, null, 1.4));
		var result = fourBar.solveClosures(["coupler", "rocker"]);
		check(result.converged && result.status == "converged" && result.degreesOfFreedom == 0,
			'four-bar closes (${result.status})');
		check(fourBar.joint("crank") == 0.6, "the driving crank is not moved");
		fourBar.checkClosures();

		var slider = new AssemblyState(sliderCrankDefinition());
		check(slider.solveClosures(["rod", "slide"]).converged, "slider-crank closes through its prismatic joint");
		slider.checkClosures();

		var chain = new AssemblyState(fixedChainDefinition());
		var welded = chain.solveClosures(["j1", "j2", "j3"]);
		check(welded.converged && welded.degreesOfFreedom == 0, "a fixed closure pins a three-link chain");
		chain.checkClosures();

		var impossible = new AssemblyState(fourBarDefinition(5000, null, null, 1.4));
		var failed = impossible.solveClosures(["coupler", "rocker"]);
		check(!failed.converged && failed.status != "converged" && failed.closureIds.join(",") == "pin",
			"an unclosable loop fails and names its closure");
		check(impossible.joint("coupler") == 0.2 && impossible.joint("rocker") == 1.4,
			"a failed solve leaves the state untouched");

		var limited = new AssemblyState(fourBarDefinition(2200, 1.8, 2.0, 1.9));
		var blocked = limited.solveClosures(["coupler", "rocker"]);
		check(blocked.status == "limit-blocked", 'a limit in the way reports limit-blocked (${blocked.status})');
		check(limited.joint("rocker") == 1.9, "a limit-blocked solve leaves the state untouched");

		throws(() -> fourBar.solveClosures(["pin"]), "a closure joint cannot be a dependent coordinate");
		throws(() -> fourBar.solveClosures(["coupler", "coupler"]), "dependent coordinates must be distinct");
	}

	static function frame(x:Float, y:Float, z:Float, ?angle:Float = 0.0):AssemblyFrame
		return {x: x, y: y, z: z, qx: 0, qy: 0, qz: Math.sin(angle / 2), qw: Math.cos(angle / 2)};

	static function bar(id:String, length:Float):AssemblyComponentDefinition
		return {id: id, connectors: [{name: "a", frame: frame(0, 0, 0)}, {name: "b", frame: frame(length, 0, 0)}]};

	static function joint(id:String, type:AssemblyJointType, role:AssemblyJointRole, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float, ?lower:Float, ?upper:Float, ?alongX:Bool = false):KinematicJoint
		return {id: id, type: type, role: role, parent: parent, parentConnector: parentConnector, child: child,
			childConnector: childConnector, axis: alongX ? {x: 1.0, y: 0.0, z: 0.0} : {x: 0.0, y: 0.0, z: 1.0},
			limits: {lower: lower, upper: upper, velocity: null, effort: null}, defaultValue: value};

	static function definition(id:String, parts:Array<AssemblyComponentDefinition>, joints:Array<KinematicJoint>):AssemblyDefinition
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: id, lengthUnit: "mm", definitions: parts,
			occurrences: [for (part in parts) {id: part.id, definition: part.id, initialPose: frame(0, 0, 0)}],
			joints: joints};

	/** Ground pivots 2000 apart, crank 1000 (driven), coupler, rocker 1500. */
	static function fourBarDefinition(coupler:Float, lower:Null<Float>, upper:Null<Float>, rocker:Float):AssemblyDefinition
		return definition("four-bar", [bar("ground", 2000), bar("crank", 1000), bar("coupler", coupler), bar("rocker", 1500)], [
			joint("crank", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "a", "crank", "a", 0.6),
			joint("coupler", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "crank", "b", "coupler", "a", 0.2),
			joint("rocker", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "b", "rocker", "a", rocker, lower, upper),
			joint("pin", AssemblyJointType.Revolute, AssemblyJointRole.Closure, "coupler", "b", "rocker", "b", 0.0)]);

	static function sliderCrankDefinition():AssemblyDefinition
		return definition("slider-crank", [bar("ground", 0), bar("crank", 500), bar("rod", 1500), bar("slider", 0)], [
			joint("crank", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "a", "crank", "a", 0.9),
			joint("rod", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "crank", "b", "rod", "a", -0.5),
			joint("slide", AssemblyJointType.Prismatic, AssemblyJointRole.Tree, "ground", "b", "slider", "a", 1500, 0, 3000, true),
			joint("pin", AssemblyJointType.Revolute, AssemblyJointRole.Closure, "rod", "b", "slider", "a", 0.0)]);

	/** Three links welded at their tip to a ground connector placed where q = (0.5, -0.9, 0.7) puts it. */
	static function fixedChainDefinition():AssemblyDefinition {
		var lengths = [800.0, 600.0, 400.0], q = [0.5, -0.9, 0.7];
		var x = 0.0, y = 0.0, angle = 0.0;
		for (i in 0...3) { angle += q[i]; x += lengths[i] * Math.cos(angle); y += lengths[i] * Math.sin(angle); }
		var ground:AssemblyComponentDefinition = {id: "ground",
			connectors: [{name: "a", frame: frame(0, 0, 0)}, {name: "b", frame: frame(x, y, 0, angle)}]};
		return definition("fixed-chain", [ground, bar("l1", 800), bar("l2", 600), bar("l3", 400)], [
			joint("j1", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "a", "l1", "a", 0.45),
			joint("j2", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "l1", "b", "l2", "a", -0.8),
			joint("j3", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "l2", "b", "l3", "a", 0.6),
			joint("weld", AssemblyJointType.Fixed, AssemblyJointRole.Closure, "l3", "b", "ground", "b", 0.0)]);
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}

	static function throws(action:Void->Void, label:String):Void {
		var threw = false;
		try action() catch (_:Dynamic) threw = true;
		if (!threw) throw label;
	}
}
