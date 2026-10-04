import machinekit.assembly.PosedParts;
import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the CoreXY pen plotter. */
class CoreXyPlotterPreview {
	public static inline var ASSEMBLY_ID:String = "corexy-plotter";

	/** Geometry, joints and initial pose of the plotter (parts with equal designations share geometry). */
	public static function plotter():Bytes
		return SceneArtifact.encode(AssemblyPreview.scene(new CoreXyPlotter(), ASSEMBLY_ID));
}

/** A pulley's coefficients on x and y: it turns `x * x + y * y` pitch radii per millimetre. */
class PulleyTerm {
	public final id:String;
	public final x:Int;
	public final y:Int;

	public function new(id:String, x:Int, y:Int) {
		this.id = id;
		this.x = x;
		this.y = y;
	}
}

/** Geometry builds, belts and pulleys agree with the head's motion, and nothing collides. */
class CoreXyPlotterChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	/** Each pulley's angle per millimetre of x and of y, in pitch radii: its belt's side times the belt's heading along each axis. */
	static final TERMS:Array<PulleyTerm> = [
		new PulleyTerm("pulleyA", 1, -1), new PulleyTerm("idlerAFront", 1, -1), new PulleyTerm("idlerARear", 1, -1),
		new PulleyTerm("pulleyB", 1, 1), new PulleyTerm("idlerBFront", 1, 1), new PulleyTerm("idlerBRear", 1, 1),
		new PulleyTerm("idlerAStart", 1, 0), new PulleyTerm("idlerAEnd", -1, 0),
		new PulleyTerm("idlerBStart", 1, 0), new PulleyTerm("idlerBEnd", -1, 0)
	];

	public static function run():Void {
		parts = new PosedParts();
		try runChecks() catch (error:Dynamic) {
			parts.close();
			throw error;
		}
		parts.close();
	}

	/** The geometry of the members posed by this run, built once each; closed when the run ends. */
	static var parts:PosedParts;

	static function runChecks():Void {
		var scene = SceneArtifact.decode(CoreXyPlotterPreview.plotter());
		var definition = scene.assemblyDefinition;
		var plotter = new CoreXyPlotter();
		if (definition == null || definition.occurrences.length != plotter.components().length)
			throw "CoreXY plotter preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'CoreXY plotter part "${part.id}" has no mass properties';
		var prismatic = [for (joint in definition.joints) if (Std.string(joint.type) == "prismatic") joint.id];
		if (prismatic.join(",") != "y,x") throw 'CoreXY plotter should have prismatic joints y, x, got $prismatic';
		var turning = [for (joint in definition.joints) if (Std.string(joint.type) == "continuous") joint.id];
		if (turning.length != TERMS.length) throw 'CoreXY plotter should have ${TERMS.length} turning pulleys, got $turning';
		var motors = [for (entry in plotter.components()) if (Std.isOfType(entry.component, machinekit.motion.NemaStepper)) entry.id];
		if (motors.join(",") != "motorA,motorB") throw 'CoreXY plotter should have motors A and B, got $motors';

		// Every pulley follows its belt: one coupling per axis it sees, of the pulley's pitch radius.
		var radius = plotter.pulleyRadius;
		var couplings = definition.couplings;
		if (couplings == null) throw "CoreXY plotter has no couplings";
		var expectedCouplings = 0;
		for (term in TERMS) expectedCouplings += (term.x != 0 ? 1 : 0) + (term.y != 0 ? 1 : 0);
		if (couplings.length != expectedCouplings) throw 'CoreXY plotter should have $expectedCouplings couplings, got ${couplings.length}';
		for (coupling in couplings) {
			var drive = plotter.transmissionFor(coupling.id);
			if (drive == null || !switch drive.source { case TimingBelt(_, pulley), BeltIdler(_, pulley): pulley + "-turn" == coupling.target; default: false; })
				throw 'coupling ${coupling.id} should be the belt drive of its pulley';
			near(Math.abs(coupling.ratio), 1 / radius, '${coupling.id} turns one pitch radius per radian', 1e-12);
		}
		// Only the motors' pulleys have two leaders.
		var leaders = new Map<String, Int>();
		for (coupling in couplings) leaders.set(coupling.target, (leaders.exists(coupling.target) ? leaders.get(coupling.target) : 0) + 1);
		for (term in TERMS) {
			var count = leaders.get(term.id + "-turn");
			var wanted = (term.x != 0 ? 1 : 0) + (term.y != 0 ? 1 : 0);
			if (count != wanted) throw '${term.id} should follow $wanted axes, got $count';
		}

		var model = new AssemblyModel("mm");
		plotter.addTo(model, "");
		var state = new AssemblyState(model.definition(CoreXyPlotterPreview.ASSEMBLY_ID));
		var base = state.worldPose("base");
		var baseSum = base.x + base.y + base.z;
		var travel = [[0.0, 0], [35, 35], [-35, 35], [35, -35], [-35, -35], [17.5, -22], [-33, 9]];
		for (at in travel) {
			state.setJoint("x", at[0]);
			state.setJoint("y", at[1]);
			state.forwardKinematics();
			var label = 'pen at ${at.join(", ")}';
			var tip = state.worldConnector("pen", "tip");
			near(tip.x, at[0], '$label, x', 1e-6);
			near(tip.y - at[1], -13, '$label, y (the pen sits 13 mm in front of the middle line of the gantry)', 1e-6);
			near(tip.z, CoreXyPlotter.PEN_BASE, '$label, height', 1e-6);
			for (term in TERMS)
				near(state.joint(term.id + "-turn"), (term.x * at[0] + term.y * at[1]) / radius, '$label: ${term.id}', 1e-9);
			near(state.joint("pulleyA-turn"), (at[0] - at[1]) / radius, '$label: motor A', 1e-9);
			near(state.joint("pulleyB-turn"), (at[0] + at[1]) / radius, '$label: motor B', 1e-9);
			near(state.worldPose("base").x + state.worldPose("base").y + state.worldPose("base").z, baseSum, "the base stays put", 1e-9);
		}
		state.setJoint("x", 0);
		state.setJoint("y", 0);
		state.forwardKinematics();
		var home = state.worldConnector("pen", "tip");
		near(home.x, 0, "the pen is over the middle of the bed at home, x", 1e-9);

		// One motor alone moves the head diagonally; both together, along an axis.
		var a = 0.25 * radius;
		state.setJoint("x", radius * a / 2);
		state.setJoint("y", -radius * a / 2);
		state.forwardKinematics();
		near(state.joint("pulleyA-turn"), a, "motor A turned alone, motor A", 1e-9);
		near(state.joint("pulleyB-turn"), 0, "motor A turned alone leaves motor B still", 1e-9);
		near(state.worldConnector("pen", "tip").x - home.x, -(state.worldConnector("pen", "tip").y - home.y), "so the head moves along a diagonal", 1e-9);

		// The belts' geometry: a belt is as long as it was while the gantry moves (the strands that
		// shorten make up for those that lengthen), whole teeth, and the strand clamped to the
		// carriage runs along the gantry.
		for (letter in ["A", "B"]) {
			var belt = beltOf(plotter, "belt" + letter);
			near(belt.length, loopAt(plotter, letter, 25).length, 'belt $letter keeps its length as the gantry moves', 1e-9);
			var strands = belt.strands();
			near(Math.abs(strands[0].dx), 1, 'belt $letter: the clamped strand runs along X', 1e-9);
			near(strands[0].dy, 0, 'belt $letter: the clamped strand runs along X', 1e-9);
			near(Math.abs(strands[1].dy), 1, 'belt $letter: the strand from the gantry to the frame runs along Y', 1e-9);
			// Moving the gantry by 25 mm lengthens the strand to the front corner by 25 and shortens the one from the rear.
			var moved = loopAt(plotter, letter, 25).strands();
			near(moved[1].length - strands[1].length, 25, 'belt $letter: the front strand lengthens with y', 1e-9);
			near(moved[4].length - strands[4].length, -25, 'belt $letter: the rear strand shortens with y', 1e-9);
			near(moved[0].length, strands[0].length, 'belt $letter: the gantry strand keeps its length', 1e-9);
		}
		// The belt feed at the motor pulley, from the belt's own path: the distance from the carriage's clamp along the
		// belt to the driving pulley changes by dy_strand1 * dy - dx_strand0 * dx, so the pulley turns by the opposite.
		for (letter in ["A", "B"]) {
			var belt = beltOf(plotter, "belt" + letter);
			var rest = belt.strands();
			var wrap = belt.wraps()[3];
			for (at in [[10.0, 0.0], [0.0, 20.0], [-15.0, 12.0]]) {
				var moved = loopAt(plotter, letter, at[1]).strands();
				var toPulley = (moved[0].length - at[0] * rest[0].dx) + moved[1].length + moved[2].length;
				var before = rest[0].length + rest[1].length + rest[2].length;
				// The clamp moves on strand 0 along its heading by x (dx is ±1): the path to the pulley is shorter by that.
				var feed = -(toPulley - before);
				var turn = wrap.side * feed / radius;
				state.setJoint("x", at[0]);
				state.setJoint("y", at[1]);
				state.forwardKinematics();
				near(state.joint("pulley" + letter + "-turn"), turn, 'belt $letter: motor pulley follows the belt feed at ${at.join(", ")}', 1e-9);
			}
		}

		// Parts of the moving gantry and carriage clear the frame at the corners of travel.
		var gantry = ["blockYLeft", "blockYRight", "bracketLeft", "bracketRight", "crossbar", "railX", "idlerAStartPin", "idlerAEndPin",
			"idlerBStartPin", "idlerBEndPin"];
		var carriage = ["blockX", "carriagePlate", "clamp", "pen"];
		var frame = ["base", "railYLeft", "railYRight", "motorA", "motorB", "pulleyA", "pulleyB", "idlerAFront", "idlerARear", "idlerBFront",
			"idlerBRear", "idlerAFrontPin", "idlerARearPin", "idlerBFrontPin", "idlerBRearPin"];
		for (at in [[35.0, 35], [-35.0, 35], [35.0, -35], [-35.0, -35], [0.0, 0.0]]) {
			checkClear(plotter, state, at, gantry.concat(carriage), frame);
			checkClear(plotter, state, at, carriage, gantry.filter(id -> id != "railX"));
		}
		// The carriage's blocks stay on their rails and each axis has room past its travel.
		for (pair in [["railYLeft", "blockYLeft"], ["railX", "blockX"]]) {
			var rail = [for (entry in plotter.components()) if (entry.id == pair[0]) cast(entry.component, machinekit.motion.LinearRail)][0];
			var reach = rail.spec.railEndMargin + rail.spec.blockLength / 2;
			for (at in [[35.0, 35], [-35.0, -35]]) {
				state.setJoint("x", at[0]);
				state.setJoint("y", at[1]);
				state.forwardKinematics();
				var local = AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(pair[0])), state.worldPose(pair[1]));
				near(local.x, 0, '${pair[1]} centred on ${pair[0]}', 1e-6);
				near(local.y, 0, '${pair[1]} seated on ${pair[0]}', 1e-6);
				if (!(local.z >= reach - 1e-6 && local.z <= rail.length - reach + 1e-6)) throw '${pair[1]} runs off ${pair[0]}';
			}
		}
		if (!(plotter.axisOvertravel("x") > 0 && plotter.axisOvertravel("y") > 0)) throw "each axis has room past its travel";

		// At home each belt is clamped by the carriage's pad on exactly its one strand, and clear of everything else.
		var clamp = 24.0 * 1.38 * CoreXyPlotter.BELT_WIDTH;
		for (letter in ["A", "B"]) {
			near(overlap(plotter, state, [0, 0], "belt" + letter, "clamp"), clamp, 'the clamp holds belt $letter\'s one strand', 0.5);
			for (other in ["base", "motorA", "motorB", "bracketLeft", "bracketRight", "crossbar", "carriagePlate", "blockX", "railX", "pen",
					"idlerAStartPin", "idlerAEndPin", "idlerBStartPin", "idlerBEndPin"])
				if (overlap(plotter, state, [0, 0], "belt" + letter, other) > 1e-3) throw 'belt $letter runs through $other';
		}
		var mass = plotter.massProperties().mass;
		Sys.println('corexy plotter: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${plotter.billOfMaterials().lines().length} BOM lines, ${Math.round(mass * 100) / 100} kg, pulley radius ${Math.round(radius * 1000) / 1000} mm');
	}

	static function beltOf(plotter:CoreXyPlotter, id:String):TimingBelt {
		for (entry in plotter.components()) if (entry.id == id) return cast entry.component;
		throw 'CoreXY plotter has no member "$id"';
	}

	/** Belt `letter`'s loop with the gantry's idlers at `y`, the frame's where they are. */
	static function loopAt(plotter:CoreXyPlotter, letter:String, y:Float):TimingBelt {
		var wraps = beltOf(plotter, "belt" + letter).wraps();
		for (index in 0...2) wraps[index] = new BeltWrap(wraps[index].x, wraps[index].y + y, wraps[index].radius, wraps[index].side);
		return new TimingBelt(GT2, CoreXyPlotter.BELT_WIDTH, wraps);
	}

	/** No member of `moving` intersects a member of `others` at the pen's position `at`. */
	static function checkClear(plotter:CoreXyPlotter, state:AssemblyState, at:Array<Float>, moving:Array<String>, others:Array<String>):Void {
		moveTo(state, at);
		var first:Array<Part> = [], second:Array<Part> = [];
		try {
			for (id in moving) first.push(posed(plotter, state, id));
			for (id in others) second.push(posed(plotter, state, id));
			var firstBoxes = [for (part in first) PosedParts.boxOf(part)], secondBoxes = [for (part in second) PosedParts.boxOf(part)];
			for (a in 0...first.length) for (b in 0...second.length) {
				var volume = PosedParts.commonVolume(first[a], firstBoxes[a], second[b], secondBoxes[b]);
				if (volume > 1e-3) throw '${moving[a]} collides with ${others[b]} at ${at.join(", ")}: ${Math.round(volume)} mm³';
			}
		} catch (error:Dynamic) {
			PosedParts.closeAll(first);
			PosedParts.closeAll(second);
			throw error;
		}
		PosedParts.closeAll(first);
		PosedParts.closeAll(second);
	}

	static function moveTo(state:AssemblyState, at:Array<Float>):Void {
		state.setJoint("x", at[0]);
		state.setJoint("y", at[1]);
		state.forwardKinematics();
	}

	static function overlap(plotter:CoreXyPlotter, state:AssemblyState, at:Array<Float>, a:String, b:String):Float {
		moveTo(state, at);
		var first = posed(plotter, state, a), second = posed(plotter, state, b);
		var volume = PosedParts.commonVolume(first, PosedParts.boxOf(first), second, PosedParts.boxOf(second));
		first.close();
		second.close();
		return volume;
	}

	static function posed(plotter:CoreXyPlotter, state:AssemblyState, id:String):Part {
		for (entry in plotter.components()) if (entry.id == id) return parts.posed(entry.component, state.worldPose(id));
		throw 'CoreXY plotter has no member "$id"';
	}
}

/** Standalone check of the plotter example. */
function main():Void CoreXyPlotterChecks.run();
