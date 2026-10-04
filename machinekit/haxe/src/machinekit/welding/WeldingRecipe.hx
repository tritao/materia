package machinekit.welding;

import machinekit.welding.WeldSeam.SeamFrame;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactTorchPose;

/** The wire a recipe is made for: what the feeder in the CAD declares (its `WireFeed` capability). */
typedef RecipeWire = {
	var diameterMm:Float;
	/** Fraction of the melted wire that reaches the weld. */
	var depositionEfficiency:Float;
	/** The feeder's top wire speed, in metres per minute. */
	var maxSpeedMPerMin:Float;
}

/** A pass's settings and tip offset along the CAD face normals, in millimetres. */
typedef WeldingPassRecipe = {
	var name:String;
	var recipe:WeldingRecipe;
	var faceA:Float;
	var faceB:Float;
	var weaveAmplitude:Float;
	/** Cycles per millimetre of seam progress. */
	var weaveFrequency:Float;
	var edgeDwell:Float;
	var interpassDwell:Float;
}

/**
 * How a weld is run, derived from the seam and the wire: the settings of a MIG fillet in solid steel wire that lay a leg
 * of the size the weldment asks for. Nothing in it is a seam's geometry; that comes from the CAD (`WeldSeam`). The
 * recipe is the generator's inverse of what the simulated welder does: it picks a wire speed for the leg, the voltage
 * that goes with it, and the travel speed at which that wire speed deposits the cross-section of an equal-leg fillet,
 * `area = leg² / 2`. It assumes the wire's deposition efficiency, which the CAD gives with the wire. The leg the weld then
 * reaches is measured, not derived: the player compares the bead the welder laid with the leg asked for, so a wrong
 * efficiency, a wire that is not the one assumed, or a torch that does not hold the speed shows as a leg that is off.
 *
 * - The wire speed grows with the leg (`1.5 · leg + 0.5` metres per minute, clamped), and must be one the feeder can
 *   feed: a leg that needs more is refused.
 * - The voltage follows the wire speed on a synergic line, `15 + 1.1 · wire speed` volts, rounded to half a volt.
 * - The travel speed is `wire speed · wire area · efficiency / area`.
 */
class WeldingRecipe {
	/** Wire speed, metres per minute. */
	public final wireSpeed:Float;
	/** Voltage setpoint, volts. */
	public final voltage:Float;
	/** Travel speed along the seam, millimetres per second. */
	public final travelSpeed:Float;
	/** The leg this recipe is for, millimetres. */
	public final legSize:Float;
	/** Seconds the torch stays at the start with the arc up before travelling. */
	public final startDwell:Float;
	/** Seconds it stays at the end with the arc up, filling the crater. */
	public final craterDwell:Float;
	/** Seconds the arc burns back, the wire stopped, before it is switched off. */
	public final burnback:Float;
	/** Distance the torch comes in from, along the wire, millimetres. */
	public final approach:Float;

	public function new(wireSpeed:Float, voltage:Float, travelSpeed:Float, legSize:Float, startDwell:Float, craterDwell:Float,
			burnback:Float, approach:Float) {
		if (!(wireSpeed > 0) || !(voltage > 0) || !(travelSpeed > 0) || !(legSize > 0) || !(startDwell >= 0) || !(craterDwell >= 0) ||
				!(burnback >= 0.05) || !(approach > 0))
			throw "A welding recipe needs a positive wire speed, voltage, travel speed, leg and approach, dwells of zero or more, and a burnback of at least 0.05 s";
		this.wireSpeed = wireSpeed;
		this.voltage = voltage;
		this.travelSpeed = travelSpeed;
		this.legSize = legSize;
		this.startDwell = startDwell;
		this.craterDwell = craterDwell;
		this.burnback = burnback;
		this.approach = approach;
	}

	/** The recipe for a fillet of `leg` millimetres in `wire`. */
	public static function fillet(leg:Float, wire:RecipeWire):WeldingRecipe {
		if (!(leg >= 2 && leg <= 8)) throw 'A single-pass fillet recipe covers legs of 2 to 8 mm, not $leg';
		if (wire == null || !(wire.diameterMm > 0) || !(wire.depositionEfficiency > 0 && wire.depositionEfficiency <= 1) || !(wire.maxSpeedMPerMin > 0))
			throw "A recipe needs the wire: its diameter, deposition efficiency and the feeder's top speed";
		var speed = Math.min(16.0, Math.max(3.0, 1.5 * leg + 0.5));
		if (speed > wire.maxSpeedMPerMin)
			throw 'A ${leg} mm fillet needs ${speed} m/min of wire, past the feeder\'s top speed of ${wire.maxSpeedMPerMin} m/min';
		var voltage = Math.round((15.0 + 1.1 * speed) * 2) / 2;
		var wireArea = Math.PI * wire.diameterMm * wire.diameterMm / 4.0;
		var feed = speed * 1000.0 / 60.0;
		var area = leg * leg / 2.0;
		return new WeldingRecipe(speed, voltage, feed * wireArea * wire.depositionEfficiency / area, leg, 0.15, 0.15, 0.1, 40);
	}

	/** Equal-leg fillet area is conserved across root, fill and cap; offsets target the previous bead's surface. */
	public static function passes(leg:Float, wire:RecipeWire, interpassDwell:Float = 0):Array<WeldingPassRecipe> {
		if (!(leg >= 2 && leg <= 12)) throw 'A fillet recipe covers legs of 2 to 12 mm, not $leg';
		if (!(interpassDwell >= 0 && interpassDwell <= 10)) throw "Interpass dwell must be between 0 and 10 seconds";
		var fractions = leg <= 8 ? [1.0] : [0.25, 0.375, 0.375];
		var result:Array<WeldingPassRecipe> = [];
		var deposited = 0.0;
		for (index in 0...fractions.length) {
			var passLeg = leg * Math.sqrt(fractions[index]);
			var recipe = fillet(passLeg, wire);
			var pass = singlePass(recipe);
			var previousLeg = leg * Math.sqrt(deposited);
			// A fillet's exposed face joins the two leg endpoints. Shift toward alternate faces on fill and cap.
			pass.name = fractions.length == 1 ? "single" : ["root", "fill", "cap"][index];
			if (index > 0) {
				pass.faceA = previousLeg * (index == 1 ? 0.65 : 0.35);
				pass.faceB = previousLeg - pass.faceA;
				pass.interpassDwell = interpassDwell;
			}
			result.push(pass);
			deposited += fractions[index];
		}
		return result;
	}

	static function singlePass(recipe:WeldingRecipe):WeldingPassRecipe {
		var leg = recipe.legSize;
		var woven = leg > 6;
		return {name: "single", recipe: recipe, faceA: woven ? leg * 0.25 : 0.0, faceB: woven ? leg * 0.25 : 0.0,
			weaveAmplitude: woven ? leg * 0.2 : 0.0, weaveFrequency: woven ? 0.1 : 0.0,
			edgeDwell: woven ? 0.05 : 0.0, interpassDwell: 0.0};
	}

	static function savedPass(pass:WeldingPassRecipe):materia.project.SceneArtifact.SceneArtifactWeldPass {
		var recipe = pass.recipe;
		var saved:materia.project.SceneArtifact.SceneArtifactWeldPass = {name: pass.name,
			offset: [pass.faceA * 0.001, pass.faceB * 0.001], interpassDwell: pass.interpassDwell,
			process: {wireSpeed: recipe.wireSpeed, voltage: recipe.voltage, travelSpeed: recipe.travelSpeed * 0.001,
				approach: recipe.approach * 0.001, startDwell: recipe.startDwell, craterDwell: recipe.craterDwell, burnback: recipe.burnback}};
		if (pass.weaveAmplitude > 0) saved.weave = {pattern: "sine", amplitude: pass.weaveAmplitude * 0.001,
			cyclesPerMetre: pass.weaveFrequency * 1000, edgeDwell: pass.edgeDwell};
		return saved;
	}

	/** Generate the shared CAD path once, then attach the area-conserving pass schedule. */
	public static function passStep(leg:Float, wire:RecipeWire, seams:Array<WeldSeam>, frame:String, metal:String,
			metresPerUnit:Float, interpassDwell:Float = 0):SceneArtifactMissionStep {
		var schedule = passes(leg, wire, interpassDwell);
		var step = schedule[0].recipe.step(seams, frame, metal, metresPerUnit);
		var weld = step.weld;
		if (weld == null) throw "A recipe produced no weld";
		weld.legSize = leg * 0.001;
		weld.passes = [for (pass in schedule) savedPass(pass)];
		return step;
	}

	/**
	 * The mission step that welds `seams` as one weld, run in order, each starting where the one before ends (a straight
	 * seam is one, a chain of the sides of a tube several; see `WeldSeams.chains`). The seams are in the millimetre frame of
	 * the occurrence `frame`, the workpiece's reference member, and the step's path is relative to it, so a player finds
	 * the workpiece where it stands. Lengths in the step are metres, whatever the scene's unit (`metresPerUnit`). The weld
	 * metal is carried by the occurrence `metal`.
	 */
	public function step(seams:Array<WeldSeam>, frame:String, metal:String, metresPerUnit:Float): SceneArtifactMissionStep {
		if (seams == null || seams.length == 0) throw "A weld needs a seam";
		function pose(seamFrame:SeamFrame):SceneArtifactTorchPose {
			var tip = seamFrame.toAssemblyFrame();
			return {position: [tip.x * metresPerUnit, tip.y * metresPerUnit, tip.z * metresPerUnit], rotation: [tip.qx, tip.qy, tip.qz, tip.qw]};
		}
		function direction(v:cadkit.modeling.Vector):Array<Float> return [v.x, v.y, v.z];
		for (index in 1...seams.length)
			if (seams[index].start.subtract(seams[index - 1].stop).length() > WeldSeams.TOUCH)
				throw 'Seam "${seams[index].name()}" does not start where "${seams[index - 1].name()}" ends: weld them as separate steps';
		var millimetre = 0.001;
		return {kind: "weld", weld: {frame: frame, metal: metal, path: [for (seam in seams) {kind: "line", seam: seam.name(), joint: switch seam.joint {
				case Fillet: "fillet";
				case Lap: "lap";
				case Butt: "butt";
				case Corner: "corner";
			}, start: pose(seam.frameAtParameter(0.0)), stop: pose(seam.frameAtParameter(1.0)),
			normals: [direction(seam.normalA), direction(seam.normalB)]}], legSize: legSize * millimetre,
			passes: [savedPass(singlePass(this))]}};
	}
}
