package machinekit.welding;

import machinekit.welding.WeldSeam.SeamFrame;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import materia.project.SceneArtifact.SceneArtifactTorchPose;

/**
 * How a seam is welded, derived from the seam: the settings of a MIG fillet in solid steel wire that lay a leg of the
 * size the weldment asks for. Nothing in it is a seam's geometry; that comes from the CAD (`WeldSeam`). The recipe is
 * the generator's inverse of what the simulated welder does: it picks a wire speed for the leg, the voltage that goes
 * with it, and the travel speed at which that wire speed deposits the cross-section of an equal-leg fillet,
 * `area = leg² / 2`. The leg the weld then reaches is a result, which the player compares with the leg asked for.
 *
 * - The wire speed grows with the leg (`1.5 · leg + 0.5` metres per minute, clamped): a bigger bead needs more
 *   metal and more current, and the speed that travel can keep up with.
 * - The voltage follows the wire speed on a synergic line, `15 + 1.1 · wire speed` volts, rounded to half a volt.
 * - The travel speed is `wire speed · wire area · efficiency / area`, with the efficiency of the deposit,
 *   `DEPOSITION_EFFICIENCY`, the same figure RobotKit's `WeldBead` deposits with.
 */
class WeldingRecipe {
	/** Fraction of the melted wire that reaches the bead (RobotKit's `WeldBead.DEPOSITION_EFFICIENCY`). */
	public static inline var DEPOSITION_EFFICIENCY:Float = 0.95;
	public static inline var DEFAULT_WIRE_DIAMETER:Float = 1.2;

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
				!(burnback >= 0) || !(approach > 0))
			throw "A welding recipe needs a positive wire speed, voltage, travel speed, leg and approach, and dwells of zero or more";
		this.wireSpeed = wireSpeed;
		this.voltage = voltage;
		this.travelSpeed = travelSpeed;
		this.legSize = legSize;
		this.startDwell = startDwell;
		this.craterDwell = craterDwell;
		this.burnback = burnback;
		this.approach = approach;
	}

	/** The recipe for a fillet of `leg` millimetres in wire of `wireDiameter` millimetres. */
	public static function fillet(leg:Float, wireDiameter:Float = DEFAULT_WIRE_DIAMETER):WeldingRecipe {
		if (!(leg >= 2 && leg <= 12)) throw 'A single-pass fillet recipe covers legs of 2 to 12 mm, not $leg';
		var wire = Math.min(16.0, Math.max(3.0, 1.5 * leg + 0.5));
		var voltage = Math.round((15.0 + 1.1 * wire) * 2) / 2;
		var wireArea = Math.PI * wireDiameter * wireDiameter / 4.0;
		var feed = wire * 1000.0 / 60.0;
		var area = leg * leg / 2.0;
		return new WeldingRecipe(wire, voltage, feed * wireArea * DEPOSITION_EFFICIENCY / area, leg, 0.15, 0.15, 0.1, 40);
	}

	/**
	 * The mission step that welds `seam`, which is in the assembly frame in millimetres: the seam as the CAD found it
	 * with this recipe's process, its weld metal carried by the occurrence `metal`. Lengths in the step are metres,
	 * whatever the scene's unit (`metresPerUnit`).
	 */
	public function step(seam:WeldSeam, metal:String, metresPerUnit:Float):SceneArtifactMissionStep {
		function pose(frame:SeamFrame):SceneArtifactTorchPose {
			var tip = frame.toAssemblyFrame();
			return {position: [tip.x * metresPerUnit, tip.y * metresPerUnit, tip.z * metresPerUnit], rotation: [tip.qx, tip.qy, tip.qz, tip.qw]};
		}
		function direction(v:cadkit.modeling.Vector):Array<Float> return [v.x, v.y, v.z];
		var millimetre = 0.001;
		return {kind: "weld", weld: {seam: seam.name(), joint: switch seam.joint {
				case Fillet: "fillet";
				case Lap: "lap";
				case Butt: "butt";
				case Corner: "corner";
			}, metal: metal, start: pose(seam.frameAtParameter(0.0)), stop: pose(seam.frameAtParameter(1.0)),
			normals: [direction(seam.normalA), direction(seam.normalB)], legSize: legSize * millimetre,
			process: {wireSpeed: wireSpeed, voltage: voltage, travelSpeed: travelSpeed * millimetre, approach: approach * millimetre,
				startDwell: startDwell, craterDwell: craterDwell, burnback: burnback}}};
	}
}
