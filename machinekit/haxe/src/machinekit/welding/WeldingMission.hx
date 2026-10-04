package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.welding.WeldSeam;
import machinekit.welding.WeldSeams;
import machinekit.welding.WeldSeams.SeamChain;
import machinekit.welding.Weldment.WeldmentSeams;
import machinekit.welding.WeldingRecipe.RecipeWire;
import machinekit.welding.WeldingRecipe;
import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** Where the torch is and which way its wire points: a place a weld starts or ends at, in the workpiece's frame (millimetres). */
typedef TorchPlace = {
	var position:Vector;
	/** Unit vector, the way the wire points. */
	var wire:Vector;
}

/**
 * Whether a seam can be welded where the weldment stands: reachable by the robot with the torch at the seam's frames, and the
 * torch and arm clear of the workpiece, fixtures, table and equipment (but for the wire tip at the seam). It belongs to
 * whoever knows the robot and its cell, so the generator asks it per seam and does not know how it decides.
 */
interface WeldAccess {
	/** Why `seam` (welded in the direction it is given) cannot be welded, or null when it can. */
	function problem(seam:WeldSeam):Null<String>;
}

/** One weld of the mission: seams that run on from one another, in the order and direction they are welded. */
class WeldRun {
	public final seams:Array<WeldSeam>;
	/** The last seam ends where the first begins (a tube's perimeter). */
	public final closed:Bool;

	public function new(seams:Array<WeldSeam>, closed:Bool) {
		this.seams = seams;
		this.closed = closed;
	}

	public function names():Array<String> return [for (seam in seams) seam.name()];

	/** Where the torch starts the run, and where it ends it. */
	public function entry():TorchPlace return place(seams[0], 0.0);

	public function exit():TorchPlace return place(seams[seams.length - 1], 1.0);

	public static function place(seam:WeldSeam, fraction:Float):TorchPlace {
		var frame = seam.frameAtParameter(fraction);
		return {position: frame.position, wire: frame.wire()};
	}
}

/**
 * The mission that welds a whole weldment, generated from the seams its declared joints have (`Weldment.find`), so that adding
 * a member adds its seams. Nothing in it is authored: the seams come from the members' geometry, the process from the leg and
 * the wire (`WeldingRecipe`), and what a seam needs of the robot is asked of a `WeldAccess`.
 *
 * - **One step per run.** `WeldSeams.chains` joins the sides that meet end to end (a tube's four sides), and each chain is one
 *   `weld` step, welded with the arc up from corner to corner. A chain whose seams ask for different legs is split where the
 *   leg changes, since a step has one process.
 * - **Order.** The runs are ordered to keep the torch from travelling and turning through the air: a greedy nearest-neighbour
 *   tour. From where the torch is (`home`, the ready pose) the next run is the one, and the way of welding it, whose start is
 *   nearest to where the torch ended, measured as the distance between the two places plus `TURN_COST` millimetres per radian
 *   of the angle between the two wire directions (reorienting the torch is slower than moving it). The way of welding is each
 *   run's choice of direction (the run, or its seams, reversed) and, for a closed run, of the corner it starts at. A greedy
 *   tour is not the shortest, but the runs of a weldment are few and the cost is only the air time between them; the tour
 *   and its totals are reported (`airTravel`, `turned`) so that a better order can be told from a worse.
 * - **Every seam must be weldable.** A joint with no seam, a seam of a joint type that is not deposited yet (only fillets
 *   are), and a seam the robot cannot reach or cannot weld clear of the work are errors in `diagnostics`, each naming the seam
 *   and why, and the mission is then not generated (`steps` is empty; `require` throws). The weldment is the cell's own CAD,
 *   which we can change, so a joint that cannot be welded is a fault in the design to be fixed, not something to skip silently
 *   and weld around. A seam is judged in both directions it can be welded in: it is unweldable only if neither works.
 */
class WeldingMission {
	/** Millimetres of air travel one radian of torch reorientation costs. */
	public static inline var TURN_COST:Float = 60.0;

	public final steps:Array<SceneArtifactMissionStep>;
	public final runs:Array<WeldRun>;
	public final diagnostics:Diagnostics;
	/** The length of the moves between runs, from `home`, in millimetres, and the angle the wire turns over them, in radians. */
	public final airTravel:Float;
	public final turned:Float;

	function new(steps:Array<SceneArtifactMissionStep>, runs:Array<WeldRun>, diagnostics:Diagnostics, airTravel:Float, turned:Float) {
		this.steps = steps;
		this.runs = runs;
		this.diagnostics = diagnostics;
		this.airTravel = airTravel;
		this.turned = turned;
	}

	/** The steps, or an exception listing every seam that could not be welded and why. */
	public function require():Array<SceneArtifactMissionStep> {
		diagnostics.throwIfErrors();
		return steps;
	}

	/**
	 * The mission for the seams `found` in `weldment`, welded with the recipes for their legs in `wire`, the weld metal carried
	 * by the occurrence `metal`, lengths in the scene's unit (`metresPerUnit` metres to one). `home` is where the torch starts;
	 * `access`, when given, judges every seam.
	 */
	public static function generate(weldment:Weldment, found:WeldmentSeams, wire:RecipeWire, metal:String, metresPerUnit:Float, ?home:TorchPlace,
			?access:WeldAccess):WeldingMission {
		var diagnostics = new Diagnostics();
		for (item in found.diagnostics.items) diagnostics.add(item.severity, item.code, item.subject, item.message);
		var seams = found.seams;
		var deposited:Array<WeldSeam> = [];
		for (seam in seams) {
			if (isFillet(seam)) deposited.push(seam);
			else diagnostics.error("weld.joint-unsupported", seam.name(), 'Seam ${seam.name()} is a ${seam.joint} joint; only fillet joints are welded so far');
		}
		var verdicts = new Map<String, Null<String>>();
		function problem(seam:WeldSeam):Null<String> {
			if (access == null) return null;
			var key = seam.name() + "@" + seam.start.x + "," + seam.start.y + "," + seam.start.z;
			if (!verdicts.exists(key)) verdicts.set(key, access.problem(seam));
			return verdicts.get(key);
		}
		// A seam is weldable when it is in at least one of the two directions.
		for (seam in deposited) {
			var forward = problem(seam), backward = forward == null ? null : problem(seam.reversed());
			if (forward != null && backward != null) diagnostics.error("weld.unweldable", seam.name(), 'Seam ${seam.name()} cannot be welded: $forward' +
				(backward == forward ? "" : '; the other way: $backward'));
		}
		if (diagnostics.hasErrors()) return new WeldingMission([], [], diagnostics, 0.0, 0.0);
		// Runs, each as the ways it can be welded.
		var chains = [for (chain in WeldSeams.chains(deposited)) for (part in split(chain)) part];
		var left:Array<Array<Array<WeldSeam>>> = [];
		for (chain in chains) {
			var ways = [for (way in ways(chain)) if (usable(way, problem)) way];
			if (ways.length == 0)
				diagnostics.error("weld.unweldable", chain.seams[0].name(), 'The seams ${[for (seam in chain.seams) seam.name()].join(", ")} meet end to end but cannot be welded as one run in either direction from any corner');
			else left.push(ways);
		}
		if (diagnostics.hasErrors()) return new WeldingMission([], [], diagnostics, 0.0, 0.0);
		// The tour.
		var runs:Array<WeldRun> = [];
		var at = home;
		var closedLeft = [for (chain in chains) chain.closed];
		while (left.length > 0) {
			var best = -1, bestWay = -1, bestCost = Math.POSITIVE_INFINITY;
			for (index in 0...left.length) for (way in 0...left[index].length) {
				var entry = WeldRun.place(left[index][way][0], 0.0);
				var cost = at == null ? 0.0 : travel(at, entry);
				if (cost < bestCost - 1e-9) {
					bestCost = cost;
					best = index;
					bestWay = way;
				}
			}
			var chosen = left[best][bestWay];
			var run = new WeldRun(chosen, closedLeft[best]);
			runs.push(run);
			at = run.exit();
			left.splice(best, 1);
			closedLeft.splice(best, 1);
		}
		var steps:Array<SceneArtifactMissionStep> = [];
		for (run in runs) {
			var leg = run.seams[0].legSize;
			steps.push(WeldingRecipe.passStep(leg, wire, run.seams, weldment.reference, metal, metresPerUnit));
		}
		var totals = home == null ? {air: 0.0, turned: 0.0} : measure(runs, home);
		return new WeldingMission(steps, runs, diagnostics, totals.air, totals.turned);
	}

	static function isFillet(seam:WeldSeam):Bool
		return switch seam.joint {
			case Fillet: true;
			case _: false;
		};

	/** The air travel between runs from `home`, in millimetres, and the angle the wire turns over those moves, in radians. */
	public static function measure(runs:Array<WeldRun>, home:TorchPlace):{air:Float, turned:Float} {
		var at = home;
		var air = 0.0, turn = 0.0;
		for (run in runs) {
			var entry = run.entry();
			air += entry.position.subtract(at.position).length();
			turn += angle(at.wire, entry.wire);
			at = run.exit();
		}
		return {air: air, turned: turn};
	}

	/** The cost of moving the torch from where it is to where the next run starts. */
	static function travel(from:TorchPlace, to:TorchPlace):Float
		return to.position.subtract(from.position).length() + TURN_COST * angle(from.wire, to.wire);

	static function angle(a:Vector, b:Vector):Float return Math.atan2(a.cross(b).length(), a.dot(b));

	/** Whether every seam of `way` can be welded in the direction the way has it. */
	static function usable(way:Array<WeldSeam>, problem:WeldSeam -> Null<String>):Bool {
		for (seam in way) if (problem(seam) != null) return false;
		return true;
	}

	/** The chain cut into runs of one leg size, a closed chain that is cut becoming open. */
	static function split(chain:SeamChain):Array<SeamChain> {
		var parts:Array<SeamChain> = [];
		var current:Array<WeldSeam> = [];
		for (seam in chain.seams) {
			if (current.length > 0 && Math.abs(current[0].legSize - seam.legSize) > 1e-9) {
				parts.push({seams: current, closed: false});
				current = [];
			}
			current.push(seam);
		}
		parts.push({seams: current, closed: chain.closed && parts.length == 0});
		return parts;
	}

	/**
	 * The ways of welding a chain: in either direction, and, for a closed chain, from any of its corners. Every way is the same
	 * seams as the chain, each running on from the one before.
	 */
	static function ways(chain:SeamChain):Array<Array<WeldSeam>> {
		var result:Array<Array<WeldSeam>> = [];
		var count = chain.seams.length;
		for (corner in 0...(chain.closed ? count : 1)) {
			var rotated = chain.seams.slice(corner).concat(chain.seams.slice(0, corner));
			result.push(rotated);
			var back = [for (index in 0...count) rotated[count - 1 - index].reversed()];
			result.push(back);
		}
		return result;
	}
}
