package cadkit.modeling;

import cadkit.solve.ConstraintDiagnosis;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSolution;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import kinematicskit.LevenbergMarquardt;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.units.LengthUnit;

/** Tolerances and iteration settings shared by the assembly solvers; omitted fields take the defaults. */
typedef AssemblySolveOptions = {
	/** Length unit of the assembly; default 1 µm. */
	@:optional var positionTolerance:Float;
	/** Radians; default 1e-6. */
	@:optional var angularTolerance:Float;
	@:optional var maxIterations:Int;
	@:optional var initialDamping:Float;
	/** Levenberg-Marquardt's rank tolerance for its steps (not the diagnosis's). */
	@:optional var rankTolerance:Float;
}

/** `AssemblySolveOptions` with every field resolved and checked. */
class AssemblySolveSettings {
	public final positionTolerance:Float;
	public final angularTolerance:Float;
	public final maxIterations:Int;
	public final initialDamping:Float;
	public final rankTolerance:Float;

	public function new(positionTolerance:Float, angularTolerance:Float, maxIterations:Int, initialDamping:Float, rankTolerance:Float) {
		if (!(positionTolerance > 0) || !Math.isFinite(positionTolerance) || !(angularTolerance > 0) || !Math.isFinite(angularTolerance) ||
			maxIterations <= 0 || !(initialDamping > 0) || !Math.isFinite(initialDamping) || !(rankTolerance > 0) ||
			!Math.isFinite(rankTolerance))
			throw "Assembly solver settings must be finite and positive";
		this.positionTolerance = positionTolerance;
		this.angularTolerance = angularTolerance;
		this.maxIterations = maxIterations;
		this.initialDamping = initialDamping;
		this.rankTolerance = rankTolerance;
	}
}

/** A solve's outcome: the solution, its status name, and the diagnosis (at a generic nearby pose when `degenerate`). */
class AssemblySolveOutcome {
	public final solution:KinematicSolution;
	/** "converged", "limit-blocked", "conflicting" or "nonconvergent". */
	public final status:String;
	public final report:DiagnosisReport;
	/**
		The rows' dependency belongs to the solved pose only (a linkage at a toggle, mates at a singular
		placement): at a nearby solved pose the rows are independent, and `report` is that pose's diagnosis.
	*/
	public final degenerate:Bool;

	public function new(solution:KinematicSolution, status:String, report:DiagnosisReport, degenerate:Bool) {
		this.solution = solution;
		this.status = status;
		this.report = report;
		this.degenerate = degenerate;
	}
}

/**
	The solve both assembly solvers share (`AssemblyLoopSolver` closes loops
	through dependent joints; `AssemblyMateSolver` places parts by mates):
	one tolerance policy, one assembly scale, Levenberg-Marquardt, the
	closure diagnosis, and the witness check for degenerate poses. The solvers
	differ only in what may move and in how a witness pose is reached.
*/
class AssemblySolve {
	static inline var DEFAULT_POSITION_TOLERANCE_METRES:Float = 1e-6;
	static inline var DEFAULT_ANGULAR_TOLERANCE:Float = 1e-6;
	static inline var DEFAULT_MAX_ITERATIONS:Int = 200;
	static inline var DEFAULT_DAMPING:Float = 1e-3;
	static inline var DEFAULT_RANK_TOLERANCE:Float = 1e-8;
	/** How far a witness pose is moved: radians, or this share of the assembly scale. */
	public static inline var WITNESS_STEP:Float = 1e-3;

	public static function settings(lengthUnit:Null<String>, options:Null<AssemblySolveOptions>):AssemblySolveSettings {
		var metresPerUnit = LengthUnit.metresPerUnit(lengthUnit == null ? "mm" : lengthUnit);
		var positionTolerance = DEFAULT_POSITION_TOLERANCE_METRES / metresPerUnit, angularTolerance = DEFAULT_ANGULAR_TOLERANCE;
		var maxIterations = DEFAULT_MAX_ITERATIONS, initialDamping = DEFAULT_DAMPING, rankTolerance = DEFAULT_RANK_TOLERANCE;
		if (options != null) {
			var position = options.positionTolerance, angular = options.angularTolerance, iterations = options.maxIterations;
			var damping = options.initialDamping, rank = options.rankTolerance;
			if (position != null) positionTolerance = position;
			if (angular != null) angularTolerance = angular;
			if (iterations != null) maxIterations = iterations;
			if (damping != null) initialDamping = damping;
			if (rank != null) rankTolerance = rank;
		}
		return new AssemblySolveSettings(positionTolerance, angularTolerance, maxIterations, initialDamping, rankTolerance);
	}

	/**
		The assembly's characteristic length, in which translations are solved: the diagonal of the box
		around its occurrences' initial origins, widened by the farthest connector from its part's origin.
		At least one unit.
	*/
	public static function scale(definition:AssemblyDefinition):Float {
		var components = new Map<String, AssemblyComponentDefinition>();
		for (component in definition.definitions) components.set(component.id, component);
		var minX = 0.0, minY = 0.0, minZ = 0.0, maxX = 0.0, maxY = 0.0, maxZ = 0.0, reach = 0.0, first = true;
		for (occurrence in definition.occurrences) {
			var pose = occurrence.initialPose;
			if (first) {
				minX = maxX = pose.x; minY = maxY = pose.y; minZ = maxZ = pose.z;
				first = false;
			}
			minX = Math.min(minX, pose.x); minY = Math.min(minY, pose.y); minZ = Math.min(minZ, pose.z);
			maxX = Math.max(maxX, pose.x); maxY = Math.max(maxY, pose.y); maxZ = Math.max(maxZ, pose.z);
			var component = components.get(occurrence.definition);
			if (component != null) for (connector in component.connectors) {
				var frame = connector.frame;
				reach = Math.max(reach, Math.sqrt(frame.x * frame.x + frame.y * frame.y + frame.z * frame.z));
			}
		}
		var diagonal = Math.sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY) + (maxZ - minZ) * (maxZ - minZ));
		return Math.max(1, diagonal + 2 * reach);
	}

	/**
		Solves `problem` from `seed`, diagnoses the result, and, when the rows are dependent at the solution,
		re-solves from `witness(solution, sign)` (a nearby seed, or null when there is none) for each sign:
		if the rows are independent there, the dependency was the pose's, and that pose's report is returned.
	*/
	public static function run(problem:KinematicProblem, seed:KinematicState, settings:AssemblySolveSettings, scale:Float,
			levenberg:Bool, witness:(KinematicState, Float)->Null<KinematicState>):AssemblySolveOutcome {
		var solution = LevenbergMarquardt.solve(problem, seed, settings.maxIterations, settings.initialDamping, settings.rankTolerance,
			scale, null, levenberg);
		var report = AssemblyClosureDiagnosis.diagnose(problem, solution.state, scale), degenerate = false;
		if (solution.converged() && report.rank < report.variables && report.dependencyGroups.length > 0) {
			for (sign in [1.0, -1.0]) {
				var nearby = witness(solution.state, sign);
				if (nearby == null) break;
				var generic = LevenbergMarquardt.solve(problem, nearby, settings.maxIterations, settings.initialDamping,
					settings.rankTolerance, scale, null, levenberg);
				if (!generic.converged()) continue;
				var genericReport = AssemblyClosureDiagnosis.diagnose(problem, generic.state, scale);
				if (genericReport.rank > report.rank) {
					report = genericReport;
					degenerate = true;
				}
				break;
			}
		}
		return new AssemblySolveOutcome(solution, status(solution), report, degenerate);
	}

	static function status(solution:KinematicSolution):String {
		if (solution.converged()) return "converged";
		return switch solution.status {
			case KinematicStatus.LimitBlocked: "limit-blocked";
			case KinematicStatus.Conflicting: "conflicting";
			default: "nonconvergent";
		};
	}
}
