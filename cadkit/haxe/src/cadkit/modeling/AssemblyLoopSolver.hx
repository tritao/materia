package cadkit.modeling;

import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicStatus;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import cadkit.solve.ConstraintDiagnosis;
import materia.units.LengthUnit;

/** Tolerances and iteration settings for joint-coordinate loop solving. */
typedef AssemblyLoopSolveOptions = {
	@:optional var positionTolerance:Float;
	@:optional var angularTolerance:Float;
	@:optional var maxIterations:Int;
	@:optional var initialDamping:Float;
	@:optional var rankTolerance:Float;
	@:optional var finiteDifferenceStep:Float;
}

/** Outcome of solving a set of assembly closure joints. */
class AssemblyLoopSolveResult {
	public final status:String;
	public final converged:Bool;
	/** Euclidean norm of all closure residuals after scaling by the requested tolerances. */
	public final residual:Float;
	public final positionResidual:Float;
	public final angularResidual:Float;
	public final degreesOfFreedom:Int;
	public final iterations:Int;
	/** Closure joint IDs that remain outside tolerance; empty on success. */
	public final closureIds:Array<String>;
	public final message:String;
	/**
		The closures' diagnosis over the dependent coordinates (see `ConstraintDiagnosis`). Owners are closure
		IDs; a planar linkage modelled with 3D closures shows its out-of-plane rows as satisfied (redundant by
		design) groups.
	*/
	public final report:Null<DiagnosisReport>;
	/**
		The closures' dependency belongs to the solved pose only (a linkage at a toggle): at a nearby pose reached
		by nudging the driven joints, the rows are independent. `report` is then that pose's diagnosis.
	*/
	public final degenerate:Bool;

	public function new(status:String, converged:Bool, residual:Float, positionResidual:Float,
		angularResidual:Float, degreesOfFreedom:Int, iterations:Int, closureIds:Array<String>, message:String,
		?report:DiagnosisReport, degenerate:Bool = false) {
		this.status = status;
		this.converged = converged;
		this.residual = residual;
		this.positionResidual = positionResidual;
		this.angularResidual = angularResidual;
		this.degreesOfFreedom = degreesOfFreedom;
		this.iterations = iterations;
		this.closureIds = closureIds.copy();
		this.message = message;
		this.report = report;
		this.degenerate = degenerate;
	}
}

/**
	Solves closed mechanisms by varying selected tree-joint coordinates. Other
	tree coordinates remain fixed and closure joints contribute geometric
	residuals; closure joints are never treated as additional FK edges. Runs
	`kinematicskit.LevenbergMarquardt` on closure tasks with analytic Jacobians.
*/
class AssemblyLoopSolver {
	static inline var DEFAULT_POSITION_TOLERANCE:Float = 1e-3;
	static inline var DEFAULT_ANGULAR_TOLERANCE:Float = 1e-5;
	static inline var DEFAULT_MAX_ITERATIONS:Int = 100;
	static inline var DEFAULT_DAMPING:Float = 1e-3;
	static inline var DEFAULT_RANK_TOLERANCE:Float = 1e-8;
	static inline var DEFAULT_FINITE_DIFFERENCE_STEP:Float = 1e-6;
	/** How far driven joints move for the witness pose: radians, or this share of the assembly size. */
	static inline var WITNESS_STEP:Float = 1e-3;

	/**
		Adjusts only the named tree-joint coordinates. The supplied state is
		updated when all closure residuals meet tolerance; on failure it is left
		at its original configuration. `finiteDifferenceStep` is validated but
		unused: Jacobians are analytic.
	*/
	public static function solve(state:AssemblyState, dependentJointIds:Array<String>,
		?options:AssemblyLoopSolveOptions):AssemblyLoopSolveResult {
		if (state == null) throw "Assembly loop solve needs a state";
		if (dependentJointIds == null || dependentJointIds.length == 0)
			throw "Assembly loop solve needs at least one dependent tree joint";
		var metresPerUnit = LengthUnit.metresPerUnit(state.definition.lengthUnit == null
			? "mm" : state.definition.lengthUnit);
		var positionTolerance = option(options, "positionTolerance",
			DEFAULT_POSITION_TOLERANCE * 0.001 / metresPerUnit);
		var angularTolerance = option(options, "angularTolerance", DEFAULT_ANGULAR_TOLERANCE);
		var maxIterations = intOption(options, "maxIterations", DEFAULT_MAX_ITERATIONS);
		var initialDamping = option(options, "initialDamping", DEFAULT_DAMPING);
		var rankTolerance = option(options, "rankTolerance", DEFAULT_RANK_TOLERANCE);
		var finiteDifferenceStep = option(options, "finiteDifferenceStep",
			DEFAULT_FINITE_DIFFERENCE_STEP * 0.001 / metresPerUnit);
		validateOptions(positionTolerance, angularTolerance, maxIterations, initialDamping,
			rankTolerance, finiteDifferenceStep);

		var kinematics = state.kinematicModel();
		var model = kinematics.model;
		if (model.closureCount() == 0) throw "Assembly loop solve needs at least one closure joint";

		var jointById = new Map<String, KinematicJoint>();
		for (joint in state.definition.joints) jointById.set(joint.id, joint);
		var dofs:Array<Int> = [];
		var seen = new Map<String, Bool>();
		for (id in dependentJointIds) {
			var joint = jointById.get(id);
			if (joint == null || joint.role != AssemblyJointRole.Tree ||
				(joint.type != AssemblyJointType.Revolute && joint.type != AssemblyJointType.Continuous &&
				joint.type != AssemblyJointType.Prismatic) || model.dofIndex(id) < 0)
				throw 'Dependent assembly coordinate "$id" is not a movable tree joint';
			if (seen.exists(id)) throw 'Dependent assembly coordinate "$id" is listed more than once';
			seen.set(id, true);
			dofs.push(model.dofIndex(id));
		}

		var problem = new KinematicProblem(model).setActiveDofs(dofs);
		for (closure in 0...model.closureCount())
			problem.add(new ClosureTask(model, closure, positionTolerance, angularTolerance));
		var solution = LevenbergMarquardt.solve(problem, state.kinematicSeed(), maxIterations, initialDamping,
			rankTolerance, assemblyScale(state));

		var positionResidual = 0.0, angularResidual = 0.0;
		for (task in solution.tasks) {
			positionResidual = Math.max(positionResidual, task.positionError);
			angularResidual = Math.max(angularResidual, task.orientationError);
		}
		var closures = {report: AssemblyClosureDiagnosis.diagnose(problem, solution.state)};
		if (solution.converged()) {
			var report = closures.report, degenerate = false;
			if (report.rank < report.variables && report.dependencyGroups.length > 0) {
				// Compare with a nearby pose of the same mechanism: nudge the inputs, close the loops again.
				var driven = [for (joint in state.definition.joints) if (joint.driven == true && model.dofIndex(joint.id) >= 0) model.dofIndex(joint.id)];
				for (sign in [1.0, -1.0]) {
					if (driven.length == 0) break;
					var seed = solution.state.copy();
					for (dof in driven) seed.q[dof] += sign * WITNESS_STEP * (model.dofIsAngular(dof) ? 1 : assemblyScale(state));
					var witness = LevenbergMarquardt.solve(problem, seed, maxIterations, initialDamping, rankTolerance, assemblyScale(state));
					if (!witness.converged()) continue;
					var generic = AssemblyClosureDiagnosis.diagnose(problem, witness.state);
					if (generic.rank > report.rank) {
						report = generic;
						degenerate = true;
					}
					break;
				}
			}
			for (dof in dofs) state.setJoint(model.dofId(dof), solution.state.q[dof]);
			return new AssemblyLoopSolveResult("converged", true, solution.residualNorm, positionResidual,
				angularResidual, solution.freeDofs, solution.iterations, [],
				degenerate ? "assembly closures converged at a degenerate pose" : "assembly closures converged", report, degenerate);
		}
		var status = switch solution.status {
			case KinematicStatus.LimitBlocked: "limit-blocked";
			case KinematicStatus.Conflicting: "conflicting";
			default: "nonconvergent";
		};
		var message = switch status {
			case "limit-blocked": "joint limits blocked further motion before the closure tolerances were met";
			case "conflicting":
				"closure residuals stopped at a locally stationary configuration; this does not prove global inconsistency";
			default: "assembly loop solve exhausted its iteration limit while a local descent direction remained";
		};
		return new AssemblyLoopSolveResult(status, false, solution.residualNorm, positionResidual, angularResidual,
			solution.freeDofs, solution.iterations, solution.unsatisfied(), message, closures.report);
	}

	static function assemblyScale(state:AssemblyState):Float {
		var minX = 1e300, minY = 1e300, minZ = 1e300;
		var maxX = -1e300, maxY = -1e300, maxZ = -1e300;
		for (occurrence in state.definition.occurrences) {
			var pose = state.worldPose(occurrence.id);
			minX = Math.min(minX, pose.x); minY = Math.min(minY, pose.y); minZ = Math.min(minZ, pose.z);
			maxX = Math.max(maxX, pose.x); maxY = Math.max(maxY, pose.y); maxZ = Math.max(maxZ, pose.z);
		}
		return Math.max(1, Math.sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY) +
			(maxZ - minZ) * (maxZ - minZ)));
	}

	static function option(options:Null<AssemblyLoopSolveOptions>, name:String, fallback:Float):Float {
		if (options == null) return fallback;
		var value:Dynamic = Reflect.field(options, name);
		return value == null ? fallback : cast value;
	}

	static function intOption(options:Null<AssemblyLoopSolveOptions>, name:String, fallback:Int):Int {
		if (options == null) return fallback;
		var value:Dynamic = Reflect.field(options, name);
		return value == null ? fallback : cast value;
	}

	static function validateOptions(positionTolerance:Float, angularTolerance:Float, maxIterations:Int,
		initialDamping:Float, rankTolerance:Float, finiteDifferenceStep:Float):Void {
		if (!Math.isFinite(positionTolerance) || positionTolerance <= 0 || !Math.isFinite(angularTolerance) ||
			angularTolerance <= 0 || maxIterations <= 0 || !Math.isFinite(initialDamping) || initialDamping <= 0 ||
			!Math.isFinite(rankTolerance) || rankTolerance <= 0 || !Math.isFinite(finiteDifferenceStep) ||
			finiteDifferenceStep <= 0)
			throw "Assembly loop solver settings must be finite and positive";
	}
}
