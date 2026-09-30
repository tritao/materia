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

	public function new(status:String, converged:Bool, residual:Float, positionResidual:Float,
		angularResidual:Float, degreesOfFreedom:Int, iterations:Int, closureIds:Array<String>, message:String,
		?report:DiagnosisReport) {
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
	/**
		An unclosable four-bar stops at its least-squares pose with ‖Jᵀr‖/‖J‖‖r‖ ≈ 1e-6 after the iteration limit;
		unfinished solves sit near 0.1–1.
	*/
	static inline var STATIONARY_RATIO:Float = 1e-4;

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
		var closures = diagnoseClosures(problem, solution.state);
		if (solution.converged()) {
			for (dof in dofs) state.setJoint(model.dofId(dof), solution.state.q[dof]);
			return new AssemblyLoopSolveResult("converged", true, solution.residualNorm, positionResidual,
				angularResidual, solution.freeDofs, solution.iterations, [], "assembly closures converged", closures.report);
		}
		var status = switch solution.status {
			case KinematicStatus.LimitBlocked: "limit-blocked";
			case KinematicStatus.Conflicting: "conflicting";
			// An unclosable loop has a large residual, so Levenberg-Marquardt approaches its least-squares pose only
			// linearly and may run out of iterations first; a residual nearly orthogonal to every direction the
			// dependent joints can move is that pose.
			default: closures.stationary ? "conflicting" : "nonconvergent";
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

	/**
		The closure rows at `state` (divided by their tolerances, so |r| <= 1 is satisfied) diagnosed over the
		problem's columns, and whether the residual is stationary there: ‖Jᵀr‖ <= `STATIONARY_RATIO` ‖J‖‖r‖,
		which does not depend on units or tolerances.
	*/
	static function diagnoseClosures(problem:KinematicProblem, state:KinematicState):{report:DiagnosisReport, stationary:Bool} {
		var model = problem.model, width = problem.layout().width, rows = problem.rowCount();
		var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
		problem.evaluate(state, new KinematicSnapshot(model), residual, jacobian);
		var owners:Array<String> = [];
		for (task in problem.tasks) {
			var id = Std.isOfType(task, ClosureTask) ? model.closureIds[(cast task : ClosureTask).closure] : "task";
			for (_ in 0...task.rowCount()) owners.push(id);
		}
		var gradient = 0.0, jacobianNorm = 0.0, residualNorm = 0.0;
		for (column in 0...width) {
			var sum = 0.0;
			for (row in 0...rows) sum += jacobian[row * width + column] * residual[row];
			gradient += sum * sum;
		}
		for (value in jacobian) jacobianNorm += value * value;
		for (value in residual) residualNorm += value * value;
		var stationary = Math.sqrt(gradient) <= STATIONARY_RATIO * Math.sqrt(jacobianNorm) * Math.sqrt(residualNorm);
		var sparse = [for (row in 0...rows) {
			var index:Array<Int> = [], value:Array<Float> = [];
			for (column in 0...width) {
				var entry = jacobian[row * width + column];
				if (entry != 0) { index.push(column); value.push(entry); }
			}
			{index: index, value: value};
		}];
		return {report: ConstraintDiagnosis.diagnoseSparse(sparse, width, owners, residual, ConstraintDiagnosis.SPARSE_TOLERANCE),
			stationary: stationary};
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
