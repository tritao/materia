package cadkit.modeling;

import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import cadkit.solve.ConstraintDiagnosis;
import cadkit.modeling.AssemblySolve.AssemblySolveOptions;

/** Tolerances and iteration settings for joint-coordinate loop solving (see `AssemblySolveOptions`). */
typedef AssemblyLoopSolveOptions = AssemblySolveOptions;

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
	/**
		Adjusts only the named tree-joint coordinates. The supplied state is
		updated when all closure residuals meet tolerance; on failure it is left
		at its original configuration.
	*/
	public static function solve(state:AssemblyState, dependentJointIds:Array<String>,
		?options:AssemblyLoopSolveOptions):AssemblyLoopSolveResult {
		if (state == null) throw "Assembly loop solve needs a state";
		if (dependentJointIds == null || dependentJointIds.length == 0)
			throw "Assembly loop solve needs at least one dependent tree joint";
		var settings = AssemblySolve.settings(state.definition.lengthUnit, options);
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
			problem.add(new ClosureTask(model, closure, settings.positionTolerance, settings.angularTolerance));
		var scale = AssemblySolve.scale(state.definition);
		// A witness pose of the same mechanism: nudge the inputs (the driven joints), close the loops again.
		var driven = [for (joint in state.definition.joints) if (joint.driven == true && model.dofIndex(joint.id) >= 0) model.dofIndex(joint.id)];
		var outcome = AssemblySolve.run(problem, state.kinematicSeed(), settings, scale, false, (solved, sign) -> {
			if (driven.length == 0) return null;
			var seed = solved.copy();
			for (dof in driven) seed.q[dof] += sign * AssemblySolve.WITNESS_STEP * (model.dofIsAngular(dof) ? 1 : scale);
			return seed;
		});
		var solution = outcome.solution;

		var positionResidual = 0.0, angularResidual = 0.0;
		for (task in solution.tasks) {
			positionResidual = Math.max(positionResidual, task.positionError);
			angularResidual = Math.max(angularResidual, task.orientationError);
		}
		if (outcome.status == "converged") {
			for (dof in dofs) state.setJoint(model.dofId(dof), solution.state.q[dof]);
			return new AssemblyLoopSolveResult("converged", true, solution.residualNorm, positionResidual,
				angularResidual, solution.freeDofs, solution.iterations, [],
				outcome.degenerate ? "assembly closures converged at a degenerate pose" : "assembly closures converged",
				outcome.report, outcome.degenerate);
		}
		var message = switch outcome.status {
			case "limit-blocked": "joint limits blocked further motion before the closure tolerances were met";
			case "conflicting":
				"closure residuals stopped at a locally stationary configuration; this does not prove global inconsistency";
			default: "assembly loop solve exhausted its iteration limit while a local descent direction remained";
		};
		return new AssemblyLoopSolveResult(outcome.status, false, solution.residualNorm, positionResidual, angularResidual,
			solution.freeDofs, solution.iterations, solution.unsatisfied(), message, outcome.report);
	}
}
