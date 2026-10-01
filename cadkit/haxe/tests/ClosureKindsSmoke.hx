import cadkit.modeling.AssemblyKinematics;
import cadkit.modeling.AssemblyState;
import cadkit.solve.JacobianCheck;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;

/** Spherical, cylindrical and planar closures close real linkages, and their rows match central differences. */
class ClosureKindsSmoke {
	public static function run():Void {
		// A four-bar pinned by a ball joint: the ball's out-of-plane row is redundant by design.
		var ball = AssemblyLoopSmoke.fourBarDefinition(2200, null, null, 1.4);
		ball.joints[0].driven = true;
		ball.joints[3].type = AssemblyJointType.Spherical;
		solveAndCheck("spherical four-bar", ball, "redundant(pin)-1");

		// A slider on a rod, held on the ground's x line by a cylindrical closure.
		var slider = AssemblyLoopSmoke.definition("cylindrical-slider", [AssemblyLoopSmoke.bar("ground", 1500),
			AssemblyLoopSmoke.bar("crank", 500), AssemblyLoopSmoke.bar("rod", 1500), AssemblyLoopSmoke.bar("slider", 0)], [
			AssemblyLoopSmoke.joint("crank", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "a", "crank", "a", 0.9),
			AssemblyLoopSmoke.joint("rod", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "crank", "b", "rod", "a", -0.5),
			AssemblyLoopSmoke.joint("swivel", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "rod", "b", "slider", "a", -0.4),
			AssemblyLoopSmoke.joint("guide", AssemblyJointType.Cylindrical, AssemblyJointRole.Closure, "slider", "a", "ground", "a", 0.0,
				null, null, true)]);
		slider.joints[0].driven = true;
		solveAndCheck("cylindrical slider", slider, "redundant(guide)-2");
		// The same guide as a prismatic closure also holds the twist about the axis: one more row, zero in a plane.
		for (joint in slider.joints) if (joint.id == "guide") joint.type = AssemblyJointType.Prismatic;
		solveAndCheck("prismatic slider", slider, "redundant(guide)-3");

		// A three-link leg whose foot stands flat on a raised floor: a planar closure.
		var ground:AssemblyComponentDefinition = {id: "ground",
			connectors: [{name: "a", frame: AssemblyLoopSmoke.frame(0, 0, 0)}, {name: "floor", frame: AssemblyLoopSmoke.frame(0, 300, 0)}]};
		var leg = AssemblyLoopSmoke.definition("planar-leg", [ground, AssemblyLoopSmoke.bar("thigh", 1000),
			AssemblyLoopSmoke.bar("shin", 800), AssemblyLoopSmoke.bar("foot", 0)], [
			AssemblyLoopSmoke.joint("hip", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "ground", "a", "thigh", "a", 0.6),
			AssemblyLoopSmoke.joint("knee", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "thigh", "b", "shin", "a", -1.2),
			AssemblyLoopSmoke.joint("ankle", AssemblyJointType.Revolute, AssemblyJointRole.Tree, "shin", "b", "foot", "a", 0.5),
			AssemblyLoopSmoke.joint("stand", AssemblyJointType.Planar, AssemblyJointRole.Closure, "ground", "floor", "foot", "a", 0.0)]);
		for (joint in leg.joints) if (joint.id == "stand") joint.axis = {x: 0, y: 1, z: 0};
		leg.joints[0].driven = true;
		solveAndCheck("planar foot", leg, "redundant(stand)-1");

		var badTree = AssemblyLoopSmoke.fourBarDefinition(2200, null, null, 1.4);
		badTree.joints[1].type = AssemblyJointType.Spherical;
		var threw = false;
		try AssemblyDefinitionCodec.validate(badTree) catch (_:Dynamic) threw = true;
		check(threw, "a spherical joint cannot be a tree joint");
	}

	/** Closes the loop from the driven joints, checks the diagnosis, then the closure rows at the solution. */
	static function solveAndCheck(label:String, definition:AssemblyDefinition, expectedGroups:String):Void {
		var state = new AssemblyState(definition);
		var result = state.solveClosures();
		var groups = result.report == null ? "none" : [for (group in result.report.dependencyGroups) group.toString()].join(" ");
		check(result.converged && groups == expectedGroups, '$label closes (${result.status}, $groups)');
		state.checkClosures();

		var model = AssemblyKinematics.compile(AssemblyDefinitionFlattener.flatten(definition)).model;
		var dofs = [for (id in state.dependentJoints()) model.dofIndex(id)];
		var problem = new KinematicProblem(model).setActiveDofs(dofs);
		for (closure in 0...model.closureCount()) problem.add(new ClosureTask(model, closure, 1e-3, 1e-5));
		var base = new KinematicState(model);
		for (dof in 0...model.dofCount()) base.q[dof] = state.joint(model.dofId(dof));
		var snapshot = new KinematicSnapshot(model);
		var rows = problem.rowCount(), width = problem.layout().width;
		var evaluate = (x:Array<Float>) -> {
			var trial = base.copy();
			for (i in 0...dofs.length) trial.q[dofs[i]] = x[i];
			var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
			problem.evaluate(trial, snapshot, residual, jacobian);
			return {residual: residual, jacobian: jacobian};
		};
		// Tasks return residual −value with Jacobian ∂value/∂q, so the value is the negated residual.
		var check = JacobianCheck.compare(x -> [for (value in evaluate(x).residual) -value], x -> evaluate(x).jacobian,
			[for (dof in dofs) base.q[dof]], 1e-5);
		if (!check.passed) throw '$label closure rows: $check';
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
