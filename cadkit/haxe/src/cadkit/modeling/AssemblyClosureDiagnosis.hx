package cadkit.modeling;

import cadkit.solve.ConstraintDiagnosis;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;

/**
	The rows of a closure (or mate) problem at a state, diagnosed over the
	problem's columns. Rows are divided by their tolerances, so |r| <= 1 is
	satisfied; owners are closure or mate IDs. Shared by `AssemblyLoopSolver`
	and `AssemblyMateSolver`.
*/
class AssemblyClosureDiagnosis {
	public static function diagnose(problem:KinematicProblem, state:KinematicState):DiagnosisReport {
		var model = problem.model, width = problem.layout().width, rows = problem.rowCount();
		var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
		problem.evaluate(state, new KinematicSnapshot(model), residual, jacobian);
		var owners:Array<String> = [];
		for (task in problem.tasks) {
			var id = Std.isOfType(task, ClosureTask) ? model.closureIds[(cast task : ClosureTask).closure] : task.label();
			for (_ in 0...task.rowCount()) owners.push(id);
		}
		var sparse = [for (row in 0...rows) {
			var index:Array<Int> = [], value:Array<Float> = [];
			for (column in 0...width) {
				var entry = jacobian[row * width + column];
				if (entry != 0) { index.push(column); value.push(entry); }
			}
			{index: index, value: value};
		}];
		return ConstraintDiagnosis.diagnoseSparse(sparse, width, owners, residual, ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE);
	}
}
