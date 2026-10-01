package cadkit.modeling;

import cadkit.solve.ConstraintDiagnosis;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.RootMotion;

/**
	The rows of a closure (or mate) problem at a state, diagnosed over the
	problem's columns. Rows are divided by their tolerances, so |r| <= 1 is
	satisfied; owners are closure or mate IDs. Shared by `AssemblyLoopSolver`
	and `AssemblyMateSolver` (through `AssemblySolve`).

	Entries that cannot matter are dropped before the diagnosis equilibrates
	the rows (which would otherwise blow roundoff up to a unit row): those for
	which moving the variable across its characteristic range (a radian, or
	`scale` in the length unit) changes the row by under `NEGLIGIBLE` of its
	tolerance. A row whose gradient vanishes at a singular pose is then the
	empty, dependent row it is.
*/
class AssemblyClosureDiagnosis {
	static inline var NEGLIGIBLE:Float = 1e-6;

	public static function diagnose(problem:KinematicProblem, state:KinematicState, scale:Float):DiagnosisReport {
		var system = rows(problem, state, scale);
		return ConstraintDiagnosis.diagnoseSparse(system.rows, system.width, system.owners, system.residual,
			ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE);
	}

	/**
		The owners among `candidates` that add nothing: without their rows the rank is the same, so every row of
		theirs is implied by the others (a mate repeated, not one that merely overlaps another, as a coaxial mate
		and a planar mate both holding one axis do).
	*/
	public static function impliedOwners(problem:KinematicProblem, state:KinematicState, scale:Float,
			candidates:Array<String>):Array<String> {
		var system = rows(problem, state, scale);
		var full = ConstraintDiagnosis.diagnoseSparse(system.rows, system.width, system.owners, system.residual,
			ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE).rank;
		var implied:Array<String> = [];
		for (candidate in candidates) {
			var kept = [for (row in 0...system.rows.length) if (system.owners[row] != candidate) row];
			if (kept.length == system.rows.length) continue;
			var rank = ConstraintDiagnosis.diagnoseSparse([for (row in kept) system.rows[row]], system.width,
				[for (row in kept) system.owners[row]], [for (row in kept) system.residual[row]],
				ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE).rank;
			if (rank == full) implied.push(candidate);
		}
		return implied;
	}

	/**
		How free each of the problem's columns is (see `ConstraintDiagnosis.freedom`), with the columns scaled to
		their characteristic range as the diagnosis's rows are: a column above `ConstraintDiagnosis.FREE_TOLERANCE`
		is a motion the rows still allow.
	*/
	public static function columnFreedom(problem:KinematicProblem, state:KinematicState, scale:Float):Array<Float> {
		var system = rows(problem, state, scale);
		return ConstraintDiagnosis.freedom(system.rows, system.width);
	}

	static function rows(problem:KinematicProblem, state:KinematicState, scale:Float):{rows:Array<{index:Array<Int>, value:Array<Float>}>,
			width:Int, owners:Array<String>, residual:Array<Float>} {
		var model = problem.model, layout = problem.layout(), width = layout.width, rows = problem.rowCount();
		var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
		problem.evaluate(state, new KinematicSnapshot(model), residual, jacobian);
		var owners:Array<String> = [];
		for (task in problem.tasks) {
			var id = Std.isOfType(task, ClosureTask) ? model.closureIds[(cast task : ClosureTask).closure] : task.label();
			for (_ in 0...task.rowCount()) owners.push(id);
		}
		var range = [for (column in 0...width) 1.0];
		for (column in 0...layout.dofs.length) if (!model.dofIsAngular(layout.dofs[column])) range[column] = scale;
		for (block in 0...layout.rootBodies.length) {
			var first = layout.rootColumns[block];
			// Planar roots: vx, vy, ωz; floating roots: v, then ω.
			var linear = layout.rootModes[block] == RootMotion.Planar ? 2 : 3;
			for (k in 0...linear) range[first + k] = scale;
		}
		var sparse = [for (row in 0...rows) {
			var index:Array<Int> = [], value:Array<Float> = [];
			for (column in 0...width) {
				var entry = jacobian[row * width + column];
				if (Math.abs(entry) * range[column] > NEGLIGIBLE) { index.push(column); value.push(entry); }
			}
			{index: index, value: value};
		}];
		return {rows: sparse, width: width, owners: owners, residual: residual};
	}
}
