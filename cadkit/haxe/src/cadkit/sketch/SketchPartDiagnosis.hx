package cadkit.sketch;

import cadkit.solve.ConstraintDiagnosis;
import cadkit.solve.ConstraintDiagnosis.RowStructure;

/** A part's diagnosis (`ConstraintDiagnosis`) and its witness pose (plan decision CS-D6). */
class SketchPartDiagnosis {
	final layout:SketchLayout;
	final equations:SketchEquations;
	final partition:SketchPartition;
	final solver:SketchPartSolver;

	public function new(layout:SketchLayout, equations:SketchEquations, partition:SketchPartition, solver:SketchPartSolver) {
		this.layout = layout;
		this.equations = equations;
		this.partition = partition;
		this.solver = solver;
	}

	/**
		Diagnoses a part with its Jacobian taken at `at` (the solution, or a
		witness pose) and feasibility judged from `set`, the residuals at the
		solution; rows count as satisfied within ten times the solve tolerance.
	*/
	public function diagnose(at:Array<Float>, part:SketchPart, set:SketchResidualSet, ?rows:RowStructure):DiagnosisReport {
		var local = [for (row in solver.rows(at, part, set.values.length)) partition.localEntries(row, part)];
		var satisfiedWithin = layout.solveTolerance * 10;
		return ConstraintDiagnosis.diagnoseSparse(local, part.variables.length, set.owners,
			[for (value in set.values) value / satisfiedWithin], layout.sketch.settings.rankTolerance, null, rows);
	}

	/**
		The part's variables (sketch-wide indices) its constraints leave free at `at` (see
		`ConstraintDiagnosis.freedom`): what still moves the part's geometry.
	*/
	public function freeVariables(at:Array<Float>, part:SketchPart, set:SketchResidualSet, ?rows:RowStructure):Array<Int> {
		var local = [for (row in solver.rows(at, part, set.values.length)) partition.localEntries(row, part)];
		var freedom = ConstraintDiagnosis.freedom(local, part.variables.length, layout.sketch.settings.rankTolerance, rows);
		return [for (index in 0...part.variables.length) if (freedom[index] > ConstraintDiagnosis.FREE_TOLERANCE) part.variables[index]];
	}

	/**
		The part's diagnosis row graph from what its constraints reference (a
		superset of any pose's nonzeros), ordered once and reusable while its
		structure stands. Null if the row count does not match `rows`.
	*/
	public function structuralRows(part:SketchPart, rows:Int):Null<RowStructure> {
		var rowVariables:Array<Array<Int>> = [];
		for (index in part.constraints)
			for (_ in 0...SketchEquations.rowCount(layout.constraintList[index].kind))
				rowVariables.push(partition.references[index]);
		if (rowVariables.length != rows) return null;
		return ConstraintDiagnosis.rowGraph([for (list in rowVariables) {index: list, value: [for (_ in list) 1.0]}]);
	}

	/**
		A pose near `x` with the same shape and no accidental coincidences: the
		part's lengths move by a fixed pseudo-random 2% of the sketch size, then
		only its shape constraints are solved again. Dimensions are left free:
		their rows' derivatives do not depend on their target values. Null when
		the shape cannot be restored.
	*/
	public function witnessPose(x:Array<Float>, part:SketchPart):Null<Array<Float>> {
		var seed = 20260930;
		var trial = x.copy();
		for (variable in part.variables) {
			seed = (seed * 1103515245 + 12345) & 0x7fffffff;
			trial[variable] += (seed / 0x7fffffff - 0.5) * 0.02 * layout.normalizationScale;
		}
		for (entity in layout.sketch.entities()) {
			var r = layout.radiusIndex.get(entity.id);
			if (r != null && partition.partOf(r) == part.id) trial[r] = Math.max(trial[r], 0.5 * x[r]);
		}
		var solved = solver.solve(trial, part, true, false);
		return solved.norm <= layout.solveTolerance ? solved.x : null;
	}
}
