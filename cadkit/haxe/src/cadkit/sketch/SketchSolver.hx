package cadkit.sketch;

import cadkit.solve.ConstraintDiagnosis;

/**
	Solves a constrained sketch and diagnoses it, entirely in Haxeon. The work
	is split by concern:
	- `SketchLayout`: variables, constraints, validation, scale, starting pose;
	- `SketchEquations`: each constraint kind's residual and analytic Jacobian;
	- `SketchPartition`: parts (constraints sharing variables) and their orderings;
	- `SketchPartSolver`: Levenberg-Marquardt over one part with a sparse Cholesky;
	- `SketchPartDiagnosis`: `ConstraintDiagnosis` per part and the witness pose;
	- `SketchSolveCache`: how parts are recognised across solves.
	This class only orchestrates: each part is reused from the seed when nothing
	it depends on changed, otherwise solved and diagnosed; reports merge.
*/
class SketchSolver {
	final layout:SketchLayout;
	final equations:SketchEquations;
	final seed:Null<SolvedSketch>;
	final diagnoseParts:Bool;

	function new(sketch:ConstrainedSketch, seed:Null<SolvedSketch>, cancellationCheck:Null<Void->Bool>, diagnose:Bool) {
		layout = new SketchLayout(sketch, cancellationCheck);
		equations = new SketchEquations(layout);
		this.seed = seed;
		diagnoseParts = diagnose;
	}

	/**
		With `diagnose` false (while dragging), parts that re-solve report their
		previous diagnosis (`SolveDiagnostic.diagnosed` is then false).
	*/
	public static function solve(sketch:ConstrainedSketch, seed:Null<SolvedSketch> = null,
		cancellationCheck:Null<Void->Bool> = null, diagnose:Bool = true):SolvedSketch {
		return new SketchSolver(sketch, seed, cancellationCheck, diagnose).run();
	}

	/**
		Test hook for `cadkit.solve.JacobianCheck`: the authored variables (point
		x/y pairs, then radii) and the residuals and analytic Jacobian as
		functions of them, in sketch units, with each row's constraint and the
		residual a solve accepts as satisfied. Tangent sides and branches are
		fixed from the authored pose, as a solve would fix them.
	*/
	public static function probe(sketch:ConstrainedSketch):{variables:Array<Float>, residuals:Array<Float>->Array<Float>,
			jacobian:Array<Float>->Array<Float>, owners:Array<String>, satisfiedWithin:Float} {
		var solver = new SketchSolver(sketch, null, null, true);
		var layout = solver.layout, equations = solver.equations;
		layout.validate();
		var x = layout.authoredPose();
		equations.residuals(x, layout.allConstraints);
		layout.normalizationScale = layout.characteristicScale(x);
		equations.fixTangentBranches(x);
		var writer = new SketchRowWriter();
		return {
			variables: x,
			residuals: values -> equations.residuals(values, layout.allConstraints).values,
			jacobian: values -> {
				var rows = equations.rows(values, layout.allConstraints, equations.residuals(values, layout.allConstraints).values.length, writer);
				// Entries are with respect to x / s; dividing by s gives them with respect to x.
				var dense = [for (_ in 0...rows.length * layout.variableCount) 0.0];
				for (row in 0...rows.length)
					for (e in 0...rows[row].index.length)
						dense[row * layout.variableCount + rows[row].index[e]] += rows[row].value[e] / layout.normalizationScale;
				return dense;
			},
			owners: equations.residuals(x, layout.allConstraints).owners,
			satisfiedWithin: sketch.settings.tolerance * 10
		};
	}

	function run():SolvedSketch {
		layout.checkCancelled();
		layout.validate();
		// Evaluating once checks every constraint's references and supported combination.
		equations.residuals(layout.authoredPose(), layout.allConstraints);
		var x = layout.seededPose(seed);
		layout.normalizationScale = layout.characteristicScale(x);
		equations.fixTangentBranches(x);
		var partition = new SketchPartition(layout);
		var solver = new SketchPartSolver(layout, equations, partition);
		var diagnosis = new SketchPartDiagnosis(layout, equations, partition, solver);
		var tolerance = layout.solveTolerance;

		var iterations = 0, failed = false, stationary = true, degenerate = false, diagnosed = true;
		var conflictingIds:Array<String> = [], failingIds:Array<String> = [];
		var reports:Array<DiagnosisReport> = [];
		var cache = new Map<String, CachedPart>(), structures = new Map<String, PartStructure>();
		var previous = seed == null ? null : seed.partCache, previousStructures = seed == null ? null : seed.structures;
		for (part in partition.parts) {
			layout.checkCancelled();
			var key = SketchSolveCache.key(layout, part), values = SketchSolveCache.values(layout, part);
			// The same structure (a drag changes only values) keeps its orderings and its last diagnosis.
			var known = previousStructures == null ? null : previousStructures.get(key);
			if (known != null) {
				part.position = known.position;
				part.first = known.first;
				part.ordered = true;
			}
			// Unchanged constraints, fixed positions and settings, seeded from its own solution: already solved.
			var cached = previous == null ? null : previous.get(key);
			if (cached != null && SketchSolveCache.sameValues(cached.values, values)) {
				reports.push(cached.report);
				if (cached.degenerate) degenerate = true;
				cache.set(key, cached);
				if (known != null) structures.set(key, known);
				continue;
			}
			var solved = solver.solve(x, part, false, seed != null);
			x = solved.x;
			iterations = iterations > solved.iterations ? iterations : solved.iterations;
			if (!diagnoseParts && known != null && solved.norm <= tolerance) {
				// Dragging: report the last diagnosis; the solve on release checks this part again.
				reports.push(known.report);
				if (known.degenerate) degenerate = true;
				diagnosed = false;
				structures.set(key, {position: part.position, first: part.first, rows: known.rows, report: known.report,
					degenerate: known.degenerate});
				continue;
			}
			var rows = known != null && known.rows != null ? known.rows : diagnosis.structuralRows(part, solved.set.values.length);
			var report = diagnosis.diagnose(x, part, solved.set, rows), partDegenerate = false;
			if (solved.norm > tolerance) {
				failed = true;
				var limit = Math.max(layout.sketch.settings.rankTolerance, tolerance * 10) * (1 + solved.norm);
				if (layout.norm(solver.gradient(x, part, solved.set)) > limit)
					stationary = false;
				for (id in report.conflictingOwners()) if (conflictingIds.indexOf(id) < 0) conflictingIds.push(id);
				for (id in failingOwners(solved.set, tolerance * 10)) if (failingIds.indexOf(id) < 0) failingIds.push(id);
			} else if (report.rank < solved.set.values.length) {
				// A dependency here may belong to this pose only: compare with the rank at a nearby pose of the same shape.
				var witness = diagnosis.witnessPose(x, part);
				if (witness != null) {
					var generic = diagnosis.diagnose(witness, part, solved.set);
					if (generic.rank > report.rank) {
						report = generic;
						partDegenerate = true;
					}
				}
			}
			if (partDegenerate) degenerate = true;
			if (solved.norm <= tolerance)
				cache.set(key, {values: values, report: report, degenerate: partDegenerate});
			structures.set(key, {position: part.position, first: part.first, rows: rows, report: report, degenerate: partDegenerate});
			reports.push(report);
		}

		var report = ConstraintDiagnosis.merge(reports, layout.variableCount);
		var currentNorm = layout.norm(equations.residuals(x, layout.allConstraints).values);
		var dof = report.degreesOfFreedom;
		if (failed) {
			var status = stationary ? "conflicting" : "nonconvergent";
			var badIds = stationary && conflictingIds.length > 0 ? conflictingIds : failingIds;
			badIds.sort(Reflect.compare);
			var message = stationary
				? "solve stopped at a locally stationary residual; the listed constraints are locally incompatible, which is not proof of global inconsistency"
				: "constraint solve exhausted its iteration limit while a local descent direction remained";
			throw new SketchSolveError(new SolveDiagnostic(status, false, currentNorm, dof, iterations, badIds, message, report));
		}
		var redundant = report.redundantOwners();
		var status = redundant.length > 0 ? "redundant" : (dof == 0 ? "fully-constrained" : "under-constrained");
		var message = status == "redundant" ? "solution converged with locally redundant constraints" : "solution converged";
		if (degenerate)
			message += "; the constraints are independent in general but the solved pose is degenerate";
		var diagnostic = new SolveDiagnostic(status, true, currentNorm, dof, iterations, redundant, message, report, degenerate, diagnosed);
		layout.checkCancelled();
		var coordinates:Map<String, Array<Float>> = new Map();
		for (point in layout.sketch.points()) { var i:Int = cast layout.pointIndex.get(point.id); coordinates.set(point.id, [x[i], x[i + 1]]); }
		var radii:Map<String, Float> = new Map();
		for (entity in layout.sketch.entities())
			if (layout.radiusIndex.exists(entity.id)) { var i:Int = cast layout.radiusIndex.get(entity.id); radii.set(entity.id, x[i]); }
		return new SolvedSketch(coordinates, radii, diagnostic, cache, structures);
	}

	static function failingOwners(set:SketchResidualSet, tolerance:Float):Array<String> {
		var out:Array<String> = [];
		for (i in 0...set.values.length)
			if (Math.abs(set.values[i]) > tolerance && out.indexOf(set.owners[i]) < 0) out.push(set.owners[i]);
		return out;
	}
}
