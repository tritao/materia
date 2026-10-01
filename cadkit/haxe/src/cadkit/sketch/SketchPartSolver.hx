package cadkit.sketch;

import cadkit.solve.EnvelopeCholesky;

/**
	Levenberg-Marquardt over one part, in lengths divided by the sketch's
	scale: JᵀJ + λI is accumulated in the part's envelope and factored by
	Cholesky there. Owns the solve's reusable buffers.
*/
class SketchPartSolver {
	/** A part seeded from a previous solution starts at this damping (Gauss-Newton). */
	static inline var WARM_DAMPING:Float = 1e-9;

	final layout:SketchLayout;
	final equations:SketchEquations;
	final partition:SketchPartition;
	final writer = new SketchRowWriter();
	final scratchIndex:Array<Int> = [];
	final scratchValue:Array<Float> = [];

	public function new(layout:SketchLayout, equations:SketchEquations, partition:SketchPartition) {
		this.layout = layout;
		this.equations = equations;
		this.partition = partition;
	}

	/** The rows of `part` at `x` (only its shape constraints when `shapeOnly`), in reused storage. */
	public function rows(x:Array<Float>, part:SketchPart, rowCount:Int, shapeOnly:Bool = false):Array<SketchSparseRow>
		return equations.rows(x, part.constraints, rowCount, writer, shapeOnly);

	/**
		Levenberg-Marquardt from `start` (the part's shape constraints only when
		`shapeOnly`), then up to three Gauss-Newton polish steps while each
		halves the residual: converging only to the tolerance leaves a visible
		length error. With `warm` (seeded from a previous solution) it starts as
		Gauss-Newton: the authored damping would swamp the soft modes of a long
		chain; a rejected step still raises the damping.
	*/
	public function solve(start:Array<Float>, part:SketchPart, shapeOnly:Bool,
			warm:Bool):{x:Array<Float>, set:SketchResidualSet, norm:Float, iterations:Int} {
		partition.order(part);
		var settings = layout.sketch.settings, tolerance = layout.solveTolerance;
		var x = start, damping = warm && !shapeOnly ? WARM_DAMPING : settings.initialDamping;
		var current = equations.residuals(x, part.constraints, shapeOnly), currentNorm = layout.norm(current.values), iterations = 0;
		while (iterations < settings.maxIterations && currentNorm > tolerance) {
			layout.checkCancelled();
			iterations++;
			var trial = dampedStep(x, current, damping, part, shapeOnly);
			if (trial == null) { damping *= 10; continue; }
			var trialSet = equations.residuals(trial, part.constraints, shapeOnly), trialNorm = layout.norm(trialSet.values);
			if (trialNorm < currentNorm) { x = trial; current = trialSet; currentNorm = trialNorm; damping = Math.max(1e-12, damping * 0.3); }
			else damping = Math.min(1e12, damping * 10);
		}
		var polish = 0;
		while (currentNorm <= tolerance && currentNorm > tolerance * 1e-3 && polish < 3) {
			layout.checkCancelled();
			polish++;
			var trial = dampedStep(x, current, 1e-12, part, shapeOnly);
			if (trial == null) break;
			var trialSet = equations.residuals(trial, part.constraints, shapeOnly), trialNorm = layout.norm(trialSet.values);
			if (!(trialNorm < currentNorm * 0.5)) break;
			x = trial; current = trialSet; currentNorm = trialNorm;
		}
		return {x: x, set: current, norm: currentNorm, iterations: iterations};
	}

	/** Jᵀr over a part: zero at a stationary point of its residual. */
	public function gradient(x:Array<Float>, part:SketchPart, set:SketchResidualSet):Array<Float> {
		var jacobian = rows(x, part, set.values.length);
		var result = [for (_ in 0...part.variables.length) 0.0];
		for (row in 0...jacobian.length) {
			var entries = partition.localEntries(jacobian[row], part);
			for (e in 0...entries.index.length) result[entries.index[e]] += entries.value[e] * set.values[row];
		}
		return result;
	}

	/** One damped step on a part; null when the damped matrix is not positive definite. */
	function dampedStep(x:Array<Float>, current:SketchResidualSet, damping:Float, part:SketchPart, shapeOnly:Bool):Null<Array<Float>> {
		var k = part.variables.length, jacobian = rows(x, part, current.values.length, shapeOnly);
		// envelope[row][column - first[row]] for first[row] <= column <= row, in RCM order; kept per part, zeroed here.
		if (part.envelope.length != k) part.envelope = EnvelopeCholesky.zero(part.first);
		var envelope = part.envelope;
		for (line in envelope) for (i in 0...line.length) line[i] = 0;
		var gradient = [for (_ in 0...k) 0.0];
		var positions = scratchIndex, values = scratchValue;
		for (row in 0...jacobian.length) {
			if (row % 16 == 0)
				layout.checkCancelled();
			// The row's entries by envelope position, repeated variables merged, in reused scratch arrays.
			var source = jacobian[row];
			positions.resize(0);
			values.resize(0);
			for (e in 0...source.index.length) {
				var variable = source.index[e];
				if (partition.partOf(variable) != part.id) continue;
				var position = part.position[partition.localIndex(variable)], at = positions.indexOf(position);
				if (at < 0) { positions.push(position); values.push(source.value[e]); } else values[at] += source.value[e];
			}
			for (a in 0...positions.length) {
				var pa = positions[a], va = values[a];
				gradient[pa] += va * current.values[row];
				var line = envelope[pa], lineFirst = part.first[pa];
				for (b in 0...positions.length) {
					var pb = positions[b];
					if (pb <= pa) line[pb - lineFirst] += va * values[b];
				}
			}
		}
		for (row in 0...k)
			envelope[row][row - part.first[row]] += damping;
		if (!EnvelopeCholesky.factor(envelope, part.first)) return null;
		var y = EnvelopeCholesky.solve(envelope, part.first, [for (value in gradient) -value]);
		var trial = x.copy();
		for (i in 0...k)
			trial[part.variables[i]] += y[part.position[i]] * layout.normalizationScale;
		return trial;
	}
}
