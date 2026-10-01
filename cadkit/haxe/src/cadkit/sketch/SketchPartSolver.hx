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

	/**
		Pulls the part towards drag `targets` (sketch-wide variables and the values the user drags them to) while its
		constraints hold, for a soft drag (plan C5.3). Each iteration takes a tangent step and projects it back:
		- the step is Gauss-Newton on the constraint rows plus a light target row `weight · (x − target)` per dragged
		  variable, so it keeps the linearized constraints and moves along them towards the targets;
		- along it, the placement is solved back onto the constraints at fractions 1 and ½, and at the minimum of the
		  parabola through those and the start, and the nearest to the targets is kept.
		The line search matters: on a curved constraint with the target out of reach (a fixed length dragged beyond
		its circle) the tangent step overshoots by the ratio of the distances, which damping alone corrects only
		linearly. A penalty without projection would crawl, as the curvature outweighs a light pull.
		Returns a placement on the constraints, as near the targets as they allow.
	*/
	public function pull(start:Array<Float>, part:SketchPart, targets:Array<{variable:Int, value:Float}>, weight:Float):Array<Float> {
		partition.order(part);
		var tolerance = layout.solveTolerance;
		var distance = (x:Array<Float>) -> {
			var sum = 0.0;
			for (target in targets) sum += (x[target.variable] - target.value) * (x[target.variable] - target.value);
			return sum;
		};
		var settled = solve(start, part, false, true);
		var x = settled.x, current = settled.set, currentDistance = distance(x);
		for (_ in 0...layout.sketch.settings.maxIterations) {
			layout.checkCancelled();
			if (currentDistance <= tolerance * tolerance) break;
			var full = dampedStep(x, current, weight * weight * 1e-6, part, false, targets, weight);
			if (full == null) break;
			var base = x;
			var along = (fraction:Float) -> {
				var trial = [for (i in 0...base.length) base[i] + fraction * (full[i] - base[i])];
				var projected = solve(trial, part, false, true);
				return projected.norm <= tolerance ? {x: projected.x, set: projected.set, distance: distance(projected.x)} : null;
			};
			var best:Null<{x:Array<Float>, set:SketchResidualSet, distance:Float}> = null;
			var consider = (candidate:Null<{x:Array<Float>, set:SketchResidualSet, distance:Float}>) -> {
				if (candidate != null && candidate.distance < currentDistance && (best == null || candidate.distance < best.distance))
					best = candidate;
			};
			var whole = along(1), half = along(0.5);
			consider(whole);
			consider(half);
			if (whole != null && half != null) {
				// The parabola through (0, d0), (½, d½), (1, d1) has its minimum at -b / 2a.
				var curvature = 2 * (whole.distance - 2 * half.distance + currentDistance);
				var slope = whole.distance - currentDistance - curvature;
				if (curvature > 0) {
					var fraction = -slope / (2 * curvature);
					if (fraction > 0 && fraction < 2 && Math.abs(fraction - 0.5) > 1e-3 && Math.abs(fraction - 1) > 1e-3)
						consider(along(fraction));
				}
			}
			var fraction = 0.25;
			while (best == null && fraction > 1e-3) {
				consider(along(fraction));
				fraction *= 0.5;
			}
			var chosen = best;
			if (chosen == null) break;
			var gained = currentDistance - chosen.distance;
			x = chosen.x;
			current = chosen.set;
			currentDistance = chosen.distance;
			if (gained <= 1e-12 * Math.max(currentDistance, tolerance * tolerance)) break;
		}
		return x;
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
	function dampedStep(x:Array<Float>, current:SketchResidualSet, damping:Float, part:SketchPart, shapeOnly:Bool,
			?targets:Array<{variable:Int, value:Float}>, weight:Float = 0):Null<Array<Float>> {
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
		// Drag target rows touch one variable each: they add to the diagonal and the gradient only.
		if (targets != null)
			for (target in targets) {
				if (partition.partOf(target.variable) != part.id) continue;
				var position = part.position[partition.localIndex(target.variable)];
				var residual = weight * (x[target.variable] - target.value) / layout.normalizationScale;
				gradient[position] += weight * residual;
				envelope[position][position - part.first[position]] += weight * weight;
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
