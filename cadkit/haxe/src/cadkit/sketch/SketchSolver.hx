package cadkit.sketch;

import cadkit.parametric.EvaluationCancelled;
import cadkit.solve.ConstraintDiagnosis;

private typedef ResidualSet = { values:Array<Float>, owners:Array<String> };

/**
	Deterministic damped nonlinear least-squares solver implemented entirely in Haxeon.
	It works in lengths divided by the sketch's characteristic size, so its
	damping and tolerances mean the same at any scale, and diagnoses the result
	with `ConstraintDiagnosis`.
*/
class SketchSolver {
	private final sketch:ConstrainedSketch;
	private final pointIndex:Map<String, Int>;
	private final radiusIndex:Map<String, Int>;
	private final points:Map<String, SketchPoint>;
	private final entities:Map<String, SketchEntity>;
	private final seed:Null<SolvedSketch>;
	private final cancellationCheck:Null<Void->Bool>;
	private final tangentBranches:Map<String, Bool>;
	private final tangentSides:Map<String, Float>;
	private var variableCount:Int;
	private var solveTolerance:Float;
	private var normalizationScale:Float;
	/** While finding a witness pose, only shape constraints (see `isShapeConstraint`) contribute residuals. */
	private var shapeOnly:Bool = false;

	private function new(sketch:ConstrainedSketch, seed:Null<SolvedSketch>, cancellationCheck:Null<Void->Bool>) {
		this.sketch = sketch; pointIndex = new Map(); radiusIndex = new Map(); points = new Map(); entities = new Map();
		this.seed = seed;
		this.cancellationCheck = cancellationCheck;
		tangentBranches = new Map();
		tangentSides = new Map();
		variableCount = 0;
		solveTolerance = sketch.settings.tolerance;
		normalizationScale = 1;
		for (point in sketch.points()) { points.set(point.id, point); pointIndex.set(point.id, variableCount); variableCount += 2; }
		for (entity in sketch.entities()) {
			entities.set(entity.id, entity);
			if (entity.kind == "circle" || entity.kind == "arc") { radiusIndex.set(entity.id, variableCount); variableCount++; }
		}
	}

	public static function solve(sketch:ConstrainedSketch, seed:Null<SolvedSketch> = null,
		cancellationCheck:Null<Void->Bool> = null):SolvedSketch {
		return new SketchSolver(sketch, seed, cancellationCheck).run();
	}

	private function checkCancelled():Void {
		if (cancellationCheck != null && cancellationCheck())
			throw new EvaluationCancelled();
	}

	private function invalid(message:String, ids:Array<String>):SketchSolveError
		return new SketchSolveError(new SolveDiagnostic("invalid", false, 1e300, variableCount, 0, ids, message));

	private function run():SolvedSketch {
		checkCancelled();
		validate();
		var x:Array<Float> = [];
		for (point in sketch.points()) {
			var value = seededPoint(point);
			x.push(value[0]);
			x.push(value[1]);
		}
		for (entity in sketch.entities())
			if (entity.kind == "circle" || entity.kind == "arc")
				x.push(seededRadius(entity));
		normalizationScale = characteristicScale(x);
		solveTolerance = sketch.settings.tolerance;
		initializeTangentBranches(x);
		var damping = sketch.settings.initialDamping;
		var current = residuals(x);
		var currentNorm = norm(current.values);
		var iterations = 0;
		while (iterations < sketch.settings.maxIterations && currentNorm > solveTolerance) {
			checkCancelled();
			iterations++;
			var trial = dampedStep(x, current, damping);
			if (trial == null) { damping *= 10; continue; }
			var trialSet = residuals(trial); var trialNorm = norm(trialSet.values);
			if (trialNorm < currentNorm) { x = trial; current = trialSet; currentNorm = trialNorm; damping = Math.max(1e-12, damping * 0.3); }
			else damping = Math.min(1e12, damping * 10);
		}
		// Converging only to the tolerance leaves a visible length error; Gauss-Newton steps are quadratic here,
		// so polish while each step at least halves the residual.
		var polish = 0;
		while (currentNorm <= solveTolerance && currentNorm > 0 && polish < 3) {
			checkCancelled();
			polish++;
			var trial = dampedStep(x, current, 1e-12);
			if (trial == null) break;
			var trialSet = residuals(trial), trialNorm = norm(trialSet.values);
			if (!(trialNorm < currentNorm * 0.5)) break;
			x = trial; current = trialSet; currentNorm = trialNorm;
		}
		checkCancelled();
		var j = jacobian(x, current.values);
		var report = diagnose(centralJacobian(x, current.values.length), current);
		var degenerate = false;
		if (currentNorm <= solveTolerance && report.rank < current.values.length) {
			// A dependency here may belong to this pose only: compare with the rank at a nearby pose of the same shape.
			var witness = witnessPose(x);
			if (witness != null) {
				var generic = diagnose(centralJacobian(witness, current.values.length), current);
				if (generic.rank > report.rank) {
					report = generic;
					degenerate = true;
				}
			}
		}
		var dof = report.degreesOfFreedom;
		var badIds = failingOwners(current, solveTolerance * 10);
		if (currentNorm > solveTolerance) {
			var gradientNorm = norm(gradient(j, current.values));
			var stationaryLimit = Math.max(sketch.settings.rankTolerance, solveTolerance * 10) * (1 + currentNorm);
			var locallyConflicting = gradientNorm <= stationaryLimit;
			var status = locallyConflicting ? "conflicting" : "nonconvergent";
			var conflicting = report.conflictingOwners();
			if (locallyConflicting && conflicting.length > 0)
				badIds = conflicting;
			var message = locallyConflicting
				? "solve stopped at a locally stationary residual; the listed constraints are locally incompatible, which is not proof of global inconsistency"
				: "constraint solve exhausted its iteration limit while a local descent direction remained";
			var diagnostic = new SolveDiagnostic(status, false, currentNorm, dof, iterations, badIds, message, report);
			throw new SketchSolveError(diagnostic);
		}
		var redundant = report.redundantOwners();
		var status = redundant.length > 0 ? "redundant" : (dof == 0 ? "fully-constrained" : "under-constrained");
		var ids = redundant.length > 0 ? redundant : [];
		var message = status == "redundant" ? "solution converged with locally redundant constraints" : "solution converged";
		if (degenerate)
			message += "; the constraints are independent in general but the solved pose is degenerate";
		var diagnostic = new SolveDiagnostic(status, true, currentNorm, dof, iterations, ids, message, report, degenerate);
		checkCancelled();
		var coordinates:Map<String, Array<Float>> = new Map();
		for (point in sketch.points()) { var i:Int = cast pointIndex.get(point.id); coordinates.set(point.id, [x[i], x[i + 1]]); }
		var radii:Map<String, Float> = new Map();
		for (entity in sketch.entities()) if (radiusIndex.exists(entity.id)) { var i:Int = cast radiusIndex.get(entity.id); radii.set(entity.id, x[i]); }
		return new SolvedSketch(coordinates, radii, diagnostic);
	}

	private function validate():Void {
		if (sketch.units == null || StringTools.trim(sketch.units) == "")
			throw invalid("sketch units must be nonempty", []);
		if (!Math.isFinite(sketch.settings.tolerance) || sketch.settings.tolerance <= 0
			|| !Math.isFinite(sketch.settings.rankTolerance) || sketch.settings.rankTolerance <= 0
			|| sketch.settings.maxIterations <= 0
			|| !Math.isFinite(sketch.settings.initialDamping) || sketch.settings.initialDamping <= 0)
			throw invalid("solver tolerances, iteration limit, and damping must be finite and positive", []);
		for (point in sketch.points())
			if (!Math.isFinite(point.x) || !Math.isFinite(point.y))
				throw invalid("point coordinates must be finite", [point.id]);
		for (entity in sketch.entities()) {
			if (!points.exists(entity.first) || (entity.kind == "line" && (entity.second == null || !points.exists(entity.second))))
				throw invalid("entity references a missing point", [entity.id]);
			if (entity.kind != "line" && entity.kind != "circle" && entity.kind != "arc") throw invalid("unsupported entity kind", [entity.id]);
			if ((entity.kind == "circle" || entity.kind == "arc") && (!Math.isFinite(entity.radius) || entity.radius <= 0))
				throw invalid("circle and arc radii must be positive", [entity.id]);
			if (entity.kind == "arc" && (!Math.isFinite(entity.startAngle) || !Math.isFinite(entity.endAngle)
				|| Math.abs(entity.endAngle - entity.startAngle) < 1e-12))
				throw invalid("arc angles must be finite and span a nonzero angle", [entity.id]);
			if (entity.kind == "line") {
				var a:SketchPoint = cast points.get(entity.first); var b:SketchPoint = cast points.get(entity.second);
				if (a.x == b.x && a.y == b.y) throw invalid("collapsed authored line", [entity.id]);
			}
		}
		var constraintIndex = 0;
		for (constraint in sketch.constraints()) {
			if (constraintIndex++ % 16 == 0)
				checkCancelled();
			if (!Math.isFinite(constraint.value)) throw invalid("constraint values must be finite", [constraint.id]);
			if ((constraint.kind == "distance" || constraint.kind == "radius") && constraint.value <= 0)
				throw invalid("distance and radius constraints must be positive", [constraint.id]);
		}
		// Evaluating once validates every constraint reference and supported combination.
		var initial:Array<Float> = [];
		for (p in sketch.points()) { initial.push(p.x); initial.push(p.y); }
		for (e in sketch.entities()) if (e.kind == "circle" || e.kind == "arc") initial.push(e.radius);
		residuals(initial);
	}

	private function residuals(x:Array<Float>):ResidualSet {
		var values:Array<Float> = []; var owners:Array<String> = [];
		var constraintIndex = 0;
		for (c in sketch.constraints()) {
			if (constraintIndex++ % 16 == 0)
				checkCancelled();
			if (shapeOnly && !isShapeConstraint(c.kind))
				continue;
			var before = values.length;
			switch (c.kind) {
				case "fixed": var p = point(c.first, x); var authored = needPoint(c.first, c.id); values.push(p[0] - authored.x); values.push(p[1] - authored.y);
				case "coincident": vectorDifference(point(c.first, x), point(needSecond(c), x), values);
				case "horizontal": var ends = line(c.first, x, c.id); values.push(ends[1][1] - ends[0][1]);
				case "vertical": var ends = line(c.first, x, c.id); values.push(ends[1][0] - ends[0][0]);
				case "distance": values.push(distance(point(c.first, x), point(needSecond(c), x)) - c.value);
				case "radius": values.push(radius(c.first, x, c.id) - c.value);
				case "equal": values.push(measure(c.first, x, c.id) - measure(needSecond(c), x, c.id));
				case "parallel": var a = direction(c.first, x, c.id); var b = direction(needSecond(c), x, c.id); values.push(cross(a, b) / lengths(a, b, c.id));
				case "perpendicular": var a = direction(c.first, x, c.id); var b = direction(needSecond(c), x, c.id); values.push(dot(a, b) / lengths(a, b, c.id));
				case "angle": var a = direction(c.first, x, c.id); var b = direction(needSecond(c), x, c.id); var scale = lengths(a, b, c.id); values.push(dot(a,b)/scale-Math.cos(c.value)); values.push(cross(a,b)/scale-Math.sin(c.value));
				case "concentric": vectorDifference(center(c.first, x, c.id), center(needSecond(c), x, c.id), values);
				case "pointOn": pointOn(c.first, needSecond(c), x, c.id, values);
				case "tangent": tangent(c.first, needSecond(c), x, c.id, values);
				case "symmetric": symmetry(c.first, needSecond(c), needThird(c), x, c.id, values);
				default: throw invalid("unsupported constraint kind: " + c.kind, [c.id]);
			}
			if (hasLinearResidual(c.kind))
				for (index in before...values.length)
					values[index] /= normalizationScale;
			for (_ in before...values.length) owners.push(c.id);
		}
		return {values: values, owners: owners};
	}

	private function point(id:String, x:Array<Float>):Array<Float> { var found = pointIndex.get(id); if (found == null) throw invalid("missing point: " + id, [id]); var i:Int = cast found; return [x[i], x[i+1]]; }
	private function needPoint(id:String, owner:String):SketchPoint { var p=points.get(id); if(p==null) throw invalid("missing point: "+id,[owner]); return p; }
	private function needSecond(c:SketchConstraint):String { if(c.second==null) throw invalid("constraint requires a second reference",[c.id]); return c.second; }
	private function needThird(c:SketchConstraint):String { if(c.third==null) throw invalid("constraint requires a third reference",[c.id]); return c.third; }
	private function entity(id:String, owner:String):SketchEntity { var e=entities.get(id); if(e==null) throw invalid("missing entity: "+id,[owner]); return e; }
	private function line(id:String,x:Array<Float>,owner:String):Array<Array<Float>> { var e=entity(id,owner); if(e.kind!="line") throw invalid("constraint requires a line",[owner]); return [point(e.first,x),point(e.second,x)]; }
	private function center(id:String,x:Array<Float>,owner:String):Array<Float> { var e=entity(id,owner); if(e.kind=="line") throw invalid("constraint requires a circle or arc",[owner]); return point(e.first,x); }
	private function radius(id:String,x:Array<Float>,owner:String):Float { var found=radiusIndex.get(id); if(found==null) throw invalid("constraint requires a circle or arc",[owner]); var i:Int = cast found; return x[i]; }
	private function direction(id:String,x:Array<Float>,owner:String):Array<Float> { var p=line(id,x,owner); return [p[1][0]-p[0][0],p[1][1]-p[0][1]]; }
	private function measure(id:String,x:Array<Float>,owner:String):Float { var e=entity(id,owner); return e.kind=="line" ? distance(point(e.first,x),point(e.second,x)) : radius(id,x,owner); }
	private function lengths(a:Array<Float>,b:Array<Float>,owner:String):Float { var v=Math.pow(dot(a,a)*dot(b,b),0.5); if(v<1e-12) throw invalid("collapsed line in constraint",[owner]); return v; }
	private function pointOn(pid:String,eid:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var p=point(pid,x); var e=entity(eid,owner);
		if(e.kind=="line") { var a=point(e.first,x); var d=direction(eid,x,owner); out.push(cross([p[0]-a[0],p[1]-a[1]],d)/Math.pow(dot(d,d),0.5)); }
		else out.push(distance(p,center(eid,x,owner))-radius(eid,x,owner));
	}
	private function tangent(aid:String,bid:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var first = entity(aid, owner);
		var second = entity(bid, owner);
		if (first.kind == "line" && second.kind == "line")
			throw invalid("line-line tangency is unsupported", [owner]);
		if (first.kind == "line" || second.kind == "line") {
			var lineId = first.kind == "line" ? aid : bid;
			var circleId = first.kind == "line" ? bid : aid;
			var ends = line(lineId, x, owner);
			var lineDirection = direction(lineId, x, owner);
			var circleCenter = center(circleId, x, owner);
			var signedDistance = cross([circleCenter[0] - ends[0][0], circleCenter[1] - ends[0][1]], lineDirection)
				/ Math.pow(dot(lineDirection, lineDirection), 0.5);
			var storedSide = tangentSides.get(owner);
			var side:Float = storedSide == null ? (signedDistance < 0 ? -1 : 1) : cast storedSide;
			out.push(signedDistance - side * radius(circleId, x, owner));
		} else {
			var separation = distance(center(aid, x, owner), center(bid, x, owner));
			var firstRadius = radius(aid, x, owner);
			var secondRadius = radius(bid, x, owner);
			var storedBranch = tangentBranches.get(owner);
			var external:Bool = storedBranch == null ? true : cast storedBranch;
			out.push(external ? separation - (firstRadius + secondRadius) : separation - Math.abs(firstRadius - secondRadius));
		}
	}

	private function initializeTangentBranches(x:Array<Float>):Void {
		for (constraint in sketch.constraints()) {
			if (constraint.kind != "tangent")
				continue;
			var second = needSecond(constraint);
			var firstEntity = entity(constraint.first, constraint.id);
			var secondEntity = entity(second, constraint.id);
			if (firstEntity.kind == "line" || secondEntity.kind == "line") {
				var lineId = firstEntity.kind == "line" ? constraint.first : second;
				var circleId = firstEntity.kind == "line" ? second : constraint.first;
				var ends = line(lineId, x, constraint.id);
				var lineDirection = direction(lineId, x, constraint.id);
				var circleCenter = center(circleId, x, constraint.id);
				var signedDistance = cross([circleCenter[0] - ends[0][0], circleCenter[1] - ends[0][1]], lineDirection)
					/ Math.pow(dot(lineDirection, lineDirection), 0.5);
				tangentSides.set(constraint.id, signedDistance < 0 ? -1 : 1);
			} else {
				var separation = distance(center(constraint.first, x, constraint.id), center(second, x, constraint.id));
				var firstRadius = radius(constraint.first, x, constraint.id);
				var secondRadius = radius(second, x, constraint.id);
				var externalResidual = Math.abs(separation - (firstRadius + secondRadius));
				var internalResidual = Math.abs(separation - Math.abs(firstRadius - secondRadius));
				tangentBranches.set(constraint.id, externalResidual <= internalResidual);
			}
		}
	}

	private function seededPoint(point:SketchPoint):Array<Float> {
		if (seed == null)
			return [point.x, point.y];
		try {
			return seed.point(point.id);
		} catch (error:Dynamic) {
			return [point.x, point.y];
		}
	}

	private function seededRadius(entity:SketchEntity):Float {
		if (seed == null)
			return entity.radius;
		try {
			return seed.radius(entity.id);
		} catch (error:Dynamic) {
			return entity.radius;
		}
	}

	private function characteristicScale(x:Array<Float>):Float {
		var minimumX = 1e300;
		var maximumX = -1e300;
		var minimumY = 1e300;
		var maximumY = -1e300;
		for (point in sketch.points()) {
			var index:Int = cast pointIndex.get(point.id);
			minimumX = Math.min(minimumX, x[index]);
			maximumX = Math.max(maximumX, x[index]);
			minimumY = Math.min(minimumY, x[index + 1]);
			maximumY = Math.max(maximumY, x[index + 1]);
		}
		var scale = sketch.points().length == 0 ? 1 : Math.max(maximumX - minimumX, maximumY - minimumY);
		for (entity in sketch.entities()) {
			if (entity.kind == "circle" || entity.kind == "arc") {
				var index:Int = cast radiusIndex.get(entity.id);
				scale = Math.max(scale, Math.abs(x[index]));
			}
		}
		for (constraint in sketch.constraints())
			if (constraint.kind == "distance" || constraint.kind == "radius")
				scale = Math.max(scale, Math.abs(constraint.value));
		return scale > 0 ? scale : 1;
	}

	private static function hasLinearResidual(kind:String):Bool {
		return kind != "parallel" && kind != "perpendicular" && kind != "angle";
	}
	private function symmetry(pa:String,pb:String,axis:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var a=point(pa,x), b=point(pb,x), ends=line(axis,x,owner), d=direction(axis,x,owner); var dd=dot(d,d); if(dd<1e-12) throw invalid("collapsed symmetry axis",[owner]);
		var mid=[(a[0]+b[0])/2,(a[1]+b[1])/2]; out.push(cross([mid[0]-ends[0][0],mid[1]-ends[0][1]],d)/Math.pow(dd,0.5)); out.push(dot([b[0]-a[0],b[1]-a[1]],d)/Math.pow(dd,0.5));
	}
	/** One Levenberg-Marquardt step from `x`, in scaled variables; null when the damped normal matrix is singular. */
	private function dampedStep(x:Array<Float>, current:ResidualSet, damping:Float):Null<Array<Float>> {
		var j = jacobian(x, current.values);
		var normal = matrix(variableCount, variableCount, 0);
		var gradient = fill(variableCount, 0);
		for (row in 0...j.length) {
			if (row % 16 == 0)
				checkCancelled();
			for (a in 0...variableCount) {
				if (a % 8 == 0)
					checkCancelled();
				gradient[a] += j[row][a] * current.values[row];
				for (b in 0...variableCount) normal[a][b] += j[row][a] * j[row][b];
			}
		}
		for (i in 0...variableCount) {
			if (i % 128 == 0)
				checkCancelled();
			normal[i][i] += damping;
		}
		var rhs:Array<Float> = [];
		for (index in 0...gradient.length) {
			if (index % 1024 == 0)
				checkCancelled();
			rhs.push(-gradient[index]);
		}
		var delta = linearSolve(normal, rhs);
		if (delta == null) return null;
		var trial = x.copy();
		for (i in 0...variableCount) {
			if (i % 1024 == 0)
				checkCancelled();
			trial[i] += delta[i] * normalizationScale;
		}
		return trial;
	}

	/** Forward differences with respect to the scaled variables (x / normalizationScale). */
	private function jacobian(x:Array<Float>, base:Array<Float>):Array<Array<Float>> {
		var j = matrix(base.length, variableCount, 0);
		for (column in 0...variableCount) {
			if (column % 8 == 0)
				checkCancelled();
			var step = 1e-6 * normalizationScale;
			var trial = x.copy();
			trial[column] += step;
			var values = residuals(trial).values;
			for (row in 0...base.length) {
				if (row % 128 == 0)
					checkCancelled();
				j[row][column] = (values[row] - base[row]) / 1e-6;
			}
		}
		return j;
	}
	private function gradient(j:Array<Array<Float>>, residual:Array<Float>):Array<Float> {
		var result = fill(variableCount, 0);
		for (row in 0...j.length) {
			if (row % 16 == 0)
				checkCancelled();
			for (column in 0...variableCount) {
				if (column % 1024 == 0)
					checkCancelled();
				result[column] += j[row][column] * residual[row];
			}
		}
		return result;
	}
	/** Central differences in the scaled variables, row-major: accurate enough for relative rank decisions. */
	private function centralJacobian(x:Array<Float>, rows:Int):Array<Float> {
		var flat = [for (_ in 0...rows * variableCount) 0.0];
		for (column in 0...variableCount) {
			if (column % 8 == 0)
				checkCancelled();
			var step = 1e-6 * normalizationScale;
			var plus = x.copy(), minus = x.copy();
			plus[column] += step;
			minus[column] -= step;
			var high = residuals(plus).values, low = residuals(minus).values;
			for (row in 0...rows)
				flat[row * variableCount + column] = (high[row] - low[row]) / 2e-6;
		}
		return flat;
	}

	/** Diagnoses a Jacobian against the solved residuals; rows count as satisfied within ten times the solve tolerance. */
	private function diagnose(jacobian:Array<Float>, set:ResidualSet):DiagnosisReport {
		var satisfiedWithin = solveTolerance * 10;
		return ConstraintDiagnosis.diagnose({jacobian: jacobian, variables: variableCount, owners: set.owners,
			residuals: [for (value in set.values) value / satisfiedWithin], rankTolerance: sketch.settings.rankTolerance});
	}

	/**
		A pose near `x` with the same shape and no accidental coincidences
		(plan decision CS-D6): every length moves by a fixed pseudo-random
		percent of the sketch size, then only the shape constraints are solved
		again. Dimensions are left free: their rows' derivatives do not depend
		on their target values. Null when the shape cannot be restored.
	*/
	private function witnessPose(x:Array<Float>):Null<Array<Float>> {
		var seed = 20260930;
		var trial = [for (value in x) {
			seed = (seed * 1103515245 + 12345) & 0x7fffffff;
			value + (seed / 0x7fffffff - 0.5) * 0.02 * normalizationScale;
		}];
		for (entity in sketch.entities())
			if (radiusIndex.exists(entity.id)) {
				var i:Int = cast radiusIndex.get(entity.id);
				trial[i] = Math.max(trial[i], 0.5 * x[i]);
			}
		shapeOnly = true;
		var current = residuals(trial), currentNorm = norm(current.values), damping = sketch.settings.initialDamping;
		var iterations = 0;
		while (currentNorm > solveTolerance && iterations < sketch.settings.maxIterations) {
			iterations++;
			var step = dampedStep(trial, current, damping);
			if (step == null) { damping *= 10; continue; }
			var stepSet = residuals(step), stepNorm = norm(stepSet.values);
			if (stepNorm < currentNorm) { trial = step; current = stepSet; currentNorm = stepNorm; damping = Math.max(1e-12, damping * 0.3); }
			else damping = Math.min(1e12, damping * 10);
		}
		shapeOnly = false;
		return currentNorm <= solveTolerance ? trial : null;
	}

	/** Constraints that fix shape rather than size: they hold for every scaled or moved copy of a solution. */
	private static function isShapeConstraint(kind:String):Bool {
		return switch kind {
			case "fixed", "distance", "radius", "angle": false;
			default: true;
		};
	}

	private function failingOwners(set:ResidualSet,t:Float):Array<String>{var out:Array<String> = [];for(i in 0...set.values.length)if(Math.abs(set.values[i])>t&&!contains(out,set.owners[i]))out.push(set.owners[i]);return out;}
	private static function contains(a:Array<String>,v:String):Bool{for(x in a)if(x==v)return true;return false;}
	private static function vectorDifference(a:Array<Float>,b:Array<Float>,out:Array<Float>):Void{out.push(a[0]-b[0]);out.push(a[1]-b[1]);}
	private static function distance(a:Array<Float>,b:Array<Float>):Float{var x=a[0]-b[0],y=a[1]-b[1];return Math.pow(x*x+y*y,0.5);}
	private static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1];
	private static function cross(a:Array<Float>,b:Array<Float>):Float return a[0]*b[1]-a[1]*b[0];
	private static function wrap(v:Float):Float{while(v>Math.PI)v-=2*Math.PI;while(v< -Math.PI)v+=2*Math.PI;return v;}
	private function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (index in 0...values.length) {
			if (index % 1024 == 0)
				checkCancelled();
			sum += values[index] * values[index];
		}
		return Math.pow(sum, 0.5);
	}

	private function fill(count:Int, value:Float):Array<Float> {
		var result:Array<Float> = [];
		for (index in 0...count) {
			if (index % 1024 == 0)
				checkCancelled();
			result.push(value);
		}
		return result;
	}

	private function matrix(rowCount:Int, columnCount:Int, value:Float):Array<Array<Float>> {
		var result:Array<Array<Float>> = [];
		for (row in 0...rowCount) {
			if (row % 16 == 0)
				checkCancelled();
			result.push(fill(columnCount, value));
		}
		return result;
	}
	private function linearSolve(a:Array<Array<Float>>, b:Array<Float>):Null<Array<Float>> {
		var n = b.length;
		var matrix:Array<Array<Float>> = [];
		for (index in 0...n) {
			if (index % 64 == 0)
				checkCancelled();
			matrix.push(a[index].copy());
			matrix[index].push(b[index]);
		}
		for (column in 0...n) {
			checkCancelled();
			var pivotRow = column;
			for (row in column...n)
				if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivotRow][column]))
					pivotRow = row;
			if (Math.abs(matrix[pivotRow][column]) < 1e-15)
				return null;
			var temporary = matrix[column];
			matrix[column] = matrix[pivotRow];
			matrix[pivotRow] = temporary;
			for (row in (column + 1)...n) {
				if (row % 16 == 0)
					checkCancelled();
				var factor = matrix[row][column] / matrix[column][column];
				for (entry in column...(n + 1))
					matrix[row][entry] -= factor * matrix[column][entry];
			}
		}
		var result = fill(n, 0);
		var row = n - 1;
		while (row >= 0) {
			checkCancelled();
			var value = matrix[row][n];
			for (column in (row + 1)...n)
				value -= matrix[row][column] * result[column];
			result[row] = value / matrix[row][row];
			row--;
		}
		return result;
	}
}
