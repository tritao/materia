package cadkit.sketch;

import cadkit.parametric.EvaluationCancelled;

/**
	A sketch's variables and constraints as one solve sees them: point x/y
	pairs then radii, their names, the constraint list, and the solve's scale
	and tolerance. Fixed for the solve; the other solver parts read it.
*/
class SketchLayout {
	public final sketch:ConstrainedSketch;
	public final pointIndex:Map<String, Int> = new Map();
	public final radiusIndex:Map<String, Int> = new Map();
	public final points:Map<String, SketchPoint> = new Map();
	public final entities:Map<String, SketchEntity> = new Map();
	public final constraintList:Array<SketchConstraint>;
	/** Every constraint index, ascending. */
	public final allConstraints:Array<Int>;
	/** Each variable's name (point id with #x/#y, entity id with #r), so a part's key pins its variable order. */
	public final variableNames:Array<String> = [];
	public var variableCount(default, null):Int = 0;
	/** Lengths are solved divided by this (see `characteristicScale`). */
	public var normalizationScale:Float = 1;
	public final solveTolerance:Float;
	final cancellationCheck:Null<Void->Bool>;

	public function new(sketch:ConstrainedSketch, cancellationCheck:Null<Void->Bool>) {
		this.sketch = sketch;
		this.cancellationCheck = cancellationCheck;
		constraintList = sketch.constraints();
		allConstraints = [for (index in 0...constraintList.length) index];
		solveTolerance = sketch.settings.tolerance;
		for (point in sketch.points()) {
			points.set(point.id, point); pointIndex.set(point.id, variableCount); variableCount += 2;
			variableNames.push(point.id + "#x"); variableNames.push(point.id + "#y");
		}
		for (entity in sketch.entities()) {
			entities.set(entity.id, entity);
			if (entity.kind == "circle" || entity.kind == "arc") {
				radiusIndex.set(entity.id, variableCount); variableCount++; variableNames.push(entity.id + "#r");
			}
		}
	}

	public function checkCancelled():Void {
		if (cancellationCheck != null && cancellationCheck())
			throw new EvaluationCancelled();
	}

	public function invalid(message:String, ids:Array<String>):SketchSolveError
		return new SketchSolveError(new SolveDiagnostic("invalid", false, 1e300, variableCount, 0, ids, message));

	/** Checks settings, points, entities and constraint values (references are checked by evaluating once). */
	public function validate():Void {
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
		for (constraint in constraintList) {
			if (constraintIndex++ % 16 == 0)
				checkCancelled();
			if (!Math.isFinite(constraint.value)) throw invalid("constraint values must be finite", [constraint.id]);
			if ((constraint.kind == "distance" || constraint.kind == "radius") && constraint.value <= 0)
				throw invalid("distance and radius constraints must be positive", [constraint.id]);
		}
	}

	/** The authored pose: point x/y pairs, then radii. */
	public function authoredPose():Array<Float> {
		var x:Array<Float> = [];
		for (point in sketch.points()) { x.push(point.x); x.push(point.y); }
		for (entity in sketch.entities()) if (entity.kind == "circle" || entity.kind == "arc") x.push(entity.radius);
		return x;
	}

	/** The pose a solve starts from: `seed`'s values where it has them, the authored ones elsewhere. */
	public function seededPose(seed:Null<SolvedSketch>):Array<Float> {
		var x:Array<Float> = [];
		for (point in sketch.points()) {
			var value = [point.x, point.y];
			if (seed != null) try value = seed.point(point.id) catch (_:Dynamic) {}
			x.push(value[0]);
			x.push(value[1]);
		}
		for (entity in sketch.entities())
			if (entity.kind == "circle" || entity.kind == "arc") {
				var radius = entity.radius;
				if (seed != null) try radius = seed.radius(entity.id) catch (_:Dynamic) {}
				x.push(radius);
			}
		return x;
	}

	/** The larger extent of the points, the largest radius or length dimension; 1 for an empty sketch. */
	public function characteristicScale(x:Array<Float>):Float {
		var minimumX = 1e300, maximumX = -1e300, minimumY = 1e300, maximumY = -1e300;
		for (point in sketch.points()) {
			var index:Int = cast pointIndex.get(point.id);
			minimumX = Math.min(minimumX, x[index]);
			maximumX = Math.max(maximumX, x[index]);
			minimumY = Math.min(minimumY, x[index + 1]);
			maximumY = Math.max(maximumY, x[index + 1]);
		}
		var scale = sketch.points().length == 0 ? 1 : Math.max(maximumX - minimumX, maximumY - minimumY);
		for (entity in sketch.entities())
			if (entity.kind == "circle" || entity.kind == "arc") {
				var index:Int = cast radiusIndex.get(entity.id);
				scale = Math.max(scale, Math.abs(x[index]));
			}
		for (constraint in constraintList)
			if (constraint.kind == "distance" || constraint.kind == "radius")
				scale = Math.max(scale, Math.abs(constraint.value));
		return scale > 0 ? scale : 1;
	}

	public function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (index in 0...values.length) {
			if (index % 1024 == 0)
				checkCancelled();
			sum += values[index] * values[index];
		}
		return Math.pow(sum, 0.5);
	}
}
