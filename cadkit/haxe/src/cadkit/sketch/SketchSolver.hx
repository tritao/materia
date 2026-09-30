package cadkit.sketch;

import cadkit.parametric.EvaluationCancelled;
import cadkit.solve.ConstraintDiagnosis;
import cadkit.solve.EnvelopeCholesky;
import cadkit.solve.ConstraintDiagnosis.RowStructure;

private typedef ResidualSet = { values:Array<Float>, owners:Array<String> };
/** One Jacobian row as (variable, entry) pairs; a variable may repeat and its entries then add. */
private typedef SparseRow = { index:Array<Int>, value:Array<Float> };
/**
	Constraints that share variables, directly or through each other, and the
	variables they reach, both ascending. `position`/`first` are filled on
	first use (a part reused from a previous solve never needs them).
	`position`/`first` describe the envelope of its normal matrix under a
	reverse Cuthill-McKee ordering: local index → row, and each row's first
	structurally nonzero column.
*/
private typedef Part = { id:Int, constraints:Array<Int>, variables:Array<Int>, position:Array<Int>, first:Array<Int>, ordered:Bool,
	envelope:Array<Array<Float>> };

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
	static inline var WARM_DAMPING:Float = 1e-9;
	/** While finding a witness pose, only shape constraints (see `isShapeConstraint`) contribute residuals. */
	private var shapeOnly:Bool = false;
	/** When not −1, only this part's constraints contribute residuals and rows. */
	private var activePart:Int = -1;
	private var partList:Array<Part> = [];
	private var allConstraints:Array<Int> = [];
	/** Each variable's part (−1 when free) and its index within that part. */
	private var variablePart:Array<Int> = [];
	private var variableLocal:Array<Int> = [];
	private var references:Array<Array<Int>> = [];
	private final constraintList:Array<SketchConstraint>;

	private final diagnoseParts:Bool;
	/** Each variable's name (point id with #x/#y, entity id with #r), so a part's key pins its variable order. */
	private final variableNames:Array<String> = [];

	private function new(sketch:ConstrainedSketch, seed:Null<SolvedSketch>, cancellationCheck:Null<Void->Bool>, diagnose:Bool = true) {
		diagnoseParts = diagnose;
		this.sketch = sketch; constraintList = sketch.constraints(); pointIndex = new Map(); radiusIndex = new Map(); points = new Map(); entities = new Map();
		this.seed = seed;
		this.cancellationCheck = cancellationCheck;
		tangentBranches = new Map();
		tangentSides = new Map();
		variableCount = 0;
		solveTolerance = sketch.settings.tolerance;
		normalizationScale = 1;
		for (point in sketch.points()) {
			points.set(point.id, point); pointIndex.set(point.id, variableCount); variableCount += 2;
			variableNames.push(point.id + "#x"); variableNames.push(point.id + "#y");
		}
		for (entity in sketch.entities()) {
			entities.set(entity.id, entity);
			if (entity.kind == "circle" || entity.kind == "arc") { radiusIndex.set(entity.id, variableCount); variableCount++; variableNames.push(entity.id + "#r"); }
		}
	}

	public static function solve(sketch:ConstrainedSketch, seed:Null<SolvedSketch> = null,
		cancellationCheck:Null<Void->Bool> = null, diagnose:Bool = true):SolvedSketch {
		return new SketchSolver(sketch, seed, cancellationCheck, diagnose).run();
	}

	/**
		Test hook for `cadkit.solve.JacobianCheck`: the authored variables (point
		x/y pairs, then radii) and this solver's residuals and analytic Jacobian
		as functions of them, in sketch units. Tangent sides and branches are
		fixed from the authored pose, as a solve would fix them.
	*/
	public static function probe(sketch:ConstrainedSketch):{variables:Array<Float>, residuals:Array<Float>->Array<Float>,
			jacobian:Array<Float>->Array<Float>} {
		var solver = new SketchSolver(sketch, null, null);
		solver.validate();
		var x:Array<Float> = [];
		for (point in sketch.points()) { x.push(point.x); x.push(point.y); }
		for (entity in sketch.entities()) if (entity.kind == "circle" || entity.kind == "arc") x.push(entity.radius);
		solver.normalizationScale = solver.characteristicScale(x);
		solver.initializeTangentBranches(x);
		return {
			variables: x,
			residuals: values -> solver.residuals(values).values,
			jacobian: values -> solver.rawJacobian(values)
		};
	}

	/** Dense Jacobian with respect to x itself (entries are with respect to x / s, so divide by s), for `probe`. */
	private function rawJacobian(values:Array<Float>):Array<Float> {
		var rows = sparseJacobian(values, residuals(values).values.length);
		var dense = [for (_ in 0...rows.length * variableCount) 0.0];
		for (row in 0...rows.length)
			for (e in 0...rows[row].index.length)
				dense[row * variableCount + rows[row].index[e]] += rows[row].value[e] / normalizationScale;
		return dense;
	}

	private function checkCancelled():Void {
		if (cancellationCheck != null && cancellationCheck())
			throw new EvaluationCancelled();
	}

	private function invalid(message:String, ids:Array<String>):SketchSolveError
		return new SketchSolveError(new SolveDiagnostic("invalid", false, 1e300, variableCount, 0, ids, message));

	/**
		Solves each part (constraints sharing variables) on its own, then
		diagnoses it; a part's dependency is checked against a witness pose
		before it is called redundant. Reports merge across parts.
	*/
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
		var iterations = 0, failed = false, stationary = true, degenerate = false;
		var conflictingIds:Array<String> = [], failingIds:Array<String> = [];
		var reports:Array<DiagnosisReport> = [];
		var cache = new Map<String, CachedPart>(), structures = new Map<String, PartStructure>();
		var previous = seed == null ? null : seed.partCache, previousStructures = seed == null ? null : seed.structures;
		var diagnosed = true;
		for (part in parts()) {
			checkCancelled();
			// A part whose constraints, fixed positions and settings are unchanged, seeded from its own previous
			// solution, is already solved: reuse its diagnosis. Dragging one profile re-solves only that profile.
			var key = partKey(part), values = partValues(part);
			// The same structure (a drag changes only values) keeps its orderings and its last diagnosis.
			var known = previousStructures == null ? null : previousStructures.get(key);
			if (known != null) {
				part.position = known.position;
				part.first = known.first;
				part.ordered = true;
			}
			var cached = previous == null ? null : previous.get(key);
			if (cached != null && sameValues(cached.values, values)) {
				reports.push(cached.report);
				if (cached.degenerate) degenerate = true;
				cache.set(key, cached);
				if (known != null) structures.set(key, known);
				continue;
			}
			var partDegenerate = false;
			var solved = solvePart(x, part, false);
			x = solved.x;
			iterations = iterations > solved.iterations ? iterations : solved.iterations;
			if (!diagnoseParts && known != null && solved.norm <= solveTolerance) {
				// Dragging: report the last diagnosis; the solve on release checks this part again.
				reports.push(known.report);
				if (known.degenerate) degenerate = true;
				diagnosed = false;
				structures.set(key, {position: part.position, first: part.first, rows: known.rows, report: known.report,
					degenerate: known.degenerate});
				continue;
			}
			var rows = known != null && known.rows != null ? known.rows : structuralRows(part, solved.set.values.length);
			var report = diagnosePart(x, x, part, solved.set, rows);
			if (solved.norm > solveTolerance) {
				failed = true;
				var limit = Math.max(sketch.settings.rankTolerance, solveTolerance * 10) * (1 + solved.norm);
				if (norm(partGradient(x, part, solved.set)) > limit)
					stationary = false;
				for (id in report.conflictingOwners()) if (!contains(conflictingIds, id)) conflictingIds.push(id);
				for (id in failingOwners(solved.set, solveTolerance * 10)) if (!contains(failingIds, id)) failingIds.push(id);
			} else if (report.rank < solved.set.values.length) {
				// A dependency here may belong to this pose only: compare with the rank at a nearby pose of the same shape.
				var witness = witnessPose(x, part);
				if (witness != null) {
					var generic = diagnosePart(witness, x, part, solved.set);
					if (generic.rank > report.rank) {
						report = generic;
						partDegenerate = true;
					}
				}
			}
			if (partDegenerate) degenerate = true;
			if (solved.norm <= solveTolerance)
				cache.set(key, {values: values, report: report, degenerate: partDegenerate});
			structures.set(key, {position: part.position, first: part.first, rows: rows, report: report, degenerate: partDegenerate});
			reports.push(report);
		}
		var report = ConstraintDiagnosis.merge(reports, variableCount);
		var current = residuals(x);
		var currentNorm = norm(current.values);
		var dof = report.degreesOfFreedom;
		if (failed) {
			var status = stationary ? "conflicting" : "nonconvergent";
			var badIds = stationary && conflictingIds.length > 0 ? conflictingIds : failingIds;
			badIds.sort(Reflect.compare);
			var message = stationary
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
		var diagnostic = new SolveDiagnostic(status, true, currentNorm, dof, iterations, ids, message, report, degenerate, diagnosed);
		checkCancelled();
		var coordinates:Map<String, Array<Float>> = new Map();
		for (point in sketch.points()) { var i:Int = cast pointIndex.get(point.id); coordinates.set(point.id, [x[i], x[i + 1]]); }
		var radii:Map<String, Float> = new Map();
		for (entity in sketch.entities()) if (radiusIndex.exists(entity.id)) { var i:Int = cast radiusIndex.get(entity.id); radii.set(entity.id, x[i]); }
		return new SolvedSketch(coordinates, radii, diagnostic, cache, structures);
	}

	/**
		The part's structure: its variables in order, constraint IDs, kinds and references, and the entities they
		reach. Numbers are in `partValues`.
	*/
	private function partKey(part:Part):String {
		var key = new StringBuf(), field = String.fromCharCode(1), record = String.fromCharCode(2);
		for (variable in part.variables) { key.add(variableNames[variable]); key.add(field); }
		key.add(record);
		for (index in part.constraints) {
			var c = constraintList[index];
			key.add(c.id); key.add(field); key.add(c.kind);
			for (id in [c.first, c.second, c.third]) {
				key.add(field);
				if (id == null) continue;
				key.add(id);
				var e = entities.get(id);
				if (e != null) { key.add("="); key.add(e.kind); key.add(":"); key.add(e.first); key.add(","); key.add(e.second == null ? "" : e.second); }
			}
			key.add(record);
		}
		return key.toString();
	}

	/** Everything numeric a part's solution depends on besides its seed: settings, dimension values, fixed positions. */
	private function partValues(part:Part):Array<Float> {
		var values = [sketch.settings.tolerance, sketch.settings.rankTolerance, sketch.settings.maxIterations, sketch.settings.initialDamping,
			normalizationScale];
		for (index in part.constraints) {
			var c = constraintList[index];
			values.push(c.value);
			if (c.kind == "fixed") {
				var p = points.get(c.first);
				if (p != null) { values.push(p.x); values.push(p.y); }
			}
		}
		return values;
	}

	private static function sameValues(a:Array<Float>, b:Array<Float>):Bool {
		if (a.length != b.length) return false;
		for (i in 0...a.length) if (a[i] != b[i]) return false;
		return true;
	}

	/**
		Levenberg-Marquardt on one part from `x` (the part's shape constraints
		only when `shape`), then up to three Gauss-Newton polish steps while
		each halves the residual: converging only to the tolerance leaves a
		visible length error.
	*/
	private function solvePart(start:Array<Float>, part:Part, shape:Bool):{x:Array<Float>, set:ResidualSet, norm:Float, iterations:Int} {
		orderPart(part);
		activePart = part.id;
		shapeOnly = shape;
		// A part seeded from a previous solution starts next to its answer: begin as Gauss-Newton. The authored
		// damping would swamp the soft modes of a long chain (its JᵀJ has eigenvalues far below 1e-3) and cost
		// several iterations for a tiny drag; a rejected step still raises the damping.
		var x = start, damping = seed != null && !shape ? WARM_DAMPING : sketch.settings.initialDamping;
		var current = residuals(x), currentNorm = norm(current.values), iterations = 0;
		while (iterations < sketch.settings.maxIterations && currentNorm > solveTolerance) {
			checkCancelled();
			iterations++;
			var trial = dampedStep(x, current, damping, part);
			if (trial == null) { damping *= 10; continue; }
			var trialSet = residuals(trial), trialNorm = norm(trialSet.values);
			if (trialNorm < currentNorm) { x = trial; current = trialSet; currentNorm = trialNorm; damping = Math.max(1e-12, damping * 0.3); }
			else damping = Math.min(1e12, damping * 10);
		}
		var polish = 0;
		while (currentNorm <= solveTolerance && currentNorm > solveTolerance * 1e-3 && polish < 3) {
			checkCancelled();
			polish++;
			var trial = dampedStep(x, current, 1e-12, part);
			if (trial == null) break;
			var trialSet = residuals(trial), trialNorm = norm(trialSet.values);
			if (!(trialNorm < currentNorm * 0.5)) break;
			x = trial; current = trialSet; currentNorm = trialNorm;
		}
		activePart = -1;
		shapeOnly = false;
		return {x: x, set: current, norm: currentNorm, iterations: iterations};
	}

	/**
		Parts from what each constraint references (points, line ends, centres,
		radii), not from Jacobian entries, which can be zero by accident.
		Variables no constraint references belong to no part: they are free.
	*/
	private function parts():Array<Part> {
		var parent = [for (i in 0...variableCount) i];
		references = [for (c in constraintList) referencedVariables(c)];
		allConstraints = [for (index in 0...constraintList.length) index];
		for (list in references)
			for (k in 1...list.length)
				union(parent, list[0], list[k]);
		var slot = [for (_ in 0...variableCount) -1];
		partList = [];
		for (index in 0...constraintList.length) {
			if (references[index].length == 0) continue;
			var root = find(parent, references[index][0]);
			if (slot[root] < 0) {
				slot[root] = partList.length;
				partList.push({id: partList.length, constraints: [], variables: [], position: [], first: [], ordered: false, envelope: []});
			}
			partList[slot[root]].constraints.push(index);
		}
		variablePart = [for (_ in 0...variableCount) -1];
		variableLocal = [for (_ in 0...variableCount) -1];
		for (variable in 0...variableCount) {
			var id = slot[find(parent, variable)];
			if (id < 0) continue;
			variablePart[variable] = id;
			variableLocal[variable] = partList[id].variables.length;
			partList[id].variables.push(variable);
		}
		return partList;
	}

	/**
		Reverse Cuthill-McKee over the part's variable graph (two variables are
		adjacent when one constraint references both), then the envelope of
		its normal matrix in that order. Chains and grids keep a narrow band.
	*/
	private function orderPart(part:Part):Void {
		if (part.ordered) return;
		var k = part.variables.length;
		var neighbours:Array<Array<Int>> = [for (_ in 0...k) []];
		for (index in part.constraints) {
			var list = [for (v in references[index]) variableLocal[v]];
			for (a in list) for (b in list)
				if (a != b && neighbours[a].indexOf(b) < 0) neighbours[a].push(b);
		}
		var ordering = EnvelopeCholesky.order(neighbours);
		part.position = ordering.position;
		part.first = ordering.first;
		part.ordered = true;
	}

	private function referencedVariables(c:SketchConstraint):Array<Int> {
		var result:Array<Int> = [];
		for (id in [c.first, c.second, c.third]) {
			if (id == null) continue;
			var p = pointIndex.get(id);
			if (p != null) { result.push(p); result.push(p + 1); continue; }
			var e = entities.get(id);
			if (e == null) continue;
			var first = pointIndex.get(e.first);
			if (first != null) { result.push(first); result.push(first + 1); }
			if (e.second != null) { var second = pointIndex.get(e.second); if (second != null) { result.push(second); result.push(second + 1); } }
			var r = radiusIndex.get(id);
			if (r != null) result.push(r);
		}
		return result;
	}

	private static function find(parent:Array<Int>, i:Int):Int {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	}

	private static function union(parent:Array<Int>, a:Int, b:Int):Void {
		var ra = find(parent, a), rb = find(parent, b);
		if (ra < rb) parent[rb] = ra; else if (rb < ra) parent[ra] = rb;
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
		for (index in activeConstraints()) {
			var c = constraintList[index];
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
	/**
		One Levenberg-Marquardt step on a part, in scaled variables: JᵀJ + λI
		is accumulated in the part's envelope and factored by Cholesky there.
		Null when the damped matrix is not positive definite.
	*/
	private function dampedStep(x:Array<Float>, current:ResidualSet, damping:Float, part:Part):Null<Array<Float>> {
		var k = part.variables.length, rows = sparseJacobian(x, current.values.length);
		// envelope[row][column - first[row]] for first[row] <= column <= row, in RCM order; kept per part, zeroed here.
		if (part.envelope.length != k) part.envelope = EnvelopeCholesky.zero(part.first);
		var envelope = part.envelope;
		for (line in envelope) for (i in 0...line.length) line[i] = 0;
		var gradient = fill(k, 0);
		var positions = scratchIndex, values = scratchValue;
		for (row in 0...rows.length) {
			if (row % 16 == 0)
				checkCancelled();
			// The row's entries by envelope position, repeated variables merged, in reused scratch arrays.
			var source = rows[row];
			positions.resize(0);
			values.resize(0);
			for (e in 0...source.index.length) {
				var variable = source.index[e];
				if (variablePart[variable] != part.id) continue;
				var position = part.position[variableLocal[variable]], at = positions.indexOf(position);
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
			trial[part.variables[i]] += y[part.position[i]] * normalizationScale;
		return trial;
	}

	private final scratchIndex:Array<Int> = [];
	private final scratchValue:Array<Float> = [];
	/** Row storage `sparseJacobian` reuses between calls; callers consume rows before asking again. */
	private final rowBuffer:Array<SparseRow> = [];

	/** A row's entries in the part's local numbering, repeated variables added together. */
	private function localEntries(row:SparseRow, part:Part):SparseRow {
		var index:Array<Int> = [], value:Array<Float> = [];
		for (e in 0...row.index.length) {
			if (variablePart[row.index[e]] != part.id) continue;
			var local = variableLocal[row.index[e]];
			var at = index.indexOf(local);
			if (at < 0) { index.push(local); value.push(row.value[e]); } else value[at] += row.value[e];
		}
		return {index: index, value: value};
	}

	/** Jᵀr over a part: zero at a stationary point of its residual. */
	private function partGradient(x:Array<Float>, part:Part, set:ResidualSet):Array<Float> {
		activePart = part.id;
		var rows = sparseJacobian(x, set.values.length);
		activePart = -1;
		var result = fill(part.variables.length, 0);
		for (row in 0...rows.length) {
			var entries = localEntries(rows[row], part);
			for (e in 0...entries.index.length) result[entries.index[e]] += entries.value[e] * set.values[row];
		}
		return result;
	}

	/** The constraints to evaluate, ascending: one part's, or all. */
	private inline function activeConstraints():Array<Int>
		return activePart < 0 ? allConstraints : partList[activePart].constraints;

	/**
		Analytic Jacobian with respect to the scaled variables (x / normalizationScale),
		as sparse rows in the order `residuals` emits them. Each block below is
		∂f/∂x of the unscaled residual f: linear rows are f / s with variables
		x / s, so their entries are ∂f/∂x unchanged; angular rows are
		dimensionless and take ∂f/∂x · s.
	*/
	private function sparseJacobian(x:Array<Float>, rows:Int):Array<SparseRow> {
		while (rowBuffer.length < rows) rowBuffer.push({index: [], value: []});
		for (r in 0...rows) { rowBuffer[r].index.resize(0); rowBuffer[r].value.resize(0); }
		var j = rows == rowBuffer.length ? rowBuffer : rowBuffer.slice(0, rows);
		var row = 0, constraintIndex = 0;
		for (index in activeConstraints()) {
			var c = constraintList[index];
			if (constraintIndex++ % 16 == 0)
				checkCancelled();
			if (shapeOnly && !isShapeConstraint(c.kind))
				continue;
			jacobianTarget = j;
			jacobianRow = row;
			jacobianScale = hasLinearResidual(c.kind) ? 1.0 : normalizationScale;
			switch (c.kind) {
				case "fixed":
					var p = pointVar(c.first);
					entry(0, p, 1); entry(1, p + 1, 1);
					row += 2;
				case "coincident", "concentric":
					var a = c.kind == "coincident" ? pointVar(c.first) : pointVar(entity(c.first, c.id).first);
					var b = c.kind == "coincident" ? pointVar(needSecond(c)) : pointVar(entity(needSecond(c), c.id).first);
					entry(0, a, 1); entry(0, b, -1); entry(1, a + 1, 1); entry(1, b + 1, -1);
					row += 2;
				case "horizontal", "vertical":
					var e = entity(c.first, c.id), offset = c.kind == "horizontal" ? 1 : 0;
					entry(0, pointVar(e.second) + offset, 1); entry(0, pointVar(e.first) + offset, -1);
					row += 1;
				case "distance":
					distanceRow(x, pointVar(c.first), pointVar(needSecond(c)), 1);
					row += 1;
				case "radius":
					entry(0, radiusVar(c.first), 1);
					row += 1;
				case "equal":
					measureRow(x, c.first, 1, c.id);
					measureRow(x, needSecond(c), -1, c.id);
					row += 1;
				case "parallel", "perpendicular":
					angleRow(x, c.first, needSecond(c), c.kind == "parallel", 0, c.id);
					row += 1;
				case "angle":
					angleRow(x, c.first, needSecond(c), false, 0, c.id);
					angleRow(x, c.first, needSecond(c), true, 1, c.id);
					row += 2;
				case "pointOn":
					var e = entity(needSecond(c), c.id);
					if (e.kind == "line")
						pointLineRow(x, pointVar(c.first), 1, 1, needSecond(c), 0, c.id);
					else {
						distanceRow(x, pointVar(c.first), pointVar(e.first), 1);
						entry(0, radiusVar(needSecond(c)), -1);
					}
					row += 1;
				case "tangent":
					tangentRow(x, c.first, needSecond(c), c.id);
					row += 1;
				case "symmetric":
					var a = pointVar(c.first), b = pointVar(needSecond(c));
					// Row 0: the midpoint lies on the axis; each point carries half the weight.
					pointLineRow(x, a, 0.5, 0, needThird(c), 0, c.id);
					pointLineRow(x, b, 0.5, 0, needThird(c), 0, c.id);
					pointLineAxisTerms(x, midpoint(x, a, b), needThird(c), 0, c.id);
					// Row 1: (b − a)·d / |d| = 0.
					alongRow(x, a, b, needThird(c), 1, c.id);
					row += 2;
				default:
					throw invalid("unsupported constraint kind: " + c.kind, [c.id]);
			}
		}
		return j;
	}

	/** Where `entry` writes: the Jacobian being built, its current constraint's first row, and that row's scale. */
	private var jacobianTarget:Array<SparseRow> = [];
	private var jacobianRow:Int = 0;
	private var jacobianScale:Float = 1;

	private inline function entry(r:Int, index:Int, value:Float):Void {
		var target = jacobianTarget[jacobianRow + r];
		target.index.push(index);
		target.value.push(value * jacobianScale);
	}

	private function pointVar(id:String):Int {
		var found = pointIndex.get(id);
		if (found == null) throw invalid("missing point: " + id, [id]);
		return found;
	}

	private function radiusVar(id:String):Int {
		var found = radiusIndex.get(id);
		if (found == null) throw invalid("constraint requires a circle or arc", [id]);
		return found;
	}

	private static function midpoint(x:Array<Float>, a:Int, b:Int):Array<Float>
		return [(x[a] + x[b]) / 2, (x[a + 1] + x[b + 1]) / 2];

	/** f = |a − b| (times `sign`): ∂f/∂a = (a − b)/|a − b|, ∂f/∂b = −∂f/∂a. */
	private function distanceRow(x:Array<Float>, a:Int, b:Int, sign:Float):Void {
		var dx = x[a] - x[b], dy = x[a + 1] - x[b + 1], length = Math.sqrt(dx * dx + dy * dy);
		if (length == 0) return;
		entry(0, a, sign * dx / length); entry(0, a + 1, sign * dy / length);
		entry(0, b, -sign * dx / length); entry(0, b + 1, -sign * dy / length);
	}

	/** A line's length or a circle's radius, times `sign`. */
	private function measureRow(x:Array<Float>, id:String, sign:Float, owner:String):Void {
		var e = entity(id, owner);
		if (e.kind == "line") distanceRow(x, pointVar(e.second), pointVar(e.first), sign);
		else entry(0, radiusVar(id), sign);
	}

	/**
		For line directions a and b with n = |a||b|: f = (a×b)/n when `cross`,
		else (a·b)/n. ∂f/∂a = ∂(a×b or a·b)/∂a / n − f a/|a|², and likewise for b;
		each direction is its second point minus its first.
	*/
	private function angleRow(x:Array<Float>, first:String, second:String, cross:Bool, r:Int, owner:String):Void {
		var e1 = entity(first, owner), e2 = entity(second, owner);
		var a0 = pointVar(e1.first), a1 = pointVar(e1.second), b0 = pointVar(e2.first), b1 = pointVar(e2.second);
		var ax = x[a1] - x[a0], ay = x[a1 + 1] - x[a0 + 1], bx = x[b1] - x[b0], by = x[b1 + 1] - x[b0 + 1];
		var aa = ax * ax + ay * ay, bb = bx * bx + by * by, n = Math.sqrt(aa * bb);
		if (n < 1e-12) return;
		var f = (cross ? ax * by - ay * bx : ax * bx + ay * by) / n;
		var dax = (cross ? by : bx) / n - f * ax / aa, day = (cross ? -bx : by) / n - f * ay / aa;
		var dbx = (cross ? -ay : ax) / n - f * bx / bb, dby = (cross ? ax : ay) / n - f * by / bb;
		entry(r, a1, dax); entry(r, a1 + 1, day); entry(r, a0, -dax); entry(r, a0 + 1, -day);
		entry(r, b1, dbx); entry(r, b1 + 1, dby); entry(r, b0, -dbx); entry(r, b0 + 1, -dby);
	}

	/**
		Signed distance of point p from a line through e0 along d = e1 − e0:
		f = (p − e0)×d / |d|. `pointWeight` scales ∂f/∂p = (dy, −dx)/|d|
		(a midpoint gives each end half). With `axisWeight` 1 the line's own
		terms are added here; symmetric rows add them once via `pointLineAxisTerms`.
	*/
	private function pointLineRow(x:Array<Float>, p:Int, pointWeight:Float, axisWeight:Float, lineId:String, r:Int,
			owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		entry(r, p, pointWeight * dy / length); entry(r, p + 1, -pointWeight * dx / length);
		if (axisWeight != 0)
			pointLineAxisTerms(x, [x[p], x[p + 1]], lineId, r, owner);
	}

	/** The line's part of ∂f/∂(e0, e1) for f = (p − e0)×d/|d| at a fixed point p. */
	private function pointLineAxisTerms(x:Array<Float>, p:Array<Float>, lineId:String, r:Int, owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		var wx = p[0] - x[e0], wy = p[1] - x[e0 + 1];
		var f = (wx * dy - wy * dx) / length;
		// ∂f/∂d, then d = e1 − e0; w = p − e0 adds −(dy, −dx)/|d| to e0.
		var ddx = -wy / length - f * dx / dd, ddy = wx / length - f * dy / dd;
		entry(r, e1, ddx); entry(r, e1 + 1, ddy);
		entry(r, e0, -ddx - dy / length); entry(r, e0 + 1, -ddy + dx / length);
	}

	/** f = (b − a)·d / |d| for the axis d = e1 − e0. */
	private function alongRow(x:Array<Float>, a:Int, b:Int, lineId:String, r:Int, owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		var vx = x[b] - x[a], vy = x[b + 1] - x[a + 1], f = (vx * dx + vy * dy) / length;
		entry(r, b, dx / length); entry(r, b + 1, dy / length); entry(r, a, -dx / length); entry(r, a + 1, -dy / length);
		var ddx = vx / length - f * dx / dd, ddy = vy / length - f * dy / dd;
		entry(r, e1, ddx); entry(r, e1 + 1, ddy); entry(r, e0, -ddx); entry(r, e0 + 1, -ddy);
	}

	/** Tangency rows, following the stored side (line-circle) or contact branch (circle-circle). */
	private function tangentRow(x:Array<Float>, aid:String, bid:String, owner:String):Void {
		var first = entity(aid, owner), second = entity(bid, owner);
		if (first.kind == "line" || second.kind == "line") {
			var lineId = first.kind == "line" ? aid : bid, circleId = first.kind == "line" ? bid : aid;
			var circle = entity(circleId, owner);
			pointLineRow(x, pointVar(circle.first), 1, 1, lineId, 0, owner);
			var storedSide = tangentSides.get(owner);
			var side:Float = storedSide == null ? 1 : storedSide;
			if (storedSide == null) {
				var e = entity(lineId, owner), c = pointVar(circle.first), e0 = pointVar(e.first), e1 = pointVar(e.second);
				var signed = (x[c] - x[e0]) * (x[e1 + 1] - x[e0 + 1]) - (x[c + 1] - x[e0 + 1]) * (x[e1] - x[e0]);
				side = signed < 0 ? -1 : 1;
			}
			entry(0, radiusVar(circleId), -side);
		} else {
			distanceRow(x, pointVar(first.first), pointVar(second.first), 1);
			var storedBranch = tangentBranches.get(owner);
			var external:Bool = storedBranch == null ? true : storedBranch;
			var ra = radiusVar(aid), rb = radiusVar(bid);
			if (external) {
				entry(0, ra, -1); entry(0, rb, -1);
			} else {
				var sign = x[ra] - x[rb] >= 0 ? 1.0 : -1.0;
				entry(0, ra, -sign); entry(0, rb, sign);
			}
		}
	}

	/**
		The part's diagnosis row graph from what its constraints reference (a
		superset of any pose's nonzeros), ordered once and reusable while its
		structure stands. Null if the row count does not match `rows`.
	*/
	private function structuralRows(part:Part, rows:Int):Null<RowStructure> {
		var rowVariables:Array<Array<Int>> = [];
		for (index in part.constraints)
			for (_ in 0...rowCount(constraintList[index].kind))
				rowVariables.push(references[index]);
		if (rowVariables.length != rows) return null;
		return ConstraintDiagnosis.rowGraph([for (list in rowVariables) {index: list, value: [for (_ in list) 1.0]}]);
	}

	/** Residual rows each constraint kind emits (see `residuals`). */
	private static function rowCount(kind:String):Int {
		return switch kind {
			case "fixed", "coincident", "concentric", "angle", "symmetric": 2;
			default: 1;
		};
	}

	/**
		Diagnoses a part with its Jacobian taken at `at` (the solution, or a
		witness pose) and feasibility judged from `set`, the residuals at the
		solution; rows count as satisfied within ten times the solve tolerance.
	*/
	private function diagnosePart(at:Array<Float>, solution:Array<Float>, part:Part, set:ResidualSet, ?rows:RowStructure):DiagnosisReport {
		activePart = part.id;
		var jacobian = sparseJacobian(at, set.values.length);
		activePart = -1;
		var local = [for (row in jacobian) localEntries(row, part)];
		var satisfiedWithin = solveTolerance * 10;
		return ConstraintDiagnosis.diagnoseSparse(local, part.variables.length, set.owners,
			[for (value in set.values) value / satisfiedWithin], sketch.settings.rankTolerance, null, rows);
	}

	/**
		A pose near `x` with the same shape and no accidental coincidences
		(plan decision CS-D6): the part's lengths move by a fixed pseudo-random
		percent of the sketch size, then only its shape constraints are solved
		again. Dimensions are left free: their rows' derivatives do not depend
		on their target values. Null when the shape cannot be restored.
	*/
	private function witnessPose(x:Array<Float>, part:Part):Null<Array<Float>> {
		var seed = 20260930;
		var trial = x.copy();
		for (variable in part.variables) {
			seed = (seed * 1103515245 + 12345) & 0x7fffffff;
			trial[variable] += (seed / 0x7fffffff - 0.5) * 0.02 * normalizationScale;
		}
		for (entity in sketch.entities()) {
			var r = radiusIndex.get(entity.id);
			if (r != null && variablePart[r] == part.id) trial[r] = Math.max(trial[r], 0.5 * x[r]);
		}
		var solved = solvePart(trial, part, true);
		return solved.norm <= solveTolerance ? solved.x : null;
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
}
