package cadkit.sketch;

/**
	Each sketch constraint kind's residual and analytic Jacobian block, side by
	side in meaning: `residuals` and `rows` walk the same constraints in the
	same order. Constraint subsets and shape-only evaluation are arguments, and
	rows go through a `SketchRowWriter`, so nothing depends on hidden state
	beyond the tangent sides and branches fixed once per solve.
*/
class SketchEquations {
	final layout:SketchLayout;
	final tangentBranches:Map<String, Bool> = new Map();
	final tangentSides:Map<String, Float> = new Map();

	public function new(layout:SketchLayout) {
		this.layout = layout;
	}

	/** Constraints that fix shape rather than size: they hold for every scaled or moved copy of a solution. */
	public static function isShapeConstraint(kind:String):Bool {
		return switch kind {
			case "fixed", "distance", "radius", "angle": false;
			default: true;
		};
	}

	/** Residual rows each constraint kind emits. */
	public static function rowCount(kind:String):Int {
		return switch kind {
			case "fixed", "coincident", "concentric", "angle", "symmetric": 2;
			default: 1;
		};
	}

	static function hasLinearResidual(kind:String):Bool {
		return kind != "parallel" && kind != "perpendicular" && kind != "angle";
	}

	/** Residuals of the given constraints (ascending indices), only shape constraints when `shapeOnly`; lengths divided by the scale. */
	public function residuals(x:Array<Float>, constraints:Array<Int>, shapeOnly:Bool = false):SketchResidualSet {
		var values:Array<Float> = []; var owners:Array<String> = [];
		var constraintIndex = 0;
		for (index in constraints) {
			var c = layout.constraintList[index];
			if (constraintIndex++ % 16 == 0)
				layout.checkCancelled();
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
				default: throw layout.invalid("unsupported constraint kind: " + c.kind, [c.id]);
			}
			if (hasLinearResidual(c.kind))
				for (index in before...values.length)
					values[index] /= layout.normalizationScale;
			for (_ in before...values.length) owners.push(c.id);
		}
		return {values: values, owners: owners};
	}

	function point(id:String, x:Array<Float>):Array<Float> { var found = layout.pointIndex.get(id); if (found == null) throw layout.invalid("missing point: " + id, [id]); var i:Int = cast found; return [x[i], x[i+1]]; }
	function needPoint(id:String, owner:String):SketchPoint { var p=layout.points.get(id); if(p==null) throw layout.invalid("missing point: "+id,[owner]); return p; }
	function needSecond(c:SketchConstraint):String { if(c.second==null) throw layout.invalid("constraint requires a second reference",[c.id]); return c.second; }
	function needThird(c:SketchConstraint):String { if(c.third==null) throw layout.invalid("constraint requires a third reference",[c.id]); return c.third; }
	function entity(id:String, owner:String):SketchEntity { var e=layout.entities.get(id); if(e==null) throw layout.invalid("missing entity: "+id,[owner]); return e; }
	function line(id:String,x:Array<Float>,owner:String):Array<Array<Float>> { var e=entity(id,owner); if(e.kind!="line") throw layout.invalid("constraint requires a line",[owner]); return [point(e.first,x),point(e.second,x)]; }
	function center(id:String,x:Array<Float>,owner:String):Array<Float> { var e=entity(id,owner); if(e.kind=="line") throw layout.invalid("constraint requires a circle or arc",[owner]); return point(e.first,x); }
	function radius(id:String,x:Array<Float>,owner:String):Float { var found=layout.radiusIndex.get(id); if(found==null) throw layout.invalid("constraint requires a circle or arc",[owner]); var i:Int = cast found; return x[i]; }
	function direction(id:String,x:Array<Float>,owner:String):Array<Float> { var p=line(id,x,owner); return [p[1][0]-p[0][0],p[1][1]-p[0][1]]; }
	function measure(id:String,x:Array<Float>,owner:String):Float { var e=entity(id,owner); return e.kind=="line" ? distance(point(e.first,x),point(e.second,x)) : radius(id,x,owner); }
	function lengths(a:Array<Float>,b:Array<Float>,owner:String):Float { var v=Math.pow(dot(a,a)*dot(b,b),0.5); if(v<1e-12) throw layout.invalid("collapsed line in constraint",[owner]); return v; }
	function pointOn(pid:String,eid:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var p=point(pid,x); var e=entity(eid,owner);
		if(e.kind=="line") { var a=point(e.first,x); var d=direction(eid,x,owner); out.push(cross([p[0]-a[0],p[1]-a[1]],d)/Math.pow(dot(d,d),0.5)); }
		else out.push(distance(p,center(eid,x,owner))-radius(eid,x,owner));
	}
	function tangent(aid:String,bid:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var first = entity(aid, owner);
		var second = entity(bid, owner);
		if (first.kind == "line" && second.kind == "line")
			throw layout.invalid("line-line tangency is unsupported", [owner]);
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

	/** Fixes each tangency's line side or contact branch from pose `x`; residuals and rows then keep them. */
	public function fixTangentBranches(x:Array<Float>):Void {
		for (constraint in layout.constraintList) {
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

	function symmetry(pa:String,pb:String,axis:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var a=point(pa,x), b=point(pb,x), ends=line(axis,x,owner), d=direction(axis,x,owner); var dd=dot(d,d); if(dd<1e-12) throw layout.invalid("collapsed symmetry axis",[owner]);
		var mid=[(a[0]+b[0])/2,(a[1]+b[1])/2]; out.push(cross([mid[0]-ends[0][0],mid[1]-ends[0][1]],d)/Math.pow(dd,0.5)); out.push(dot([b[0]-a[0],b[1]-a[1]],d)/Math.pow(dd,0.5));
	}
	/**
		Analytic Jacobian with respect to the scaled variables (x / layout.normalizationScale),
		as sparse rows in the order `residuals` emits them, written through `w`. Each block below is
		∂f/∂x of the unscaled residual f: linear rows are f / s with variables
		x / s, so their entries are ∂f/∂x unchanged; angular rows are
		dimensionless and take ∂f/∂x · s.
	*/
	public function rows(x:Array<Float>, constraints:Array<Int>, rowCount:Int, w:SketchRowWriter, shapeOnly:Bool = false):Array<SketchSparseRow> {
		w.reset(rowCount);
		var row = 0, constraintIndex = 0;
		for (index in constraints) {
			var c = layout.constraintList[index];
			if (constraintIndex++ % 16 == 0)
				layout.checkCancelled();
			if (shapeOnly && !isShapeConstraint(c.kind))
				continue;
			w.begin(row, hasLinearResidual(c.kind) ? 1.0 : layout.normalizationScale);
			switch (c.kind) {
				case "fixed":
					var p = pointVar(c.first);
					w.entry(0, p, 1); w.entry(1, p + 1, 1);
					row += 2;
				case "coincident", "concentric":
					var a = c.kind == "coincident" ? pointVar(c.first) : pointVar(entity(c.first, c.id).first);
					var b = c.kind == "coincident" ? pointVar(needSecond(c)) : pointVar(entity(needSecond(c), c.id).first);
					w.entry(0, a, 1); w.entry(0, b, -1); w.entry(1, a + 1, 1); w.entry(1, b + 1, -1);
					row += 2;
				case "horizontal", "vertical":
					var e = entity(c.first, c.id), offset = c.kind == "horizontal" ? 1 : 0;
					w.entry(0, pointVar(e.second) + offset, 1); w.entry(0, pointVar(e.first) + offset, -1);
					row += 1;
				case "distance":
					distanceRow(w, x, pointVar(c.first), pointVar(needSecond(c)), 1);
					row += 1;
				case "radius":
					w.entry(0, radiusVar(c.first), 1);
					row += 1;
				case "equal":
					measureRow(w, x, c.first, 1, c.id);
					measureRow(w, x, needSecond(c), -1, c.id);
					row += 1;
				case "parallel", "perpendicular":
					angleRow(w, x, c.first, needSecond(c), c.kind == "parallel", 0, c.id);
					row += 1;
				case "angle":
					angleRow(w, x, c.first, needSecond(c), false, 0, c.id);
					angleRow(w, x, c.first, needSecond(c), true, 1, c.id);
					row += 2;
				case "pointOn":
					var e = entity(needSecond(c), c.id);
					if (e.kind == "line")
						pointLineRow(w, x, pointVar(c.first), 1, 1, needSecond(c), 0, c.id);
					else {
						distanceRow(w, x, pointVar(c.first), pointVar(e.first), 1);
						w.entry(0, radiusVar(needSecond(c)), -1);
					}
					row += 1;
				case "tangent":
					tangentRow(w, x, c.first, needSecond(c), c.id);
					row += 1;
				case "symmetric":
					var a = pointVar(c.first), b = pointVar(needSecond(c));
					// Row 0: the midpoint lies on the axis; each point carries half the weight.
					pointLineRow(w, x, a, 0.5, 0, needThird(c), 0, c.id);
					pointLineRow(w, x, b, 0.5, 0, needThird(c), 0, c.id);
					pointLineAxisTerms(w, x, midpoint(x, a, b), needThird(c), 0, c.id);
					// Row 1: (b − a)·d / |d| = 0.
					alongRow(w, x, a, b, needThird(c), 1, c.id);
					row += 2;
				default:
					throw layout.invalid("unsupported constraint kind: " + c.kind, [c.id]);
			}
		}
		return w.rows;
	}

	function pointVar(id:String):Int {
		var found = layout.pointIndex.get(id);
		if (found == null) throw layout.invalid("missing point: " + id, [id]);
		return found;
	}

	function radiusVar(id:String):Int {
		var found = layout.radiusIndex.get(id);
		if (found == null) throw layout.invalid("constraint requires a circle or arc", [id]);
		return found;
	}

	static function midpoint(x:Array<Float>, a:Int, b:Int):Array<Float>
		return [(x[a] + x[b]) / 2, (x[a + 1] + x[b + 1]) / 2];

	/** f = |a − b| (times `sign`): ∂f/∂a = (a − b)/|a − b|, ∂f/∂b = −∂f/∂a. */
	function distanceRow(w:SketchRowWriter, x:Array<Float>, a:Int, b:Int, sign:Float):Void {
		var dx = x[a] - x[b], dy = x[a + 1] - x[b + 1], length = Math.sqrt(dx * dx + dy * dy);
		if (length == 0) return;
		w.entry(0, a, sign * dx / length); w.entry(0, a + 1, sign * dy / length);
		w.entry(0, b, -sign * dx / length); w.entry(0, b + 1, -sign * dy / length);
	}

	/** A line's length or a circle's radius, times `sign`. */
	function measureRow(w:SketchRowWriter, x:Array<Float>, id:String, sign:Float, owner:String):Void {
		var e = entity(id, owner);
		if (e.kind == "line") distanceRow(w, x, pointVar(e.second), pointVar(e.first), sign);
		else w.entry(0, radiusVar(id), sign);
	}

	/**
		For line directions a and b with n = |a||b|: f = (a×b)/n when `cross`,
		else (a·b)/n. ∂f/∂a = ∂(a×b or a·b)/∂a / n − f a/|a|², and likewise for b;
		each direction is its second point minus its first.
	*/
	function angleRow(w:SketchRowWriter, x:Array<Float>, first:String, second:String, cross:Bool, r:Int, owner:String):Void {
		var e1 = entity(first, owner), e2 = entity(second, owner);
		var a0 = pointVar(e1.first), a1 = pointVar(e1.second), b0 = pointVar(e2.first), b1 = pointVar(e2.second);
		var ax = x[a1] - x[a0], ay = x[a1 + 1] - x[a0 + 1], bx = x[b1] - x[b0], by = x[b1 + 1] - x[b0 + 1];
		var aa = ax * ax + ay * ay, bb = bx * bx + by * by, n = Math.sqrt(aa * bb);
		if (n < 1e-12) return;
		var f = (cross ? ax * by - ay * bx : ax * bx + ay * by) / n;
		var dax = (cross ? by : bx) / n - f * ax / aa, day = (cross ? -bx : by) / n - f * ay / aa;
		var dbx = (cross ? -ay : ax) / n - f * bx / bb, dby = (cross ? ax : ay) / n - f * by / bb;
		w.entry(r, a1, dax); w.entry(r, a1 + 1, day); w.entry(r, a0, -dax); w.entry(r, a0 + 1, -day);
		w.entry(r, b1, dbx); w.entry(r, b1 + 1, dby); w.entry(r, b0, -dbx); w.entry(r, b0 + 1, -dby);
	}

	/**
		Signed distance of point p from a line through e0 along d = e1 − e0:
		f = (p − e0)×d / |d|. `pointWeight` scales ∂f/∂p = (dy, −dx)/|d|
		(a midpoint gives each end half). With `axisWeight` 1 the line's own
		terms are added here; symmetric rows add them once via `pointLineAxisTerms`.
	*/
	function pointLineRow(w:SketchRowWriter, x:Array<Float>, p:Int, pointWeight:Float, axisWeight:Float, lineId:String, r:Int,
			owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		w.entry(r, p, pointWeight * dy / length); w.entry(r, p + 1, -pointWeight * dx / length);
		if (axisWeight != 0)
			pointLineAxisTerms(w, x, [x[p], x[p + 1]], lineId, r, owner);
	}

	/** The line's part of ∂f/∂(e0, e1) for f = (p − e0)×d/|d| at a fixed point p. */
	function pointLineAxisTerms(w:SketchRowWriter, x:Array<Float>, p:Array<Float>, lineId:String, r:Int, owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		var wx = p[0] - x[e0], wy = p[1] - x[e0 + 1];
		var f = (wx * dy - wy * dx) / length;
		// ∂f/∂d, then d = e1 − e0; w = p − e0 adds −(dy, −dx)/|d| to e0.
		var ddx = -wy / length - f * dx / dd, ddy = wx / length - f * dy / dd;
		w.entry(r, e1, ddx); w.entry(r, e1 + 1, ddy);
		w.entry(r, e0, -ddx - dy / length); w.entry(r, e0 + 1, -ddy + dx / length);
	}

	/** f = (b − a)·d / |d| for the axis d = e1 − e0. */
	function alongRow(w:SketchRowWriter, x:Array<Float>, a:Int, b:Int, lineId:String, r:Int, owner:String):Void {
		var e = entity(lineId, owner), e0 = pointVar(e.first), e1 = pointVar(e.second);
		var dx = x[e1] - x[e0], dy = x[e1 + 1] - x[e0 + 1], dd = dx * dx + dy * dy, length = Math.sqrt(dd);
		if (length < 1e-12) return;
		var vx = x[b] - x[a], vy = x[b + 1] - x[a + 1], f = (vx * dx + vy * dy) / length;
		w.entry(r, b, dx / length); w.entry(r, b + 1, dy / length); w.entry(r, a, -dx / length); w.entry(r, a + 1, -dy / length);
		var ddx = vx / length - f * dx / dd, ddy = vy / length - f * dy / dd;
		w.entry(r, e1, ddx); w.entry(r, e1 + 1, ddy); w.entry(r, e0, -ddx); w.entry(r, e0 + 1, -ddy);
	}

	/** Tangency rows, following the stored side (line-circle) or contact branch (circle-circle). */
	function tangentRow(w:SketchRowWriter, x:Array<Float>, aid:String, bid:String, owner:String):Void {
		var first = entity(aid, owner), second = entity(bid, owner);
		if (first.kind == "line" || second.kind == "line") {
			var lineId = first.kind == "line" ? aid : bid, circleId = first.kind == "line" ? bid : aid;
			var circle = entity(circleId, owner);
			pointLineRow(w, x, pointVar(circle.first), 1, 1, lineId, 0, owner);
			var storedSide = tangentSides.get(owner);
			var side:Float = storedSide == null ? 1 : storedSide;
			if (storedSide == null) {
				var e = entity(lineId, owner), c = pointVar(circle.first), e0 = pointVar(e.first), e1 = pointVar(e.second);
				var signed = (x[c] - x[e0]) * (x[e1 + 1] - x[e0 + 1]) - (x[c + 1] - x[e0 + 1]) * (x[e1] - x[e0]);
				side = signed < 0 ? -1 : 1;
			}
			w.entry(0, radiusVar(circleId), -side);
		} else {
			distanceRow(w, x, pointVar(first.first), pointVar(second.first), 1);
			var storedBranch = tangentBranches.get(owner);
			var external:Bool = storedBranch == null ? true : storedBranch;
			var ra = radiusVar(aid), rb = radiusVar(bid);
			if (external) {
				w.entry(0, ra, -1); w.entry(0, rb, -1);
			} else {
				var sign = x[ra] - x[rb] >= 0 ? 1.0 : -1.0;
				w.entry(0, ra, -sign); w.entry(0, rb, sign);
			}
		}
	}

	static function vectorDifference(a:Array<Float>,b:Array<Float>,out:Array<Float>):Void{out.push(a[0]-b[0]);out.push(a[1]-b[1]);}
	static function distance(a:Array<Float>,b:Array<Float>):Float{var x=a[0]-b[0],y=a[1]-b[1];return Math.pow(x*x+y*y,0.5);}
	static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1];
	static function cross(a:Array<Float>,b:Array<Float>):Float return a[0]*b[1]-a[1]*b[0];
}
