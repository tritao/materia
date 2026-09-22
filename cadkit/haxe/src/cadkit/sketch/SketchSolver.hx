package cadkit.sketch;

private typedef ResidualSet = { values:Array<Float>, owners:Array<String> };

/** Deterministic damped nonlinear least-squares solver implemented entirely in Haxeon. */
class SketchSolver {
	private final sketch:ConstrainedSketch;
	private final pointIndex:Map<String, Int>;
	private final radiusIndex:Map<String, Int>;
	private final points:Map<String, SketchPoint>;
	private final entities:Map<String, SketchEntity>;
	private var variableCount:Int;

	private function new(sketch:ConstrainedSketch) {
		this.sketch = sketch; pointIndex = new Map(); radiusIndex = new Map(); points = new Map(); entities = new Map();
		variableCount = 0;
		for (point in sketch.points()) { points.set(point.id, point); pointIndex.set(point.id, variableCount); variableCount += 2; }
		for (entity in sketch.entities()) {
			entities.set(entity.id, entity);
			if (entity.kind == "circle" || entity.kind == "arc") { radiusIndex.set(entity.id, variableCount); variableCount++; }
		}
	}

	public static function solve(sketch:ConstrainedSketch):SolvedSketch {
		return new SketchSolver(sketch).run();
	}

	private function invalid(message:String, ids:Array<String>):SketchSolveError
		return new SketchSolveError(new SolveDiagnostic("invalid", false, 1e300, variableCount, 0, ids, message));

	private function run():SolvedSketch {
		validate();
		var x:Array<Float> = [];
		for (point in sketch.points()) { x.push(point.x); x.push(point.y); }
		for (entity in sketch.entities()) if (entity.kind == "circle" || entity.kind == "arc") x.push(entity.radius);
		var damping = sketch.settings.initialDamping;
		var current = residuals(x);
		var currentNorm = norm(current.values);
		var iterations = 0;
		while (iterations < sketch.settings.maxIterations && currentNorm > sketch.settings.tolerance) {
			iterations++;
			var j = jacobian(x, current.values);
			var normal = matrix(variableCount, variableCount, 0);
			var gradient = fill(variableCount, 0);
			for (row in 0...j.length) for (a in 0...variableCount) {
				gradient[a] += j[row][a] * current.values[row];
				for (b in 0...variableCount) normal[a][b] += j[row][a] * j[row][b];
			}
			for (i in 0...variableCount) normal[i][i] += damping;
			var rhs:Array<Float> = []; for (v in gradient) rhs.push(-v);
			var delta = linearSolve(normal, rhs);
			if (delta == null) { damping *= 10; continue; }
			var trial = x.copy(); for (i in 0...variableCount) trial[i] += delta[i];
			var trialSet = residuals(trial); var trialNorm = norm(trialSet.values);
			if (trialNorm < currentNorm) { x = trial; current = trialSet; currentNorm = trialNorm; damping = Math.max(1e-12, damping * 0.3); }
			else damping = Math.min(1e12, damping * 10);
		}
		var j = jacobian(x, current.values);
		var rankValue = rank(j, sketch.settings.rankTolerance);
		var dof = variableCount - rankValue;
		var badIds = failingOwners(current, sketch.settings.tolerance * 10);
		if (currentNorm > sketch.settings.tolerance) {
			var gradientNorm = norm(gradient(j, current.values));
			var stationaryLimit = Math.max(sketch.settings.rankTolerance, sketch.settings.tolerance * 10) * (1 + currentNorm);
			var locallyConflicting = gradientNorm <= stationaryLimit;
			var status = locallyConflicting ? "conflicting" : "nonconvergent";
			var message = locallyConflicting
				? "solve stopped at a locally stationary residual; the listed constraints are locally incompatible, which is not proof of global inconsistency"
				: "constraint solve exhausted its iteration limit while a local descent direction remained";
			var diagnostic = new SolveDiagnostic(status, false, currentNorm, dof, iterations, badIds, message);
			throw new SketchSolveError(diagnostic);
		}
		var redundant = redundantIds(x, j, rankValue);
		var status = redundant.length > 0 ? "redundant" : (dof == 0 ? "fully-constrained" : "under-constrained");
		var ids = redundant.length > 0 ? redundant : [];
		var diagnostic = new SolveDiagnostic(status, true, currentNorm, dof, iterations, ids,
			status == "redundant" ? "solution converged with locally redundant constraints" : "solution converged");
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
		for (constraint in sketch.constraints()) {
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
		for (c in sketch.constraints()) {
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
		var a=entity(aid,owner), b=entity(bid,owner);
		if(a.kind=="line" && b.kind=="line") throw invalid("line-line tangency is unsupported",[owner]);
		if(a.kind=="line" || b.kind=="line") { var l=a.kind=="line"?aid:bid; var q=a.kind=="line"?bid:aid; var ends=line(l,x,owner); var d=direction(l,x,owner); var c=center(q,x,owner); out.push(Math.abs(cross([c[0]-ends[0][0],c[1]-ends[0][1]],d))/Math.pow(dot(d,d),0.5)-radius(q,x,owner)); }
		else { var separation=distance(center(aid,x,owner),center(bid,x,owner)); var ra=radius(aid,x,owner), rb=radius(bid,x,owner); var external=Math.abs(separation-(ra+rb)); var internal=Math.abs(separation-Math.abs(ra-rb)); out.push(external<internal?separation-(ra+rb):separation-Math.abs(ra-rb)); }
	}
	private function symmetry(pa:String,pb:String,axis:String,x:Array<Float>,owner:String,out:Array<Float>):Void {
		var a=point(pa,x), b=point(pb,x), ends=line(axis,x,owner), d=direction(axis,x,owner); var dd=dot(d,d); if(dd<1e-12) throw invalid("collapsed symmetry axis",[owner]);
		var mid=[(a[0]+b[0])/2,(a[1]+b[1])/2]; out.push(cross([mid[0]-ends[0][0],mid[1]-ends[0][1]],d)/Math.pow(dd,0.5)); out.push(dot([b[0]-a[0],b[1]-a[1]],d)/Math.pow(dd,0.5));
	}
	private function jacobian(x:Array<Float>, base:Array<Float>):Array<Array<Float>> { var j=matrix(base.length,variableCount,0); for(col in 0...variableCount){var h=1e-6*Math.max(1,Math.abs(x[col]));var t=x.copy();t[col]+=h;var r=residuals(t).values;for(row in 0...base.length)j[row][col]=(r[row]-base[row])/h;}return j; }
	private function gradient(j:Array<Array<Float>>, residual:Array<Float>):Array<Float> { var result=fill(variableCount,0);for(row in 0...j.length)for(column in 0...variableCount)result[column]+=j[row][column]*residual[row];return result; }
	private function redundantIds(x:Array<Float>, full:Array<Array<Float>>, fullRank:Int):Array<String> { var result:Array<String> = []; var set=residuals(x); for(c in sketch.constraints()){var reduced:Array<Array<Float>> = [];for(i in 0...full.length)if(set.owners[i]!=c.id)reduced.push(full[i]);if(rank(reduced,sketch.settings.rankTolerance)==fullRank)result.push(c.id);}return result; }
	private function failingOwners(set:ResidualSet,t:Float):Array<String>{var out:Array<String> = [];for(i in 0...set.values.length)if(Math.abs(set.values[i])>t&&!contains(out,set.owners[i]))out.push(set.owners[i]);return out;}
	private static function contains(a:Array<String>,v:String):Bool{for(x in a)if(x==v)return true;return false;}
	private static function vectorDifference(a:Array<Float>,b:Array<Float>,out:Array<Float>):Void{out.push(a[0]-b[0]);out.push(a[1]-b[1]);}
	private static function distance(a:Array<Float>,b:Array<Float>):Float{var x=a[0]-b[0],y=a[1]-b[1];return Math.pow(x*x+y*y,0.5);}
	private static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1];
	private static function cross(a:Array<Float>,b:Array<Float>):Float return a[0]*b[1]-a[1]*b[0];
	private static function wrap(v:Float):Float{while(v>Math.PI)v-=2*Math.PI;while(v< -Math.PI)v+=2*Math.PI;return v;}
	private static function norm(v:Array<Float>):Float{var s=0.0;for(x in v)s+=x*x;return Math.pow(s,0.5);}
	private static function fill(n:Int,value:Float):Array<Float>{var a:Array<Float> = [];for(_ in 0...n)a.push(value);return a;}
	private static function matrix(r:Int,c:Int,value:Float):Array<Array<Float>>{var m:Array<Array<Float>> = [];for(_ in 0...r)m.push(fill(c,value));return m;}
	private static function linearSolve(a:Array<Array<Float>>,b:Array<Float>):Null<Array<Float>>{var n=b.length,m:Array<Array<Float>> = [];for(i in 0...n){m.push(a[i].copy());m[i].push(b[i]);}for(k in 0...n){var p=k;for(i in k...n)if(Math.abs(m[i][k])>Math.abs(m[p][k]))p=i;if(Math.abs(m[p][k])<1e-15)return null;var tmp=m[k];m[k]=m[p];m[p]=tmp;for(i in (k+1)...n){var f=m[i][k]/m[k][k];for(j in k...(n+1))m[i][j]-=f*m[k][j];}}var x=fill(n,0);var i=n-1;while(i >= 0){var s=m[i][n];for(j in (i+1)...n)s-=m[i][j]*x[j];x[i]=s/m[i][i];i--;}return x;}
	private static function rank(input:Array<Array<Float>>,tol:Float):Int{if(input.length==0)return 0;var a:Array<Array<Float>> = [];for(r in input)a.push(r.copy());var rows=a.length,cols=a[0].length,row=0,col=0;while(row<rows&&col<cols){var p=row;for(i in row...rows)if(Math.abs(a[i][col])>Math.abs(a[p][col]))p=i;if(Math.abs(a[p][col])<=tol){col++;continue;}var tmp=a[row];a[row]=a[p];a[p]=tmp;var pivot=a[row][col];for(j in col...cols)a[row][j]/=pivot;for(i in 0...rows)if(i!=row){var f=a[i][col];for(j in col...cols)a[i][j]-=f*a[row][j];}row++;col++;}return row;}
}
