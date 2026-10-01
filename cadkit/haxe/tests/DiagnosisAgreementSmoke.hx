import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolver;
import cadkit.sketch.SolvedSketch;
import cadkit.solve.ConstraintDiagnosis;

/**
	The sparse diagnosis (Gram Cholesky with dropped rows) and the dense one
	(column-pivoted QR) must agree on real systems: random sketches with
	implied, duplicate and conflicting constraints, at their authored pose and,
	when they solve, at their solution.
*/
class DiagnosisAgreementSmoke {
	static var seed = 424242;

	public static function run():Void {
		var compared = 0, withGroups = 0;
		for (trial in 0...40) {
			var sketch = randomSketch(trial);
			var probe = SketchSolver.probe(sketch);
			var poses = [probe.variables];
			var solved:Null<SolvedSketch> = null;
			try solved = sketch.solve() catch (_:Dynamic) {}
			if (solved != null) poses.push(solvedVariables(sketch, solved));
			for (pose in poses) {
				var residuals = [for (value in probe.residuals(pose)) value / probe.satisfiedWithin];
				var jacobian = probe.jacobian(pose), variables = pose.length, rows = probe.owners.length;
				var dense = ConstraintDiagnosis.diagnose({jacobian: jacobian, variables: variables, owners: probe.owners,
					residuals: residuals, rankTolerance: ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE});
				var sparseRows = [for (row in 0...rows) {
					var index:Array<Int> = [], value:Array<Float> = [];
					for (column in 0...variables)
						if (jacobian[row * variables + column] != 0) { index.push(column); value.push(jacobian[row * variables + column]); }
					{index: index, value: value};
				}];
				var sparse = ConstraintDiagnosis.diagnoseSparse(sparseRows, variables, probe.owners, residuals,
					ConstraintDiagnosis.DEFAULT_RANK_TOLERANCE);
				if (sparse.rank != dense.rank || groups(sparse) != groups(dense) || sparse.unsatisfied.join(",") != dense.unsatisfied.join(",")
					|| sparse.subsystems.length != dense.subsystems.length)
					throw 'trial $trial: sparse rank ${sparse.rank} ${groups(sparse)} unsatisfied ${sparse.unsatisfied}, '
						+ 'dense rank ${dense.rank} ${groups(dense)} unsatisfied ${dense.unsatisfied}';
				compared++;
				if (dense.dependencyGroups.length > 0) withGroups++;
			}
		}
		if (withGroups < 10) throw 'too few random systems had dependencies to compare ($withGroups of $compared)';
	}

	static function groups(report:DiagnosisReport):String
		return [for (group in report.dependencyGroups) group.toString()].join(" ");

	static function solvedVariables(sketch:ConstrainedSketch, solved:SolvedSketch):Array<Float> {
		var x:Array<Float> = [];
		for (point in sketch.points()) { x.push(solved.x(point.id)); x.push(solved.y(point.id)); }
		for (entity in sketch.entities()) if (entity.kind == "circle" || entity.kind == "arc") x.push(solved.radius(entity.id));
		return x;
	}

	static function random():Float {
		seed = (seed * 1103515245 + 12345) & 0x7fffffff;
		return seed / 0x7fffffff;
	}

	/** Rectangles and triangles, some chained by a shared corner, then random extra constraints. */
	static function randomSketch(trial:Int):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var shapes = 1 + Std.int(random() * 4), previous:Null<String> = null;
		var widths:Array<{shape:String, a:String, b:String, top:String, topEnd:String, length:Float}> = [];
		for (k in 0...shapes) {
			var prefix = 's$k.', x0 = k * 25.0 + random(), y0 = random() * 3;
			if (random() < 0.6) {
				var w = 5 + random() * 10, h = 3 + random() * 6;
				var p = [for (i in 0...4) prefix + "p" + i];
				sketch.addPoint(new SketchPoint(p[0], x0, y0)).addPoint(new SketchPoint(p[1], x0 + w + random(), y0 + random() * 0.3))
					.addPoint(new SketchPoint(p[2], x0 + w + random() * 0.3, y0 + h)).addPoint(new SketchPoint(p[3], x0 - random() * 0.3, y0 + h + random() * 0.3));
				sketch.addEntity(SketchEntity.line(prefix + "bottom", p[0], p[1])).addEntity(SketchEntity.line(prefix + "right", p[1], p[2]))
					.addEntity(SketchEntity.line(prefix + "top", p[2], p[3])).addEntity(SketchEntity.line(prefix + "left", p[3], p[0]));
				sketch.addConstraint(SketchConstraint.horizontal(prefix + "h0", prefix + "bottom"))
					.addConstraint(SketchConstraint.horizontal(prefix + "h1", prefix + "top"))
					.addConstraint(SketchConstraint.vertical(prefix + "v0", prefix + "right"))
					.addConstraint(SketchConstraint.vertical(prefix + "v1", prefix + "left"))
					.addConstraint(SketchConstraint.distance(prefix + "width", p[0], p[1], w))
					.addConstraint(SketchConstraint.distance(prefix + "height", p[1], p[2], h));
				anchor(sketch, prefix, p[0], previous);
				widths.push({shape: prefix, a: p[0], b: p[1], top: p[2], topEnd: p[3], length: w});
				previous = p[1];
			} else {
				var p = [for (i in 0...3) prefix + "p" + i];
				sketch.addPoint(new SketchPoint(p[0], x0, y0)).addPoint(new SketchPoint(p[1], x0 + 8 + random(), y0 + random()))
					.addPoint(new SketchPoint(p[2], x0 + 3 + random(), y0 + 5 + random()));
				sketch.addEntity(SketchEntity.line(prefix + "base", p[0], p[1]));
				sketch.addConstraint(SketchConstraint.horizontal(prefix + "level", prefix + "base"))
					.addConstraint(SketchConstraint.distance(prefix + "ab", p[0], p[1], 8))
					.addConstraint(SketchConstraint.distance(prefix + "ac", p[0], p[2], 6))
					.addConstraint(SketchConstraint.distance(prefix + "bc", p[1], p[2], 5.5));
				anchor(sketch, prefix, p[0], previous);
				previous = p[1];
			}
		}
		// Extra constraints: implied or duplicated (redundant), or contradicting (conflicting).
		for (extra in 0...Std.int(random() * 3)) {
			if (widths.length == 0) break;
			var target = widths[Std.int(random() * widths.length)], id = target.shape + "extra" + extra, roll = random();
			if (roll < 0.35) sketch.addConstraint(SketchConstraint.distance(id, target.top, target.topEnd, target.length));
			else if (roll < 0.55) sketch.addConstraint(SketchConstraint.distance(id, target.a, target.b, target.length));
			else if (roll < 0.75) sketch.addConstraint(SketchConstraint.parallel(id, target.shape + "bottom", target.shape + "top"));
			else sketch.addConstraint(SketchConstraint.distance(id, target.top, target.topEnd, target.length + 1 + random()));
		}
		return sketch;
	}

	/** The first shape is fixed at its corner; later ones either share the previous shape's corner or get their own fix. */
	static function anchor(sketch:ConstrainedSketch, prefix:String, corner:String, previous:Null<String>):Void {
		if (previous != null && random() < 0.5) sketch.addConstraint(SketchConstraint.coincident(prefix + "joint", corner, previous));
		else sketch.addConstraint(SketchConstraint.fixed(prefix + "origin", corner));
	}
}
