import ConstrainedSlottedBracket;
// Root same-package modules needed by this standalone Haxeon compilation entry.
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.features.ConstrainedSketchSupportFaceChange;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.ProfileError;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSession;

/** Repeatable timing probe for sketch solve, document evaluation, and viewport tessellation. */
class SketchEditBenchmark {
	static inline var WARMUP:Int = 5;
	static inline var SAMPLES:Int = 25;

	static function setDimension(sketch:ConstrainedSketch, id:String, value:Float):Void {
		for (constraint in sketch.constraints()) {
			if (constraint.id == id) {
				sketch.replaceConstraint(SketchConstraint.raw(constraint.id, constraint.kind, constraint.first,
					constraint.second, constraint.third, value));
				return;
			}
		}
		throw "unknown benchmark dimension: " + id;
	}

	static function summarize(label:String, values:Array<Float>):Void {
		values.sort(function(first, second) return first < second ? -1 : first > second ? 1 : 0);
		var p50 = values[Std.int((values.length - 1) * 0.50)];
		var p95 = values[Std.int((values.length - 1) * 0.95)];
		Sys.println(label + "_ms p50=" + milliseconds(p50) + " p95=" + milliseconds(p95)
			+ " max=" + milliseconds(values[values.length - 1]));
	}

	static function milliseconds(seconds:Float):String
		return Std.string(Math.round(seconds * 100000) / 100);

	public static function main():Void {
		var model = new ConstrainedSlottedBracket();
		var authored = model.profile.sketch();
		var session = new SketchSession(authored);
		var solveTimes:Array<Float> = [];
		for (index in 0...(WARMUP + SAMPLES)) {
			var width = 80 + (index % 7) * 0.25;
			var started = Sys.time();
			if (!session.edit(function(draft) setDimension(draft, "body.width", width)))
				throw "benchmark sketch edit did not converge";
			var elapsed = Sys.time() - started;
			if (index >= WARMUP)
				solveTimes.push(elapsed);
		}

		var recomputeTimes:Array<Float> = [];
		var documentSolveTimes:Array<Float> = [];
		var profileTimes:Array<Float> = [];
		var fineTessellationTimes:Array<Float> = [];
		var previewTessellationTimes:Array<Float> = [];
		for (index in 0...(WARMUP + SAMPLES)) {
			model.document.parameter("bracket.width").set(80 + (index % 7) * 0.25);
			model.document.recompute();
			if (index < WARMUP)
				continue;
			recomputeTimes.push(model.document.lastRecomputeSeconds);
			documentSolveTimes.push(model.document.lastSketchSolveSeconds);
			profileTimes.push(model.document.lastSketchProfileSeconds);
			var shape = model.finish.currentShape();
			var started = Sys.time();
			shape.tessellate(0.1, 0.35);
			fineTessellationTimes.push(Sys.time() - started);
			started = Sys.time();
			shape.tessellate(0.5, 0.75);
			previewTessellationTimes.push(Sys.time() - started);
		}

		var solved = session.solution;
		var cancellationChecks = 0;
		var cancellationStarted = Sys.time();
		var cancelled = false;
		try {
			authored.solve(null, function() {
				cancellationChecks++;
				return cancellationChecks >= 8;
			});
		} catch (_:EvaluationCancelled) {
			cancelled = true;
		}
		if (!cancelled)
			throw "solver did not stop at the cancellation probe";
		var variableCount = authored.points().length * 2;
		for (entity in authored.entities())
			if (entity.kind == "circle" || entity.kind == "arc")
				variableCount++;
		Sys.println("fixture=ConstrainedSlottedBracket points=" + authored.points().length
			+ " entities=" + authored.entities().length + " constraints=" + authored.constraints().length
			+ " variables=" + variableCount
			+ " samples=" + SAMPLES + " warmup=" + WARMUP
			+ " iterations=" + (solved == null ? -1 : solved.diagnostic.iterations)
			+ " diagnostic=" + (solved == null ? "none" : solved.diagnostic.status));
		summarize("solve_only", solveTimes);
		summarize("document_solve", documentSolveTimes);
		summarize("profile_build", profileTimes);
		summarize("document_recompute", recomputeTimes);
		summarize("viewport_tessellation_0_1_0_35", fineTessellationTimes);
		summarize("preview_tessellation_0_5_0_75", previewTessellationTimes);
		Sys.println("solver_cancelled=" + cancelled + " callback_checks=" + cancellationChecks
			+ " cancel_ms=" + milliseconds(Sys.time() - cancellationStarted));
		model.close();
		scaling();
	}

	/**
		Solve cost against sketch size: rectangles of 4 points each, either
		independent (each with its own fixed corner: many parts) or chained
		(each shares a corner with the last: one part). Cold is a solve from
		the authored pose; edit changes one width and re-solves from the
		previous solution.
	*/
	static function scaling():Void {
		var cases:Array<{points:Int, chained:Bool, redundant:Bool}> = [];
		for (points in [48, 200, 1000]) {
			cases.push({points: points, chained: false, redundant: false});
			cases.push({points: points, chained: true, redundant: false});
		}
		// One implied constraint in a connected sketch: diagnosis must find and explain the dependency.
		cases.push({points: 200, chained: true, redundant: true});
		cases.push({points: 1000, chained: true, redundant: true});
		// The same redundant sketch through the dense QR: a rank tolerance below 1e-6 keeps dependencies off the sparse path.
		var dense = rectangles(250, true, 10, new cadkit.sketch.SolverSettings(1e-8, 1e-7));
		dense.addConstraint(SketchConstraint.distance('r125.top-length', 'r125.p2', 'r125.p3', 10));
		var denseStarted = Sys.time();
		var denseSolved = dense.solve();
		Sys.println("scaling chained-redundant-dense-qr points=1000 cold_ms=" + milliseconds(Sys.time() - denseStarted)
			+ " diagnostic=" + denseSolved.diagnostic.status);
		for (entry in cases) {
				var points = entry.points, chained = entry.chained;
				var sketch = rectangles(Std.int(points / 4), chained, 10);
				if (entry.redundant) {
					var middle = Std.int(points / 8);
					sketch.addConstraint(SketchConstraint.distance('r$middle.top-length', 'r$middle.p2', 'r$middle.p3', 10));
				}
				var started = Sys.time();
				var solved = sketch.solve();
				var cold = Sys.time() - started;
				setDimension(sketch, "r0.width", 10.5);
				started = Sys.time();
				var edited = sketch.solve(solved);
				var edit = Sys.time() - started;
				// A drag: successive edits, each seeded from the last solution.
				var drag:Array<Float> = [], previous = edited;
				for (step in 0...20) {
					setDimension(sketch, "r0.width", 10.5 + 0.01 * (step + 1));
					started = Sys.time();
					previous = sketch.solve(previous);
					drag.push(Sys.time() - started);
				}
				drag.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
				Sys.println("scaling " + (chained ? "chained" : "independent") + (entry.redundant ? "-redundant" : "") + " points=" + points
					+ " cold_ms=" + milliseconds(cold) + " edit_ms=" + milliseconds(edit) + " drag_p50_ms=" + milliseconds(drag[10])
					+ " iterations=" + edited.diagnostic.iterations + " diagnostic=" + edited.diagnostic.status
					+ " subsystems=" + (edited.diagnostic.report == null ? 0 : edited.diagnostic.report.subsystems.length));
			}
	}

	static function rectangles(count:Int, chained:Bool, width:Float, ?settings:cadkit.sketch.SolverSettings):ConstrainedSketch {
		var sketch = new ConstrainedSketch(null, "mm", settings);
		for (r in 0...count) {
			var p = [for (i in 0...4) 'r$r.p$i'], x = r * (width + (chained ? 0 : 5)), y = 0.0;
			sketch.addPoint(new SketchPoint(p[0], x + 0.1, y - 0.1)).addPoint(new SketchPoint(p[1], x + width - 0.2, y + 0.2))
				.addPoint(new SketchPoint(p[2], x + width + 0.3, y + 5.1)).addPoint(new SketchPoint(p[3], x - 0.2, y + 4.9));
			sketch.addEntity(SketchEntity.line('r$r.bottom', p[0], p[1])).addEntity(SketchEntity.line('r$r.right', p[1], p[2]))
				.addEntity(SketchEntity.line('r$r.top', p[2], p[3])).addEntity(SketchEntity.line('r$r.left', p[3], p[0]));
			if (chained && r > 0)
				sketch.addConstraint(SketchConstraint.coincident('r$r.joint', p[0], 'r${r - 1}.p1'));
			else
				sketch.addConstraint(SketchConstraint.fixed('r$r.origin', p[0]));
			sketch.addConstraint(SketchConstraint.horizontal('r$r.h0', 'r$r.bottom'))
				.addConstraint(SketchConstraint.horizontal('r$r.h1', 'r$r.top'))
				.addConstraint(SketchConstraint.vertical('r$r.v0', 'r$r.right'))
				.addConstraint(SketchConstraint.vertical('r$r.v1', 'r$r.left'))
				.addConstraint(SketchConstraint.distance('r$r.width', p[0], p[1], width))
				.addConstraint(SketchConstraint.distance('r$r.height', p[1], p[2], 5));
		}
		return sketch;
	}
}
