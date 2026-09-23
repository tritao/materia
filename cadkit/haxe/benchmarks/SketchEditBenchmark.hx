import ConstrainedSlottedBracket;
// Root same-package modules needed by this standalone Haxeon compilation entry.
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.EvaluationCancelled;
import cadkit.parametric.features.ConstrainedSketchSupportFaceChange;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.ProfileError;
import cadkit.sketch.SketchConstraint;
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
	}
}
