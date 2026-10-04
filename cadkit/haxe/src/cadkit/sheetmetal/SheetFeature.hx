package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;

/** A document feature whose graph and folded solid publish together after successful recompute. */
class SheetFeature extends Feature {
	var committedSheet:Null<Sheet>;
	var stagedSheet:Null<Sheet>;
	var stagedContext:Null<EvaluationContext>;

	public function new() {
		super();
		committedSheet = null;
		stagedSheet = null;
		stagedContext = null;
	}

	/** An isolated copy of the last committed graph; edits cannot bypass document parameters. */
	public function sheet():Sheet {
		if (committedSheet == null)
			throw new SheetDiagnosis("sheet.feature-not-evaluated", "Sheet feature has no committed graph", ["feature:" + id.toInt()]);
		return committedSheet.copy();
	}

	/** Downstream features consume the graph corresponding to the context's staged or committed solid. */
	public function sheetForEvaluation(context:EvaluationContext):Sheet {
		if (document != context.owner())
			throw new SheetDiagnosis("sheet.feature-owner", "Sheet feature belongs to a different document", ["feature:" + id.toInt()]);
		var shape = context.shape(this);
		if (stagedContext == context && stagedSheet != null)
			return stagedSheet.copy();
		return sheet();
	}

	/** Concrete features derive a fresh graph from authored data and the previous sheet feature. */
	public function buildSheet(context:EvaluationContext):Sheet {
		throw new SheetDiagnosis("sheet.feature-no-builder", "Sheet feature has no graph evaluator", ["feature:" + id.toInt()]);
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var graph = buildSheet(context);
		var shape = SheetSolid.build(graph, graph.folded());
		stagedSheet = graph;
		stagedContext = context;
		return EvaluationResult.fromShape(shape);
	}

	override public function commitEvaluation():Void {
		committedSheet = stagedSheet;
		stagedSheet = null;
		stagedContext = null;
	}

	override public function discardEvaluation():Void {
		stagedSheet = null;
		stagedContext = null;
	}

	override public function onClose():Void {
		committedSheet = null;
		discardEvaluation();
	}
}
