package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;

/** A rip names a permanent bend identity and cuts its child-side seam without removing strip material. */
class SheetRipFeature extends SheetFeature {
	public final source:SheetFeature;
	public final bend:Int;
	public function new(source:SheetFeature, bend:Int) {
		super();
		if (bend <= 0) throw new SheetDiagnosis("sheet.missing-bend", "A rip requires a positive bend identity");
		this.source = source; this.bend = bend;
	}
	override public function identityLedger():SheetIdentityLedger { return source.identityLedger(); }
	override public function dependencies():Array<FeatureId> { return [source.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source]; }
	override public function serializationType():String { return "sheet-rip"; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context); var cut = graph.rip(bend); return graph;
	}
}
