package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;

/** Authored corner selections and dimensions rebuild one closed, relieved material graph. */
class SheetCornerFeature extends SheetFeature {
	public final source:SheetFeature;
	public final first:SheetCornerEdge;
	public final second:SheetCornerEdge;
	public final join:CornerJoin;
	public final relief:CornerRelief;
	public final gap:Parameter;
	public final overlap:Parameter;
	public final reliefSize:Parameter;
	public function new(source:SheetFeature, first:SheetCornerEdge, second:SheetCornerEdge, join:CornerJoin,
		gapValue:Float, overlapValue:Float, relief:CornerRelief, reliefSizeValue:Float) {
		super();
		this.source = source; this.first = first; this.second = second; this.join = join; this.relief = relief;
		gap = new Parameter(this, "sheet.corner.gap", gapValue, -1e300, false, 1e300, ParameterKind.Length);
		overlap = new Parameter(this, "sheet.corner.overlap", overlapValue, -1e300, false, 1e300, ParameterKind.Length);
		reliefSize = new Parameter(this, "sheet.corner.relief-size", reliefSizeValue, -1e300, false, 1e300, ParameterKind.Length);
	}
	override public function identityLedger():SheetIdentityLedger { return source.identityLedger(); }
	override public function dependencies():Array<FeatureId> { return [source.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source]; }
	override public function serializationType():String { return "sheet-corner"; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		return SheetCorner.apply(source.sheetForEvaluation(context), first, second, join, gap.value, overlap.value,
			relief, reliefSize.value, "feature:" + id.toInt() + ":corner");
	}
}
