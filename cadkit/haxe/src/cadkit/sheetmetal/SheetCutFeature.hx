package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.features.ConstrainedSketchFeature;

/** Sheet cuts consume the staged exact sketch profile on an authored wall and sheet side. */
class SheetCutFeature extends SheetFeature {
	public final source:SheetFeature;
	public final profile:ConstrainedSketchFeature;
	public final wall:Int;
	public final side:SheetSide;
	public function new(source:SheetFeature, profile:ConstrainedSketchFeature, wall:Int, side:SheetSide = A) {
		super();
		if (wall <= 0) throw new SheetDiagnosis("sheet.missing-wall", "A sheet cut requires a positive wall identity");
		this.source = source; this.profile = profile; this.wall = wall; this.side = side;
	}
	override public function identityLedger():SheetIdentityLedger { return source.identityLedger(); }
	override public function dependencies():Array<FeatureId> { return [source.id, profile.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source, profile]; }
	override public function serializationType():String { return "sheet-cut"; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context); var geometry = profile.profileForEvaluation(context);
		var placement = graph.evaluate(graph.folded()).wall(wall).plane;
		var offset = placement.toLocal(geometry.plane.origin).z;
		if (Math.abs(geometry.plane.normal.dot(placement.normal)) < 1 - 1e-9
			|| Math.abs(offset - (side == A ? 0 : -graph.thickness())) > 1e-7)
			throw new SheetDiagnosis("sheet.cut-plane", "Cut sketch does not lie on the selected wall side", ["wall:" + wall]);
		for (region in geometry.regions()) graph = SheetCutGeometry.apply(graph, wall, SheetProfile.region(region, geometry.plane, placement),
			"feature:" + id.toInt() + ":region:" + region.id);
		return graph;
	}
}
