package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ProfileCurve;
import cadkit.sheetmetal.SheetMaterial.SheetMechanical;

/** Base sheet from one validated constrained-sketch region; the sketch can describe either sheet surface. */
class SheetBaseFeature extends SheetFeature {
	public final source:ConstrainedSketchFeature;
	public final thickness:Parameter;
	public final side:SheetSide;
	public final materialId:String;
	public final bendRule:Null<BendRule>;
	public final mechanical:Null<SheetMechanical>;
	public function new(source:ConstrainedSketchFeature, stock:SheetMaterial, side:SheetSide = A) {
		super();
		this.source = source;
		this.side = side;
		materialId = stock.id;
		bendRule = stock.bendRule;
		mechanical = stock.mechanical;
		thickness = new Parameter(this, "sheet.thickness", stock.thickness, 0, false, 1e300, ParameterKind.Length);
	}
	public function stock():SheetMaterial {
		return new SheetMaterial(materialId, thickness.value, bendRule, mechanical);
	}
	override public function serializationType():String { return "sheet-base"; }
	override public function dependencies():Array<FeatureId> { return [source.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source]; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var geometry = source.profileForEvaluation(context);
		var regions = geometry.regions();
		if (regions.length != 1)
			throw new SheetDiagnosis("sheet.base-region-count", "A sheet base requires one connected region, got " + regions.length,
				["feature:" + source.id.toInt()]);
		var outer = loop(regions[0].outer(), false);
		var holes = [for (curves in regions[0].holes()) loop(curves, true)];
		var plane = side == SheetSide.A ? geometry.plane : geometry.plane.offset(thickness.value);
		var graph = new Sheet(stock(), plane.location());
		var identities = [outer.identities];
		for (hole in holes) identities.push(hole.identities);
		var wall = graph.addWall(new WallOutline(outer.segments, [for (hole in holes) hole.segments], identities));
		return graph;
	}
	static function loop(curves:Array<ProfileCurve>, clockwise:Bool):{segments:Array<Segment>, identities:Array<String>} {
		var result:Array<Segment> = [];
		var identities:Array<String> = [];
		var area = 0.0;
		for (curve in curves) {
			var segment:Segment;
			if (curve.center == null)
				segment = Segment.line(new Point2(curve.start.x, curve.start.y), new Point2(curve.end.x, curve.end.y));
			else {
				var center = curve.center;
				segment = Segment.arc(new Point2(center.x, center.y), curve.radius, curve.startAngle, curve.sweep);
			}
			result.push(segment);
			identities.push("sketch:" + curve.entityId);
			area += segment.area();
		}
		if ((area < 0) != clockwise) {
			var reversed:Array<Segment> = [];
			var index = result.length;
			while (index > 0) {
				var segment = result[--index];
				if (segment.center == null) reversed.push(Segment.line(segment.end, segment.start));
				else {
					var center = segment.center;
					reversed.push(Segment.arc(center, segment.radius, segment.startAngle + segment.sweep, -segment.sweep));
				}
			}
			identities.reverse();
			return {segments: reversed, identities: identities};
		}
		return {segments: result, identities: identities};
	}
}
