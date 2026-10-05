package cadkit.sheetmetal;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.features.ConstrainedSketchFeature;
import materia.project.MaterialDef.MaterialMechanical;

/** Base sheet from one validated constrained-sketch region; the sketch can describe either sheet surface. */
class SheetBaseFeature extends SheetFeature {
	public final source:ConstrainedSketchFeature;
	public final thickness:Parameter;
	public final side:SheetSide;
	public final materialId:String;
	public final bendRule:Null<BendRule>;
	public final mechanical:Null<MaterialMechanical>;
	public final defaultRadiusRatio:Null<Float>;
	public final minRadiusRatio:Null<Float>;
	public final grainAngle:Null<Float>;
	public function new(source:ConstrainedSketchFeature, stock:SheetMaterial, side:SheetSide = A, ?identities:SheetIdentityLedger) {
		super(identities);
		this.source = source;
		this.side = side;
		materialId = stock.id;
		bendRule = stock.bendRule;
		mechanical = stock.mechanical;
		defaultRadiusRatio = stock.defaultRadiusRatio;
		minRadiusRatio = stock.minRadiusRatio;
		grainAngle = stock.grainAngle;
		thickness = new Parameter(this, "sheet.thickness", stock.thickness, 0, false, 1e300, ParameterKind.Length);
	}
	public function stock():SheetMaterial {
		return new SheetMaterial(materialId, thickness.value, bendRule, mechanical, defaultRadiusRatio, minRadiusRatio, grainAngle);
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
		var plane = side == SheetSide.A ? geometry.plane : geometry.plane.offset(thickness.value);
		var graph = new Sheet(stock(), plane.location());
		var wall = graph.addWall(SheetProfile.region(regions[0], geometry.plane, geometry.plane));
		return graph;
	}
}
