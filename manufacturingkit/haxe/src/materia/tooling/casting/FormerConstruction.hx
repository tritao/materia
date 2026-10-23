package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingSection.CastingSectionBand;
import materia.tooling.panel.PanelStockSpec;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;

/** Board choices and cutter dimensions are recipe inputs, never planner defaults.
 * Lengths are mm. A zero taper is the ordinary straight router cutter.
 */
typedef FormerCutter = {
	var id:String;
	var tipDiameterMm:Float;
	var taperDegrees:Float;
	var fluteLengthMm:Float;
	var stickOutMm:Float;
}

typedef FormerPocketScrew = {
	var diameterMm:Float;
	var lengthMm:Float;
	var jigAngleDegrees:Float;
	var edgeDistanceMm:Float;
	var maximumPitchMm:Float;
}

typedef FormerRecipe = {
	var pocketScrew:FormerPocketScrew;
	var stocks:Array<PanelStockSpec>;
	var cutters:Array<FormerCutter>;
	/** Sacrificial radial stock for the rough contour; never a tolerance budget. */
	var finishingStockMm:Float;
	var toleranceMm:Float;
}

enum FormerBuild {
	Solid(stock:PanelStockSpec, facingDepthMm:Float, cutter:FormerCutter);
	PanelBox(stock:PanelStockSpec, panels:PanelAssemblyRecord, body:FormerBoxExtent);
	Unsupported(reason:String);
}

typedef FormerBoxExtent = {
	var pose:AssemblyFrame;
	var widthMm:Float;
	var depthMm:Float;
	var heightMm:Float;
}

typedef FormerSheetAttachment = {
	var id:String;
	var stock:PanelStockSpec;
	var region:materia.casting.CastingRegion;
	var pose:AssemblyFrame;
}

/** The envelope is the existing cavity-shaping former. Construction describes
 * its board material separately, including the open interior of a panel box.
 */
typedef FormerConstructionPlan = {
	var stockToMold:AssemblyFrame;
	var widthMm:Float;
	var depthMm:Float;
	var heightMm:Float;
	var build:FormerBuild;
	var attachments:Array<FormerSheetAttachment>;
	var airRegions:Array<materia.casting.CastingRegion>;
	var sealingRegions:Array<materia.casting.CastingRegion>;
}

/** Native section extraction supplies these neutral records. CamKit owns
 * compensation and consumes exactly these same line/arc contours.
 */
typedef FormerMachiningPlan = {
	var id:String;
	var stockToMold:AssemblyFrame;
	var widthMm:Float;
	var depthMm:Float;
	var heightMm:Float;
	var stock:PanelStockSpec;
	var bands:Array<CastingSectionBand>;
	var draftDegrees:Float;
	var cutter:FormerCutter;
	var roughingCutter:FormerCutter;
	var finishingStockMm:Float;
	var toleranceMm:Float;
}

class FormerConstruction {
	public static function recipeSnapshot(raw:Dynamic):FormerRecipe {
		materia.casting.CastingFeatureCodec.fields(raw,["stocks","cutters","finishingStockMm","toleranceMm","pocketScrew"]);
		var stocks=[for (row in materia.casting.CastingRecordCodec.array(raw,"stocks"))
			materia.tooling.ToolingCodec.stockSnapshot(row)];
		var cutters:Array<FormerCutter> = [],ids:Map<String,Bool> = [];
		for (row in materia.casting.CastingRecordCodec.array(raw,"cutters")) {
			materia.casting.CastingFeatureCodec.fields(row,["id","tipDiameterMm","taperDegrees","fluteLengthMm","stickOutMm"]);
			var tool:FormerCutter={id:materia.casting.CastingRecordCodec.string(row,"id"),
				tipDiameterMm:materia.casting.CastingRecordCodec.number(row,"tipDiameterMm"),
				taperDegrees:materia.casting.CastingRecordCodec.number(row,"taperDegrees"),
				fluteLengthMm:materia.casting.CastingRecordCodec.number(row,"fluteLengthMm"),
				stickOutMm:materia.casting.CastingRecordCodec.number(row,"stickOutMm")};
			materia.casting.CastingFeatureCodec.identifier(tool.id);
			if (ids.exists(tool.id) || tool.tipDiameterMm<=0 || tool.taperDegrees<0 || tool.taperDegrees>=90
				|| tool.fluteLengthMm<=0 || tool.stickOutMm<tool.fluteLengthMm)
				throw "Invalid former cutter: "+tool.id;
			if (cutters.filter(existing -> Math.abs(existing.taperDegrees-tool.taperDegrees)<1e-7).length>0)
				throw "Ambiguous former cutter for draft "+tool.taperDegrees;
			ids.set(tool.id,true);cutters.push(tool);
		}
		if (stocks.length==0 || stocks.length>16 || cutters.length==0 || cutters.length>32)
			throw "Invalid former stock/cutter count";
		stocks.sort((a,b) -> Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b)));
		for (index in 1...stocks.length)
			if (materia.tooling.ToolingCodec.canonical(stocks[index-1])==materia.tooling.ToolingCodec.canonical(stocks[index]))
				throw "Duplicate former stock";
		cutters.sort((a,b) -> Reflect.compare(a.id,b.id));
		var allowance=materia.casting.CastingRecordCodec.number(raw,"finishingStockMm");
		var tolerance=materia.casting.CastingRecordCodec.number(raw,"toleranceMm");
		if (allowance<0 || tolerance<=0) throw "Invalid former finishing stock/tolerance";
		var rawScrew=Reflect.field(raw,"pocketScrew");
		materia.casting.CastingFeatureCodec.fields(rawScrew,["diameterMm","lengthMm","jigAngleDegrees","edgeDistanceMm","maximumPitchMm"]);
		var screw:FormerPocketScrew={diameterMm:materia.casting.CastingRecordCodec.number(rawScrew,"diameterMm"),
			lengthMm:materia.casting.CastingRecordCodec.number(rawScrew,"lengthMm"),
			jigAngleDegrees:materia.casting.CastingRecordCodec.number(rawScrew,"jigAngleDegrees"),
			edgeDistanceMm:materia.casting.CastingRecordCodec.number(rawScrew,"edgeDistanceMm"),
			maximumPitchMm:materia.casting.CastingRecordCodec.number(rawScrew,"maximumPitchMm")};
		if (screw.diameterMm<=0 || screw.lengthMm<=screw.diameterMm || screw.jigAngleDegrees<=0 || screw.jigAngleDegrees>=45
			|| screw.edgeDistanceMm<2*screw.diameterMm || screw.maximumPitchMm<2*screw.edgeDistanceMm)
			throw "Invalid former pocket screw/jig choice";
		return {stocks:stocks,cutters:cutters,finishingStockMm:allowance,toleranceMm:tolerance,pocketScrew:screw};
	}

	public static function buildRecord(value:FormerBuild):Dynamic return switch value {
		case Solid(stock,facingDepth,cutter):{kind:"solid-board",stock:stock,facingDepthMm:facingDepth,cutter:cutter};
		case PanelBox(stock,panels,body):{kind:"panel-box",stock:stock,panels:panels,body:body};
		case Unsupported(reason):{kind:"unsupported",reason:reason};
	};

	/** Smallest sufficient declared sheet, independent of declaration order.
	 * Returning null selects the panel-box construction, never a laminated block.
	 */
	public static function solidStock(stocks:Array<PanelStockSpec>,heightMm:Float,widthMm:Float,depthMm:Float):Null<PanelStockSpec> {
		if (!Math.isFinite(heightMm) || heightMm<=0) throw "Invalid former height";
		var suitable=stocks.filter(stock -> stock.measuredThickness>=heightMm-1e-7
			&& ((stock.allowedRotations.indexOf(0)>=0 && widthMm<=stock.width && depthMm<=stock.height)
				|| (stock.allowedRotations.indexOf(90)>=0 && depthMm<=stock.width && widthMm<=stock.height)));
		suitable.sort((a,b) -> a.measuredThickness==b.measuredThickness
			? Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b))
			: Reflect.compare(a.measuredThickness,b.measuredThickness));
		return suitable.length==0 ? null : suitable[0];
	}

	public static function cutter(recipe:FormerRecipe,draftDegrees:Float,id:String):Null<FormerCutter> {
		var matching=recipe.cutters.filter(tool -> Math.abs(tool.taperDegrees-draftDegrees)<1e-7);
		return matching.length==0 ? null : matching[0];
	}
}
