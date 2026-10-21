package materia.sheet;

/** Authored cutting layout, separate from physical inventory and execution history. */
typedef SheetCutPlan = {
	var schemaVersion:Int;
	var id:String;
	var revision:Int;
	var stockSpecId:String;
	var stockSpecRevision:Int;
	var stockSpecFingerprint:String;
	var lengthUnit:String;
	var kerf:Float;
	var edgeMargin:Float;
	var requirements:Array<SheetRequirementReference>;
	var placements:Array<SheetPartPlacement>;
	var cuts:Array<SheetCut>;
	/** Leaf regions explicitly retained as reusable stock; other unplaced leaves are waste. */
	var retainedRegionIds:Array<String>;
	/** Version 2 is a free-form layout, with no invented guillotine cuts/leaf regions. */
	@:optional var layout:String;
	@:optional var batchId:String;
	@:optional var sheetIndex:Int;
	@:optional var nestGap:Float;
	@:optional var freePlacements:Array<materia.sheet.SheetPolygon.SheetShapePlacement>;
	@:optional var remnants:Array<materia.sheet.SheetPolygon.SheetRemnant>;
}
