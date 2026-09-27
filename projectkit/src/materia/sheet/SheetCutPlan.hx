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
}
