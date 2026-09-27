package materia.sheet;

typedef SheetBlankOutput = {
	var id:String;
	var placementId:String;
	var requirementId:String;
	var partId:String;
	var partRevision:Int;
	var sourcePieceId:String;
	var sourceStockSpecId:String;
	var widthMm:Float;
	var heightMm:Float;
	var thicknessMm:Float;
	var materialId:String;
}

/** Immutable audit record of one confirmed cut operation. Areas are in mm². */
typedef SheetCutExecution = {
	var id:String;
	var planId:String;
	var planRevision:Int;
	var planFingerprint:String;
	var consumedPieceId:String;
	var blanks:Array<SheetBlankOutput>;
	var remnantPieceIds:Array<String>;
	var kerfLossAreaMm2:Float;
	var marginWasteAreaMm2:Float;
	var discardedAreaMm2:Float;
	var completedAtUnixMilliseconds:Float;
}
