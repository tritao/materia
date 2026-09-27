package materia.sheet;

/** A designed part's rectangular cutting blank and acceptable stock. */
typedef SheetPartRequirement = {
	var id:String;
	var revision:Int;
	var partId:String;
	var partRevision:Int;
	var quantity:Int;
	var materialId:String;
	var lengthUnit:String;
	var thickness:Float;
	var thicknessToleranceMinus:Float;
	var thicknessTolerancePlus:Float;
	var blankWidth:Float;
	var blankHeight:Float;
	var widthToleranceMinus:Float;
	var widthTolerancePlus:Float;
	var heightToleranceMinus:Float;
	var heightTolerancePlus:Float;
	var allowedRotations:Array<Int>;
	/** `free` permits either axis; `width-aligned` keeps the part's width on stock X. */
	var directionConstraint:String;
}
