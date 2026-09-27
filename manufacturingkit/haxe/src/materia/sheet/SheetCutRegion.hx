package materia.sheet;

typedef SheetCutRegion = {
	var id:String;
	var xMm:Float;
	var yMm:Float;
	var widthMm:Float;
	var heightMm:Float;
	var retained:Bool;
	@:optional var placementId:String;
}
