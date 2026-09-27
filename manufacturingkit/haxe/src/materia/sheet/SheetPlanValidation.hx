package materia.sheet;

typedef SheetPlanValidation = {
	var valid:Bool;
	var messages:Array<String>;
	var regions:Array<SheetCutRegion>;
	var cutLines:Array<SheetCutLine>;
	var kerfLossAreaMm2:Float;
	var marginWasteAreaMm2:Float;
	var discardedAreaMm2:Float;
	var retainedAreaMm2:Float;
	var partAreaMm2:Float;
	var sourceAreaMm2:Float;
	var areaBalanceErrorMm2:Float;
}
