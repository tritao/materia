package cadkit.sheetmetal.process;

/** Authored tooling and measurements only; predictions are derived on every rebuild. */
@:wire typedef ProcessRuleRecord = {
	@:id(1) var tooling:ProcessToolingRecord;
	@:id(2) var lot:Null<String>;
	@:id(3) var calibration:Array<ProcessCalibrationRecord>;
}

@:wire typedef ProcessToolingRecord = {
	@:id(1) var punchId:String;
	@:id(2) var dieId:String;
	@:id(3) var punchRadius:Float;
	@:id(4) var punchAngle:Float;
	@:id(5) var vWidth:Float;
	@:id(6) var dieAngle:Float;
	@:id(7) var shoulderRadius:Float;
}

@:wire typedef ProcessCalibrationRecord = {
	@:id(1) var lot:String;
	@:id(2) var thickness:Float;
	@:id(3) var punchId:String;
	@:id(4) var dieId:String;
	@:id(5) var firstAngle:Float;
	@:id(6) var lastAngle:Float;
	@:id(7) var depthOffset:Float;
	@:id(8) var innerRadius:Float;
}
