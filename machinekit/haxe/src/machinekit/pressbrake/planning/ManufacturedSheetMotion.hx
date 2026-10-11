package machinekit.pressbrake.planning;

import cadkit.sheetmetal.ManufacturedSheetState;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.process.ProcessTooling;

/** Physical contact motion of a fixed blank; prior manufacturing errors and tool limits remain explicit. */
class ManufacturedSheetMotion {
	public final bend:Int;
	public final contactDepth:Float;
	public final limitDepth:Float;
	final sheet:Sheet;
	final initial:ManufacturedSheetState;
	final tooling:ProcessTooling;
	final station:PressStationGeometry;
	final orientation:BendOrientation;
	final inverse:PressDepthInversion;
	public function new(sheet:Sheet, initial:ManufacturedSheetState, bend:Int, tooling:ProcessTooling,
			station:PressStationGeometry, orientation:BendOrientation) {
		this.sheet = sheet.copy(); this.initial = new ManufacturedSheetState(initial);
		this.bend = bend; this.tooling = tooling; this.station = station; this.orientation = orientation;
		if (sheet.isRipped(bend)) throw new SheetDiagnosis("sheet.ripped-stroke", "A ripped bend cannot carry a manufacturing stroke", ["bend:" + bend]);
		var angle = Math.PI - Math.max(tooling.punchAngle, tooling.dieAngle) - 1e-8;
		if (sheet.bend(bend).angle < 0) angle = -angle;
		inverse = new PressDepthInversion(bend, angle, atAngle);
		contactDepth = inverse.contactDepth; limitDepth = inverse.limitDepth;
	}
	public function atDepth(depth:Float):PressSheetPose return inverse.atDepth(depth);
	function atAngle(angle:Float):PressSheetPose {
		var actual = new ManufacturedSheetState(initial).set(bend, angle);
		return PressSheetPose.manufactured(sheet, actual, bend, tooling, station, orientation);
	}
}
