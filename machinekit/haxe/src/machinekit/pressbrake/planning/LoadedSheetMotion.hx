package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.process.BendProcessResult;
import cadkit.sheetmetal.process.ProcessTooling;

/** Sheet motion during tool contact, driven by measured depth in mm below the die top. */
class LoadedSheetMotion {
	final sheet:Sheet;
	final initial:SheetState;
	final bend:Int;
	final process:BendProcessResult;
	final tooling:ProcessTooling;
	final station:PressStationGeometry;
	final orientation:BendOrientation;
	public final contactDepth:Float;
	public final loadedDepth:Float;
	final inverse:PressDepthInversion;

	public function new(sheet:Sheet, initial:SheetState, bend:Int, process:BendProcessResult,
			tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation) {
		this.sheet = sheet.copy();
		this.initial = new SheetState();
		for (id in initial.ids()) this.initial.set(id, initial.angle(id));
		this.bend = bend; this.process = process; this.tooling = tooling;
		this.station = station; this.orientation = orientation;
		inverse = new PressDepthInversion(bend, process.loadedAngle, atAngle);
		contactDepth = inverse.contactDepth; loadedDepth = inverse.limitDepth;
	}

	/** Outside contact, the caller owns free motion and springback; neither is inferred here. */
	public function atDepth(depth:Float):PressSheetPose return inverse.atDepth(depth);

	function atAngle(angle:Float):PressSheetPose {
		var state = new SheetState();
		for (id in initial.ids()) state.set(id, initial.angle(id));
		state.set(bend, angle);
		return PressSheetPose.at(sheet, state, bend, process, tooling, station, orientation);
	}
}
