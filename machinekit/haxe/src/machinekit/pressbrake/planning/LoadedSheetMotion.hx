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

	public function new(sheet:Sheet, initial:SheetState, bend:Int, process:BendProcessResult,
			tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation) {
		this.sheet = sheet.copy();
		this.initial = new SheetState();
		for (id in initial.ids()) this.initial.set(id, initial.angle(id));
		this.bend = bend; this.process = process; this.tooling = tooling;
		this.station = station; this.orientation = orientation;
		contactDepth = atAngle(0).depth;
		loadedDepth = atAngle(process.loadedAngle).depth;
		if (loadedDepth <= contactDepth)
			throw new SheetDiagnosis("press.stroke-depth", "Loaded depth must exceed first contact", ["bend:" + bend]);
	}

	/** Outside contact, the caller owns free motion and springback; neither is inferred here. */
	public function atDepth(depth:Float):PressSheetPose {
		if (!Math.isFinite(depth) || depth < contactDepth || depth > loadedDepth)
			throw new SheetDiagnosis("press.measured-depth", "Measured depth lies outside the loaded contact stroke", ["bend:" + bend]);
		if (depth == contactDepth) return atAngle(0);
		if (depth == loadedDepth) return atAngle(process.loadedAngle);
		var low = 0.0, high = 1.0, lowDepth = contactDepth, highDepth = loadedDepth;
		// Invert the canonical construction rather than duplicating its contact formula.
		for (iteration in 0...48) {
			var fraction = (low + high) / 2;
			var pose = atAngle(process.loadedAngle * fraction);
			if (pose.depth < lowDepth || pose.depth > highDepth)
				throw new SheetDiagnosis("press.stroke-depth", "Contact depth is not monotonic across the loaded stroke", ["bend:" + bend]);
			if (Math.abs(pose.depth - depth) <= 1e-12) return pose;
			if (pose.depth < depth) { low = fraction; lowDepth = pose.depth; }
			else { high = fraction; highDepth = pose.depth; }
		}
		throw new SheetDiagnosis("press.measured-depth", "Measured depth inversion did not converge", ["bend:" + bend]);
	}

	function atAngle(angle:Float):PressSheetPose {
		var state = new SheetState();
		for (id in initial.ids()) state.set(id, initial.angle(id));
		state.set(bend, angle);
		return PressSheetPose.at(sheet, state, bend, process, tooling, station, orientation);
	}
}
