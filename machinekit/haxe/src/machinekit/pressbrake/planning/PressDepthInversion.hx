package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** One bracketed inverse of the canonical contact construction for predicted or actual stock. */
class PressDepthInversion {
	public final contactDepth:Float;
	public final limitDepth:Float;
	final bend:Int;
	final limitAngle:Float;
	final pose:Float->PressSheetPose;
	public function new(bend:Int, limitAngle:Float, pose:Float->PressSheetPose) {
		this.bend = bend; this.limitAngle = limitAngle; this.pose = pose;
		contactDepth = pose(0).depth; limitDepth = pose(limitAngle).depth;
		if (limitDepth <= contactDepth)
			throw new SheetDiagnosis("press.stroke-depth", "Loaded depth must exceed first contact", ["bend:" + bend]);
	}
	public function atDepth(depth:Float):PressSheetPose {
		if (!Math.isFinite(depth) || depth < contactDepth || depth > limitDepth)
			throw new SheetDiagnosis("press.measured-depth", "Measured depth lies outside the contact stroke", ["bend:" + bend]);
		if (depth == contactDepth) return pose(0);
		if (depth == limitDepth) return pose(limitAngle);
		var low = 0.0, high = 1.0, lowDepth = contactDepth, highDepth = limitDepth;
		for (iteration in 0...48) {
			var fraction = (low + high) / 2, sample = pose(limitAngle * fraction);
			if (sample.depth < lowDepth || sample.depth > highDepth)
				throw new SheetDiagnosis("press.stroke-depth", "Contact depth is not monotonic across the stroke", ["bend:" + bend]);
			if (Math.abs(sample.depth - depth) <= 1e-12) return sample;
			if (sample.depth < depth) { low = fraction; lowDepth = sample.depth; }
			else { high = fraction; highDepth = sample.depth; }
		}
		throw new SheetDiagnosis("press.measured-depth", "Measured depth inversion did not converge", ["bend:" + bend]);
	}
}
