package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetDiagnosis;

/** Measured final radius and additive depth correction for one lot and half-open angle band. */
class CalibrationEntry {
	public final lot:String;
	public final thickness:Float;
	public final punchId:String;
	public final dieId:String;
	public final firstAngle:Float;
	public final lastAngle:Float;
	public final depthOffset:Float;
	public final innerRadius:Float;

	public function new(lot:String, thickness:Float, punchId:String, dieId:String,
		firstAngle:Float, lastAngle:Float, depthOffset:Float, innerRadius:Float) {
		if (lot == null || lot.length == 0 || punchId == null || punchId.length == 0 || dieId == null || dieId.length == 0
			|| !Math.isFinite(thickness) || thickness <= 0 || !Math.isFinite(depthOffset)
			|| !Math.isFinite(innerRadius) || innerRadius <= 0
			|| !Math.isFinite(firstAngle) || !Math.isFinite(lastAngle)
			|| firstAngle < 0 || lastAngle > Math.PI || firstAngle >= lastAngle)
			throw new SheetDiagnosis("sheet.invalid-calibration", "Calibration needs explicit lot and tools, positive mm dimensions and a valid angle band");
		this.lot = lot; this.thickness = thickness; this.punchId = punchId; this.dieId = dieId;
		this.firstAngle = firstAngle; this.lastAngle = lastAngle;
		this.depthOffset = depthOffset; this.innerRadius = innerRadius;
	}
}
