package cadkit.sheetmetal.process;

import cadkit.sheetmetal.SheetDiagnosis;

/** Tool contact dimensions in millimetres; included tool angles are radians. */
class ProcessTooling {
	public final punchId:String;
	public final dieId:String;
	public final punchRadius:Float;
	public final punchAngle:Float;
	public final vWidth:Float;
	public final dieAngle:Float;
	public final shoulderRadius:Float;

	public function new(punchId:String, dieId:String, punchRadius:Float, punchAngle:Float,
		vWidth:Float, dieAngle:Float, shoulderRadius:Float) {
		if (punchId == null || punchId.length == 0 || dieId == null || dieId.length == 0
			|| !Math.isFinite(punchRadius) || punchRadius < 0
			|| !Math.isFinite(shoulderRadius) || shoulderRadius < 0
			|| !Math.isFinite(vWidth) || vWidth <= 0
			|| !Math.isFinite(punchAngle) || punchAngle <= 0 || punchAngle >= Math.PI
			|| !Math.isFinite(dieAngle) || dieAngle <= 0 || dieAngle >= Math.PI)
			throw new SheetDiagnosis("sheet.invalid-process-tooling", "Tool identities, positive opening and valid contact dimensions are required");
		this.punchId = punchId; this.dieId = dieId;
		this.punchRadius = punchRadius; this.punchAngle = punchAngle;
		this.vWidth = vWidth; this.dieAngle = dieAngle; this.shoulderRadius = shoulderRadius;
	}
}
