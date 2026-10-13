package cadkit.sheetmetal;

/** Immutable wall-local coordinates, in millimetres. */
class Point2 {
	public final x:Float;
	public final y:Float;
	public function new(x:Float, y:Float) {
		if (!Math.isFinite(x) || !Math.isFinite(y))
			throw new SheetDiagnosis("sheet.invalid-outline", "Outline coordinates must be finite");
		this.x = x;
		this.y = y;
	}
	public function distance(other:Point2):Float {
		var dx = x - other.x;
		var dy = y - other.y;
		return Math.sqrt(dx * dx + dy * dy);
	}
}
