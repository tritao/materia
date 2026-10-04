package cadkit.sheetmetal;

/** Hem dimensions derive one circular bend and one straight return, without hidden clearance gaps. */
class HemGeometry {
	public static function derive(type:HemType, length:Float, gap:Float, thickness:Float, radius:Null<Float>):HemDimensions {
		BendDimensions.validate(thickness, 0, 0);
		if (!Math.isFinite(length) || length <= 0 || !Math.isFinite(gap) || gap < 0)
			throw new SheetDiagnosis("sheet.invalid-hem", "Hem length must be positive and its gap nonnegative");
		if (type == Closed) {
			if (gap != 0 || radius != null) throw new SheetDiagnosis("sheet.invalid-hem", "A closed hem derives zero gap and zero inner radius");
			return new HemDimensions(Math.PI, 0);
		}
		if (type == Open) {
			if (gap <= 0 || radius != null) throw new SheetDiagnosis("sheet.invalid-hem", "An open hem derives its radius from a positive gap");
			return new HemDimensions(Math.PI, gap / 2);
		}
		if (radius == null || !Math.isFinite(radius) || radius <= 0 || gap >= 2 * radius)
			throw new SheetDiagnosis("sheet.invalid-hem", "A teardrop hem needs a positive bulb radius and a tip gap below its diameter");
		var magnitude = Math.sqrt(radius * radius + length * length);
		var angle = 2 * Math.PI - Math.acos((radius - gap) / magnitude) - Math.atan2(length, radius);
		if (gap - thickness * Math.cos(angle) < -1e-9)
			throw new SheetDiagnosis("sheet.hem-clearance", "The return tip would penetrate the parent surface");
		return new HemDimensions(angle, radius);
	}
}

class HemDimensions {
	public final angle:Float;
	public final radius:Float;
	public function new(angle:Float, radius:Float) { this.angle = angle; this.radius = radius; }
}
