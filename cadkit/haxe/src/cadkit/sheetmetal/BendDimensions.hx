package cadkit.sheetmetal;

/** Bend dimensions in millimetres and radians, measured from the concave surface for either sign. */
class BendDimensions {
	public static function validate(thickness:Float, radius:Float, angle:Float):Void {
		if (!Math.isFinite(thickness) || thickness <= 0)
			throw new SheetDiagnosis("sheet.invalid-thickness", "Thickness must be finite and positive");
		if (!Math.isFinite(radius) || radius < 0)
			throw new SheetDiagnosis("sheet.invalid-radius", "Inner radius must be finite and nonnegative");
		if (!Math.isFinite(angle) || Math.abs(angle) >= 2 * Math.PI)
			throw new SheetDiagnosis("sheet.invalid-angle", "Bend angle must be finite and below one full turn");
	}

	public static function allowance(thickness:Float, radius:Float, angle:Float, rule:Null<BendRule>):Float {
		validate(thickness, radius, angle);
		if (rule == null)
			throw new SheetDiagnosis("sheet.no-bend-rule", "Choose a bend rule before developing a sheet");
		return switch (rule) {
			case KFactor(table): Math.abs(angle) * (radius + table.at(radius / thickness) * thickness);
		};
	}

	public static function outsideSetback(thickness:Float, radius:Float, angle:Float):Float {
		validate(thickness, radius, angle);
		sharpAngle(angle);
		return (radius + thickness) * Math.tan(Math.abs(angle) / 2);
	}

	public static function deduction(thickness:Float, radius:Float, angle:Float, rule:Null<BendRule>):Float {
		return 2 * outsideSetback(thickness, radius, angle) - allowance(thickness, radius, angle, rule);
	}

	public static function leg(dimension:Float, spec:LengthSpec, thickness:Float, radius:Float, angle:Float):Float {
		validate(thickness, radius, angle);
		if (!Math.isFinite(dimension))
			throw new SheetDiagnosis("sheet.invalid-flange-length", "Flange dimension must be finite");
		var offset = switch (spec) {
			case Leg: 0.0;
			case OuterSharp: outsideSetback(thickness, radius, angle);
			case InnerSharp:
				sharpAngle(angle);
				radius * Math.tan(Math.abs(angle) / 2);
			case Tangential:
				Math.abs(angle) > Math.PI / 2 ? radius + thickness : outsideSetback(thickness, radius, angle);
		};
		var result = dimension - offset;
		if (result <= 0)
			throw new SheetDiagnosis("sheet.invalid-flange-length", "The drawing dimension leaves no straight flange");
		return result;
	}

	static function sharpAngle(angle:Float):Void {
		if (Math.abs(angle) > 179 * Math.PI / 180)
			throw new SheetDiagnosis("sheet.sharp-length-angle", "Sharp dimensions are undefined above 179 degrees");
	}
}
