package cadkit.parametric;

import cadkit.units.LengthUnits;

/** Explicit conversion to canonical mm, rad, count, mm², and mm³ values. */
class UnitConversion {
	public static function canonicalUnit(kind:String):String {
		return switch ParameterKind.validate(kind) {
			case ParameterKind.Length: "mm";
			case ParameterKind.Angle: "rad";
			case ParameterKind.Count: "count";
			case ParameterKind.Area: "mm2";
			case ParameterKind.Volume: "mm3";
			default: "1";
		};
	}

	public static function toCanonical(value:Float, kind:String, unit:String):Float {
		if (!Math.isFinite(value))
			throw new ParametricError("parameter value must be finite");
		var factor = factorFor(kind, unit);
		var result = value * factor;
		if (kind == ParameterKind.Count && result != Std.int(result))
			throw new ParametricError("count parameter must be an integer");
		return result;
	}

	public static function fromCanonical(value:Float, kind:String, unit:String):Float {
		return value / factorFor(kind, unit);
	}

	public static function validateUnit(kind:String, unit:String):String {
		factorFor(kind, unit);
		return unit;
	}

	private static function factorFor(kind:String, unit:String):Float {
		ParameterKind.validate(kind);
		return switch kind {
			case ParameterKind.Scalar:
				if (unit == "1") 1 else invalid(kind, unit);
			case ParameterKind.Count:
				if (unit == "count" || unit == "1") 1 else invalid(kind, unit);
			case ParameterKind.Length:
				var factor = LengthUnits.factorToMillimetres(unit);
				factor == null ? invalid(kind, unit) : factor;
			case ParameterKind.Angle: switch unit {
				case "rad": 1; case "deg": Math.PI / 180; default: invalid(kind, unit);
			};
			case ParameterKind.Area: switch unit {
				case "mm2": 1; case "cm2": 100; case "m2": 1000000; case "in2": 25.4 * 25.4; case "ft2": 304.8 * 304.8; default: invalid(kind, unit);
			};
			case ParameterKind.Volume: switch unit {
				case "mm3": 1; case "cm3": 1000; case "m3": 1000000000; case "in3": 25.4 * 25.4 * 25.4;
				case "ft3": 304.8 * 304.8 * 304.8; default: invalid(kind, unit);
			};
			default: invalid(kind, unit);
		};
	}

	private static function invalid(kind:String, unit:String):Float {
		throw new ParametricError("unit " + unit + " is not valid for " + kind);
	}
}
