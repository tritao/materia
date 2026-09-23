package cadkit.units;

/** Conversion factors for supported authored length units to canonical millimetres. */
class LengthUnits {
	public static function factorToMillimetres(unit:String):Null<Float> {
		return switch unit {
			case "mm": 1.0;
			case "cm": 10.0;
			case "m": 1000.0;
			case "in": 25.4;
			case "ft": 304.8;
			default: null;
		};
	}
}
