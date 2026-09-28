package machinekit.component;

/** Formatting for dimensions embedded in designations and part numbers. */
class Dimension {
	/** Decimal text for `value` rounded to 0.001 mm. */
	public static function format(value:Float):String {
		return Std.string(Math.fround(value * 1000) / 1000);
	}
}
