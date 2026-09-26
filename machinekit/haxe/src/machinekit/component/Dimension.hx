package machinekit.component;

/** Formatting for dimensions embedded in designations and part numbers. */
class Dimension {
	/** Shortest decimal text for `value` rounded to 0.001 mm: `12.7`, `6.35`, `20`.
	 * haxeon's float-to-string does not round-trip (12.7 prints as 12.699999999999999), and part
	 * numbers key BOM aggregation, so designations must never interpolate raw fractional floats.
	 */
	public static function format(value:Float):String {
		var scaled:Int = Math.round(Math.abs(value) * 1000);
		var sign = value < 0 && scaled != 0 ? "-" : "";
		var whole:Int = Std.int(scaled / 1000);
		var frac = scaled - whole * 1000;
		if (frac == 0) return sign + Std.string(whole);
		var digits = Std.string(frac);
		while (digits.length < 3) digits = "0" + digits;
		while (digits.charAt(digits.length - 1) == "0") digits = digits.substr(0, digits.length - 1);
		return '$sign$whole.$digits';
	}
}
