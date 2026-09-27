package cadkit.parametric;

/** Shared physical quantity types used by CAD parameters and persistent properties. */
class QuantityKind {
	public static inline var Scalar:String = "scalar";
	public static inline var Length:String = "length";
	public static inline var Angle:String = "angle";
	public static inline var Count:String = "count";
	public static inline var Area:String = "area";
	public static inline var Volume:String = "volume";

	public static function validate(value:String):String {
		// Literal names avoid collisions with similarly named enum constructors in combined Haxeon builds.
		if (value == "scalar" || value == "length" || value == "angle" || value == "count" ||
			value == "area" || value == "volume")
			return value;
		throw new ParametricError("unsupported quantity type: " + value);
	}
}
