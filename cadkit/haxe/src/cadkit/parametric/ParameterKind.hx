package cadkit.parametric;

/** Supported named-parameter quantities and their canonical document units. */
class ParameterKind {
	public static inline var Scalar:String = "scalar";
	public static inline var Length:String = "length";
	public static inline var Angle:String = "angle";
	public static inline var Count:String = "count";
	public static inline var Area:String = "area";
	public static inline var Volume:String = "volume";

	public static function validate(value:String):String {
		return switch value {
			case Scalar, Length, Angle, Count, Area, Volume: value;
			default: throw new ParametricError("unsupported parameter type: " + value);
		};
	}
}
