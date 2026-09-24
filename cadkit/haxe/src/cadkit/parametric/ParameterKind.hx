package cadkit.parametric;

/** Backward-compatible names for quantity types used by CAD parameters. */
class ParameterKind {
	public static inline var Scalar:String = QuantityKind.Scalar;
	public static inline var Length:String = QuantityKind.Length;
	public static inline var Angle:String = QuantityKind.Angle;
	public static inline var Count:String = QuantityKind.Count;
	public static inline var Area:String = QuantityKind.Area;
	public static inline var Volume:String = QuantityKind.Volume;

	public static function validate(value:String):String
		return QuantityKind.validate(value);
}
