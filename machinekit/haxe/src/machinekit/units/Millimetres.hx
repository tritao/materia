package machinekit.units;

abstract Millimetres(Float) from Float to Float {
	public inline function new(value:Float) this = value;
	public inline function metres():Metres return new Metres((this : Float) * 0.001);
	@:op(A + B) public static function add(a:Millimetres, b:Millimetres):Millimetres
		return new Millimetres((a : Float) + (b : Float));
	@:op(A - B) public static function subtract(a:Millimetres, b:Millimetres):Millimetres
		return new Millimetres((a : Float) - (b : Float));
}
