package machinekit.units;

abstract Millimetres(Float) {
	public inline function new(value:Float) this = value;
	public inline function raw():Float return (this : Float);
	public inline function metres():Metres return new Metres((this : Float) * 0.001);
	@:op(A + B) public static function add(a:Millimetres, b:Millimetres):Millimetres
		return new Millimetres((a : Float) + (b : Float));
	@:op(A - B) public static function subtract(a:Millimetres, b:Millimetres):Millimetres
		return new Millimetres((a : Float) - (b : Float));
}
