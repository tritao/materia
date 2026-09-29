package machinekit.units;

abstract Metres(Float) {
	public inline function new(value:Float) this = value;
	public inline function raw():Float return (this : Float);
	public inline function millimetres():Millimetres return new Millimetres((this : Float) * 1000);
	@:op(A + B) public static function add(a:Metres, b:Metres):Metres
		return new Metres(a.raw() + b.raw());
	@:op(A - B) public static function subtract(a:Metres, b:Metres):Metres
		return new Metres(a.raw() - b.raw());
}
