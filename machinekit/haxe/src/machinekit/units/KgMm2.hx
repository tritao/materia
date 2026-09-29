package machinekit.units;

abstract KgMm2(Float) {
	public inline function new(value:Float) this = value;
	public inline function raw():Float return (this : Float);
	public inline function kgM2():Float return (this : Float) * 1e-6;
	@:op(A + B) public static function add(a:KgMm2, b:KgMm2):KgMm2
		return new KgMm2(a.raw() + b.raw());
	@:op(A - B) public static function subtract(a:KgMm2, b:KgMm2):KgMm2
		return new KgMm2(a.raw() - b.raw());
}
