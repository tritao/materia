package machinekit.units;

abstract Kilograms(Float) {
	public inline function new(value:Float) this = value;
	public inline function raw():Float return (this : Float);
	@:op(A + B) public static function add(a:Kilograms, b:Kilograms):Kilograms
		return new Kilograms(a.raw() + b.raw());
	@:op(A - B) public static function subtract(a:Kilograms, b:Kilograms):Kilograms
		return new Kilograms(a.raw() - b.raw());
}
