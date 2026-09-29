package machinekit.units;

abstract KgMm2(Float) from Float to Float {
	public inline function new(value:Float) this = value;
	public inline function kgM2():Float return (this : Float) * 1e-6;
}
