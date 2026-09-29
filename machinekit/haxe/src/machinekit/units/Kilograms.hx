package machinekit.units;

abstract Kilograms(Float) from Float to Float {
	public inline function new(value:Float) this = value;
}
