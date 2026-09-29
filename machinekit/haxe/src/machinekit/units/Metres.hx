package machinekit.units;

abstract Metres(Float) from Float to Float {
	public inline function new(value:Float) this = value;
	public inline function millimetres():Millimetres return new Millimetres((this : Float) * 1000);
}
