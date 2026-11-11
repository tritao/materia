package cadkit.parametric.recording;

import cadkit.parametric.NamedParameter;

/** A builder dimension: a shared named parameter, or a plain value that belongs to its feature alone. */
abstract Dimension(DimensionSource) {
  inline function new(source:DimensionSource) {
    this = source;
  }

  @:from public static function ofValue(value:Float):Dimension return new Dimension(new DimensionSource(value, null));

  @:from public static function ofInt(value:Int):Dimension return new Dimension(new DimensionSource(value, null));

  @:from public static function ofNamed(named:NamedParameter):Dimension return new Dimension(new DimensionSource(named.value, named));

  public inline function number():Float return this.value;

  public inline function named():Null<NamedParameter> return this.named;
}

private class DimensionSource {
  public final value:Float;
  public final named:Null<NamedParameter>;

  public function new(value:Float, named:Null<NamedParameter>) {
    this.value = value;
    this.named = named;
  }
}
