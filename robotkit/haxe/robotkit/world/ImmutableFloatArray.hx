package robotkit.world;

/** Owning, read-only view over a finite sequence of floating-point values. */
class ImmutableFloatArray {
  final values:Array<Float>;

  public var length(get, never):Int;

  public function new(source:Null<Array<Float>>) {
    values = source == null ? [] : source.copy();
  }

  inline function get_length():Int return values.length;

  @:arrayAccess
  public inline function get(index:Int):Float return values[index];

  /** Returns independent mutable storage for adapters or serialization. */
  public function toArray():Array<Float> return values.copy();

  public function iterator():Iterator<Float> return values.iterator();
}
