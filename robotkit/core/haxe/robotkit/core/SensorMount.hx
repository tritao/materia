package robotkit.core;

/** Immutable authored sensor pose, shared by observations of the same sensor. */
class SensorMount {
  public final position:ImmutableFloatArray;
  public final rotation:ImmutableFloatArray;

  public function new(?position:Array<Float>, ?rotation:Array<Float>) {
    this.position = new ImmutableFloatArray(position == null ? [0.0, 0.0, 0.0] : position);
    this.rotation = new ImmutableFloatArray(rotation == null ? [0.0, 0.0, 0.0, 1.0] : rotation);
  }
}
