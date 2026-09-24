package robotkit.mobile;

/** Immutable x/y coordinate in a base footprint, measured in metres. */
class FootprintPoint {
  public final x:Float;
  public final y:Float;

  public function new(x:Float, y:Float) {
    if (!Math.isFinite(x) || !Math.isFinite(y))
      throw "Footprint points must be finite";
    this.x = x;
    this.y = y;
  }
}
