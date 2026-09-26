package motionkit.axis;

/** A machine-coordinate target for one logical motion axis. */
class AxisTarget {
  public final axis:String;
  public final position:Float;

  public function new(axis:String, position:Float) {
    if (axis == null || StringTools.trim(axis).length == 0)
      throw "Axis target needs a non-empty axis ID";
    if (!Math.isFinite(position)) throw "Axis target position must be finite";
    this.axis = axis;
    this.position = position;
  }
}
