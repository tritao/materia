package motionkit.path;

/** Lateral material-frame axis and its seam-distance derivatives, in the base frame. */
class WeaveDirection {
  public final axis:Array<Float>;
  public final first:Array<Float>;
  public final second:Array<Float>;
  public function new(axis:Array<Float>, first:Array<Float>, second:Array<Float>) {
    for (vector in [axis, first, second]) {
      if (vector == null || vector.length != 3) throw "Weave frame requires three-component vectors";
      for (value in vector) if (!Math.isFinite(value)) throw "Weave frame requires finite vectors";
    }
    var norm = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    if (Math.abs(norm - 1.0) > 1e-8) throw "Weave lateral axis must be a unit vector";
    this.axis = axis.copy(); this.first = first.copy(); this.second = second.copy();
  }
}
