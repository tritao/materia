package robotkit.runtime;

import robotkit.core.SensorFrame;

/** Digital state and an optional captured closing edge, in the source joint coordinate.
 * Sequence, timestamp and clock remain on the immutable SensorFrame. */
class JointSwitchFrame {
  public final frame:SensorFrame;
  public final active:Bool;
  public final closingEdgePosition:Null<Float>;
  public final closingEdges:Int;

  public function new(frame:SensorFrame) {
    if (frame == null || frame.kind != "joint_switch" || !valid(frame.values.toArray()))
      throw "Invalid joint switch frame";
    this.frame = frame;
    active = frame.values.get(0) == 1.0;
    closingEdgePosition = frame.values.length == 4 && frame.values.get(1) == 1.0 ? frame.values.get(2) : null;
    closingEdges = frame.values.length == 4 ? Std.int(frame.values.get(3)) : 0;
  }

  /** Legacy digital-only frames carry no capture. Extended payload: active, valid, position, count. */
  public static function valid(values:Array<Float>):Bool {
    if (values == null || (values.length != 1 && values.length != 4)) return false;
    for (value in values) if (!Math.isFinite(value)) return false;
    if (values[0] != 0.0 && values[0] != 1.0) return false;
    if (values.length == 1) return true;
    if ((values[1] != 0.0 && values[1] != 1.0) || values[3] < 0.0 ||
        values[3] > 2147483647.0 || values[3] != Std.int(values[3])) return false;
    return values[1] == 1.0 ? values[3] > 0.0 : values[3] == 0.0 && values[2] == 0.0;
  }
}
