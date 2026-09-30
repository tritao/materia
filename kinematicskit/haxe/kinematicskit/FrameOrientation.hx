package kinematicskit;

/** Which part of a target orientation a `FrameTask` holds. */
enum FrameOrientation {
  /** The whole orientation (three rows). */
  Full;
  /** Only the direction of one local axis (two rows); rotation about it is free. */
  Axis(x:Float, y:Float, z:Float);
  /** None (no rows). */
  Free;
}
