package motionkit.program;

/** Junction policy requested between consecutive motion operations. */
enum Blend {
  ExactStop;
  ToleranceBlend(metres:Float);
  /**
   * Pass a planned joint target without stopping where the curved motion through it can be
   * certified clear; otherwise stop there. Joins consecutive planned `MoveJ` operations.
   */
  FlyBy;
}
