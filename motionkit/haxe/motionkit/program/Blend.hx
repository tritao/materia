package motionkit.program;

/** Junction policy requested between consecutive motion operations. */
enum Blend {
  ExactStop;
  ToleranceBlend(metres:Float);
}
