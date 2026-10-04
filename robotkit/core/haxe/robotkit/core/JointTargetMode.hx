package robotkit.core;



/** Interpretation of a joint target value, independent of its transport. */
enum JointTargetMode {
  Position;
  Velocity;
  Effort;
  /** Position target with velocity target, stiffness, damping and feedforward effort. */
  Servo;
}
