package motionkit.planner;

enum BindingConstraintKind {
  JointVelocity;
  JointAcceleration;
  SpeedCap;
}

/** Constraint that sets a reference timing span's path-speed bound. */
class BindingConstraint {
  public final spanIndex:Int;
  /** -1 identifies an authored speed cap. */
  public final jointIndex:Int;
  public final kind:BindingConstraintKind;
  public final limit:Float;

  public function new(spanIndex:Int, jointIndex:Int, kind:BindingConstraintKind,
      limit:Float) {
    this.spanIndex = spanIndex;
    this.jointIndex = jointIndex;
    this.kind = kind;
    this.limit = limit;
  }
}
