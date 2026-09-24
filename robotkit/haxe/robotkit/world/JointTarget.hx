package robotkit.world;

/** Immutable target for one joint inside an atomic RobotCommand batch. */
class JointTarget {
  /** Maximum targets accepted by the fixed-size RobotRuntime ABI. */
  public static inline final MAX_BATCH_SIZE:Int = 64;

  public final joint:Int;
  public final mode:JointTargetMode;
  public final target:Float;

  public function new(joint:Int, mode:JointTargetMode, target:Float) {
    if (joint < 0 || joint >= MAX_BATCH_SIZE)
      throw 'Invalid joint target index $joint';
    if (mode == null)
      throw "Joint target mode is required";
    if (!Math.isFinite(target))
      throw "Joint target must be finite";
    this.joint = joint;
    this.mode = mode;
    this.target = target;
  }

  public static function position(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Position, target);

  public static function velocity(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Velocity, target);

  public static function effort(joint:Int, target:Float):JointTarget
    return new JointTarget(joint, JointTargetMode.Effort, target);

  public function copy():JointTarget return new JointTarget(joint, mode, target);

  /** Validates and copies a complete batch so callers cannot mutate it mid-submit. */
  public static function copyBatch(values:Array<JointTarget>):Array<JointTarget> {
    if (values == null || values.length == 0 || values.length > MAX_BATCH_SIZE)
      throw 'Joint target batch must contain 1..$MAX_BATCH_SIZE targets';
    var seen = new Map<Int, Bool>();
    var result:Array<JointTarget> = [];
    for (value in values) {
      if (value == null)
        throw "Joint target batch cannot contain null values";
      if (seen.exists(value.joint))
        throw 'Joint target batch contains joint ${value.joint} more than once';
      seen.set(value.joint, true);
      result.push(value.copy());
    }
    return result;
  }
}
