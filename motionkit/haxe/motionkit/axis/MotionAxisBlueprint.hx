package motionkit.axis;

/**
 * Stable logical-axis mapping authored separately from runtime joint indices.
 * Multiple joint IDs are supported so dual-motor axes do not need a different
 * public MotionSystem API later.
 */
class MotionAxisBlueprint {
  public final id:String;
  public final jointIds:Array<String>;
  public final lowerLimit:Float;
  public final upperLimit:Float;
  public final maxVelocity:Float;
  public final maxAcceleration:Float;
  public final homePosition:Float;
  public final jointScales:Array<Float>;
  public final jointOffsets:Array<Float>;

  public function new(id:String, jointIds:Array<String>, lowerLimit:Float,
      upperLimit:Float, ?maxVelocity:Float = 0.0, ?maxAcceleration:Float = 0.0,
      ?homePosition:Float, ?jointScales:Array<Float>, ?jointOffsets:Array<Float>) {
    if (id == null || StringTools.trim(id).length == 0)
      throw "Motion axis needs a non-empty ID";
    if (jointIds == null || jointIds.length == 0)
      throw 'Motion axis "$id" needs at least one joint';
    if (!Math.isFinite(lowerLimit) || !Math.isFinite(upperLimit) || lowerLimit > upperLimit)
      throw 'Motion axis "$id" has invalid logical limits';
    requireNonnegative(maxVelocity, "maximum velocity", id);
    requireNonnegative(maxAcceleration, "maximum acceleration", id);
    var scales = jointScales == null ? [for (_ in jointIds) 1.0] : jointScales.copy();
    var offsets = jointOffsets == null ? [for (_ in jointIds) 0.0] : jointOffsets.copy();
    if (scales.length != jointIds.length || offsets.length != jointIds.length)
      throw 'Motion axis "$id" joint mapping lengths do not match';
    for (scale in scales)
      if (!Math.isFinite(scale) || scale == 0.0) throw 'Motion axis "$id" has an invalid joint scale';
    for (offset in offsets)
      if (!Math.isFinite(offset)) throw 'Motion axis "$id" has an invalid joint offset';
    var chosenHome = homePosition == null ? lowerLimit : homePosition;
    if (!Math.isFinite(chosenHome) || chosenHome < lowerLimit || chosenHome > upperLimit)
      throw 'Motion axis "$id" home position is outside its limits';
    this.id = id;
    this.jointIds = jointIds.copy();
    this.lowerLimit = lowerLimit;
    this.upperLimit = upperLimit;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.homePosition = chosenHome;
    this.jointScales = scales;
    this.jointOffsets = offsets;
  }

  static function requireNonnegative(value:Float, label:String, id:String):Void {
    if (!Math.isFinite(value) || value < 0.0)
      throw 'Motion axis "$id" $label must be finite and non-negative';
  }
}
