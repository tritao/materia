package robotkit.material;

/** Named robot joint and permitted target interval for one fork axis. */
class ForkAxisConfig {
  public final jointName:String;
  public final minimum:Float;
  public final maximum:Float;

  public function new(jointName:String, minimum:Float, maximum:Float) {
    if (jointName == null || jointName.length == 0 ||
        !Math.isFinite(minimum) || !Math.isFinite(maximum) || minimum > maximum)
      throw "Fork axis requires a joint name and finite ordered limits";
    this.jointName = jointName;
    this.minimum = minimum;
    this.maximum = maximum;
  }

  public function validate(target:Float):Void {
    if (!Math.isFinite(target) || target < minimum || target > maximum)
      throw 'Fork target $target is outside [$minimum, $maximum] for $jointName';
  }
}
