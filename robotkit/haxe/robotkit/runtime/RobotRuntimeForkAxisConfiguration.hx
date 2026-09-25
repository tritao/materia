package robotkit.runtime;

/** A fork role resolved to a runtime joint slot and authored joint limits. */
class RobotRuntimeForkAxisConfiguration {
  public final jointIndex:Int;
  public final jointName:String;
  public final minimum:Float;
  public final maximum:Float;

  public function new(jointIndex:Int, jointName:String, minimum:Float, maximum:Float) {
    this.jointIndex = jointIndex;
    this.jointName = jointName;
    this.minimum = minimum;
    this.maximum = maximum;
  }
}
