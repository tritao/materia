package robotkit.protocol;

/** One transport representation inside an atomic JointTargets message. */
@:wire
class JointTargetValue {
  @:id(1) public var joint:Int;
  /** 1=position, 2=velocity, 3=effort. */
  @:id(2) public var mode:Int;
  @:id(3) public var target:Float;

  public function new(?joint:Int = 0, ?mode:Int = 1, ?target:Float = 0.0) {
    this.joint = joint;
    this.mode = mode;
    this.target = target;
  }
}
