package robotkit.protocol;

@:wire
class RobotCapabilities {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var jointCount:Int;
  @:id(3) public var supportsPosition:Bool;
  @:id(4) public var supportsVelocity:Bool;
  @:id(5) public var supportsEffort:Bool;
  @:id(6) public var supportsPrediction:Bool;

  public function new(?robotId:haxe.Int64 = null, ?jointCount:Int = 0,
      ?supportsPosition:Bool = true, ?supportsVelocity:Bool = false,
      ?supportsEffort:Bool = false, ?supportsPrediction:Bool = false) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.jointCount = jointCount;
    this.supportsPosition = supportsPosition;
    this.supportsVelocity = supportsVelocity;
    this.supportsEffort = supportsEffort;
    this.supportsPrediction = supportsPrediction;
  }
}
