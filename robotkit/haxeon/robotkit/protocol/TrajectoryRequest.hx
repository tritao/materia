package robotkit.protocol;

@:wire
class TrajectoryRequest {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var jointValues:Array<Float>;
  @:id(3) public var durationSec:Float;
  @:id(4) public var expiryNs:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null,
      ?jointValues:Array<Float> = null, ?durationSec:Float = 0.0,
      ?expiryNs:haxe.Int64 = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.jointValues = jointValues == null ? [] : jointValues;
    this.durationSec = durationSec;
    this.expiryNs = expiryNs == null ? haxe.Int64.ofInt(0) : expiryNs;
  }
}
