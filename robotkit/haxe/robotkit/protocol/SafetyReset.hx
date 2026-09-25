package robotkit.protocol;

@:wire
class SafetyReset {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var reason:String;

  public function new(?robotId:haxe.Int64 = null,
      ?reason:String = "application acknowledged safety stop") {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.reason = reason;
  }
}
