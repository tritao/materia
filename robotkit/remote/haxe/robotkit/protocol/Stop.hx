package robotkit.protocol;

@:wire
class Stop {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var reason:String;
  @:id(3) public var emergency:Bool;

  public function new(?robotId:haxe.Int64 = null, ?reason:String = "",
      ?emergency:Bool = false) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.reason = reason;
    this.emergency = emergency;
  }
}
