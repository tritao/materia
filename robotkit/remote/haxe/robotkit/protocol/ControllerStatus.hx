package robotkit.protocol;

@:wire
class ControllerStatus {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var controller:String;
  @:id(3) public var state:String;
  @:id(4) public var active:Bool;

  public function new(?robotId:haxe.Int64 = null, ?controller:String = "",
      ?state:String = "idle", ?active:Bool = false) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.controller = controller;
    this.state = state;
    this.active = active;
  }
}
