package robotkit.protocol;

/** Native path lifecycle action: 1=HOLD, 2=RESUME, 3=ABORT. */
@:wire
class PathControl {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var action:Int;

  public function new(?robotId:haxe.Int64, ?action:Int = 0) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.action = action;
  }
}
