package robotkit.protocol;

@:wire
class RobotDescription {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var name:String;
  @:id(3) public var links:Array<String>;
  @:id(4) public var joints:Array<String>;

  public function new(?robotId:haxe.Int64 = null, ?name:String = "",
      ?links:Array<String> = null, ?joints:Array<String> = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.name = name;
    this.links = links == null ? [] : links;
    this.joints = joints == null ? [] : joints;
  }
}
