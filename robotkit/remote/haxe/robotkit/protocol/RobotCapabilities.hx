package robotkit.protocol;
@:wire
class RobotCapabilities {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var jointCount:Int;
  @:id(3) public var controlModes:Array<String>;
  @:id(4) public var execution:ExecutionCapabilitiesMsg;
  @:id(5) public var timing:TimingCapabilitiesMsg;
  @:id(6) public var streams:Array<String>;
  public function new() {}
}
