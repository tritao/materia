package robotkit.protocol;
@:wire
class TimingCapabilitiesMsg {
  @:id(1) public var deadlines:Bool;
  @:id(2) public var clockMapping:Bool;
  @:id(3) public var prediction:String;
  @:id(4) public var samplingResolutionNs:haxe.Int64;
  public function new() {}
}
