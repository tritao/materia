package robotkit.protocol;
@:wire
class ExecutionCapabilitiesMsg {
  @:id(1) public var plans:Bool;
  @:id(2) public var maximumPolynomialDegree:Int;
  @:id(3) public var maximumJoints:Int;
  @:id(4) public var maximumSegments:Int;
  @:id(5) public var timedEvents:Bool;
  @:id(6) public var replacementBoundaries:Bool;
  @:id(7) public var holdResume:Bool;
  @:id(8) public var polynomialLimits:String;
  @:id(9) public var samplingResolutionNs:haxe.Int64;
  public function new() {}
}
