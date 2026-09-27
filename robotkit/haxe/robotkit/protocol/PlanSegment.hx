package robotkit.protocol;

/** Polynomial coefficients are flattened joint-major for MessagePack. */
@:wire
class PlanSegment {
  @:id(1) public var timeFromStartNs:haxe.Int64;
  @:id(2) public var durationNs:haxe.Int64;
  @:id(3) public var jointCount:Int;
  @:id(4) public var degree:Int;
  @:id(5) public var coefficients:Array<Float>;

  public function new(?timeFromStartNs:haxe.Int64, ?durationNs:haxe.Int64,
      ?jointCount:Int = 0, ?degree:Int = 0, ?coefficients:Array<Float>) {
    this.timeFromStartNs = timeFromStartNs == null ? haxe.Int64.ofInt(0) : timeFromStartNs;
    this.durationNs = durationNs == null ? haxe.Int64.ofInt(0) : durationNs;
    this.jointCount = jointCount;
    this.degree = degree;
    this.coefficients = coefficients == null ? [] : coefficients;
  }
}
