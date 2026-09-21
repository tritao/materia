package robotkit.protocol;

@:wire
class BaseTwist {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var vx:Float;
  @:id(3) public var vy:Float;
  @:id(4) public var wz:Float;
  @:id(5) public var expiryNs:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null, ?vx:Float = 0.0,
      ?vy:Float = 0.0, ?wz:Float = 0.0, ?expiryNs:haxe.Int64 = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.vx = vx;
    this.vy = vy;
    this.wz = wz;
    this.expiryNs = expiryNs == null ? haxe.Int64.ofInt(0) : expiryNs;
  }
}
