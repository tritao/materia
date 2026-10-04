package robotkit.protocol;

/** Renews one controller's lease without relying on synchronized clocks. */
@:wire
class ControlHeartbeat {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var leaseId:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null, ?leaseId:haxe.Int64 = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.leaseId = leaseId == null ? haxe.Int64.ofInt(0) : leaseId;
  }
}
