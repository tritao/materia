package robotkit.protocol;

@:wire
class Welcome {
  @:id(1) public var protocolVersion:Int;
  @:id(2) public var serverName:String;
  @:id(3) public var sessionId:haxe.Int64;
  @:id(4) public var robotId:haxe.Int64;
  @:id(5) public var controlGranted:Bool;
  @:id(6) public var leaseId:haxe.Int64;
  @:id(7) public var leaseTimeoutMs:Int;

  public function new(?protocolVersion:Int = 1, ?serverName:String = "robotd",
      ?sessionId:haxe.Int64 = null, ?robotId:haxe.Int64 = null,
      ?controlGranted:Bool = false, ?leaseId:haxe.Int64 = null,
      ?leaseTimeoutMs:Int = 0) {
    this.protocolVersion = protocolVersion;
    this.serverName = serverName;
    this.sessionId = sessionId == null ? haxe.Int64.ofInt(0) : sessionId;
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.controlGranted = controlGranted;
    this.leaseId = leaseId == null ? haxe.Int64.ofInt(0) : leaseId;
    this.leaseTimeoutMs = leaseTimeoutMs;
  }
}
