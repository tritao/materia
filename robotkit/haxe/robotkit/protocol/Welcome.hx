package robotkit.protocol;

@:wire
class Welcome {
  @:id(1) public var protocolVersion:Int;
  @:id(2) public var serverName:String;
  @:id(3) public var sessionId:haxe.Int64;
  @:id(4) public var robotId:haxe.Int64;

  public function new(?protocolVersion:Int = 1, ?serverName:String = "robotd",
      ?sessionId:haxe.Int64 = null, ?robotId:haxe.Int64 = null) {
    this.protocolVersion = protocolVersion;
    this.serverName = serverName;
    this.sessionId = sessionId == null ? haxe.Int64.ofInt(0) : sessionId;
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
  }
}
