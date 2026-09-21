package robotkit.protocol;

@:wire
class Hello {
  @:id(1) public var protocolVersion:Int;
  @:id(2) public var clientName:String;
  @:id(3) public var schemaFingerprint:String;
  @:id(4) public var requestedRole:String;

  public function new(?protocolVersion:Int = 1, ?clientName:String = "",
      ?schemaFingerprint:String = "", ?requestedRole:String = "controller") {
    this.protocolVersion = protocolVersion;
    this.clientName = clientName;
    this.schemaFingerprint = schemaFingerprint;
    this.requestedRole = requestedRole;
  }
}
