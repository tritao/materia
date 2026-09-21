package robotkit.protocol;

@:wire
class Hello {
  @:id(1) public var protocolVersion:Int;
  @:id(2) public var clientName:String;
  @:id(3) public var schemaFingerprint:String;

  public function new(?protocolVersion:Int = 1, ?clientName:String = "",
      ?schemaFingerprint:String = "") {
    this.protocolVersion = protocolVersion;
    this.clientName = clientName;
    this.schemaFingerprint = schemaFingerprint;
  }
}
