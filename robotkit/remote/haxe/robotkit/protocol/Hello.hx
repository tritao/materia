package robotkit.protocol;

@:wire
class Hello {
  @:id(1) public var protocolVersion:Int;
  @:id(2) public var clientName:String;
  @:id(3) public var schemaFingerprint:String;
  @:id(4) public var requestedRole:String;
  @:id(5) public var subscriptions:Array<StreamSubscription>;

  @:id(6) public var identity:String;
  @:id(7) public var token:String;

  public function new(?protocolVersion:Int = RobotFrame.VERSION, ?clientName:String = "",
      ?schemaFingerprint:String = "", ?requestedRole:String = "controller",
      ?subscriptions:Array<StreamSubscription>, ?identity:String = "", ?token:String = "") {
    this.identity = identity; this.token = token;
    this.protocolVersion = protocolVersion;
    this.clientName = clientName;
    this.schemaFingerprint = schemaFingerprint;
    this.requestedRole = requestedRole;
    this.subscriptions = subscriptions == null ? [] : subscriptions;
  }
}
