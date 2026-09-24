package robotkit.model;

class Joint {
  public final id:JointId;
  public var name:String;
  public final type:JointType;
  public final parent:Link;
  public final child:Link;
  public var limits:JointLimits;
  public var drive:Null<Actuator>;
  public var parentFramePosition:Array<Float> = [0.0, 0.0, 0.0];
  public var parentFrameRotation:Array<Float> = [0.0, 0.0, 0.0, 1.0];
  public var childFramePosition:Array<Float> = [0.0, 0.0, 0.0];
  public var childFrameRotation:Array<Float> = [0.0, 0.0, 0.0, 1.0];
  public var axis:Array<Float> = [0.0, 0.0, 1.0];

  public function new(name:String, type:JointType, parent:Link, child:Link, ?id:JointId) {
    // Legacy callers use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
    this.type = type;
    this.parent = parent;
    this.child = child;
    this.limits = new JointLimits();
  }
}
