package robotkit.model;

class Joint {
  public final name:String;
  public final type:JointType;
  public final parent:Link;
  public final child:Link;
  public var limits:JointLimits;
  public var drive:Null<Actuator>;

  public function new(name:String, type:JointType, parent:Link, child:Link) {
    this.name = name;
    this.type = type;
    this.parent = parent;
    this.child = child;
    this.limits = new JointLimits();
  }
}
