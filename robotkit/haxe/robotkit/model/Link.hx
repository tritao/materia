package robotkit.model;

class Link {
  public final id:LinkId;
  public var name:String;
  public var mass:Float = 1.0;
  public var centerOfMass:Array<Float> = [0.0, 0.0, 0.0];
  /** Row-major symmetric inertia tensor in kg m², relative to center of mass. */
  public var inertiaTensor:Array<Float> = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0];
  public var visualGeometry:Null<String> = null;
  public var collisionGeometry:Null<String> = null;

  public function new(name:String, ?id:LinkId) {
    // Legacy callers use the initial name once; imports pass the stored ID.
    this.id = id == null ? name : id;
    this.name = name;
  }
}
