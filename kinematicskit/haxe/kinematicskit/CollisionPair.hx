package kinematicskit;

/** Two collision objects, the lower id first. */
class CollisionPair {
  public final a:Int;
  public final b:Int;

  public function new(a:Int, b:Int) {
    this.a = a;
    this.b = b;
  }
}
