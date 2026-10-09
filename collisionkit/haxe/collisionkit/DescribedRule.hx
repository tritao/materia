package collisionkit;

/** A declared rule between two bodies (-1: the world) or two objects of a `CollisionDescription`, with its reason. */
class DescribedRule {
  public final a:Int;
  public final b:Int;
  public final rule:CollisionPairRule;
  public final reason:CollisionPairStatus;

  public function new(a:Int, b:Int, rule:CollisionPairRule, reason:CollisionPairStatus) {
    this.a = a;
    this.b = b;
    this.rule = rule;
    this.reason = reason;
  }
}
