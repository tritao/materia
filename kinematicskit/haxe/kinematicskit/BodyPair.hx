package kinematicskit;

/** Two bodies of a model, the lower index first, and how they are related. */
class BodyPair {
  public final a:Int;
  public final b:Int;
  public final relation:BodyPairRelation;

  public function new(a:Int, b:Int, relation:BodyPairRelation) {
    this.a = a;
    this.b = b;
    this.relation = relation;
  }
}
