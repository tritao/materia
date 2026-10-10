package kinematicskit;

/**
 * The body pairs of a model that a collision world should not check, with
 * their reasons (COLLISION.md CL-D1, CL-D3): rigid, adjacent and closure
 * pairs. They follow from the model's structure alone; the caller turns
 * them into its collision world's rules. Overlap at a reference
 * configuration is the world's to find, since it needs the geometry.
 */
class KinematicBodyPairs {
  /** Every related pair of `model`'s bodies, once each, in body order; a pair takes its first reason of rigid, adjacent, closure. */
  public static function of(model:KinematicModel):Array<BodyPair> {
    var group = rigidGroups(model);
    var count = model.bodyCount();
    var relation:Array<Int> = [for (_ in 0...count * count) -1];
    function mark(a:Int, b:Int, kind:BodyPairRelation):Void {
      if (a == b) return;
      var i = a < b ? a * count + b : b * count + a;
      if (relation[i] < 0 || relation[i] > (kind : Int)) relation[i] = kind;
    }
    // Groups joined by one movable joint, and groups meeting through a closure.
    var groupAdjacent:Array<Array<Int>> = [];
    for (joint in 0...model.jointCount()) {
      if (model.jointKind[joint] == JointKind.Fixed) continue;
      groupAdjacent.push([group[model.jointParent[joint]], group[model.jointChild[joint]]]);
    }
    var groupClosure:Array<Array<Int>> = [];
    for (closure in 0...model.closureCount()) {
      groupClosure.push([group[model.frameBody[model.closureFrameA[closure]]],
        group[model.frameBody[model.closureFrameB[closure]]]]);
    }
    for (a in 0...count) for (b in a + 1...count) {
      if (group[a] == group[b]) {
        mark(a, b, Rigid);
        continue;
      }
      for (link in groupAdjacent) if (sameGroups(link, group[a], group[b])) mark(a, b, Adjacent);
      for (link in groupClosure) if (sameGroups(link, group[a], group[b])) mark(a, b, Closure);
    }
    var pairs:Array<BodyPair> = [];
    for (a in 0...count) for (b in a + 1...count) {
      var kind = relation[a * count + b];
      if (kind >= 0) pairs.push(new BodyPair(a, b, cast kind));
    }
    return pairs;
  }

  /** Per body: the top body of the group it is rigidly fixed to (through fixed joints). */
  public static function rigidGroups(model:KinematicModel):Array<Int> {
    var group = [for (body in 0...model.bodyCount()) body];
    for (body in model.bodyOrder) {
      var joint = model.bodyParentJoint[body];
      if (joint >= 0 && model.jointKind[joint] == JointKind.Fixed) group[body] = group[model.jointParent[joint]];
    }
    return group;
  }

  static function sameGroups(link:Array<Int>, g:Int, h:Int):Bool
    return (link[0] == g && link[1] == h) || (link[0] == h && link[1] == g);
}
