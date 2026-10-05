package robotkit.runtime;

/** Project named home-axis displacements through compiled physical coupling terms. */
class PowerUpOffsets {
  public static function resolve(blueprint:RobotRuntimeBlueprint, named:Map<String, Float>):Array<Float> {
    if (blueprint == null || blueprint.identity == null || named == null)
      throw "Power-up offsets require a compiled joint identity";
    var values = [for (_ in 0...blueprint.jointCount) 0.0];
    var ready = [for (_ in values) true];
    for (term in blueprint.couplings) ready[term.follower] = false;
    for (id in named.keys()) {
      var joint = -1;
      for (i in 0...values.length) if (blueprint.identity.jointId(i) == id) joint = i;
      if (joint < 0 || !ready[joint]) throw 'Power-up axis "$id" must name an independent runtime joint';
      var home = false;
      for (contact in blueprint.switches) if (contact.joint == id && contact.role == "home") home = true;
      var value = named.get(id);
      if (!home || value == null || !Math.isFinite(value)) throw 'Power-up axis "$id" needs a home and finite displacement';
      values[joint] = value;
    }
    for (_ in values) {
      var changed = false;
      for (joint in 0...values.length) if (!ready[joint]) {
        var complete = true, sum = 0.0;
        for (term in blueprint.couplings) if (term.follower == joint) {
          if (!ready[term.leader]) complete = false;
          sum += term.ratio * values[term.leader];
        }
        if (complete) {
          if (!Math.isFinite(sum)) throw "Power-up follower displacement overflow";
          values[joint] = sum; ready[joint] = true; changed = true;
        }
      }
      if (!changed) break;
    }
    for (resolved in ready) if (!resolved) throw "Power-up coupling graph could not resolve";
    return values;
  }
}
