package robotkit.model;

import robotkit.model.EngineeringAssumptions.QuantityAssumption;

/**
 * One term of a follower joint's coordinate, in SI units: the follower is the sum of its couplings,
 * `Σ (ratio * leader + offset)`. Most followers have one coupling (a lead screw follows its axis); a
 * follower with several, such as a CoreXY motor that follows both axes, has one coupling per leader,
 * each with its own efficiency, stiffness, backlash and drag. A leader appears once per follower, and
 * the couplings never form a cycle (see `cycleThrough`).
 */
class JointCoupling {
  public var assumptions:Array<QuantityAssumption> = [];
  public final id:String;
  /** Assumed engineering inputs carried from the model's source. */
  public var assumed:Array<String> = [];
  public final leader:JointId;
  public final follower:JointId;
  public final ratio:Float;
  public final offset:Float;
  /** Share of power passed from leader to follower or back, such as a lead screw's 0.4; 1 is lossless. */
  public var efficiency:Float = 1.0;
  /**
   * How stiff the coupling is, as force at the leader per unit of the leader's travel (N/m for a
   * sliding leader, N m/rad for a turning one), such as a timing belt's stretch. Zero means rigid.
   */
  public var stiffness:Float = 0.0;
  /** Lost motion when the coupling reverses, in the leader's units, such as a lead screw nut's backlash. */
  public var backlash:Float = 0.0;
  /**
   * Constant resisting effort the coupling adds at the follower while it moves, in the follower's
   * units (N m for a turning follower), such as a screw nut's drag.
   */
  public var drag:Float = 0.0;

  public function new(id:String, leader:JointId, follower:JointId,
      ratio:Float, offset:Float) {
    if (id == null || StringTools.trim(id).length == 0 ||
        leader == null || StringTools.trim(leader).length == 0 ||
        follower == null || StringTools.trim(follower).length == 0 ||
        leader == follower || !Math.isFinite(ratio) || ratio == 0.0 ||
        !Math.isFinite(offset))
      throw "Joint coupling needs distinct joints, nonzero finite ratio and finite offset";
    this.id = id;
    this.leader = leader;
    this.follower = follower;
    this.ratio = ratio;
    this.offset = offset;
  }

  /**
   * The ID of a joint that depends on itself through the couplings' leader-to-follower links, or null
   * when there is none.
   */
  public static function cycleThrough(couplings:Array<JointCoupling>):Null<JointId> {
    var state = new Map<JointId, Int>();
    for (start in couplings) {
      if (state.get(start.follower) == 2) continue;
      // Iterative depth first from the follower back through its leaders; 1 is on the path, 2 is done.
      var stack = [start.follower], cursor = [0];
      state.set(start.follower, 1);
      while (stack.length > 0) {
        var joint = stack[stack.length - 1];
        var leaders = [for (coupling in couplings) if (coupling.follower == joint) coupling.leader];
        var at = cursor[cursor.length - 1];
        if (at >= leaders.length) {
          state.set(joint, 2);
          stack.pop();
          cursor.pop();
          continue;
        }
        cursor[cursor.length - 1] = at + 1;
        var leader = leaders[at], mark = state.get(leader);
        if (mark == 1) return leader;
        if (mark == 2) continue;
        state.set(leader, 1);
        stack.push(leader);
        cursor.push(0);
      }
    }
    return null;
  }
}
