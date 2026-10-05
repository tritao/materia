package motionkit.robot;

import motionkit.trajectory.PlanDiagnostic.PlanCheckResult;
import motionkit.trajectory.PlanDiagnostic.PlanSlip;
import robotkit.core.CoupledJoint;

/**
 * Carries out what a plan check predicts for steppers: where the motors of an axis would lose sync, the
 * simulated axis falls behind its command and keeps the error, as a stepper that lost steps does. The
 * slip is a per-joint offset applied to the commands in the simulation's endpoint, not a change to the
 * plan, so the runtime and a monitor still see the commanded positions and the machine the real ones.
 *
 * A plan the check passes has no slip, so it never changes what the machine does. The caller feeds it
 * each plan's check when the plan starts and the plan's elapsed time as it runs. Joints coupled to a
 * slipping axis get the offset through their couplings.
 */
class StepperSlip {
  final couplings:Array<CoupledJoint>;
  final jointOfAxis:Map<String, Int>;
  final sink:(joint:Int, offset:Float) -> Void;
  /** Distance each axis is behind its command, with the running plan's share, by axis id. */
  final lostNow = new Map<String, Float>();
  final lostBefore = new Map<String, Float>();
  final stepsBefore = new Map<String, Float>();
  var running:Array<PlanSlip> = [];
  var touched = new Map<Int, Float>();
  var referenceOffsets = new Map<Int, Float>();

  /**
   * `jointOfAxis` maps an axis id to its robot joint index; `couplings` are the robot's, to carry the
   * offset to coupled joints; `sink` sets one joint's offset in the simulation.
   */
  public function new(couplings:Array<CoupledJoint>, jointOfAxis:Map<String, Int>, sink:(joint:Int, offset:Float) -> Void) {
    this.couplings = couplings.copy();
    this.jointOfAxis = jointOfAxis;
    this.sink = sink;
  }

  /** A plan starts: what the check found for it is carried out as it runs. */
  public function start(checked:Null<PlanCheckResult>):Void {
    settle();
    running = checked == null ? [] : checked.slips;
  }

  /** The running plan is `elapsed` seconds in. */
  public function update(elapsed:Float):Void {
    for (slip in running) {
      var before = lostBefore.get(slip.axis);
      lostNow.set(slip.axis, (before == null ? 0.0 : before) + slip.lostAt(elapsed));
    }
    apply();
  }

  /** The plan ended, completed or not: what was lost so far is kept. */
  public function settle():Void {
    for (slip in running) {
      var now = lostNow.get(slip.axis);
      lostBefore.set(slip.axis, now == null ? 0.0 : now);
    }
    running = [];
  }

  /** Distance axis `axis` is behind its command, in its units, since the last reset: signed along the motion. */
  public function lost(axis:String):Float {
    var found = lostNow.get(axis);
    return found == null ? 0.0 : found;
  }

  /** Whether any axis has lost steps. */
  public function slipped():Bool {
    for (axis in lostNow.keys()) if (lostNow.get(axis) != 0.0) return true;
    return false;
  }

  /** Forgets all lost steps, as homing the machine does, and puts every joint back on its command. */
  public function reset():Void {
    referenceOffsets.clear();
    running = [];
    for (axis in lostNow.keys()) lostNow.set(axis, 0.0);
    lostBefore.clear();
    apply();
  }

  /** Clear loss history after calibration without changing any physical endpoint offset. */
  public function rebaseAfterHoming():Void {
    running = [];
    lostNow.clear(); lostBefore.clear(); stepsBefore.clear();
    referenceOffsets = new Map<Int, Float>();
    for (joint in touched.keys()) referenceOffsets.set(joint, touched.get(joint));
  }

  /** Writes each axis's offset (behind its command, so negative along the motion) to its joint and what couples to it. */
  function apply():Void {
    var offsets = new Map<Int, Float>();
    for (axis in lostNow.keys()) {
      var joint = jointOfAxis.get(axis);
      if (joint == null) continue;
      offsets.set(joint, -lostNow.get(axis));
    }
    // Followers move by their ratios times their leaders (summed over several), so offsets propagate down the couplings.
    var followers:Array<Int> = [];
    for (coupling in couplings) if (followers.indexOf(coupling.follower) < 0) followers.push(coupling.follower);
    var changed = true, passes = 0;
    while (changed && passes++ <= couplings.length) {
      changed = false;
      for (follower in followers) {
        var wanted = 0.0, any = false;
        for (coupling in couplings) if (coupling.follower == follower) {
          var leader = offsets.get(coupling.leader);
          if (leader == null) continue;
          wanted += coupling.ratio * leader;
          any = true;
        }
        if (!any) continue;
        var current = offsets.get(follower);
        if (current == null || current != wanted) {
          offsets.set(follower, wanted);
          changed = true;
        }
      }
    }
    for (joint in referenceOffsets.keys()) {
      var loss = offsets.get(joint);
      offsets.set(joint, referenceOffsets.get(joint) + (loss == null ? 0.0 : loss));
    }
    for (joint in offsets.keys()) {
      var previous = touched.get(joint);
      var offset = offsets.get(joint);
      if (previous == null || previous != offset) sink(joint, offset);
    }
    for (joint in touched.keys()) if (!offsets.exists(joint)) sink(joint, 0.0);
    touched = offsets;
  }
}
