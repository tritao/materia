package processkit;

import processkit.ContactProbeRunner.ContactProbeRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.OrientationPolicy;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Prepare a normal contact search with checked air access and a reachable bounded sensing corridor. */
class ProbePosePlanner {
  public final motion:ProbeMotionPlanner;
  public function new(motion:ProbeMotionPlanner) {
    if (motion == null) throw "Probe pose preparation needs a checked motion planner";
    this.motion = motion;
  }
  static function pose(frame:Transform3):Pose3 {
    var p = frame.translation, q = frame.rotation;
    return new Pose3(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  }
  static function orientations(outward:Vec3, current:Quat, rolls:Int):Array<Quat> {
    var z = outward.scale(-1);
    var preferred = current.rotate(new Vec3(1, 0, 0));
    var projected = preferred.sub(z.scale(preferred.dot(z)));
    var fallback = Math.abs(z.x) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0);
    var x = projected.norm() > 1e-8 ? projected.normalized() : fallback.sub(z.scale(fallback.dot(z))).normalized();
    var y = z.cross(x);
    var base = Quat.fromRotationMatrix([x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z]);
    var result = [for (i in 0...rolls) base.multiply(Quat.fromAxisAngle(new Vec3(0, 0, 1), 2 * Math.PI * i / rolls))];
    result.sort((a, b) -> Reflect.compare(a.angularDistance(current), b.angularDistance(current)));
    return result;
  }

  /** Candidate screening only: a clear endpoint does not authorize the approach trajectory. */
  public function hasClearApproachConfiguration(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>):Bool {
    return checkedApproachLift(point, outward, normalTravel, start) != null;
  }

  /** Prefer configurations reached by continuation from observed joints; global discovery remains a separate fallback. */
  public function hasClearObservedApproachConfiguration(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>):Bool {
    return observedApproachLift(point, outward, normalTravel, start) != null;
  }

  /** Additional standoff lift that lets a geometric candidate pass the inexpensive reach/clearance screen. */
  public function checkedApproachLift(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>,
      maxLift:Float = 0.04, airMargin:Float = 0.003):Null<Float> {
    return clearApproachLift(point, outward, normalTravel, start, true, maxLift, airMargin);
  }

  /** The corresponding lift on the observed IK branch, without global discovery. */
  public function observedApproachLift(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>,
      maxLift:Float = 0.04, airMargin:Float = 0.003):Null<Float> {
    return clearApproachLift(point, outward, normalTravel, start, false, maxLift, airMargin);
  }

  function clearApproachLift(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>, discover:Bool,
      maxLift:Float, airMargin:Float):Null<Float> {
    if (point == null || outward == null || Math.abs(outward.norm() - 1) > 1e-8 ||
        !Math.isFinite(normalTravel) || normalTravel < 0 || start == null || start.length != motion.arm.group.count() ||
        !Math.isFinite(maxLift) || maxLift < 0 || maxLift > 0.04 || !Math.isFinite(airMargin) || !(airMargin > 0))
      throw "Probe candidate screening needs a unit normal, observed joints and finite search bounds";
    var current = motion.arm.tcpPose(start).rotation;
    var fullTolerance = motion.compiler.ikTolerance;
    // This is a cheap candidate screen, not the proof of a probe motion. A
    // free-about-tool solve tests the constrained position and wire axis while
    // avoiding sixteen full-orientation solves for every lattice point. The
    // selected points still go through prepareChecked, which searches rolls,
    // plans the air move and proves the complete sensing corridor. A global
    // screen gets one free-roll sample; exhaustive branch search belongs to
    // the selected point, not every lattice candidate.
    var screenTolerance = new IkTolerance(fullTolerance.position, fullTolerance.orientation,
      Std.int(Math.min(64, fullTolerance.maxIterations)), fullTolerance.damping, fullTolerance.candidateSeparation);
    for (lift in [0.0, 0.005, 0.01, 0.02, 0.04]) {
      if (lift > maxLift + 1e-9) break;
      var at = point.add(outward.scale(normalTravel + airMargin + lift + motion.airPoseReserve));
      var targets = [for (rotation in orientations(outward, current, 16)) pose(new Transform3(at, rotation))];
      for (q in axisGoals(targets[0], start, discover, 1, screenTolerance))
        if (axisMatches(q, at, outward) && motion.violation(q) == null) return lift;
      // Try every roll on the observed branch before paying for global branch discovery.
      for (target in targets) {
        var q = motion.inverse.solvePose(target, start, motion.compiler.ikTolerance);
        if (q != null && motion.violation(q) == null) return lift;
      }
    }
    return null;
  }

  function axisGoals(target:Pose3, start:Array<Float>, discover:Bool, maxCandidates:Int = 12,
      ?tolerance:IkTolerance):Array<Array<Float>> {
    // EAIK solves fixed orientations. The shared roll grid below supplies
    // that freedom; do not route a supported arm back through numeric IK.
    if(Std.isOfType(motion.inverse,motionkit.robot.EaikKinematics))return [];
    var solveTolerance = tolerance == null ? motion.compiler.ikTolerance : tolerance;
    if (discover) return motion.inverse.sampleCandidates(target, maxCandidates,
      solveTolerance, OrientationPolicy.FreeAboutTool);
    var q = motion.inverse.solvePose(target, start, solveTolerance, OrientationPolicy.FreeAboutTool);
    return q == null ? [] : [q];
  }

  function axisMatches(q:Array<Float>, point:Vec3, outward:Vec3):Bool {
    var actual = motion.arm.tcpPose(q);
    var inward = actual.rotation.rotate(new Vec3(0, 0, 1));
    var angle = Math.acos(Math.max(-1.0, Math.min(1.0, inward.dot(outward.scale(-1)))));
    return actual.translation.sub(point).norm() <= motion.compiler.ikTolerance.position
      && angle <= motion.compiler.ikTolerance.orientation;
  }

  /** Wire +Z points inward; roll is chosen by reach and clearance, nearest the observed torch orientation first. */
  public function prepare(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>, contactOffset:Float,
      airMargin:Float = 0.003, rolls:Int = 16, initialLift:Float = 0.0):ContactProbeRequest {
    return prepareChecked(point, outward, normalTravel, start, contactOffset, airMargin, rolls, initialLift, true);
  }

  /** Local refinement must not trigger global discovery when a checked continuation fails. */
  public function prepareObserved(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>, contactOffset:Float,
      airMargin:Float = 0.003, rolls:Int = 16, initialLift:Float = 0.0):ContactProbeRequest {
    return prepareChecked(point, outward, normalTravel, start, contactOffset, airMargin, rolls, initialLift, false);
  }

  function prepareChecked(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>, contactOffset:Float,
      airMargin:Float, rolls:Int, initialLift:Float, discover:Bool):ContactProbeRequest {
    if (point == null || outward == null || Math.abs(outward.norm() - 1) > 1e-8 ||
        !Math.isFinite(normalTravel) || normalTravel < 0 || start == null || start.length != motion.arm.group.count() ||
        !Math.isFinite(airMargin) || !(airMargin > 0) || !Math.isFinite(initialLift) || initialLift < 0 || initialLift > 0.04 ||
        rolls < 1 || rolls > 64)
      throw "Probe pose preparation needs a unit plane normal, observed joints and finite search bounds";
    var current = motion.arm.tcpPose(start).rotation;
    var candidates = orientations(outward, current, rolls);
    var reasons:Array<String> = [];
    function note(error:Dynamic):Void {
      var reason = Std.string(error);
      if (reasons.length < 8 && reasons.indexOf(reason) < 0) reasons.push(reason);
    }
    // The CAD uncertainty sets the farthest possible contact plane. A low
    // standoff can still put the torch housing into a neighbouring member, so
    // try bounded extra lifts. Extend the commanded sensing distance by the
    // same lift: this moves the start pose outward without widening the
    // uncertainty interval the probe is allowed to search.
    var lifts = [initialLift];
    for (lift in [0.0, 0.005, 0.01, 0.02, 0.04]) if (lift > initialLift + 1e-9) lifts.push(lift);
    for (lift in lifts) {
      var standoff = airMargin + lift;
      var approachPoint = point.add(outward.scale(normalTravel + standoff + motion.airPoseReserve));
      var distance = 2 * normalTravel + standoff + motion.airPoseReserve;
      for (pass in 0...(discover ? 2 : 1)) {
        // The contact task fixes the wire axis; a clear reachable roll is a choice,
        // not an extra orientation constraint. Lock the selected actual pose below.
        var axisTarget = pose(new Transform3(approachPoint, candidates[0]));
        for (q in axisGoals(axisTarget, start, pass != 0)) {
          try {
            if (!axisMatches(q, approachPoint, outward) || motion.violation(q) != null)
              throw "Probe axis candidate is outside the task or clearance bounds";
            var checked = pass == 0 ? motion.observedApproachJoints(q, start)
              : motion.approachJoints(q, start);
            if (!motion.corridorReachable(checked.endJoints, outward.scale(-1), distance))
              throw "Probe sensing corridor leaves the reachable IK branch";
            return new ContactProbeRequest(motion.arm.tcpPose(checked.endJoints), outward.scale(-1), distance,
              0.01, 0.0005, 0.002, contactOffset, checked.endJoints);
          } catch (error:Dynamic) note(error);
        }
        // Position-only candidates keep search cheap when they work. If none
        // survives the checked route, try the explicit tool rolls on the same
        // branch first, then on globally discovered branches.
        for (rotation in candidates) {
          try {
            var approach = new Transform3(approachPoint, rotation);
            var checked = pass == 0 ? motion.observedApproach(approach, start) : motion.approach(approach, start);
            // Contact is not yet localized, so a nominal contact move could penetrate the real plane.
            // Check corridor kinematics here; CAD region screening and measured servo/brake checks own obstruction checks.
            if (!motion.corridorReachable(checked.endJoints, outward.scale(-1), distance))
              throw "Probe sensing corridor leaves the reachable IK branch";
            return new ContactProbeRequest(approach, outward.scale(-1), distance, 0.01, 0.0005, 0.002, contactOffset, checked.endJoints);
          } catch (error:Dynamic) note(error);
        }
      }
    }
    throw 'Probe has no checked reachable normal orientation: ${reasons.join("; ")}';
  }
}
