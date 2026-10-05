package processkit;

import processkit.ContactProbeRunner.ContactProbeRequest;
import motionkit.kinematics.Pose3;
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

  /** Wire +Z points inward; roll is chosen by reach and clearance, nearest the observed torch orientation first. */
  public function prepare(point:Vec3, outward:Vec3, normalTravel:Float, start:Array<Float>, contactOffset:Float,
      airMargin:Float = 0.003, rolls:Int = 16):ContactProbeRequest {
    if (point == null || outward == null || Math.abs(outward.norm() - 1) > 1e-8 ||
        !Math.isFinite(normalTravel) || normalTravel < 0 || start == null || start.length != motion.arm.group.count() ||
        !Math.isFinite(airMargin) || !(airMargin > 0) || rolls < 1 || rolls > 64)
      throw "Probe pose preparation needs a unit plane normal, observed joints and finite search bounds";
    var current = motion.arm.tcpPose(start).rotation;
    var z = outward.scale(-1);
    var preferred = current.rotate(new Vec3(1, 0, 0));
    var projected = preferred.sub(z.scale(preferred.dot(z)));
    var fallback = Math.abs(z.x) < 0.9 ? new Vec3(1, 0, 0) : new Vec3(0, 1, 0);
    var x = projected.norm() > 1e-8 ? projected.normalized() : fallback.sub(z.scale(fallback.dot(z))).normalized();
    var y = z.cross(x);
    var base = Quat.fromRotationMatrix([x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z]);
    var orientations = [for (i in 0...rolls) base.multiply(Quat.fromAxisAngle(new Vec3(0, 0, 1), 2 * Math.PI * i / rolls))];
    orientations.sort((a, b) -> Reflect.compare(a.angularDistance(current), b.angularDistance(current)));
    var approachPoint = point.add(outward.scale(normalTravel + airMargin));
    var distance = 2 * normalTravel + airMargin;
    var reasons:Array<String> = [];
    for (rotation in orientations) {
      try {
        var approach = new Transform3(approachPoint, rotation);
        var request = new ContactProbeRequest(approach, outward.scale(-1), distance, 0.01, 0.0005, 0.002, contactOffset);
        var checked = motion.approach(approach, start);
        // Contact is not yet localized, so a nominal contact move could penetrate the real plane.
        // Check corridor kinematics here; CAD region screening and measured servo/brake checks own obstruction checks.
        var q = checked.endJoints.copy();
        var steps = Std.int(Math.max(1.0, Math.ceil(distance / 0.001)));
        for (step in 1...steps + 1) {
          var at = approachPoint.sub(outward.scale(distance * step / steps));
          var next = motion.compiler.solver.solvePose(pose(new Transform3(at, rotation)), q, motion.compiler.ikTolerance);
          if (next == null) throw "Probe sensing corridor leaves the reachable IK branch";
          q = next;
        }
        return request;
      } catch (error:Dynamic) reasons.push(Std.string(error));
    }
    throw 'Probe has no checked reachable normal orientation: ${reasons.join("; ")}';
  }
}
