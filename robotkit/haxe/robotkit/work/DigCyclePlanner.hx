package robotkit.work;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;

/**
 * Plans one excavator dig cycle (entry -> cut -> curl -> lift -> swing ->
 * dump) as a `Toolpath`, for a 4-DOF chain shaped like `slew (Z) -> boom (Y)
 * -> stick (Y) -> bucket (Y)`: because every non-slew joint turns about a
 * parallel axis, the chain's *only* reachable tip orientations are
 * `Rz(slew) * Ry(totalPitch)` (see `poseAt`, and ARCHITECTURE.md's
 * "Simulated excavator (M12)" section for the derivation from
 * `KinematicChain.evaluate`'s own composition order). Every waypoint this
 * planner emits sets its orientation from `poseAt`, so it always lies
 * exactly on that manifold; the existing generic (undamped-least-squares)
 * `InverseKinematics`/`ToolpathExecutor` then converges to near-zero
 * position *and* orientation error with no weighting or closed-form solver
 * of its own, since a target already on the manifold leaves the "extra" two
 * 6-DOF error components at zero by construction.
 */
class DigCyclePlanner {
  /** `Rz(atan2(y, x)) * Ry(pitch)`: the one tip orientation this chain shape can reach at `(x, y)`. */
  public static function poseAt(x:Float, y:Float, z:Float, pitch:Float):Transform3 {
    var slew = Math.atan2(y, x);
    var rotation = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), slew).multiply(Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), pitch));
    return new Transform3(new Vec3(x, y, z), rotation);
  }

  /**
   * `entry`/`exit` are the cutting edge's XY (chain-base frame) at the start
   * and end of the cut; `groundZ` the terrain elevation there before this
   * cycle; `depth` how far below `groundZ` to cut. The bucket lifts clear at
   * `clearanceZ`, swings through `swingSteps` intermediate waypoints (each
   * built from `poseAt`, so the swing stays on the reachable manifold rather
   * than relying on `CartesianTrajectory`'s slerp across a wide arc) to
   * `dump` at `dumpZ`, then opens to `dumpPitch`. Returns both the
   * `Toolpath` and the full swept-segment parameters
   * (`BucketSweep.apply(heightMap, sweepFrom, sweepTo, sweepHalfWidth,
   * sweepEdgeHeight)`) a caller applies once the cycle's execution succeeds.
   * The engaged cutting edge is inset by half the bucket width at each
   * longitudinal trench wall, while the material sweep remains clipped to the
   * authored region. This keeps the tool out of the wall and accounts for the
   * whole bucket footprint in the terrain update.
   */
  public static function planCycle(frameId:String, entry:Point2, exit:Point2, groundZ:Float, depth:Float,
      clearanceZ:Float, dump:Point2, dumpZ:Float, bucketHalfWidth:Float,
      digPitch:Float, curlPitch:Float, dumpPitch:Float, feedRate:Float, ?swingSteps:Int = 8):DigCyclePlan {
    if (frameId == null || frameId.length == 0) throw "Dig cycle planning requires a non-empty frame id";
    if (entry == null || exit == null || dump == null) throw "Dig cycle planning requires entry, exit, and dump points";
    if (!Math.isFinite(groundZ)) throw "Dig cycle ground elevation must be finite";
    if (!Math.isFinite(depth) || depth <= 0.0) throw "Dig cycle depth must be positive and finite";
    if (!Math.isFinite(clearanceZ) || clearanceZ <= groundZ) throw "Dig cycle clearance height must be above ground";
    if (!Math.isFinite(dumpZ)) throw "Dig cycle dump elevation must be finite";
    if (!Math.isFinite(bucketHalfWidth) || bucketHalfWidth <= 0.0) throw "Dig cycle bucket half-width must be positive and finite";
    if (!Math.isFinite(feedRate) || feedRate <= 0.0) throw "Dig cycle feed rate must be positive and finite";
    if (swingSteps < 1) throw "Dig cycle swing requires at least one step";

    var cutDepthZ = groundZ - depth;
    var sweepFrom = entry;
    var sweepTo = exit;
    var dx = exit.x - entry.x, dy = exit.y - entry.y;
    var length = Math.sqrt(dx * dx + dy * dy);
    if (length > 2.0 * bucketHalfWidth + 1e-9) {
      var offsetX = dx / length * bucketHalfWidth;
      var offsetY = dy / length * bucketHalfWidth;
      sweepFrom = new Point2(entry.x + offsetX, entry.y + offsetY);
      sweepTo = new Point2(exit.x - offsetX, exit.y - offsetY);
    }
    var points:Array<ToolpathPoint> = [];
    // 1. Entry: cutting edge just above the ground at the entry point.
    points.push(new ToolpathPoint(poseAt(sweepFrom.x, sweepFrom.y, groundZ + 0.05, digPitch), feedRate, false));
    // 2. Cut: advance along the trench line down to the target depth, engaged with the ground.
    points.push(new ToolpathPoint(poseAt(sweepTo.x, sweepTo.y, cutDepthZ, digPitch), feedRate, true));
    // 3. Curl: still engaged, rotate the bucket closed to retain the cut material.
    points.push(new ToolpathPoint(poseAt(sweepTo.x, sweepTo.y, cutDepthZ, curlPitch), feedRate, true));
    // 4. Lift: raise clear of the trench, bucket still closed.
    points.push(new ToolpathPoint(poseAt(sweepTo.x, sweepTo.y, clearanceZ, curlPitch), feedRate, false));
    // 5. Swing: sweep toward the dump point at clearance height, dense enough to stay on the manifold.
    for (step in 1...(swingSteps + 1)) {
      var t = step / swingSteps;
      var x = sweepTo.x + (dump.x - sweepTo.x) * t;
      var y = sweepTo.y + (dump.y - sweepTo.y) * t;
      points.push(new ToolpathPoint(poseAt(x, y, clearanceZ, curlPitch), feedRate, false));
    }
    // 6. Descend to the dump elevation, still curled (holding the material).
    points.push(new ToolpathPoint(poseAt(dump.x, dump.y, dumpZ, curlPitch), feedRate, false));
    // 7. Dump: open the bucket in place (position held, only pitch changes) --
    // kept as its own point (rather than combined with the descent above) so
    // every waypoint changes at most one of "position" or "pitch" at a time,
    // keeping each step's own 3R (boom/stick/bucket) sub-problem well away
    // from the reachable-workspace boundary a combined jump could cross.
    points.push(new ToolpathPoint(poseAt(dump.x, dump.y, dumpZ, dumpPitch), feedRate, false));

    var toolpath = new Toolpath(frameId, points);
    return new DigCyclePlan(toolpath, entry, exit, bucketHalfWidth, cutDepthZ, sweepFrom, sweepTo);
  }
}
