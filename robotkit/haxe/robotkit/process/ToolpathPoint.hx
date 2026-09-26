package robotkit.process;

import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * One waypoint of a `Toolpath`: a TCP pose expressed in the toolpath's own
 * frame (`work_T_tcp`, following `a_T_b`), the feed rate for the move that
 * reaches it, whether the tool process is on for that move, and an optional
 * desired surface normal/standoff a generator may attach for downstream use
 * (e.g. reachability or coverage checks).
 */
class ToolpathPoint {
  public final work_T_tcp:Transform3;
  public final feedRate:Float;
  public final processOn:Bool;
  public final normal:Null<Vec3>;
  public final standoff:Null<Float>;
  public final positionTolerance:Float;
  public final orientationTolerance:Float;

  public function new(work_T_tcp:Transform3, feedRate:Float, processOn:Bool,
      ?normal:Vec3, ?standoff:Float, ?positionTolerance:Float = 1e-3,
      ?orientationTolerance:Float = 1e-2) {
    if (work_T_tcp == null) throw "Toolpath point requires a work_T_tcp pose";
    if (!Math.isFinite(feedRate) || feedRate <= 0.0) throw "Toolpath point feed rate must be positive and finite";
    if (!Math.isFinite(positionTolerance) || positionTolerance < 0.0)
      throw "Toolpath point position tolerance must be finite and non-negative";
    if (!Math.isFinite(orientationTolerance) || orientationTolerance < 0.0)
      throw "Toolpath point orientation tolerance must be finite and non-negative";
    this.work_T_tcp = work_T_tcp;
    this.feedRate = feedRate;
    this.processOn = processOn;
    this.normal = normal;
    this.standoff = standoff;
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
  }
}
