package processkit;

import processkit.WeldStationPlanner.WeldStationCandidate;
import robotkit.mobile.Pose2;
import robotkit.spatial.Vec3;

/**
 * Parking candidates around the bounds of CAD seam endpoints in their work frame.
 * Standoff is measured from the mounted arm, not the chassis: a long carrier can park its
 * arm close to work while keeping its deck farther away. The work frame defines the grid's
 * axes, so translating or rotating the work translates or rotates every candidate.
 * These are proposals only; navigation, reach and swept clearance decide their coverage.
 */
class WeldStationCandidates {
  public static function around(points:Array<Vec3>, map_T_work:Pose2, chassis_T_arm:Vec3,
      minimumStandoff:Float, maximumStandoff:Float, radialSteps:Int = 3, headings:Int = 12):Array<WeldStationCandidate> {
    if (points == null || points.length == 0 || map_T_work == null || chassis_T_arm == null)
      throw "Station candidates require seam endpoints, work placement and the mounted arm offset";
    if (!Math.isFinite(minimumStandoff) || !Math.isFinite(maximumStandoff) || minimumStandoff <= 0 ||
        maximumStandoff < minimumStandoff || radialSteps < 1 || headings < 4)
      throw "Station candidates require positive standoffs, ordered bounds and a finite sampling grid";
    var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY;
    var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
    for (point in points) {
      if (point == null || !Math.isFinite(point.x) || !Math.isFinite(point.y) || !Math.isFinite(point.z))
        throw "Station candidate seam endpoints must be finite";
      minX = Math.min(minX, point.x); minY = Math.min(minY, point.y);
      maxX = Math.max(maxX, point.x); maxY = Math.max(maxY, point.y);
    }
    if (!Math.isFinite(chassis_T_arm.x) || !Math.isFinite(chassis_T_arm.y) || !Math.isFinite(chassis_T_arm.z))
      throw "Station candidate arm offset must be finite";
    var x = minX / 2 + maxX / 2, y = minY / 2 + maxY / 2;
    var result:Array<WeldStationCandidate> = [];
    for (heading in 0...headings) {
      var yaw = 2 * Math.PI * heading / headings;
      var c = Math.cos(yaw), s = Math.sin(yaw);
      for (radial in 0...radialSteps) {
        var distance = radialSteps == 1 ? minimumStandoff :
          minimumStandoff + (maximumStandoff - minimumStandoff) * radial / (radialSteps - 1);
        var armX = x - c * distance, armY = y - s * distance;
        var baseX = armX - (c * chassis_T_arm.x - s * chassis_T_arm.y);
        var baseY = armY - (s * chassis_T_arm.x + c * chassis_T_arm.y);
        result.push(new WeldStationCandidate('station:$heading:$radial', map_T_work.compose(new Pose2(baseX, baseY, yaw))));
      }
    }
    return result;
  }
}
