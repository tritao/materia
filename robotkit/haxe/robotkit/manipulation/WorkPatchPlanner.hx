package robotkit.manipulation;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.mobile.Pose2;
import robotkit.work.WorkSurface;
import robotkit.work.Polygon2;
import robotkit.work.Point2;
import robotkit.work.Provenance;
import robotkit.work.SourceKind;
import robotkit.work.RasterToolpathGenerator;
import robotkit.process.Toolpath;

/**
 * Splits a `WorkSurface` into axis-aligned column patches (no wider than
 * `maxPatchWidth`, mirroring `RasterToolpathGenerator`'s axis-aligned
 * scope), builds a raster `Toolpath` per patch, and searches a grid of
 * candidate base placements — in front of the wall, in a fixed standoff
 * band, facing the wall, at least `clearance` from every `BaseObstacle` — to
 * find one from which the whole patch toolpath is reachable
 * (`ReachabilityChecker`). Base motion between the returned patches is the
 * existing planar `Navigator`/`GoTo`, driven by each patch's `basePose`;
 * this planner does no joint base+arm optimization.
 */
class WorkPatchPlanner {
  public static function plan(design:WorkSurface, map_T_surface:Transform3, manipulator:Manipulator,
      maxPatchWidth:Float, toolWidth:Float, overlap:Float, standoff:Float, feedRate:Float, leadInOut:Float,
      standoffDistanceMin:Float, standoffDistanceMax:Float, seed:Array<Float>,
      ?standoffSteps:Int = 3, ?lateralSteps:Int = 5, ?clearance:Float = 0.4,
      ?obstacles:Array<BaseObstacle>, ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3):WorkPatchPlanResult {
    if (design == null) throw "Work patch planning requires a design work surface";
    if (map_T_surface == null) throw "Work patch planning requires a map_T_surface transform";
    if (manipulator == null) throw "Work patch planning requires a manipulator";
    if (!Math.isFinite(maxPatchWidth) || maxPatchWidth <= 0.0) throw "Work patch planning max patch width must be positive and finite";
    if (!Math.isFinite(standoffDistanceMin) || !Math.isFinite(standoffDistanceMax) || standoffDistanceMin > standoffDistanceMax)
      throw "Work patch planning requires standoffDistanceMin <= standoffDistanceMax";
    var obstacleList = obstacles == null ? [] : obstacles;

    var bounds = design.boundary.bounds();
    var width = bounds.maxX - bounds.minX;
    var columns = Math.ceil(width / maxPatchWidth);
    if (columns < 1) columns = 1;
    var columnWidth = width / columns;
    var patchCenterY = (bounds.minY + bounds.maxY) * 0.5;

    var patches:Array<WorkPatch> = [];
    var fullyPlanned = true;

    for (col in 0...columns) {
      var colMinX = bounds.minX + col * columnWidth;
      var colMaxX = colMinX + columnWidth;
      var patchBoundary = new Polygon2([
        new Point2(colMinX, bounds.minY), new Point2(colMaxX, bounds.minY),
        new Point2(colMaxX, bounds.maxY), new Point2(colMinX, bounds.maxY)
      ]);
      var patchExclusions:Array<Polygon2> = [];
      for (exclusion in design.exclusions) {
        var exclusionBounds = exclusion.bounds();
        var clippedMinX = Math.max(exclusionBounds.minX, colMinX);
        var clippedMaxX = Math.min(exclusionBounds.maxX, colMaxX);
        if (clippedMaxX - clippedMinX <= 1e-9) continue;
        patchExclusions.push(new Polygon2([
          new Point2(clippedMinX, exclusionBounds.minY), new Point2(clippedMaxX, exclusionBounds.minY),
          new Point2(clippedMaxX, exclusionBounds.maxY), new Point2(clippedMinX, exclusionBounds.maxY)
        ]));
      }

      var patchSurface = new WorkSurface('${design.id}:patch$col', design.frameId, design.frame_T_surface,
        patchBoundary, patchExclusions, design.tolerance, design.materialTag,
        new Provenance(design.provenance.designElementId, SourceKind.Work), '${design.surfaceFrameId}:patch$col');
      var patchToolpath = RasterToolpathGenerator.generate(patchSurface, toolWidth, overlap, standoff, feedRate, leadInOut);

      var midX = (colMinX + colMaxX) * 0.5;
      var patchWidthHalf = (colMaxX - colMinX) * 0.5;

      var best:Null<ReachabilityResult> = null;
      var bestPose:Null<Pose2> = null;
      for (standoffStep in 0...standoffSteps) {
        var d = standoffSteps == 1 ? standoffDistanceMin :
          standoffDistanceMin + (standoffDistanceMax - standoffDistanceMin) * (standoffStep / (standoffSteps - 1));
        for (lateralStep in 0...lateralSteps) {
          var lateral = lateralSteps == 1 ? 0.0 :
            -patchWidthHalf + (2.0 * patchWidthHalf) * (lateralStep / (lateralSteps - 1));
          var candidateLocal = new Vec3(midX + lateral, patchCenterY, d);
          var candidatePosMap = map_T_surface.transformPoint(candidateLocal);
          var wallPointMap = map_T_surface.transformPoint(new Vec3(midX + lateral, patchCenterY, 0.0));

          var blocked = false;
          for (obstacle in obstacleList) {
            var dx = candidatePosMap.x - obstacle.centerX;
            var dy = candidatePosMap.y - obstacle.centerY;
            if (Math.sqrt(dx * dx + dy * dy) < obstacle.radius + clearance) { blocked = true; break; }
          }
          if (blocked) continue;

          var yaw = Math.atan2(wallPointMap.y - candidatePosMap.y, wallPointMap.x - candidatePosMap.x);
          var candidateTransform = new Transform3(candidatePosMap, Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), yaw));
          var candidatePose = candidateTransform.toPose2();
          var base_T_work = candidateTransform.inverse().compose(map_T_surface);
          var result = ReachabilityChecker.check(manipulator, patchToolpath, base_T_work, seed,
            positionTolerance, orientationTolerance);

          if (best == null || result.reachableFraction > best.reachableFraction) {
            best = result;
            bestPose = candidatePose;
          }
          if (result.fullyReachable()) break;
        }
        if (best != null && best.fullyReachable()) break;
      }

      if (best == null) throw 'Work patch planner found no viable base candidate for patch $col (all candidates blocked by obstacles)';
      if (!best.fullyReachable()) fullyPlanned = false;
      patches.push(new WorkPatch(patchSurface, patchToolpath, bestPose, best.reachableFraction));
    }

    return new WorkPatchPlanResult(patches, fullyPlanned);
  }
}
