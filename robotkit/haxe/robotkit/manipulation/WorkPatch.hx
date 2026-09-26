package robotkit.manipulation;

import robotkit.mobile.Pose2;
import robotkit.work.WorkSurface;
import robotkit.process.Toolpath;

/** One planned patch: its own sub-`WorkSurface`, raster `Toolpath`, and the base pose it is reachable from. */
class WorkPatch {
  public final surface:WorkSurface;
  public final toolpath:Toolpath;
  public final basePose:Pose2;
  public final reachableFraction:Float;

  public function new(surface:WorkSurface, toolpath:Toolpath, basePose:Pose2, reachableFraction:Float) {
    this.surface = surface;
    this.toolpath = toolpath;
    this.basePose = basePose;
    this.reachableFraction = reachableFraction;
  }
}
