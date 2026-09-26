package robotkit.process;

/**
 * An ordered sequence of `ToolpathPoint`s expressed in one named frame
 * (`frameId`; a `WorkSurface` frame, or any other frame a `FrameTree3` can
 * resolve into the manipulator's base frame). Points may be empty only for
 * a segment produced by `segmentByProcess`.
 */
class Toolpath {
  public final frameId:String;
  public final points:Array<ToolpathPoint>;

  public function new(frameId:String, points:Array<ToolpathPoint>) {
    if (frameId == null || frameId.length == 0) throw "Toolpath requires a non-empty frame id";
    if (points == null) throw "Toolpath requires a points array";
    this.frameId = frameId;
    this.points = points.copy();
  }

  /** Sum of straight-line distances between consecutive points. */
  public function length():Float {
    var total = 0.0;
    for (i in 0...(points.length - 1))
      total += points[i + 1].work_T_tcp.translation.sub(points[i].work_T_tcp.translation).norm();
    return total;
  }

  /**
   * Sum of straight-line distances for moves whose *departing* point has
   * `processOn == true` (the point's process state governs the move to the
   * next point, matching `CartesianTrajectory`'s per-segment sampling).
   */
  public function processOnLength():Float {
    var total = 0.0;
    for (i in 0...(points.length - 1)) if (points[i].processOn)
      total += points[i + 1].work_T_tcp.translation.sub(points[i].work_T_tcp.translation).norm();
    return total;
  }

  /**
   * Splits into a leading `approach` run (processOn == false), the middle
   * `process` run from the first to the last processOn == true point
   * inclusive, and a trailing `retract` run. If no point has processOn ==
   * true, `approach` holds every point and `process`/`retract` are empty.
   */
  public function segmentByProcess():ToolpathSegments {
    var firstOn = -1, lastOn = -1;
    for (i in 0...points.length) if (points[i].processOn) {
      if (firstOn < 0) firstOn = i;
      lastOn = i;
    }
    if (firstOn < 0)
      return new ToolpathSegments(new Toolpath(frameId, points), new Toolpath(frameId, []), new Toolpath(frameId, []));
    return new ToolpathSegments(
      new Toolpath(frameId, points.slice(0, firstOn)),
      new Toolpath(frameId, points.slice(firstOn, lastOn + 1)),
      new Toolpath(frameId, points.slice(lastOn + 1, points.length))
    );
  }
}

/** Result of `Toolpath.segmentByProcess`; each part shares the source toolpath's frame. */
class ToolpathSegments {
  public final approach:Toolpath;
  public final process:Toolpath;
  public final retract:Toolpath;

  public function new(approach:Toolpath, process:Toolpath, retract:Toolpath) {
    this.approach = approach;
    this.process = process;
    this.retract = retract;
  }
}
