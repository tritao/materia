package robotkit.world;

import haxe.Int64;

/** Immutable transport-neutral batch of timestamped trajectory samples. */
class TrajectoryChunk {
  public static inline final MAX_POINTS:Int = 256;
  public final points:Array<TrajectoryPoint>;
  public final jointCount:Int;
  public final tag:Int64;

  public function new(points:Array<TrajectoryPoint>, ?tag:Int64) {
    if (points == null || points.length == 0 || points.length > MAX_POINTS)
      throw 'Trajectory chunk needs one to $MAX_POINTS points';
    var first = points[0];
    if (first == null) throw "Trajectory chunk cannot contain null points";
    jointCount = first.positions.length;
    var previous = Int64.ofInt(0);
    var copied:Array<TrajectoryPoint> = [];
    for (index in 0...points.length) {
      var point = points[index];
      if (point == null) throw "Trajectory chunk cannot contain null points";
      if (point.positions.length != jointCount)
        throw "Trajectory chunk points must have the same joint count";
      if (index > 0 && Int64.compare(point.timeFromStartNs, previous) < 0)
        throw "Trajectory chunk point times must be monotonic";
      previous = point.timeFromStartNs;
      copied.push(point.copy());
    }
    this.points = copied;
    this.tag = tag == null ? Int64.ofInt(0) : tag;
  }

  public function copy():TrajectoryChunk return new TrajectoryChunk(points, tag);
}
