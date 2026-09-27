package robotkit.world;

import haxe.Int64;

/**
 * Immutable transport-neutral batch of samples or polynomial segments.
 *
 * A chunk normally appends to the runtime's queued path. With a non-zero
 * `spliceTag`, it instead replaces queued motion from the knot
 * `spliceTimeNs` into the queued chunk tagged `spliceTag`; the runtime drops
 * a splice whose point its trajectory clock has already reached.
 */
class TrajectoryChunk {
  public static inline final MAX_POINTS:Int = 256;
  public final points:Array<TrajectoryPoint>;
  public final segments:Array<TrajectorySegment>;
  public final jointCount:Int;
  public final tag:Int64;
  public final spliceTag:Int64;
  public final spliceTimeNs:Int64;

  public function new(points:Array<TrajectoryPoint>, ?tag:Int64, ?spliceTag:Int64,
      ?spliceTimeNs:Int64, ?segments:Array<TrajectorySegment>) {
    if (segments != null) {
      if (points != null && points.length > 0)
        throw "Trajectory chunk cannot mix points and segments";
      if (segments.length == 0 || segments.length > 128)
        throw "Trajectory chunk needs one to 128 segments";
      var first = segments[0];
      if (first == null) throw "Trajectory chunk cannot contain null segments";
      jointCount = first.jointCount;
      var expected = Int64.ofInt(0);
      var coefficients = 0;
      var copied:Array<TrajectorySegment> = [];
      for (segment in segments) {
        if (segment == null || segment.jointCount != jointCount ||
            Int64.compare(segment.timeFromStartNs, expected) != 0)
          throw "Trajectory segments must be contiguous with one joint count";
        expected = Int64.add(expected, segment.durationNs);
        coefficients += jointCount * (segment.degree + 1);
        if (coefficients > 4096) throw "Trajectory chunk coefficient budget exceeded";
        copied.push(segment.copy());
      }
      this.points = [];
      this.segments = copied;
    } else {
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
      this.segments = [];
    }
    this.tag = tag == null ? Int64.ofInt(0) : tag;
    this.spliceTag = spliceTag == null ? Int64.ofInt(0) : spliceTag;
    this.spliceTimeNs = spliceTimeNs == null ? Int64.ofInt(0) : spliceTimeNs;
    if (Int64.compare(this.spliceTimeNs, Int64.ofInt(0)) < 0)
      throw "Trajectory chunk splice time must be non-negative";
  }

  public static function fromSegments(segments:Array<TrajectorySegment>, ?tag:Int64,
      ?spliceTag:Int64, ?spliceTimeNs:Int64):TrajectoryChunk
    return new TrajectoryChunk([], tag, spliceTag, spliceTimeNs, segments);

  public function copy():TrajectoryChunk
    return new TrajectoryChunk(points, tag, spliceTag, spliceTimeNs,
      segments.length == 0 ? null : segments);
}
