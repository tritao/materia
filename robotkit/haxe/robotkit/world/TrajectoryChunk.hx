package robotkit.world;

import haxe.Int64;

/** Immutable transport-neutral batch of contiguous polynomial segments. */
class TrajectoryChunk {
  public final segments:Array<TrajectorySegment>;
  public final jointCount:Int;
  public final tag:Int64;

  public function new(segments:Array<TrajectorySegment>, ?tag:Int64) {
    if (segments == null || segments.length == 0 || segments.length > 128)
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
    this.segments = copied;
    this.tag = tag == null ? Int64.ofInt(0) : tag;
  }

  public static function fromSegments(segments:Array<TrajectorySegment>, ?tag:Int64):TrajectoryChunk
    return new TrajectoryChunk(segments, tag);

  public function copy():TrajectoryChunk
    return new TrajectoryChunk(segments, tag);
}
