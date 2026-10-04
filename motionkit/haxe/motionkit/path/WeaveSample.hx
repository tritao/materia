package motionkit.path;

/** Offset and exact first/second derivatives with respect to seam progress. */
class WeaveSample {
  public final offset:Float;
  public final first:Float;
  public final second:Float;
  public function new(offset:Float, first:Float, second:Float) {
    this.offset = offset; this.first = first; this.second = second;
  }
}
