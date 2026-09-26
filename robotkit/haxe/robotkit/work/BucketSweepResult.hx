package robotkit.work;

/** Outcome of one `BucketSweep.apply` call. */
class BucketSweepResult {
  public final removedVolume:Float;
  public final verticesLowered:Int;

  public function new(removedVolume:Float, verticesLowered:Int) {
    if (!Math.isFinite(removedVolume) || removedVolume < 0.0)
      throw "Bucket sweep removed volume must be finite and non-negative";
    if (verticesLowered < 0) throw "Bucket sweep vertices-lowered count must be non-negative";
    this.removedVolume = removedVolume;
    this.verticesLowered = verticesLowered;
  }
}
