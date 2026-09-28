package toolpathkit.motion;

/** Restart settings for machining, independent of dispensing rates. */
class MachiningRecipe {
  public final approachFeed:Float;
  public final restartBackoff:Float;

  public function new(approachFeed:Float, ?restartBackoff:Float = 0.0) {
    if (!Math.isFinite(approachFeed) || approachFeed <= 0.0 ||
        !Math.isFinite(restartBackoff) || restartBackoff < 0.0)
      throw "machining recipe needs positive approach feed and nonnegative backoff";
    this.approachFeed = approachFeed;
    this.restartBackoff = restartBackoff;
  }
}
