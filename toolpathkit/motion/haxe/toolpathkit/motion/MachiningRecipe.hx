package toolpathkit.motion;

/** Restart settings for machining, independent of dispensing rates. */
class MachiningRecipe {
  public final approachFeed:Float;
  public final restartBackoff:Float;
  /**
    Height in the motion frame that clears the work and its fixtures. With
    it, a restart retracts there, moves across and descends to the restart
    point; without it, it moves straight there.
  **/
  public final clearanceZ:Null<Float>;

  public function new(approachFeed:Float, ?restartBackoff:Float = 0.0, ?clearanceZ:Float) {
    if (!Math.isFinite(approachFeed) || approachFeed <= 0.0 ||
        !Math.isFinite(restartBackoff) || restartBackoff < 0.0 ||
        (clearanceZ != null && !Math.isFinite(clearanceZ)))
      throw "machining recipe needs positive approach feed, nonnegative backoff and a finite clearance";
    this.approachFeed = approachFeed;
    this.restartBackoff = restartBackoff;
    this.clearanceZ = clearanceZ;
  }
}
