package camkit;

/** Planned cutting pass, mm/radians/seconds. Excludes positioning and spindle settling. */
class HobbingPass {
  public final depth:Float;
  public final shift:Float;
  public final centreDistance:Float;
  public final engagement:HobbingEngagement;
  public final zStart:Float;
  public final zEnd:Float;
  public final revolutions:Float;
  public final seconds:Float;
  public final tablePhase:Float;
  public function new(depth:Float,shift:Float,centreDistance:Float,engagement:HobbingEngagement,
      zStart:Float,zEnd:Float,revolutions:Float,seconds:Float,tablePhase:Float) {
    this.depth=depth;this.shift=shift;this.centreDistance=centreDistance;this.engagement=engagement;
    this.zStart=zStart;this.zEnd=zEnd;this.revolutions=revolutions;this.seconds=seconds;this.tablePhase=tablePhase;
  }
}
