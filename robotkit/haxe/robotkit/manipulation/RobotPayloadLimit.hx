package robotkit.manipulation;

/** Allowed carried mass and gravity moment about the flange at one pose. */
class RobotPayloadLimit {
  public final maxMassKg:Float;
  public final maxFlangeMomentNm:Float;

  public function new(maxMassKg:Float, maxFlangeMomentNm:Float) {
    if (!Math.isFinite(maxMassKg) || maxMassKg <= 0 ||
        !Math.isFinite(maxFlangeMomentNm) || maxFlangeMomentNm < 0)
      throw "Robot payload limits require positive mass and non-negative moment";
    this.maxMassKg = maxMassKg;
    this.maxFlangeMomentNm = maxFlangeMomentNm;
  }
}
