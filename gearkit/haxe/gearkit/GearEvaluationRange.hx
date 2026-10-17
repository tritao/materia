package gearkit;

/** Explicit geometric evaluation window. This does not imply ISO evaluation rules. */
class GearEvaluationRange {
  public final radiusMin:Float;
  public final radiusMax:Float;
  public final axialMin:Float;
  public final axialMax:Float;
  public function new(radiusMin:Float, radiusMax:Float, axialMin:Float, axialMax:Float) {
    if (!Math.isFinite(radiusMin) || !Math.isFinite(radiusMax)
      || radiusMin <= 0 || radiusMax <= radiusMin) throw "evaluation radii must be finite, positive and increasing";
    if (!Math.isFinite(axialMin) || !Math.isFinite(axialMax) || axialMin < 0
      || axialMax <= axialMin) throw "evaluation axial limits must be finite, non-negative and increasing";
    this.radiusMin = radiusMin;
    this.radiusMax = radiusMax;
    this.axialMin = axialMin;
    this.axialMax = axialMax;
  }
}
