package robotkit.tool;

import robotkit.spatial.Transform3;

/** One sealed suction contact. Local +Z points from the cup toward the part.
 * Vacuum is a guaranteed minimum measured at the cup; friction and moment
 * rating must describe this cup and workpiece surface at that vacuum. */
class SuctionGrip {
  public final flangeTCup:Transform3;
  public final effectiveAreaM2:Float;
  public final minimumVacuumKpa:Float;
  public final frictionCoefficient:Float;
  public final safetyFactor:Float;
  public final ratedMomentNm:Null<Float>;

  public function new(flangeTCup:Transform3, effectiveAreaM2:Float,
      minimumVacuumKpa:Float, frictionCoefficient:Float, safetyFactor:Float,
      ?ratedMomentNm:Float) {
    if (flangeTCup == null || !Math.isFinite(effectiveAreaM2) || effectiveAreaM2 <= 0 ||
        !Math.isFinite(minimumVacuumKpa) || minimumVacuumKpa <= 0 ||
        minimumVacuumKpa > 101.325 || !Math.isFinite(frictionCoefficient) ||
        frictionCoefficient <= 0 || !Math.isFinite(safetyFactor) || safetyFactor < 1.5)
      throw "Suction grip requires contact pose, sealed area, cup vacuum, friction and safety factor >= 1.5";
    if (ratedMomentNm != null && (!Math.isFinite(ratedMomentNm) || ratedMomentNm <= 0))
      throw "Suction grip moment rating must be positive and finite";
    this.flangeTCup = flangeTCup;
    this.effectiveAreaM2 = effectiveAreaM2;
    this.minimumVacuumKpa = minimumVacuumKpa;
    this.frictionCoefficient = frictionCoefficient;
    this.safetyFactor = safetyFactor;
    this.ratedMomentNm = ratedMomentNm;
  }

  public function normalCapacityN():Float return minimumVacuumKpa * 1000.0 * effectiveAreaM2;
}
