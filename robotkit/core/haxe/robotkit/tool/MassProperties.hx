package robotkit.tool;

import robotkit.spatial.Inertia3;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Mass, centre and optional centroidal inertia in one stated frame. */
class MassProperties {
  public final massKg:Float;
  public final centerOfMass:Vec3;
  public final inertia:Null<Inertia3>;

  public function new(massKg:Float, centerOfMass:Vec3, ?inertia:Inertia3) {
    if (!Math.isFinite(massKg) || massKg < 0 || centerOfMass == null)
      throw "Mass properties require non-negative mass and a centre";
    this.massKg = massKg;
    this.centerOfMass = centerOfMass;
    this.inertia = inertia;
  }

  public static function zero():MassProperties
    return new MassProperties(0.0, Vec3.zero(), Inertia3.zero());

  public function transformed(parent_T_local:Transform3):MassProperties {
    if (parent_T_local == null) throw "Mass transformation requires a pose";
    return new MassProperties(massKg, parent_T_local.transformPoint(centerOfMass),
      inertia == null ? null : inertia.rotated(parent_T_local.rotation));
  }

  /** Combine two masses expressed in the same frame via the parallel-axis theorem. */
  public function combined(other:MassProperties):MassProperties {
    if (other == null) throw "Mass combination requires another body";
    var total = massKg + other.massKg;
    if (total == 0.0) return zero();
    var centre = centerOfMass.scale(massKg / total)
      .add(other.centerOfMass.scale(other.massKg / total));
    var first = massKg == 0.0 ? Inertia3.zero() : inertia;
    var second = other.massKg == 0.0 ? Inertia3.zero() : other.inertia;
    return new MassProperties(total, centre,
      first == null || second == null ? null : first.shifted(massKg, centerOfMass.sub(centre))
        .add(second.shifted(other.massKg, other.centerOfMass.sub(centre))));
  }
}
