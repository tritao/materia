package machinekit.structural;

/**
 * Audit of `MineralCasting.analyticVolume` against integrating the real B-rep, which it replaces in `massProperties`.
 * Tolerances are integration rounding, not modelling allowances: relative on volume and on inertia (to the largest
 * principal moment), absolute on the centroid.
 */
class CastingVolumeAgreement {
  public static inline final RELATIVE = 1e-9;
  public static inline final CENTRE_MM = 1e-6;

  /** Throws when `casting` has no analytic volume or it differs from the solid; `label` names it in the error. */
  public static function check(casting:MineralCasting, label:String):Void {
    var analytic = casting.analyticVolume();
    if (analytic == null) throw "Casting volume has no analytic form: " + label;
    var part = casting.geometry(), solid = part.massProperties();
    part.close();
    var c = solid.centerOfMass, i = solid.inertia, a = analytic.inertia;
    if (Math.abs(analytic.volume - solid.volume) > RELATIVE * solid.volume) throw 'Analytic casting volume differs: $label: ${analytic.volume} vs ${solid.volume}';
    var dc = Math.sqrt(Math.pow(c.x - analytic.centre.x, 2) + Math.pow(c.y - analytic.centre.y, 2)
      + Math.pow(c.z - analytic.centre.z, 2));
    if (dc > CENTRE_MM) throw 'Analytic casting centre differs by $dc mm: $label';
    var scale = Math.max(i.xx, Math.max(i.yy, i.zz));
    for (pair in [[i.xx, a.xx], [i.yy, a.yy], [i.zz, a.zz], [i.xy, a.xy], [i.xz, a.xz], [i.yz, a.yz]]) {
      if (Math.abs(pair[0] - pair[1]) > RELATIVE * scale) throw 'Analytic casting inertia differs: $label: $pair';
    }
  }
}
