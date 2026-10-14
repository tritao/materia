package processkit.perception;

import robotkit.spatial.Vec3;

/** Allocate a rigid-registration error budget over a measured 3-2-1 pattern and its work extent. */
class ContactRegistrationPrecision {
  public static inline final DEFAULT_WORK_ERROR = 0.0001;
  final targets:Array<Vec3>;
  final workError:Float;

  public function new(targets:Array<Vec3>, workError:Float = DEFAULT_WORK_ERROR) {
    if (targets == null || targets.length == 0 || !Math.isFinite(workError) || !(workError > 0))
      throw "Registration precision needs a work extent and positive finite error budget";
    for (point in targets) if (point == null || !Math.isFinite(point.x) || !Math.isFinite(point.y) || !Math.isFinite(point.z))
      throw "Registration precision has a non-finite work point";
    this.targets = targets.copy(); this.workError = workError;
  }

  /** Smaller patterns need finer contacts; near-parallel planes also consume the pose-error budget. */
  public function contactError(points:Array<Vec3>, normal:Vec3, previous:Array<Vec3>, maximumError:Float):Float {
    if (points == null || previous == null || points.length != 3 - previous.length || points.length < 1 ||
        normal == null || !Math.isFinite(normal.norm()) || normal.norm() < 1e-12 ||
        !Math.isFinite(maximumError) || !(maximumError > 0))
      throw "Registration precision needs an observable 3-2-1 stage and a total observation-error bound";
    var n = normal.normalized();
    var gain = 1.0;
    if (points.length == 3) {
      var a = points[0], edge = points[1].sub(a), other = points[2].sub(a);
      var area = edge.cross(other).dot(n);
      if (!Math.isFinite(area) || Math.abs(area) < 1e-12) throw "Registration precision has a degenerate triangle";
      var gv = other.cross(n).scale(1.0 / area), gw = n.cross(edge).scale(1.0 / area);
      var gu = gv.add(gw).scale(-1);
      for (target in targets) {
        var delta = target.sub(a);
        var v = gv.dot(delta), w = gw.dot(delta), u = 1 - v - w;
        // Barycentric interpolation bounds plane displacement. Its gradient also
        // bounds the lateral displacement of points above or below that plane.
        gain = Math.max(gain, Math.abs(u) + Math.abs(v) + Math.abs(w) +
          Math.abs(n.dot(delta)) * (gu.norm() + gv.norm() + gw.norm()));
      }
    } else if (points.length == 2) {
      var cross = previous[0].normalized().cross(n);
      var independence = cross.norm();
      if (!(independence > 1e-3)) throw "Registration precision needs independent planes";
      var axis = cross.scale(1.0 / independence);
      var span = Math.abs(points[1].sub(points[0]).dot(axis));
      if (!(span > 1e-12)) throw "Registration precision has a degenerate pair";
      var middle = points[0].add(points[1]).scale(0.5);
      for (target in targets)
        gain = Math.max(gain, (1 + 2 * target.sub(middle).norm() / span) / independence);
    } else {
      var independence = Math.abs(previous[0].normalized().cross(previous[1].normalized()).dot(n));
      if (!(independence > 1e-3)) throw "Registration precision needs three independent planes";
      gain = 1.0 / independence;
    }
    return Math.min(maximumError, workError / (3 * gain));
  }
}
