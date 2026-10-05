package processkit.perception;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;

/** A nominal work-frame plane and a measured contact point in the observer frame, in metres. */
class PlaneContact {
  public final normal:Vec3;
  public final offset:Float;
  public final measured:Vec3;

  public function new(normal:Vec3, offset:Float, measured:Vec3) {
    if (normal == null || measured == null || !Math.isFinite(offset) || !Math.isFinite(normal.norm()) || normal.norm() < 1e-12)
      throw "Contact registration needs a finite plane and measured point";
    this.normal = normal.normalized();
    this.offset = offset / normal.norm();
    this.measured = measured;
  }
}

/** An incomplete estimate may guide another probe, but only an accepted frame can authorize welding. */
class ContactRegistrationResult {
  public final estimate:Transform3;
  public final correction:Transform3;
  public final rank:Int;
  public final maxResidual:Float;
  public final accepted:Bool;
  public final reason:Null<String>;

  public function new(estimate:Transform3, correction:Transform3, rank:Int, maxResidual:Float,
      accepted:Bool, reason:Null<String>) {
    this.estimate = estimate; this.correction = correction; this.rank = rank;
    this.maxResidual = maxResidual; this.accepted = accepted; this.reason = reason;
  }

  public function require():Transform3 {
    if (!accepted) throw 'Contact registration rejected: $reason';
    return estimate;
  }
}

/** Rigid point-to-plane registration. Inputs contain observations, never a live work-body pose. */
class ContactRegistration {
  /** Symmetric pseudoinverse: update only observable directions, including for provisional fits. */
  static function solve(matrix:Array<Array<Float>>, rhs:Array<Float>):{step:Array<Float>, rank:Int} {
    var a = [for (row in matrix) row.copy()];
    var vectors = [for (i in 0...6) [for (j in 0...6) i == j ? 1.0 : 0.0]];
    for (_ in 0...128) {
      var p = 0, q = 1, off = 0.0, magnitude = 0.0;
      for (i in 0...6) {
        magnitude = Math.max(magnitude, Math.abs(a[i][i]));
        for (j in i + 1...6) if (Math.abs(a[i][j]) > off) { off = Math.abs(a[i][j]); p = i; q = j; }
      }
      if (off <= Math.max(1e-14, magnitude * 1e-12)) break;
      var angle = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
      var c = Math.cos(angle), s = Math.sin(angle);
      var pp = a[p][p], qq = a[q][q], pq = a[p][q];
      for (i in 0...6) {
        if (i != p && i != q) {
          var ip = a[i][p], iq = a[i][q];
          a[i][p] = c * ip - s * iq; a[p][i] = a[i][p];
          a[i][q] = s * ip + c * iq; a[q][i] = a[i][q];
        }
        var vp = vectors[i][p], vq = vectors[i][q];
        vectors[i][p] = c * vp - s * vq;
        vectors[i][q] = s * vp + c * vq;
      }
      a[p][p] = c * c * pp - 2 * s * c * pq + s * s * qq;
      a[q][q] = s * s * pp + 2 * s * c * pq + c * c * qq;
      a[p][q] = 0; a[q][p] = 0;
    }
    var largest = 0.0;
    for (i in 0...6) largest = Math.max(largest, a[i][i]);
    var step = [for (_ in 0...6) 0.0];
    var rank = 0;
    for (column in 0...6) if (a[column][column] > Math.max(1e-14, largest * 1e-9)) {
      rank++;
      var projection = 0.0;
      for (row in 0...6) projection += vectors[row][column] * rhs[row];
      for (row in 0...6) step[row] += vectors[row][column] * projection / a[column][column];
    }
    return {step: step, rank: rank};
  }

  public static function fit(contacts:Array<PlaneContact>, nominal:Transform3,
      maxResidual:Float = 0.0005, maxTranslation:Float = 0.15, maxRotation:Float = 0.1,
      iterations:Int = 32):ContactRegistrationResult {
    if (contacts == null || contacts.length == 0 || nominal == null || !Math.isFinite(maxResidual) ||
        !(maxResidual > 0) || !Math.isFinite(maxTranslation) || !(maxTranslation > 0) ||
        !Math.isFinite(maxRotation) || !(maxRotation > 0) || iterations < 1)
      throw "Contact registration needs observations, a nominal frame and positive finite limits";
    for (contact in contacts) if (contact == null)
      throw "Contact registration cannot contain a missing observation";
    var inverse = nominal.inverse();
    var points = [for (contact in contacts) inverse.transformPoint(contact.measured)];
    var centre = new Vec3();
    for (point in points) centre = centre.add(point.scale(1.0 / points.length));
    var spread = 0.0;
    for (point in points) spread += point.sub(centre).dot(point.sub(centre)) / points.length;
    var scale = Math.max(0.001, Math.sqrt(spread));
    var position = new Vec3(), rotation = Quat.identity();
    var rank = 0, converged = false;
    for (_ in 0...iterations) {
      var matrix = [for (_ in 0...6) [for (_ in 0...6) 0.0]];
      var rhs = [for (_ in 0...6) 0.0];
      for (index in 0...contacts.length) {
        var normal = rotation.rotate(contacts[index].normal);
        var relative = points[index].sub(position);
        var residual = normal.dot(relative) - contacts[index].offset;
        var angular = normal.cross(relative).scale(1.0 / scale);
        var row = [-normal.x, -normal.y, -normal.z, angular.x, angular.y, angular.z];
        for (i in 0...6) {
          rhs[i] -= row[i] * residual;
          for (j in 0...6) matrix[i][j] += row[i] * row[j];
        }
      }
      var found = solve(matrix, rhs); rank = found.rank;
      var shift = new Vec3(found.step[0], found.step[1], found.step[2]);
      var turn = new Vec3(found.step[3] / scale, found.step[4] / scale, found.step[5] / scale);
      position = position.add(shift);
      var angle = turn.norm();
      if (angle > 1e-12) rotation = Quat.fromAxisAngle(turn.scale(1.0 / angle), angle).multiply(rotation);
      if (shift.norm() < 1e-9 && angle < 1e-9) { converged = true; break; }
    }
    var correction = new Transform3(position, rotation);
    var worst = 0.0;
    for (index in 0...contacts.length)
      worst = Math.max(worst, Math.abs(rotation.rotate(contacts[index].normal).dot(points[index].sub(position)) - contacts[index].offset));
    var angle = 2 * Math.acos(Math.min(1.0, Math.abs(rotation.w)));
    var reason:Null<String> = null;
    if (!converged) reason = "the bounded fit did not converge";
    else if (rank < 6) reason = 'only $rank of six pose components are observable';
    else if (!(worst <= maxResidual)) reason = 'contact residual $worst m exceeds $maxResidual m';
    else if (!(position.norm() <= maxTranslation) || !(angle <= maxRotation)) reason = "correction exceeds the configured uncertainty bounds";
    return new ContactRegistrationResult(nominal.compose(correction), correction, rank, worst, reason == null, reason);
  }
}
