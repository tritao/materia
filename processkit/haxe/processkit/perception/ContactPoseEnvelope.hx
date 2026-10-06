package processkit.perception;

import processkit.perception.ContactRegistration.PlaneContact;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;

private class Span {
  public final lo:Float;
  public final hi:Float;
  public function new(lo:Float, hi:Float) {
    // Expand arithmetic ranges so roundoff at a closed constraint boundary cannot discard a feasible pose.
    var pad = 1e-14 * Math.max(1.0, Math.max(Math.abs(lo), Math.abs(hi)));
    this.lo = lo - pad; this.hi = hi + pad;
  }
  public function add(other:Span):Span return new Span(lo + other.lo, hi + other.hi);
  public function scale(value:Float):Span return value >= 0 ? new Span(lo * value, hi * value) : new Span(hi * value, lo * value);
  public function multiply(other:Span):Span {
    var values = [lo * other.lo, lo * other.hi, hi * other.lo, hi * other.hi];
    var low = values[0], high = values[0];
    for (value in values) { low = Math.min(low, value); high = Math.max(high, value); }
    return new Span(low, high);
  }
  public function sin():Span return new Span(Math.sin(lo), Math.sin(hi));
  public function cos():Span return new Span(Math.min(Math.cos(lo), Math.cos(hi)), lo <= 0 && hi >= 0 ? 1.0 : Math.max(Math.cos(lo), Math.cos(hi)));
  public function radius():Float return Math.max(Math.abs(lo), Math.abs(hi));
}

private typedef PoseBox = {var lo:Array<Float>; var hi:Array<Float>;}
private typedef PlaneConstraint = {var normal:Vec3; var point:Vec3; var offset:Float; var translated:Bool; var tolerance:Float;}

/** Conservative bounds of a search ray's intersection with its nominal CAD plane, in metres. */
class ContactRegionBounds {
  public final halfU:Float;
  public final halfV:Float;
  public final normalTravel:Float;
  public final tiltU:Float;
  public final tiltV:Float;
  public function new(halfU:Float, halfV:Float, normalTravel:Float, tiltU:Float, tiltV:Float) {
    this.halfU = halfU; this.halfV = halfV; this.normalTravel = normalTravel; this.tiltU = tiltU; this.tiltV = tiltV;
  }
}

/**
 * Set-valued contact registration. A bounded chassis-pose error transforms measured points back into
 * the nominal chassis frame. Interval constraints retain every pose consistent with calibrated
 * contact error; a search budget may leave wider boxes, never silently remove unknown components.
 * Euler intervals are restricted to +/-90 degrees so their sine ranges are monotone.
 */
class ContactPoseEnvelope {
  public final nominalWork:Transform3;
  public final measurementError:Float;
  public var cells(get, never):Int;
  var boxes:Array<PoseBox>;

  public function new(nominalWork:Transform3, translation:Vec3, rotation:Vec3, measurementError:Float) {
    if (nominalWork == null || translation == null || rotation == null || translation.x < 0 || translation.y < 0 || translation.z < 0 ||
        rotation.x < 0 || rotation.y < 0 || rotation.z < 0 || rotation.x >= Math.PI / 2 || rotation.y >= Math.PI / 2 || rotation.z >= Math.PI / 2 ||
        !Math.isFinite(measurementError) || !(measurementError > 0))
      throw "Contact pose envelope needs a finite bounded chassis error and positive calibrated observation error";
    this.nominalWork = nominalWork; this.measurementError = measurementError;
    var limits = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z];
    boxes = [{lo: [for (limit in limits) -limit], hi: limits.copy()}];
  }

  function get_cells():Int return boxes.length;
  static function dot(vector:Array<Span>, axis:Vec3):Span return vector[0].scale(axis.x).add(vector[1].scale(axis.y)).add(vector[2].scale(axis.z));

  /** Z-Y-X rotation of a constant vector, with conservative interval products. */
  static function rotate(box:PoseBox, point:Vec3):Array<Span> {
    var roll = new Span(box.lo[3], box.hi[3]), pitch = new Span(box.lo[4], box.hi[4]), yaw = new Span(box.lo[5], box.hi[5]);
    var cr = roll.cos(), sr = roll.sin(), cp = pitch.cos(), sp = pitch.sin(), cy = yaw.cos(), sy = yaw.sin();
    var rxY = cr.scale(point.y).add(sr.scale(-point.z));
    var rxZ = sr.scale(point.y).add(cr.scale(point.z));
    var ryX = cp.scale(point.x).add(sp.multiply(rxZ));
    var ryZ = sp.scale(-point.x).add(cp.multiply(rxZ));
    return [cy.multiply(ryX).add(sy.multiply(rxY).scale(-1)), sy.multiply(ryX).add(cy.multiply(rxY)), ryZ];
  }

  static function transformed(box:PoseBox, point:Vec3, translated:Bool):Array<Span> {
    var result = rotate(box, point);
    if (translated) for (i in 0...3) result[i] = result[i].add(new Span(box.lo[i], box.hi[i]));
    return result;
  }

  static function compatible(box:PoseBox, constraints:Array<PlaneConstraint>):Bool {
    for (constraint in constraints) {
      var residual = dot(transformed(box, constraint.point, constraint.translated), constraint.normal).add(new Span(-constraint.offset, -constraint.offset));
      if (residual.lo > constraint.tolerance || residual.hi < -constraint.tolerance) return false;
    }
    return true;
  }

  /** Exact linear contraction of each translation coordinate, conservative in all other variables. */
  static function contract(box:PoseBox, constraints:Array<PlaneConstraint>):Bool {
    for (_ in 0...3) for (constraint in constraints) if (constraint.translated) {
      var rotated = dot(rotate(box, constraint.point), constraint.normal).add(new Span(-constraint.offset, -constraint.offset));
      var normal = constraint.normal.toArray();
      for (axis in 0...3) if (Math.abs(normal[axis]) > 1e-12) {
        var others = rotated;
        for (i in 0...3) if (i != axis) others = others.add(new Span(box.lo[i], box.hi[i]).scale(normal[i]));
        var possible = new Span(-constraint.tolerance - others.hi, constraint.tolerance - others.lo).scale(1.0 / normal[axis]);
        box.lo[axis] = Math.max(box.lo[axis], possible.lo); box.hi[axis] = Math.min(box.hi[axis], possible.hi);
        if (box.lo[axis] > box.hi[axis]) return false;
      }
    }
    return compatible(box, constraints);
  }

  /** Refine with all observations collected so far. Unsupported poses or inconsistent observations fail explicitly. */
  public function refine(contacts:Array<PlaneContact>, splitBudget:Int = 4096, maxCells:Int = 256,
      translationResolution:Float = 0.00025, rotationResolution:Float = 0.00001):Void {
    if (contacts == null || contacts.length == 0 || splitBudget < 0 || maxCells < 1 ||
        !Math.isFinite(translationResolution) || !(translationResolution > 0) ||
        !Math.isFinite(rotationResolution) || !(rotationResolution > 0)) throw "Contact envelope refinement needs observations and finite positive resolutions";
    var constraints:Array<PlaneConstraint> = [];
    for (contact in contacts) {
      if (contact == null) throw "Contact envelope cannot contain a missing observation";
      var normal = nominalWork.rotation.rotate(contact.normal);
      constraints.push({normal: normal, point: contact.measured, offset: contact.offset + normal.dot(nominalWork.translation),
        translated: true, tolerance: measurementError});
    }
    // Differences on one plane eliminate unknown translation before angular refinement.
    for (i in 0...contacts.length) for (j in i + 1...contacts.length)
      if (contacts[i].normal.sub(contacts[j].normal).norm() < 1e-12)
        constraints.push({normal: nominalWork.rotation.rotate(contacts[i].normal), point: contacts[i].measured.sub(contacts[j].measured),
          offset: contacts[i].offset - contacts[j].offset, translated: false, tolerance: 2 * measurementError});
    var pending:Array<PoseBox> = [for (box in boxes) {lo: box.lo.copy(), hi: box.hi.copy()}];
    if (pending.length > maxCells) {
      var hull:PoseBox = {lo: pending[0].lo.copy(), hi: pending[0].hi.copy()};
      for (box in pending) for (i in 0...6) { hull.lo[i] = Math.min(hull.lo[i], box.lo[i]); hull.hi[i] = Math.max(hull.hi[i], box.hi[i]); }
      pending = [hull];
    }
    var kept:Array<PoseBox> = [];
    var splits = 0;
    var sensitivities = [for (_ in 0...6) 0.0];
    for (constraint in constraints) {
      if (constraint.translated) {
        var normal = constraint.normal.toArray();
        for (i in 0...3) sensitivities[i] = Math.max(sensitivities[i], Math.abs(normal[i]));
      }
      var radius = constraint.point.norm();
      for (i in 3...5) sensitivities[i] = Math.max(sensitivities[i], radius);
      // A yaw derivative is Z cross the rotated point. A horizontal plane cannot observe it.
      var horizontalNormal = Math.sqrt(constraint.normal.x * constraint.normal.x + constraint.normal.y * constraint.normal.y);
      sensitivities[5] = Math.max(sensitivities[5], radius * horizontalNormal);
    }
    while (pending.length > 0) {
      var box:PoseBox = cast pending.shift();
      if (!contract(box, constraints)) continue;
      var axis = -1, width = 1.0;
      for (i in 0...6) if (sensitivities[i] > 1e-12) {
        var ratio = (box.hi[i] - box.lo[i]) / (i < 3 ? translationResolution : rotationResolution);
        if (ratio > width) { axis = i; width = ratio; }
      }
      if (axis < 0 || splits >= splitBudget || kept.length + pending.length + 2 > maxCells) { kept.push(box); continue; }
      var middle = (box.lo[axis] + box.hi[axis]) / 2;
      var left:PoseBox = {lo: box.lo.copy(), hi: box.hi.copy()}, right:PoseBox = {lo: box.lo.copy(), hi: box.hi.copy()};
      left.hi[axis] = middle; right.lo[axis] = middle;
      // Prune before using the cell budget; sparse feasible intervals should not waste it on rejected children.
      if (compatible(left, constraints)) pending.push(left);
      if (compatible(right, constraints)) pending.push(right);
      splits++;
    }
    if (kept.length == 0) throw "Contact observations are inconsistent with the bounded chassis pose error";
    boxes = kept;
  }

  /** A provisional centre for placing probes; this never authorizes welding or discards the set-valued bounds. */
  public function estimate():Transform3 {
    var low = boxes[0].lo.copy(), high = boxes[0].hi.copy();
    for (box in boxes) for (i in 0...6) { low[i] = Math.min(low[i], box.lo[i]); high[i] = Math.max(high[i], box.hi[i]); }
    var middle = [for (i in 0...6) (low[i] + high[i]) / 2];
    var error = new Transform3(new Vec3(middle[0], middle[1], middle[2]), Quat.fromRollPitchYaw(middle[3], middle[4], middle[5]));
    return error.inverse().compose(nominalWork);
  }

  /** Bound the actual work-frame intersection of a probe placed using the current provisional estimate. */
  public function region(point:Vec3, normal:Vec3, u:Vec3, v:Vec3, estimate:Transform3):ContactRegionBounds {
    if (point == null || normal == null || u == null || v == null || estimate == null || Math.abs(normal.norm() - 1) > 1e-8 ||
        Math.abs(u.norm() - 1) > 1e-8 || Math.abs(v.norm() - 1) > 1e-8 || Math.abs(normal.dot(u)) > 1e-8 ||
        Math.abs(normal.dot(v)) > 1e-8 || Math.abs(u.dot(v)) > 1e-8) throw "Contact region needs a unit orthogonal CAD plane frame and provisional pose";
    var command = estimate.transformPoint(point), outward = estimate.rotation.rotate(normal);
    var nominalNormal = nominalWork.rotation.rotate(normal), nominalU = nominalWork.rotation.rotate(u), nominalV = nominalWork.rotation.rotate(v);
    var planeOffset = nominalNormal.dot(nominalWork.transformPoint(point));
    var halfU = 0.0, halfV = 0.0, travel = 0.0, tiltU = 0.0, tiltV = 0.0;
    for (box in boxes) {
      var located = transformed(box, command, true), direction = rotate(box, outward);
      var residual = dot(located, nominalNormal).add(new Span(-planeOffset, -planeOffset));
      var projection = dot(direction, nominalNormal);
      if (!(projection.lo > 0)) throw "Contact ray can be tangent or opposed within the remaining pose uncertainty";
      var lambda = residual.scale(-1).multiply(new Span(1.0 / projection.hi, 1.0 / projection.lo));
      var shiftU = dot(located, nominalU).add(new Span(-nominalU.dot(nominalWork.transformPoint(point)), -nominalU.dot(nominalWork.transformPoint(point))));
      var shiftV = dot(located, nominalV).add(new Span(-nominalV.dot(nominalWork.transformPoint(point)), -nominalV.dot(nominalWork.transformPoint(point))));
      var du = dot(direction, nominalU), dv = dot(direction, nominalV);
      halfU = Math.max(halfU, shiftU.add(du.multiply(lambda)).radius());
      halfV = Math.max(halfV, shiftV.add(dv.multiply(lambda)).radius());
      travel = Math.max(travel, lambda.radius()); tiltU = Math.max(tiltU, du.radius()); tiltV = Math.max(tiltV, dv.radius());
    }
    return new ContactRegionBounds(halfU, halfV, travel, tiltU, tiltV);
  }
}
