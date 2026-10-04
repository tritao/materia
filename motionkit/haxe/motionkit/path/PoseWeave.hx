package motionkit.path;

import motionkit.kinematics.Pose3;

/** A single smooth span of a weave, measured in the base primitive's seam progress. */
class PoseWeave implements PosePrimitive {
  final base:PosePrimitive;
  final from:Float;
  final to:Float;
  final offset:Float;
  final profile:WeaveProfile;
  final frame:WeaveFrame;
  final total:Float;
  final taper:Float;
  final phase:Float;

  public function new(base:PosePrimitive, from:Float, to:Float, offset:Float,
      profile:WeaveProfile, frame:WeaveFrame, total:Float, taper:Float, phase:Float) {
    if (base == null || profile == null || frame == null || from < 0.0 || to > base.length() || to <= from)
      throw "Pose weave needs a positive span inside its base primitive";
    this.base = base; this.from = from; this.to = to; this.offset = offset;
    this.profile = profile; this.frame = frame; this.total = total; this.taper = taper; this.phase = phase;
  }

  public function length():Float return to - from;
  public function speedLimit():Float return base.speedLimit();
  public function orientationPolicy():OrientationPolicy return base.orientationPolicy();
  public function startWaypoint():PoseWaypoint return waypointAt(0.0);
  public function endWaypoint():PoseWaypoint return waypointAt(length());

  function local(distance:Float):Float {
    if (!Math.isFinite(distance) || distance < -1e-12 || distance > length() + 1e-12)
      throw "Pose-weave distance outside span";
    return Math.max(from, Math.min(to, from + distance));
  }

  function wave(distance:Float, arriving:Bool):WeaveSample {
    var w = profile.at(distance + phase, arriving);
    if (taper <= 0.0) return w;
    var a = ramp(distance), b = ramp(total - distance);
    var value = a.offset * b.offset;
    var first = a.first * b.offset - a.offset * b.first;
    var second = a.second * b.offset - 2.0 * a.first * b.first + a.offset * b.second;
    return new WeaveSample(w.offset * value, w.first * value + w.offset * first,
      w.second * value + 2.0 * w.first * first + w.offset * second);
  }

  /** Quintic endpoint envelope: zero offset, velocity and acceleration at the seam ends. */
  function ramp(distance:Float):WeaveSample {
    if (distance >= taper) return new WeaveSample(1.0, 0.0, 0.0);
    var u = Math.max(0.0, distance / taper);
    return new WeaveSample(u * u * u * (10.0 + u * (-15.0 + 6.0 * u)),
      30.0 * u * u * (1.0 - u) * (1.0 - u) / taper,
      60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (taper * taper));
  }

  public function waypointAt(distance:Float):PoseWaypoint {
    var at = local(distance), s = offset + at;
    var original = base.waypointAt(at), pose = original.pose;
    var w = wave(s, distance >= length()), axis = frame.at(s).axis;
    return new PoseWaypoint(new Pose3(pose.x + w.offset * axis[0], pose.y + w.offset * axis[1],
      pose.z + w.offset * axis[2], pose.qx, pose.qy, pose.qz, pose.qw),
      original.positionTolerance, original.orientationTolerance);
  }

  public function derivativesAt(distance:Float):PoseDerivatives {
    var at = local(distance), s = offset + at;
    var original = base.derivativesAt(at), direction = frame.at(s);
    var w = wave(s, distance >= length());
    return new PoseDerivatives([for (i in 0...3) original.linear[i] + direction.axis[i] * w.first + direction.first[i] * w.offset],
      original.angular.copy(), [for (i in 0...3) original.linearSecond[i] + direction.axis[i] * w.second +
        2.0 * direction.first[i] * w.first + direction.second[i] * w.offset], original.angularSecond.copy());
  }
}
