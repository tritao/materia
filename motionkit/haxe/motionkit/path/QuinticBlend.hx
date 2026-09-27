package motionkit.path;

/** Planar quintic Hermite blend, parameterized numerically by arc length. */
class QuinticBlend implements PathPrimitive {
  final controls:Array<PathPoint>;
  final distances:Array<Float>;
  final blendLength:Float;
  static inline var SAMPLES = 128;

  public function new(before:PathPrimitive, beforeDistance:Float,
      after:PathPrimitive, afterDistance:Float, scale:Float) {
    var a = before.pointAt(beforeDistance);
    var b = after.pointAt(afterDistance);
    var ta = before.tangentAt(beforeDistance);
    var tb = after.tangentAt(afterDistance);
    var ka = before.curvatureAt(beforeDistance);
    var kb = after.curvatureAt(afterDistance);
    var d0 = new PathPoint(ta[0] * scale, ta[1] * scale, 0.0);
    var d1 = new PathPoint(tb[0] * scale, tb[1] * scale, 0.0);
    var dd0 = new PathPoint(-ta[1] * ka * scale * scale,
      ta[0] * ka * scale * scale, 0.0);
    var dd1 = new PathPoint(-tb[1] * kb * scale * scale,
      tb[0] * kb * scale * scale, 0.0);
    controls = [a,
      new PathPoint(a.x + d0.x / 5.0, a.y + d0.y / 5.0, a.z),
      new PathPoint(a.x + 2.0 * d0.x / 5.0 + dd0.x / 20.0,
        a.y + 2.0 * d0.y / 5.0 + dd0.y / 20.0, a.z),
      new PathPoint(b.x - 2.0 * d1.x / 5.0 + dd1.x / 20.0,
        b.y - 2.0 * d1.y / 5.0 + dd1.y / 20.0, b.z),
      new PathPoint(b.x - d1.x / 5.0, b.y - d1.y / 5.0, b.z), b];
    distances = [0.0];
    var previous = a;
    var total = 0.0;
    for (i in 1...(SAMPLES + 1)) {
      var next = bezier(i / SAMPLES, controls);
      total += previous.distanceTo(next);
      distances.push(total);
      previous = next;
    }
    blendLength = total;
  }

  public function length():Float return blendLength;

  public function pointAt(distance:Float):PathPoint return bezier(parameter(distance), controls);

  public function tangentAt(distance:Float):Array<Float> {
    var d = derivative(parameter(distance));
    var magnitude = Math.sqrt(d.x * d.x + d.y * d.y);
    return magnitude <= 0.0 ? [0.0, 0.0, 0.0] :
      [d.x / magnitude, d.y / magnitude, 0.0];
  }

  public function curvatureAt(distance:Float):Float {
    var t = parameter(distance);
    var d = derivative(t);
    var dd = secondDerivative(t);
    var magnitudeSquared = d.x * d.x + d.y * d.y;
    return magnitudeSquared <= 1e-24 ? 0.0 :
      (d.x * dd.y - d.y * dd.x) / (magnitudeSquared * Math.sqrt(magnitudeSquared));
  }

  function parameter(distance:Float):Float {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > blendLength)
      throw "Quintic-blend distance is outside its length";
    var low = 0, high = SAMPLES;
    while (high - low > 1) {
      var middle = (low + high) >> 1;
      if (distances[middle] < distance) low = middle;
      else high = middle;
    }
    var span = distances[high] - distances[low];
    return (low + (span <= 0.0 ? 0.0 : (distance - distances[low]) / span)) / SAMPLES;
  }

  function derivative(t:Float):PathPoint {
    var differences = [for (i in 0...5)
      new PathPoint(5.0 * (controls[i + 1].x - controls[i].x),
        5.0 * (controls[i + 1].y - controls[i].y), 0.0)];
    return bezier(t, differences);
  }

  function secondDerivative(t:Float):PathPoint {
    var differences = [for (i in 0...4)
      new PathPoint(20.0 * (controls[i + 2].x - 2.0 * controls[i + 1].x + controls[i].x),
        20.0 * (controls[i + 2].y - 2.0 * controls[i + 1].y + controls[i].y), 0.0)];
    return bezier(t, differences);
  }

  static function bezier(t:Float, points:Array<PathPoint>):PathPoint {
    var work = points.copy();
    for (remaining in 1...points.length)
      for (i in 0...(points.length - remaining))
        work[i] = work[i].lerp(work[i + 1], t);
    return work[0];
  }
}
