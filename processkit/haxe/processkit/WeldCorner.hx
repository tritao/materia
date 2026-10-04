package processkit;

import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** What the arm's wrist can do, in radians per second and per second squared: the limits of the torch's turn. */
typedef WristLimits = {
  var angularSpeed:Float;
  var angularAcceleration:Float;
}

/**
 * How the torch turns where one weld segment meets the next at an angle (a corner of a tube, the sides of a post). The
 * turn is part of the travel: the torch turns half of the way on the last stretch of one segment and half on the first
 * stretch of the next, at the travel speed, so the wire feed that keeps the deposit per length constant stays right and no
 * metal piles up where the torch would otherwise stand still to turn.
 *
 * The length of the turn comes from the arm and the angle, not from a constant. The torch must reorient by the angle `a`
 * between the two segments' orientations while the tip keeps its travel speed `v`, and the wrist can turn the torch at no
 * more than `HEADROOM` of its angular speed `w` and acceleration `α` (the rest is for the wrist's share of moving the tip).
 * A rest-to-rest turn of `a` under those limits takes
 *
 *   T = a / w' + w' / α'   if a ≥ w'² / α'   (the speed limit is reached),
 *   T = 2 · sqrt(a / α')   otherwise          (it is not),
 *
 * with w' = HEADROOM · w and α' = HEADROOM · α, and the tip travels `v · T` while it does, half of it on each side of the
 * corner. A sharper corner is a longer turn; a straight join (no change of orientation) has none. The length is kept within
 * `MIN_TURN` and `MAX_TURN` so that a sensible turn is never smeared along a whole seam or shorter than the path can
 * follow, and what a segment gives to a corner is at most `SHARE` of its own length.
 */
class WeldCorner {
  /** The part of the wrist's speed and acceleration a turn may use. */
  public static inline var HEADROOM:Float = 0.5;
  /** The shortest turn on each side of a corner, in metres. */
  public static inline var MIN_TURN:Float = 0.004;
  /** The longest turn on each side of a corner, in metres. */
  public static inline var MAX_TURN:Float = 0.025;
  /** The most of a segment a corner may take, as a fraction of its length. */
  public static inline var SHARE:Float = 0.45;
  /** Orientations closer than this, in radians, are the same. */
  public static inline var STRAIGHT:Float = 1e-6;
  /** The ways to turn, in the order a planner tries them: around the corner's edge, along the shortest arc of the wire, and by the rotations' own interpolation. */
  public static inline var AROUND:Int = 0;
  public static inline var ARC:Int = 1;
  public static inline var ROTATION:Int = 2;
  public static inline var STYLES:Int = 3;

  /** The time a rest-to-rest turn by `angle` radians takes under the wrist's limits, in seconds. */
  public static function turnTime(angle:Float, limits:WristLimits):Float {
    if (!(angle > STRAIGHT)) return 0.0;
    if (!(limits.angularSpeed > 0.0) || !(limits.angularAcceleration > 0.0)) throw "A corner turn needs a wrist speed and acceleration";
    var speed = HEADROOM * limits.angularSpeed, acceleration = HEADROOM * limits.angularAcceleration;
    return angle >= speed * speed / acceleration ? angle / speed + speed / acceleration : 2.0 * Math.sqrt(angle / acceleration);
  }

  /**
   * How far along each side of a corner the torch turns, for an orientation change of `angle` radians at `travelSpeed`
   * metres per second: zero for a straight join, otherwise `v · T / 2` within `MIN_TURN` and `MAX_TURN`.
   */
  public static function turnLength(angle:Float, travelSpeed:Float, limits:WristLimits):Float {
    if (!(angle > STRAIGHT)) return 0.0;
    if (!(travelSpeed > 0.0)) throw "A corner turn needs a travel speed";
    return Math.min(MAX_TURN, Math.max(MIN_TURN, travelSpeed * turnTime(angle, limits) / 2.0));
  }

  /** What a segment of `segmentLength` metres gives to a corner whose turn is `turn` long on each side. */
  public static function given(turn:Float, segmentLength:Float):Float
    return turn <= 0.0 ? 0.0 : Math.min(turn, SHARE * segmentLength);

  /** Roll about `to`'s wire that carries `from` to the new wire by its shortest swing, without extra wrist twist. */
  public static function continuationRoll(from:Quat, to:Quat):Float {
    var z = new Vec3(0.0, 0.0, 1.0);
    var wireFrom = from.rotate(z), wireTo = to.rotate(z);
    var aligned = arc(wireFrom, wireTo).multiply(from);
    var residual = to.conjugate().multiply(aligned);
    var roll = 2.0 * Math.atan2(residual.z, residual.w);
    while (roll > Math.PI) roll -= 2.0 * Math.PI;
    while (roll <= -Math.PI) roll += 2.0 * Math.PI;
    return roll;
  }

  /**
   * The torch's orientation a fraction `t` of the way through a turn from `from` to `to`, at a corner whose two segments run
   * along `travelA` and `travelB`. The wire swings *around the corner*: about the axis square to both directions of travel (the
   * corner's edge, vertical for the posts of a frame), keeping its angle to that axis as it goes (changing evenly if the two
   * differ). That is how a torch goes round an outside corner with its nozzle the same distance from the edge all the way; the
   * shortest arc between the two wire directions instead bulges toward the axis, and puts the nozzle into the corner. What is
   * left of the turn after the wire is where it should be is a roll about the wire, evenly. This is not the interpolation of
   * the two rotations (`Quat.slerp`), which turns about one axis and takes the wire off the path when the segments are rolled
   * differently. Segments in a line (no corner axis) swing the wire along the shortest arc. Those are the first two `style`s
   * (`AROUND`, `ARC`); `ROTATION` is that plain interpolation. Which clears the work at a corner depends on the torch and the
   * corner, so a planner tries them in turn.
   */
  public static function orientationAt(from:Quat, to:Quat, t:Float, ?travelA:Vec3, ?travelB:Vec3, ?style:Int = 0):Quat {
    if (style == ROTATION) return from.slerp(to, t);
    var z = new Vec3(0.0, 0.0, 1.0);
    var w1 = from.rotate(z), w2 = to.rotate(z);
    var wireAt = greatCircle(w1, w2, t);
    if (style == AROUND && travelA != null && travelB != null) {
      var corner = travelA.cross(travelB);
      if (corner.norm() > 1e-6) {
        var k = corner.normalized();
        var a1 = w1.dot(k), a2 = w2.dot(k);
        var p1 = w1.sub(k.scale(a1)), p2 = w2.sub(k.scale(a2));
        if (p1.norm() > 1e-6 && p2.norm() > 1e-6) {
          var u1 = p1.normalized(), u2 = p2.normalized();
          var azimuth = Math.atan2(k.dot(u1.cross(u2)), u1.dot(u2));
          var a = a1 + (a2 - a1) * t;
          wireAt = k.scale(a).add(Quat.fromAxisAngle(k, azimuth * t).rotate(u1).scale(Math.sqrt(Math.max(0.0, 1.0 - a * a))));
        }
      }
    }
    var swing = arc(w1, wireAt).multiply(from);
    var full = arc(w1, w2).multiply(from);
    return Quat.fromAxisAngle(wireAt, twistOf(full, to, w2) * t).multiply(swing);
  }

  /** The point a fraction `t` of the shortest arc from `a` to `b` (unit vectors). */
  static function greatCircle(a:Vec3, b:Vec3, t:Float):Vec3 {
    var cosine = a.dot(b);
    var sine = a.cross(b).norm();
    if (sine < 1e-9) return a;
    return Quat.fromAxisAngle(a.cross(b).scale(1.0 / sine), Math.atan2(sine, cosine) * t).rotate(a);
  }

  /** The shortest rotation taking the unit vector `a` to `b`. */
  static function arc(a:Vec3, b:Vec3):Quat {
    var axis = a.cross(b);
    var sine = axis.norm();
    if (sine < 1e-9) return Quat.identity();
    return Quat.fromAxisAngle(axis.scale(1.0 / sine), Math.atan2(sine, a.dot(b)));
  }

  /** The roll about `wire` that takes `reached`, whose wire is `wire`, to `to`, in (-pi, pi]. */
  static function twistOf(reached:Quat, to:Quat, wire:Vec3):Float {
    var residual = to.multiply(reached.conjugate());
    var twist = 2.0 * Math.atan2(residual.x * wire.x + residual.y * wire.y + residual.z * wire.z, residual.w);
    while (twist > Math.PI) twist -= 2.0 * Math.PI;
    while (twist <= -Math.PI) twist += 2.0 * Math.PI;
    return twist;
  }
}
