package toolpathkit.tool;

import toolpathkit.tool.CutterSegment;
import toolpathkit.tool.CutterZone;

/**
  A rotating tool as a surface of revolution about its axis. Segments run
  from the tip (r = 0, z = 0) upwards and are continuous; the solid is closed
  by the axis and by a flat top at the last segment's height. Heights never
  decrease along the profile, so horizontal steps (a flat bottom, a shank or
  holder shoulder) are allowed but overhangs are not. Units are metres.
**/
class CutterProfile {
  public final segments:Array<CutterSegment>;

  public function new(segments:Array<CutterSegment>) {
    if (segments.length == 0) throw "cutter profile needs at least one segment";
    var r = 0.0, z = 0.0;
    for (segment in segments) {
      var start = startOf(segment), end = endOf(segment);
      if (Math.abs(start.r - r) > 1e-12 || Math.abs(start.z - z) > 1e-12)
        throw "cutter profile segments must be continuous from the tip at (0, 0)";
      if (!Math.isFinite(end.r) || !Math.isFinite(end.z) || end.r < 0.0)
        throw "cutter profile points need finite, non-negative radii";
      if (end.z < start.z - 1e-12)
        throw "cutter profile heights must not decrease";
      switch segment {
        case Arc(cr, cz, r0, z0, r1, z1, _):
          var a = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
          var b = Math.sqrt((r1 - cr) * (r1 - cr) + (z1 - cz) * (z1 - cz));
          if (!(a > 0.0) || Math.abs(a - b) > 1e-9 * Math.max(1.0, a))
            throw "cutter profile arc endpoints must be equidistant from its centre";
        case Line(_, _, _, _, _):
      }
      r = end.r;
      z = end.z;
    }
    if (!(z > 0.0) || !(r > 0.0))
      throw "cutter profile must end above the tip and away from the axis";
    this.segments = segments;
  }

  /** Flat end mill; the flutes run `fluteLength` up from the tip. */
  public static function flat(diameter:Float, fluteLength:Float):CutterProfile {
    var r = positive(diameter, "diameter") / 2;
    positive(fluteLength, "flute length");
    return new CutterProfile([
      Line(0, 0, r, 0, Cutting),
      Line(r, 0, r, fluteLength, Cutting)
    ]);
  }

  /** Ball end mill: a hemispherical tip of the cutter's radius. */
  public static function ball(diameter:Float, fluteLength:Float):CutterProfile {
    var r = positive(diameter, "diameter") / 2;
    if (!(fluteLength >= r)) throw "ball end mill flutes must cover the ball";
    var segments = [Arc(0, r, 0, 0, r, r, Cutting)];
    if (fluteLength > r) segments.push(Line(r, r, r, fluteLength, Cutting));
    return new CutterProfile(segments);
  }

  /** Bull-nose (corner radius) end mill. */
  public static function bullNose(diameter:Float, cornerRadius:Float,
      fluteLength:Float):CutterProfile {
    var r = positive(diameter, "diameter") / 2;
    positive(cornerRadius, "corner radius");
    if (cornerRadius >= r) throw "bull-nose corner radius must be below the cutter radius; use ball()";
    if (!(fluteLength >= cornerRadius)) throw "bull-nose flutes must cover the corner";
    var flatEnd = r - cornerRadius;
    var segments = [
      Line(0, 0, flatEnd, 0, Cutting),
      Arc(flatEnd, cornerRadius, flatEnd, 0, r, cornerRadius, Cutting)
    ];
    if (fluteLength > cornerRadius)
      segments.push(Line(r, cornerRadius, r, fluteLength, Cutting));
    return new CutterProfile(segments);
  }

  /** V-bit with a sharp tip; `includedAngle` is the full cone angle in radians. */
  public static function vee(diameter:Float, includedAngle:Float,
      fluteLength:Float):CutterProfile {
    var r = positive(diameter, "diameter") / 2;
    if (!(includedAngle > 0.0 && includedAngle < Math.PI))
      throw "V-bit included angle must be between 0 and pi";
    var coneHeight = r / Math.tan(includedAngle / 2);
    if (!(fluteLength >= coneHeight)) throw "V-bit flutes must cover the cone";
    var segments = [Line(0, 0, r, coneHeight, Cutting)];
    if (fluteLength > coneHeight)
      segments.push(Line(r, coneHeight, r, fluteLength, Cutting));
    return new CutterProfile(segments);
  }

  /**
    Tapered ball mill: a ball tip of `tipDiameter` blending tangentially into a
    cone of `taperAngle` (half-angle from the axis) up to `diameter`.
  **/
  public static function taperedBall(tipDiameter:Float, taperAngle:Float,
      diameter:Float, fluteLength:Float):CutterProfile {
    var tip = positive(tipDiameter, "tip diameter") / 2;
    var r = positive(diameter, "diameter") / 2;
    if (!(taperAngle > 0.0 && taperAngle < Math.PI / 2))
      throw "taper angle must be between 0 and pi/2";
    var tangentR = tip * Math.cos(taperAngle);
    var tangentZ = tip * (1 - Math.sin(taperAngle));
    if (!(r > tangentR)) throw "tapered ball diameter must exceed its ball";
    var coneTop = tangentZ + (r - tangentR) / Math.tan(taperAngle);
    if (!(fluteLength >= coneTop)) throw "tapered ball flutes must cover the taper";
    var segments = [
      Arc(0, tip, 0, 0, tangentR, tangentZ, Cutting),
      Line(tangentR, tangentZ, r, coneTop, Cutting)
    ];
    if (fluteLength > coneTop)
      segments.push(Line(r, coneTop, r, fluteLength, Cutting));
    return new CutterProfile(segments);
  }

  /** Appends a cylindrical non-cutting section, with a shoulder if the radius changes. */
  public function withShank(diameter:Float, length:Float):CutterProfile
    return withCylinder(diameter, length, Shank);

  public function withHolder(diameter:Float, length:Float):CutterProfile
    return withCylinder(diameter, length, Holder);

  public function height():Float
    return endOf(segments[segments.length - 1]).z;

  public function topRadius():Float
    return endOf(segments[segments.length - 1]).r;

  /** Widest diameter of the cutting zone. */
  public function cuttingDiameter():Float {
    var widest = 0.0;
    for (segment in segments) {
      if (zoneOf(segment) != Cutting) continue;
      widest = Math.max(widest, Math.max(startOf(segment).r, endOf(segment).r));
      switch segment {
        case Arc(cr, cz, r0, z0, r1, z1, _):
          // An arc bulges outwards past its ends when it crosses angle 0.
          var arc = arcAngles(cr, cz, r0, z0, r1, z1);
          if (Math.min(arc.start, arc.end) < 0.0 && Math.max(arc.start, arc.end) > 0.0)
            widest = Math.max(widest, cr + arc.radius);
        case Line(_, _, _, _, _):
      }
    }
    return 2 * widest;
  }

  /** Height of the top of the cutting zone. */
  public function fluteLength():Float {
    var top = 0.0;
    for (segment in segments)
      if (zoneOf(segment) == Cutting) top = endOf(segment).z;
    return top;
  }

  /**
    The part of the tool up to `height` above its tip, closed by a flat top
    there: the tool as seen by stock whose surface is `height` above the tip.
  **/
  public function below(height:Float):CutterProfile {
    if (!(height > 0.0)) throw "cutter profile can only be cut above its tip";
    var kept:Array<CutterSegment> = [];
    for (segment in segments) {
      var start = startOf(segment), end = endOf(segment);
      if (start.z >= height) break;
      if (end.z <= height) {
        kept.push(segment);
        continue;
      }
      switch segment {
        case Line(r0, z0, r1, z1, zone):
          kept.push(Line(r0, z0, r0 + (r1 - r0) * (height - z0) / (z1 - z0), height, zone));
        case Arc(cr, cz, r0, z0, r1, z1, zone):
          // The branch of the circle that lies between the arc's end radii.
          var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
          var half = Math.sqrt(Math.max(0.0, radius * radius - (height - cz) * (height - cz)));
          var low = Math.min(r0, r1) - 1e-12, high = Math.max(r0, r1) + 1e-12;
          var r = cr + half >= low && cr + half <= high ? cr + half : cr - half;
          kept.push(Arc(cr, cz, r0, z0, r, height, zone));
      }
      break;
    }
    return new CutterProfile(kept);
  }

  /** Area between the axis and the profile: half the tool's cross-section. */
  public function halfSectionArea():Float {
    // Green's theorem: the axis (r = 0) and the flat top (dz = 0) add nothing
    // to the integral of r dz, leaving only the profile.
    var area = 0.0;
    for (segment in segments) switch segment {
      case Line(r0, z0, r1, z1, _): area += (r0 + r1) / 2 * (z1 - z0);
      case Arc(cr, cz, r0, z0, r1, z1, _):
        var arc = arcAngles(cr, cz, r0, z0, r1, z1);
        var R = arc.radius;
        // r = cr + R cos t, dz = R cos t dt
        area += cr * R * (Math.sin(arc.end) - Math.sin(arc.start))
          + R * R * (cosSquaredIntegral(arc.end) - cosSquaredIntegral(arc.start));
    }
    return area;
  }

  /** Volume of the solid tool. */
  public function volume():Float {
    var volume = 0.0;
    for (segment in segments) switch segment {
      case Line(r0, z0, r1, z1, _):
        volume += Math.PI * (z1 - z0) * (r0 * r0 + r0 * r1 + r1 * r1) / 3;
      case Arc(cr, cz, r0, z0, r1, z1, _):
        var arc = arcAngles(cr, cz, r0, z0, r1, z1);
        var R = arc.radius;
        // pi * integral of (cr + R cos t)^2 R cos t dt
        function antiderivative(t:Float):Float {
          var s = Math.sin(t);
          return cr * cr * s + 2 * cr * R * cosSquaredIntegral(t)
            + R * R * (s - s * s * s / 3);
        }
        volume += Math.PI * R * (antiderivative(arc.end) - antiderivative(arc.start));
    }
    return volume;
  }

  static function cosSquaredIntegral(t:Float):Float
    return t / 2 + Math.sin(2 * t) / 4;

  /** Start and end angles of a minor arc, with |end - start| < pi. */
  static function arcAngles(cr:Float, cz:Float, r0:Float, z0:Float, r1:Float,
      z1:Float):{start:Float, end:Float, radius:Float} {
    var start = Math.atan2(z0 - cz, r0 - cr);
    var sweep = Math.atan2(z1 - cz, r1 - cr) - start;
    while (sweep > Math.PI) sweep -= 2 * Math.PI;
    while (sweep <= -Math.PI) sweep += 2 * Math.PI;
    return {start: start, end: start + sweep,
      radius: Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz))};
  }

  /** Radius never shrinks going up, so the tool's silhouette narrows only towards the tip. */
  public function isRadiallyMonotonic():Bool {
    for (segment in segments) switch segment {
      case Line(r0, _, r1, _, _): if (r1 < r0 - 1e-12) return false;
      case Arc(cr, _, r0, _, r1, _, _):
        if (r1 < r0 - 1e-12) return false;
        // A minor arc whose ends both lie left of its centre bulges inwards.
        if (r0 < cr - 1e-12 && r1 < cr - 1e-12) return false;
    }
    return true;
  }

  public static function startOf(segment:CutterSegment):{r:Float, z:Float}
    return switch segment {
      case Line(r0, z0, _, _, _): {r: r0, z: z0};
      case Arc(_, _, r0, z0, _, _, _): {r: r0, z: z0};
    };

  public static function endOf(segment:CutterSegment):{r:Float, z:Float}
    return switch segment {
      case Line(_, _, r1, z1, _): {r: r1, z: z1};
      case Arc(_, _, _, _, r1, z1, _): {r: r1, z: z1};
    };

  public static function zoneOf(segment:CutterSegment):CutterZone
    return switch segment {
      case Line(_, _, _, _, zone): zone;
      case Arc(_, _, _, _, _, _, zone): zone;
    };

  function withCylinder(diameter:Float, length:Float,
      zone:CutterZone):CutterProfile {
    var r = positive(diameter, "diameter") / 2;
    positive(length, "length");
    var z = height(), top = topRadius();
    var next = segments.copy();
    if (Math.abs(r - top) > 1e-12) next.push(Line(top, z, r, z, zone));
    next.push(Line(r, z, r, z + length, zone));
    return new CutterProfile(next);
  }

  static function positive(value:Float, name:String):Float {
    if (!(value > 0.0) || !Math.isFinite(value))
      throw 'cutter $name must be positive and finite';
    return value;
  }
}
