package oracle;

import toolpathkit.path.PathGeometry;
import toolpathkit.tool.CutterProfile;
import stockkit.CutMove;

/** A closed interval of heights along a +Z ray, in metres. */
typedef Span = {lo:Float, hi:Float};

/**
  Sets along a ray that bracket an unknown one: `inner` is certainly inside
  it and `outer` certainly contains it. Both are sorted and disjoint.
**/
typedef SpanBounds = {inner:Array<Span>, outer:Array<Span>};

/** One tool motion for the reference: the tool shape and its tip path. */
typedef ReferenceMove = {profile:CutterProfile, geometry:PathGeometry};

/**
  Brute-force reference for the material one move sweeps along a vertical ray,
  for moves the exact OCCT oracle refuses (ramps, helices). It shares no code
  with the simulator core and bounds the answer instead of approximating it.

  The tool axis is +Z and the tool tip follows the path, whose parameter
  `t` runs from 0 to 1 at constant speed. Poses are sampled at `t = i / N`.

  - Inner: the union over samples of the tool's exact intersection with the
    ray. Every sampled pose is part of the sweep, so this lies inside it.
  - Outer: between samples the tool is at most `rho = length / (2N)` from
    the nearest sampled pose (a chord is never longer than its arc), so the
    sweep lies inside the union of sampled tools grown by `rho`. For a solid
    of revolution, the distance from a point to the tool is the 2D distance
    from its (radius, height) to the tool's full, mirrored cross-section, so
    each grown tool meets the ray in a set computed in closed form.

  Checks against these bounds are set containment, so a wrong reference can
  only raise a false alarm, never pass a wrong answer.
**/
class SampledReference {
  /** Bounds on the set of heights that `geometry` sweeps `profile` through along the ray at (x, y). */
  public static function sweep(profile:CutterProfile, geometry:PathGeometry,
      x:Float, y:Float, samples:Int):SpanBounds {
    if (samples < 1) throw "sampled reference needs at least one sample step";
    var path = new SampledPath(geometry);
    var section = new Section(profile);
    // Relative margin for rounding in the pose positions.
    var rho = path.length / (2 * samples) * (1 + 1e-9);
    var inner:Array<Span> = [], outer:Array<Span> = [];
    for (i in 0...samples + 1) {
      var t = i / samples;
      var dx = x - path.x(t), dy = y - path.y(t);
      var d = Math.sqrt(dx * dx + dy * dy);
      if (d > section.reach + rho) continue;
      var tipZ = path.z(t);
      var pose = section.inside(d);
      for (span in pose) inner.push({lo: span.lo + tipZ, hi: span.hi + tipZ});
      for (span in pose) outer.push({lo: span.lo + tipZ, hi: span.hi + tipZ});
      if (rho > 0)
        for (span in section.near(d, rho))
          outer.push({lo: span.lo + tipZ, hi: span.hi + tipZ});
    }
    return {inner: merge(inner), outer: merge(outer)};
  }

  /**
    Bounds on the material left on the ray at (x, y) after cutting `initial`
    with `moves`: `inner` (initial minus every move's outer sweep) certainly
    remains and `outer` (initial minus every inner sweep) contains all that
    remains.
  **/
  public static function remaining(initial:Array<Span>, moves:Array<ReferenceMove>,
      x:Float, y:Float, samples:Int):SpanBounds {
    var inner = merge(initial), outer = merge(initial);
    for (move in moves) {
      var swept = sweep(move.profile, move.geometry, x, y, samples);
      inner = subtract(inner, swept.outer);
      outer = subtract(outer, swept.inner);
    }
    return {inner: inner, outer: outer};
  }

  /** Reference moves cutting with each move's whole tool profile. */
  public static function fromCutMoves(moves:Array<CutMove>):Array<ReferenceMove>
    return [for (move in moves) switch move.motion {
      case Path(geometry): {profile: move.tool.profile(), geometry: geometry};
    }];

  /**
    True when `inner` ⊆ `candidate` ⊆ `outer`, allowing `slack` metres of
    rounding: each set is shrunk by `slack` before it must fit inside the
    next one grown by `slack`.
  **/
  public static function contains(inner:Array<Span>, candidate:Array<Span>,
      outer:Array<Span>, slack:Float):Bool
    return subset(erode(inner, slack), dilate(candidate, slack))
      && subset(erode(candidate, slack), dilate(outer, slack));

  /** Total length of a set of spans. */
  public static function measure(spans:Array<Span>):Float {
    var total = 0.0;
    for (span in merge(spans)) total += span.hi - span.lo;
    return total;
  }

  /** Sorted, disjoint union; touching spans join. */
  public static function merge(spans:Array<Span>):Array<Span> {
    var sorted = [for (span in spans) if (span.hi >= span.lo) {lo: span.lo, hi: span.hi}];
    sorted.sort((a, b) -> a.lo < b.lo ? -1 : a.lo > b.lo ? 1 : 0);
    var merged:Array<Span> = [];
    for (span in sorted) {
      var last = merged.length == 0 ? null : merged[merged.length - 1];
      if (last != null && span.lo <= last.hi) last.hi = Math.max(last.hi, span.hi);
      else merged.push({lo: span.lo, hi: span.hi});
    }
    return merged;
  }

  /** `a` minus the open interior of `b`: a closed set minus a closed set, closed again. */
  public static function subtract(a:Array<Span>, b:Array<Span>):Array<Span> {
    var cut = merge(b);
    var result:Array<Span> = [];
    for (span in merge(a)) {
      var lo = span.lo;
      var alive = true;
      for (hole in cut) {
        if (hole.hi <= lo || hole.lo >= span.hi) continue;
        if (hole.lo > lo) result.push({lo: lo, hi: hole.lo});
        if (hole.hi >= span.hi) {
          alive = false;
          break;
        }
        lo = hole.hi;
      }
      if (alive) result.push({lo: lo, hi: span.hi});
    }
    return merge(result);
  }

  static function dilate(spans:Array<Span>, by:Float):Array<Span>
    return merge([for (span in spans) {lo: span.lo - by, hi: span.hi + by}]);

  static function erode(spans:Array<Span>, by:Float):Array<Span>
    return [for (span in merge(spans)) if (span.hi - span.lo > 2 * by)
      {lo: span.lo + by, hi: span.hi - by}];

  /** Every span of `a` lies inside one span of `b` (both merged). */
  static function subset(a:Array<Span>, b:Array<Span>):Bool {
    for (span in a) {
      var covered = false;
      for (other in b) if (other.lo <= span.lo && span.hi <= other.hi) {
        covered = true;
        break;
      }
      if (!covered) return false;
    }
    return true;
  }
}

/** A tool-tip path with constant-speed parameter t in [0, 1]. */
private class SampledPath {
  public final length:Float;
  final geometry:PathGeometry;

  public function new(geometry:PathGeometry) {
    this.geometry = geometry;
    length = switch geometry {
      case Line(a, b):
        Math.sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)
          + (b.z - a.z) * (b.z - a.z));
      case Arc(_, radius, _, sweep): Math.abs(radius * sweep);
      case Circular(_, radius, _, sweep, plane, rise):
        if (plane != XY) throw "sampled reference only sweeps XY arcs and helices";
        Math.sqrt(radius * sweep * radius * sweep + rise * rise);
    };
  }

  public function x(t:Float):Float
    return switch geometry {
      case Line(a, b): a.x + (b.x - a.x) * t;
      case Arc(c, radius, start, sweep): c.x + radius * Math.cos(start + sweep * t);
      case Circular(c, radius, start, sweep, _, _): c.x + radius * Math.cos(start + sweep * t);
    };

  public function y(t:Float):Float
    return switch geometry {
      case Line(a, b): a.y + (b.y - a.y) * t;
      case Arc(c, radius, start, sweep): c.y + radius * Math.sin(start + sweep * t);
      case Circular(c, radius, start, sweep, _, _): c.y + radius * Math.sin(start + sweep * t);
    };

  public function z(t:Float):Float
    return switch geometry {
      case Line(a, b): a.z + (b.z - a.z) * t;
      case Arc(c, _, _, _): c.z;
      case Circular(c, _, _, _, _, rise): c.z + rise * t;
    };
}

/**
  The tool's cross-section in (r, z) with the tip at the origin. `inside`
  and `near` answer for the vertical line at radius `d` >= 0.
**/
private class Section {
  /** No part of the tool is further than this from its axis. */
  public final reach:Float;
  final profile:CutterProfile;
  final height:Float;
  final top:Float;

  public function new(profile:CutterProfile) {
    this.profile = profile;
    height = profile.height();
    top = profile.topRadius();
    var widest = 0.0;
    for (segment in profile.segments) switch segment {
      case Line(r0, _, r1, _, _): widest = Math.max(widest, Math.max(r0, r1));
      case Arc(cr, cz, r0, z0, _, _, _):
        widest = Math.max(widest, cr + Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz)));
    }
    reach = widest;
  }

  /**
    Heights where the line r = d is inside the tool. Heights never decrease
    along the profile, so the tool at height z is r <= R(z), and each segment
    contributes the heights in its range where its radius reaches d.
  **/
  public function inside(d:Float):Array<Span> {
    var spans:Array<Span> = [];
    for (segment in profile.segments) switch segment {
      case Line(r0, z0, r1, z1, _):
        if (z1 == z0) {
          if (Math.max(r0, r1) >= d) spans.push({lo: z0, hi: z0});
        } else if (r1 == r0) {
          if (r0 >= d) spans.push({lo: z0, hi: z1});
        } else {
          // r(z) = r0 + (r1 - r0) s with s = (z - z0) / (z1 - z0) in [0, 1].
          var s = (d - r0) / (r1 - r0);
          if (r1 > r0) {
            if (s <= 1) spans.push({lo: z0 + (z1 - z0) * Math.max(0.0, s), hi: z1});
          } else if (s >= 0) {
            spans.push({lo: z0, hi: z0 + (z1 - z0) * Math.min(1.0, s)});
          }
        }
      case Arc(cr, cz, r0, z0, r1, z1, _):
        var arc = ArcGeometry.of(cr, cz, r0, z0, r1, z1);
        // Points of the arc with cr + R cos(a) >= d.
        var k = (d - cr) / arc.radius;
        if (k > 1) continue;
        var half = k <= -1 ? Math.PI : Math.acos(k);
        for (turn in -1...2) {
          var lo = Math.max(arc.low, -half + 2 * Math.PI * turn);
          var hi = Math.min(arc.high, half + 2 * Math.PI * turn);
          if (lo <= hi) spans.push(arc.heights(lo, hi));
        }
    }
    return SampledReference.merge(spans);
  }

  /**
    Heights on the line r = d within `rho` of the boundary of the full,
    mirrored section: the profile, its mirror image and the flat top.
  **/
  public function near(d:Float, rho:Float):Array<Span> {
    var spans:Array<Span> = [];
    for (mirror in [1.0, -1.0]) for (segment in profile.segments) switch segment {
      case Line(r0, z0, r1, z1, _):
        capsule(mirror * r0, z0, mirror * r1, z1, d, rho, spans);
      case Arc(cr, cz, r0, z0, r1, z1, _):
        annularSector(mirror * cr, cz, mirror * r0, z0, mirror * r1, z1, d, rho, spans);
    }
    capsule(-top, height, top, height, d, rho, spans);
    return SampledReference.merge(spans);
  }

  /** The rho-neighbourhood of segment A-B, on the line r = d. */
  static function capsule(ar:Float, az:Float, br:Float, bz:Float, d:Float,
      rho:Float, spans:Array<Span>):Void {
    disk(ar, az, rho, d, spans);
    disk(br, bz, rho, d, spans);
    var length = Math.sqrt((br - ar) * (br - ar) + (bz - az) * (bz - az));
    if (length == 0) return;
    var ur = (br - ar) / length, uz = (bz - az) / length;
    // Points (d, z) with 0 <= u.(p - A) <= length and -rho <= n.(p - A) <= rho.
    var clip = new Clip();
    var along = ur * (d - ar) - uz * az;
    clip.within(along, uz, 0, length);
    var across = -uz * (d - ar) - ur * az;
    clip.within(across, ur, -rho, rho);
    if (clip.lo <= clip.hi) spans.push({lo: clip.lo, hi: clip.hi});
  }

  /** The rho-neighbourhood of a minor arc, on the line r = d. */
  static function annularSector(cr:Float, cz:Float, r0:Float, z0:Float, r1:Float,
      z1:Float, d:Float, rho:Float, spans:Array<Span>):Void {
    disk(r0, z0, rho, d, spans);
    disk(r1, z1, rho, d, spans);
    var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
    var outerR = radius + rho, innerR = radius - rho;
    var dr = d - cr;
    if (Math.abs(dr) > outerR) return;
    var h = Math.sqrt(outerR * outerR - dr * dr);
    var rings:Array<Span> = [];
    if (innerR <= 0 || Math.abs(dr) >= innerR) {
      rings.push({lo: cz - h, hi: cz + h});
    } else {
      var g = Math.sqrt(innerR * innerR - dr * dr);
      rings.push({lo: cz - h, hi: cz - g});
      rings.push({lo: cz + g, hi: cz + h});
    }
    // The wedge swept by a minor arc is convex: the two half-planes on the
    // arc's side of its end radii.
    var ar = r0 - cr, az = z0 - cz, br = r1 - cr, bz = z1 - cz;
    var turn = ar * bz - az * br >= 0 ? 1.0 : -1.0;
    for (ring in rings) {
      var clip = new Clip();
      clip.lo = ring.lo;
      clip.hi = ring.hi;
      // cross(A, p - c) * turn >= 0 and cross(p - c, B) * turn >= 0, with
      // p - c = (dr, z - cz) and cross(u, v) = u.r v.z - u.z v.r.
      clip.within(turn * (-ar * cz - az * dr), turn * ar, 0, Math.POSITIVE_INFINITY);
      clip.within(turn * (dr * bz + cz * br), -turn * br, 0, Math.POSITIVE_INFINITY);
      if (clip.lo <= clip.hi) spans.push({lo: clip.lo, hi: clip.hi});
    }
  }

  static function disk(cr:Float, cz:Float, rho:Float, d:Float, spans:Array<Span>):Void {
    var dr = d - cr;
    if (Math.abs(dr) > rho) return;
    var h = Math.sqrt(rho * rho - dr * dr);
    spans.push({lo: cz - h, hi: cz + h});
  }
}

/** Heights z where `offset + slope * z` lies in [min, max], narrowed in place. */
private class Clip {
  public var lo = Math.NEGATIVE_INFINITY;
  public var hi = Math.POSITIVE_INFINITY;

  public function new() {}

  public function within(offset:Float, slope:Float, min:Float, max:Float):Void {
    if (slope == 0) {
      if (offset < min || offset > max) {
        lo = Math.POSITIVE_INFINITY;
        hi = Math.NEGATIVE_INFINITY;
      }
      return;
    }
    var a = (min - offset) / slope, b = (max - offset) / slope;
    lo = Math.max(lo, Math.min(a, b));
    hi = Math.min(hi, Math.max(a, b));
  }
}

/** A profile arc as a centre, radius and angle range [low, high]. */
private class ArcGeometry {
  public final cz:Float;
  public final radius:Float;
  public final low:Float;
  public final high:Float;

  function new(cz:Float, radius:Float, low:Float, high:Float) {
    this.cz = cz;
    this.radius = radius;
    this.low = low;
    this.high = high;
  }

  public static function of(cr:Float, cz:Float, r0:Float, z0:Float, r1:Float,
      z1:Float):ArcGeometry {
    var start = Math.atan2(z0 - cz, r0 - cr);
    var sweep = Math.atan2(z1 - cz, r1 - cr) - start;
    while (sweep > Math.PI) sweep -= 2 * Math.PI;
    while (sweep <= -Math.PI) sweep += 2 * Math.PI;
    var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
    return new ArcGeometry(cz, radius, Math.min(start, start + sweep),
      Math.max(start, start + sweep));
  }

  /** Range of cz + R sin(a) for a in [lo, hi]. */
  public function heights(lo:Float, hi:Float):Span {
    var a = cz + radius * Math.sin(lo), b = cz + radius * Math.sin(hi);
    var min = Math.min(a, b), max = Math.max(a, b);
    for (turn in -2...3) {
      var top = Math.PI / 2 + 2 * Math.PI * turn;
      if (top >= lo && top <= hi) max = cz + radius;
      var bottom = -Math.PI / 2 + 2 * Math.PI * turn;
      if (bottom >= lo && bottom <= hi) min = cz - radius;
    }
    return {lo: min, hi: max};
  }
}
