package stockkit;

import gearkit.GearSpec;
import gearkit.ToothProfile;
import gearkit.Point2;

/** Samples an ideal, undeformed spur gear on an existing ring lattice.
    Only angular crossings are solved; there is no angular tessellation error. */
class RingGearTarget {
  public static function create(grid:RingGrid, spec:GearSpec):RingStock {
    var profile = new ToothProfile(spec), stock = RingStock.annulus(grid);
    try {
      var first = 0;
      while (first < grid.axialCount && grid.z(first) < 0) first++;
      var end = first;
      while (end < grid.axialCount && grid.z(end) <= spec.faceWidth) end++;
      for (r in 0...grid.radialCount) {
        var cuts:Array<RingCut> = [];
        if (first > 0) cuts.push(new RingCut(r, 0, first, 0, 2 * Math.PI, 0, 0));
        if (end < grid.axialCount) cuts.push(new RingCut(r, end, grid.axialCount - end, 0, 2 * Math.PI, 0, 0));
        if (end > first) {
          var material = angles(profile, grid.radius(r));
          var cursor = 0.0;
          for (span in material) {
            if (span.lo > cursor) cuts.push(new RingCut(r, first, end - first, cursor, span.lo - cursor, 0, 0));
            cursor = span.hi;
          }
          if (cursor < 2 * Math.PI) cuts.push(new RingCut(r, first, end - first, cursor, 2 * Math.PI - cursor, 0, 0));
        }
        stock.cutRegion(cuts);
      }
      return stock;
    } catch (error:Dynamic) { stock.dispose(); throw error; }
  }

  static function angles(profile:ToothProfile, radius:Float):Array<RingInterval> {
    var spec = profile.spec, full = 2 * Math.PI;
    if ((!spec.internal && radius <= spec.rootRadius()) || (spec.internal && radius >= spec.rootRadius()))
      return [new RingInterval(0, full, -1, -1, -1, -1)];
    if ((!spec.internal && radius > spec.tipRadius()) || (spec.internal && radius < spec.tipRadius())) return [];
    var lo = Math.min(profile.rootJoinRadius, profile.tipJoinRadius);
    var hi = Math.max(profile.rootJoinRadius, profile.tipJoinRadius);
    var point:Point2;
    if (radius >= lo && radius <= hi) point = profile.flankAtRadius(radius);
    else {
      var root = spec.sign() * (radius - profile.rootJoinRadius) < 0;
      var a = 0.0, b = 1.0;
      // Root parameter runs flank->root; tip parameter runs flank->tip.
      var increasing = root ? spec.internal : !spec.internal;
      for (_ in 0...64) {
        var t = (a + b) / 2;
        var p = root ? profile.rootFillet(t) : profile.tipFillet(t);
        if ((p.radius() < radius) == increasing) a = t; else b = t;
      }
      point = root ? profile.rootFillet((a + b) / 2) : profile.tipFillet((a + b) / 2);
    }
    var half = Math.atan2(point.x, point.y), pitch = full / spec.teeth;
    var spans:Array<RingInterval> = [];
    for (tooth in 0...spec.teeth) {
      var centre = (Math.PI / 2 + tooth * pitch) % full;
      var left = centre - half, right = centre + half;
      if (left < 0) {
        spans.push(new RingInterval(0, right, -1, -1, -1, -1));
        spans.push(new RingInterval(left + full, full, -1, -1, -1, -1));
      } else if (right > full) {
        spans.push(new RingInterval(left, full, -1, -1, -1, -1));
        spans.push(new RingInterval(0, right - full, -1, -1, -1, -1));
      } else spans.push(new RingInterval(left, right, -1, -1, -1, -1));
    }
    spans.sort((a, b) -> a.lo < b.lo ? -1 : a.lo > b.lo ? 1 : 0);
    return spans;
  }
}
