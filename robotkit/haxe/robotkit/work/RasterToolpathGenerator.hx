package robotkit.work;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.process.ToolpathPoint;
import robotkit.process.Toolpath;

/**
 * Boustrophedon raster over a `WorkSurface`'s boundary minus its exclusions,
 * clipped row by row with polygon scanline intersection. Each exclusion is
 * expanded by the tool radius before its intervals are removed. The expanded
 * intervals are the exact horizontal slices of the polygon's Minkowski sum
 * with a disk: the polygon interior, an offset strip around every edge, and a
 * disk around every vertex are unioned. This handles rotated and concave
 * exclusions without relying on their bounding boxes. The tool is off while
 * transiting between rows and across exclusion gaps; lead-in/lead-out points
 * bracket the whole path off-process. Curved surfaces are out of scope;
 * geometry stays planar.
 */
class RasterToolpathGenerator {
  public static function generate(surface:WorkSurface, toolWidth:Float, overlap:Float,
      standoff:Float, feedRate:Float, leadInOut:Float):Toolpath {
    if (surface == null) throw "Raster toolpath generation requires a work surface";
    if (!Math.isFinite(toolWidth) || toolWidth <= 0.0) throw "Raster tool width must be positive and finite";
    if (!Math.isFinite(overlap) || overlap < 0.0 || overlap >= 1.0) throw "Raster overlap must be in [0, 1)";
    if (!Math.isFinite(standoff) || standoff < 0.0) throw "Raster standoff must be finite and non-negative";
    if (!Math.isFinite(feedRate) || feedRate <= 0.0) throw "Raster feed rate must be positive and finite";
    if (!Math.isFinite(leadInOut) || leadInOut < 0.0) throw "Raster lead-in/out must be finite and non-negative";

    var halfWidth = toolWidth * 0.5;
    var bounds = surface.boundary.bounds();
    var rowSpacing = toolWidth * (1.0 - overlap);
    var rows:Array<Float> = [];
    if (bounds.minY + halfWidth > bounds.maxY - halfWidth) {
      rows.push((bounds.minY + bounds.maxY) * 0.5);
    } else {
      var y = bounds.minY + halfWidth;
      while (y <= bounds.maxY - halfWidth + 1e-9) {
        rows.push(y);
        y += rowSpacing;
      }
      var lastRowY = bounds.maxY - halfWidth;
      if (rows.length == 0 || rows[rows.length - 1] < lastRowY - 1e-9) rows.push(lastRowY);
    }

    var points:Array<ToolpathPoint> = [];
    var forward = true;
    var firstStart = 0.0, firstRowY = 0.0, firstDir = 1.0;
    var lastEnd = 0.0, lastRowY = 0.0, lastDir = 1.0;
    var wroteAny = false;

    for (rowY in rows) {
      var allowed:Array<Interval1D> = [];
      for (raw in surface.boundary.scanlineIntervals(rowY))
        allowed.push(new Interval1D(raw[0], raw[1]));
      var forbidden:Array<Array<Float>> = [];
      for (exclusion in surface.exclusions)
        forbidden = forbidden.concat(expandedExclusionIntervals(exclusion, rowY, halfWidth));
      allowed = subtractIntervals(allowed, mergeIntervals(forbidden));
      var kept:Array<Array<Float>> = [];
      for (interval in allowed) {
        if (interval.end - interval.start > 1e-6) kept.push([interval.start, interval.end]);
      }
      if (kept.length == 0) continue;
      if (!forward) kept.reverse();

      for (interval in kept) {
        var a = forward ? interval[0] : interval[1];
        var b = forward ? interval[1] : interval[0];
        if (!wroteAny) {
          firstStart = a;
          firstRowY = rowY;
          firstDir = forward ? 1.0 : -1.0;
          wroteAny = true;
        }
        points.push(new ToolpathPoint(surfacePose(a, rowY, standoff), feedRate, true));
        points.push(new ToolpathPoint(surfacePose(b, rowY, standoff), feedRate, false));
        lastEnd = b;
        lastRowY = rowY;
        lastDir = forward ? 1.0 : -1.0;
      }
      forward = !forward;
    }

    if (!wroteAny) throw "Raster toolpath generator found no reachable area inside the boundary";

    points.unshift(new ToolpathPoint(surfacePose(firstStart - firstDir * leadInOut, firstRowY, standoff), feedRate, false));
    points.push(new ToolpathPoint(surfacePose(lastEnd + lastDir * leadInOut, lastRowY, standoff), feedRate, false));

    return new Toolpath(surface.surfaceFrameId, points);
  }

  /** TCP at surface-local `(x, y, standoff)`, facing into the surface (tool +Z opposite the surface's +Z normal). */
  static function surfacePose(x:Float, y:Float, standoff:Float):Transform3 {
    var faceSurface = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI);
    return new Transform3(new Vec3(x, y, standoff), faceSurface);
  }

  /** Returns the horizontal slice of an exclusion expanded by `radius`. */
  static function expandedExclusionIntervals(exclusion:Polygon2, y:Float,
      radius:Float):Array<Array<Float>> {
    var intervals = exclusion.scanlineIntervals(y);
    var vertices = exclusion.vertices();
    for (index in 0...vertices.length) {
      var a = vertices[index];
      var b = vertices[(index + 1) % vertices.length];
      var dy = y - a.y;
      if (Math.abs(dy) <= radius + 1e-12) {
        var halfChord = Math.sqrt(Math.max(0.0, radius * radius - dy * dy));
        intervals.push([a.x - halfChord, a.x + halfChord]);
      }

      var edgeX = b.x - a.x, edgeY = b.y - a.y;
      var edgeLength = Math.sqrt(edgeX * edgeX + edgeY * edgeY);
      if (edgeLength <= 1e-12) continue;
      var normalX = -edgeY / edgeLength, normalY = edgeX / edgeLength;
      var strip = new Polygon2([
        new Point2(a.x - normalX * radius, a.y - normalY * radius),
        new Point2(b.x - normalX * radius, b.y - normalY * radius),
        new Point2(b.x + normalX * radius, b.y + normalY * radius),
        new Point2(a.x + normalX * radius, a.y + normalY * radius)
      ]);
      intervals = intervals.concat(strip.scanlineIntervals(y));
    }
    return mergeIntervals(intervals);
  }

  /** Merges overlapping or touching horizontal intervals in ascending order. */
  static function mergeIntervals(input:Array<Array<Float>>):Array<Array<Float>> {
    var sorted:Array<Array<Float>> = [];
    for (interval in input) if (interval != null && interval.length >= 2 &&
        interval[1] - interval[0] > 1e-12)
      sorted.push([interval[0], interval[1]]);
    sorted.sort(function(left, right) {
      if (left[0] != right[0]) return left[0] < right[0] ? -1 : 1;
      return left[1] < right[1] ? -1 : (left[1] > right[1] ? 1 : 0);
    });
    var result:Array<Array<Float>> = [];
    for (interval in sorted) {
      if (result.length == 0 || interval[0] > result[result.length - 1][1] + 1e-10) {
        result.push(interval);
      } else if (interval[1] > result[result.length - 1][1]) {
        result[result.length - 1][1] = interval[1];
      }
    }
    return result;
  }

  /** Subtracts sorted, disjoint removal intervals from the base intervals. */
  static function subtractIntervals(base:Array<Interval1D>, remove:Array<Array<Float>>):Array<Interval1D> {
    var result:Array<Interval1D> = [];
    for (b in base) {
      var cursor = b.start;
      for (r in remove) {
        if (r[1] <= cursor + 1e-10) continue;
        if (r[0] >= b.end - 1e-10) break;
        if (r[0] > cursor) result.push(new Interval1D(cursor, Math.min(r[0], b.end)));
        if (r[1] > cursor) cursor = r[1];
        if (cursor >= b.end - 1e-10) break;
      }
      if (cursor < b.end - 1e-10) result.push(new Interval1D(cursor, b.end));
    }
    return result;
  }
}

/** A 1D interval left after clipping a scanline. */
private class Interval1D {
  public final start:Float;
  public final end:Float;

  public function new(start:Float, end:Float) {
    this.start = start;
    this.end = end;
  }
}
