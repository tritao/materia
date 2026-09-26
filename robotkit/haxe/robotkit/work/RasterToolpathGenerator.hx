package robotkit.work;

import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.process.ToolpathPoint;
import robotkit.process.Toolpath;

/**
 * Boustrophedon raster over a `WorkSurface`'s boundary minus its exclusions,
 * clipped row by row with polygon scanline intersection. An interval end
 * created by cutting into an exclusion is pulled inward by half the tool
 * width before laying down points, so the tool's own footprint radius (not
 * just its center) stays clear of the exclusion; an end that is the outer
 * boundary itself is left alone, since a footprint bulging past the
 * boundary edge is harmless. This is a 1D stand-in for a full polygon
 * offset, adequate for the axis-aligned/rectangular surfaces this milestone
 * targets. The tool is off while transiting between rows and across
 * exclusion gaps; lead-in/lead-out points bracket the whole path
 * off-process. Curved surfaces are out of scope; geometry stays planar.
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
      for (raw in surface.boundary.scanlineIntervals(rowY)) allowed.push(new Interval1D(raw[0], raw[1], false, false));
      // Subtract by the exclusion's own bounding-box X-range whenever the
      // row's *footprint band* (not just its centerline) reaches the
      // exclusion's Y bounds: a row whose scanline misses a nearby
      // exclusion can still graze it with the tool's radius. This is exact
      // for axis-aligned rectangular exclusions and conservative otherwise.
      for (exclusion in surface.exclusions) {
        var exclusionBounds = exclusion.bounds();
        if (rowY + halfWidth >= exclusionBounds.minY && rowY - halfWidth <= exclusionBounds.maxY)
          allowed = subtractIntervals(allowed, [[exclusionBounds.minX, exclusionBounds.maxX]]);
      }
      var kept:Array<Array<Float>> = [];
      for (interval in allowed) {
        var start = interval.start + (interval.startIsExclusion ? halfWidth : 0.0);
        var end = interval.end - (interval.endIsExclusion ? halfWidth : 0.0);
        if (end - start > 1e-6) kept.push([start, end]);
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

  /**
   * Subtracts raw exclusion intervals from `base`. A remainder edge created
   * by the cut is flagged `IsExclusion = true`; an untouched original edge
   * keeps its existing flag.
   */
  static function subtractIntervals(base:Array<Interval1D>, remove:Array<Array<Float>>):Array<Interval1D> {
    var result = base;
    for (r in remove) {
      var next:Array<Interval1D> = [];
      for (b in result) {
        if (r[1] <= b.start || r[0] >= b.end) {
          next.push(b);
          continue;
        }
        if (r[0] > b.start) next.push(new Interval1D(b.start, Math.min(r[0], b.end), b.startIsExclusion, true));
        if (r[1] < b.end) next.push(new Interval1D(Math.max(r[1], b.start), b.end, true, b.endIsExclusion));
      }
      result = next;
    }
    return result;
  }
}

/** A 1D interval with per-end provenance: does this end touch an exclusion cut, or the outer boundary? */
private class Interval1D {
  public final start:Float;
  public final end:Float;
  public final startIsExclusion:Bool;
  public final endIsExclusion:Bool;

  public function new(start:Float, end:Float, startIsExclusion:Bool, endIsExclusion:Bool) {
    this.start = start;
    this.end = end;
    this.startIsExclusion = startIsExclusion;
    this.endIsExclusion = endIsExclusion;
  }
}
