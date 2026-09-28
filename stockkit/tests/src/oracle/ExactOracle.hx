package oracle;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Axis;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import stockkit.CutMove;
import toolpathkit.tool.CutterProfile;

/**
  Exact reference results built with OCCT booleans through CadKit. Tools point
  along +Z with the tip at the programmed point. Supported moves are the ones
  whose swept volume has a closed B-rep form: horizontal lines, vertical
  lines, and XY arcs at constant height. Everything else is rejected rather
  than approximated, so a passing comparison is always against exact geometry.
**/
class ExactOracle {
  /** The tool as a solid with its tip at `tip`. */
  public static function toolSolid(profile:CutterProfile, tip:Point3):Part {
    var section = sectionFace(profile, new Vector(tip.x, tip.y, tip.z),
      Vector.X(), 0.0, true);
    try {
      var result = section.revolve(new Axis(new Vector(tip.x, tip.y, tip.z),
        Vector.Z()));
      section.close();
      return result;
    } catch (error:Dynamic) {
      section.close();
      throw error;
    }
  }

  /**
    The region swept by the tool along one move. Each result is checked
    against its closed-form volume, so a boolean that returns a valid but
    wrong solid fails here instead of skewing a comparison.
  **/
  public static function sweptSolid(profile:CutterProfile,
      geometry:PathGeometry):Part {
    if (!profile.isRadiallyMonotonic())
      throw "exact oracle needs a tool whose radius never shrinks upwards";
    var section = 2 * profile.halfSectionArea(), tool = profile.volume();
    var expected = 0.0;
    var solid = switch geometry {
      case Line(start, end):
        var dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
        var horizontal = Math.sqrt(dx * dx + dy * dy);
        if (horizontal <= 1e-12 && Math.abs(dz) <= 1e-12) {
          expected = tool;
          toolSolid(profile, start);
        } else if (Math.abs(dz) <= 1e-12) {
          expected = section * horizontal + tool;
          horizontalLine(profile, start, end);
        } else if (horizontal <= 1e-12) {
          var top = profile.topRadius();
          expected = tool + Math.PI * top * top * Math.abs(dz);
          verticalLine(profile, start.z < end.z ? start : end, Math.abs(dz));
        } else
          throw "exact oracle cannot sweep a line that moves in XY and Z together";
      case Arc(center, radius, startAngle, sweep):
        // Pappus: the section is symmetric about the arc radius.
        expected = Math.abs(sweep) >= 2 * Math.PI - 1e-12
          ? section * radius * 2 * Math.PI
          : section * radius * Math.abs(sweep) + tool;
        arc(profile, center, radius, startAngle, sweep);
      case Circular(_, _, _, _, _, _):
        throw "exact oracle cannot sweep helical or non-XY circular moves";
    };
    var actual = solid.volume();
    if (Math.abs(actual - expected) > VOLUME_TOLERANCE * expected) {
      solid.close();
      throw 'exact oracle swept solid has volume $actual, expected $expected';
    }
    return solid;
  }

  /** Relative agreement required between OCCT and closed-form volumes. */
  public static inline final VOLUME_TOLERANCE = 1e-7;

  /**
    Stock minus every move's swept volume, subtracted one move at a time. Each
    step must remove no more than that move's (already checked) swept volume
    and never add material; a boolean that goes wrong between moves fails at
    the move that caused it rather than skewing the final stock.
  **/
  public static function cut(stock:Part, moves:Array<CutMove>):Part {
    var current = new Part(stock.shape.cloneShape());
    var volume = current.volume();
    try {
      for (index in 0...moves.length) {
        var sweep = switch moves[index].motion {
          case Path(geometry): sweptSolid(moves[index].tool.profile(), geometry);
        };
        var sweepVolume = sweep.volume();
        var next:Part;
        try {
          next = current.subtract(sweep);
        } catch (error:Dynamic) {
          sweep.close();
          throw 'exact oracle failed to cut move $index: $error';
        }
        sweep.close();
        current.close();
        current = next;
        if (!current.valid()) throw 'exact oracle produced invalid stock at move $index';
        var remaining = current.volume();
        var removed = volume - remaining;
        var slack = VOLUME_TOLERANCE * sweepVolume;
        if (removed < -slack || removed > sweepVolume + slack)
          throw 'exact oracle move $index removed $removed, outside 0..$sweepVolume';
        volume = remaining;
      }
      return current;
    } catch (error:Dynamic) {
      current.close();
      throw error;
    }
  }

  /** Moves of one tool along plain geometry, for fixtures without a program. */
  public static function pathMoves(profile:CutterProfile,
      geometries:Array<PathGeometry>):Array<CutMove> {
    var tool = Tool.shaped(1, 0.0, profile);
    return [for (index in 0...geometries.length)
      new CutMove(tool, Path(geometries[index]), Cut, index,
        new Provenance(index + 1, 1, 0))];
  }

  /**
    Material intervals along a ray, as distances from `origin` along the unit
    `direction`, clipped to `length`. This is what a dexel stores, so it is
    the exact reference for comparing simulated rays.
  **/
  public static function rayIntervals(part:Part, origin:Vector,
      direction:Vector, length:Float):Array<{enter:Float, exit:Float}> {
    var unit = direction.normalized();
    var probe = Curve.line(origin, origin.add(unit.scale(length)));
    var inside:Null<Shape> = null;
    try {
      inside = probe.shape.common(part.shape);
      var intervals:Array<{enter:Float, exit:Float}> = [];
      for (i in 0...inside.subshapeCount(CadKit.ShapeKind.Edge)) {
        var edge = inside.subshape(CadKit.ShapeKind.Edge, i);
        var a = Math.POSITIVE_INFINITY, b = Math.NEGATIVE_INFINITY;
        for (j in 0...edge.subshapeCount(CadKit.ShapeKind.Vertex)) {
          var vertex = edge.subshape(CadKit.ShapeKind.Vertex, j);
          var t = Vector.fromNative(vertex.position()).subtract(origin).dot(unit);
          vertex.close();
          a = Math.min(a, t);
          b = Math.max(b, t);
        }
        edge.close();
        if (b > a) intervals.push({enter: a, exit: b});
      }
      intervals.sort((p, q) -> p.enter < q.enter ? -1 : p.enter > q.enter ? 1 : 0);
      var merged:Array<{enter:Float, exit:Float}> = [];
      for (interval in intervals) {
        var last = merged.length == 0 ? null : merged[merged.length - 1];
        if (last != null && interval.enter <= last.exit + 1e-12)
          last.exit = Math.max(last.exit, interval.exit);
        else
          merged.push({enter: interval.enter, exit: interval.exit});
      }
      inside.close();
      probe.close();
      return merged;
    } catch (error:Dynamic) {
      if (inside != null) inside.close();
      probe.close();
      throw error;
    }
  }

  static function horizontalLine(profile:CutterProfile, start:Point3,
      end:Point3):Part {
    var a = new Vector(start.x, start.y, start.z);
    var b = new Vector(end.x, end.y, end.z);
    var along = b.subtract(a);
    var side = new Vector(-along.y, along.x, 0).normalized();
    var parts:Array<Part> = [];
    try {
      // Turning `side` by +90 degrees about +Z points backwards along the move.
      parts.push(halfTool(profile, a, side, true));
      parts.push(halfTool(profile, b, side, false));
      var section = sectionFace(profile, a, side, 0.0, false);
      try {
        parts.push(Part.fromOperation(section.shape.extrudeOperation(along.native())));
      } catch (error:Dynamic) {
        section.close();
        throw error;
      }
      section.close();
      return fuse(parts);
    } catch (error:Dynamic) {
      for (part in parts) part.close();
      throw error;
    }
  }

  /**
    Half of the tool, on one side of the vertical plane through its axis and
    `side`: the half reached by turning `side` counter-clockwise about +Z, or
    clockwise. End caps built this way meet the swept body on a shared planar
    face, which OCCT fuses reliably; whole tools would touch a revolved body
    only tangentially, which it does not.
  **/
  static function halfTool(profile:CutterProfile, tip:Vector, side:Vector,
      counterClockwise:Bool):Part {
    var section = sectionFace(profile, tip, side, 0.0, true);
    try {
      var result = section.revolve(new Axis(tip,
        counterClockwise ? Vector.Z() : Vector.Z().scale(-1)), Math.PI);
      section.close();
      return result;
    } catch (error:Dynamic) {
      section.close();
      throw error;
    }
  }

  /**
    With a radius that never shrinks upwards, the widest slice at any height is
    the one from the lowest tip position; above that tool's top, the top radius
    continues to the highest position's top.
  **/
  static function verticalLine(profile:CutterProfile, lower:Point3,
      rise:Float):Part {
    var parts:Array<Part> = [];
    try {
      parts.push(toolSolid(profile, lower));
      var top = lower.z + profile.height();
      parts.push(Part.cylinderSpan(profile.topRadius(), top, top + rise,
        lower.x, lower.y));
      return fuse(parts);
    } catch (error:Dynamic) {
      for (part in parts) part.close();
      throw error;
    }
  }

  static function arc(profile:CutterProfile, center:Point3, radius:Float,
      startAngle:Float, sweep:Float):Part {
    var maxRadius = 0.0;
    for (segment in profile.segments)
      maxRadius = Math.max(maxRadius, CutterProfile.endOf(segment).r);
    if (radius < maxRadius - 1e-12)
      throw "exact oracle needs an arc radius at least the tool radius";
    if (Math.abs(sweep) <= 1e-12) throw "exact oracle needs a non-zero arc sweep";
    // If a curved part of the tool reaches the arc's axis, the section meets
    // the axis tangentially and revolves into a horn torus, which OCCT builds
    // invalid at some positions. Straight segments meet the axis at a corner
    // and revolve reliably.
    for (segment in profile.segments) switch segment {
      case Arc(_, _, r0, _, r1, _, _):
        if (Math.max(r0, r1) >= radius - 1e-12)
          throw "exact oracle cannot sweep an arc whose radius a curved tool edge reaches";
      case Line(_, _, _, _, _):
    }
    // Each half-tool cap reaches asin(R / radius) past its end of the arc. If
    // the caps reach round into the swept body, the closed-form volume check no
    // longer holds, so such arcs are rejected rather than left unchecked.
    var capAngle = Math.asin(Math.min(1.0, maxRadius / radius));
    if (Math.abs(sweep) < 2 * Math.PI - 1e-12
        && Math.abs(sweep) > 2 * Math.PI - 2 * capAngle)
      throw "exact oracle cannot check an arc whose end caps overlap its own sweep";
    var c = new Vector(center.x, center.y, center.z);
    var u = new Vector(Math.cos(startAngle), Math.sin(startAngle), 0);
    var parts:Array<Part> = [];
    try {
      var section = sectionFace(profile, c, u, radius, false);
      try {
        var full = Math.abs(sweep) >= 2 * Math.PI - 1e-12;
        parts.push(section.revolve(new Axis(c, sweep > 0 ? Vector.Z() : Vector.Z().scale(-1)),
          full ? 2 * Math.PI : Math.abs(sweep)));
      } catch (error:Dynamic) {
        section.close();
        throw error;
      }
      section.close();
      if (Math.abs(sweep) < 2 * Math.PI - 1e-12) {
        // Radial directions at the ends; the caps lie outside the swept angle.
        var endAngle = startAngle + sweep;
        var v = new Vector(Math.cos(endAngle), Math.sin(endAngle), 0);
        parts.push(halfTool(profile, c.add(u.scale(radius)), u, sweep < 0));
        parts.push(halfTool(profile, c.add(v.scale(radius)), v, sweep > 0));
      }
      return fuse(parts);
    } catch (error:Dynamic) {
      for (part in parts) part.close();
      throw error;
    }
  }

  /**
    The tool's cross-section in the vertical plane through `origin` spanned by
    `side` and +Z: material between `offset - r(z)` and `offset + r(z)` along
    `side`. With `halfOnly` the inner side is the tool axis itself, which is
    what revolving the tool needs.
  **/
  static function sectionFace(profile:CutterProfile, origin:Vector,
      side:Vector, offset:Float, halfOnly:Bool):Sketch {
    var up = Vector.Z();
    function at(s:Float, z:Float):Vector
      return origin.add(side.scale(offset + s)).add(up.scale(z));
    var curves:Array<Curve> = [];
    function add(curve:Curve):Void curves.push(curve);
    function segmentCurve(segment:toolpathkit.tool.CutterSegment, sign:Float,
        reversed:Bool):Void {
      switch segment {
        case Line(r0, z0, r1, z1, _):
          if (Math.abs(r1 - r0) > 1e-15 || Math.abs(z1 - z0) > 1e-15)
            add(reversed ? Curve.line(at(sign * r1, z1), at(sign * r0, z0))
              : Curve.line(at(sign * r0, z0), at(sign * r1, z1)));
        case Arc(cr, cz, r0, z0, r1, z1, _):
          var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
          var mr = (r0 + r1) / 2 - cr, mz = (z0 + z1) / 2 - cz;
          var m = Math.sqrt(mr * mr + mz * mz);
          var midR = cr + radius * mr / m, midZ = cz + radius * mz / m;
          add(reversed
            ? Curve.arc(at(sign * r1, z1), at(sign * midR, midZ), at(sign * r0, z0))
            : Curve.arc(at(sign * r0, z0), at(sign * midR, midZ), at(sign * r1, z1)));
      }
    }
    try {
      for (segment in profile.segments) segmentCurve(segment, 1.0, false);
      var height = profile.height(), top = profile.topRadius();
      if (halfOnly) {
        add(Curve.line(at(top, height), at(0, height)));
        add(Curve.line(at(0, height), at(0, 0)));
      } else {
        add(Curve.line(at(top, height), at(-top, height)));
        var index = profile.segments.length;
        while (--index >= 0) segmentCurve(profile.segments[index], -1.0, true);
      }
      var wire = Curve.wire(curves);
      for (curve in curves) curve.close();
      curves = [];
      try {
        var face = Sketch.face(wire, [], new Plane(origin, side,
          side.cross(up).normalized()));
        wire.close();
        return face;
      } catch (error:Dynamic) {
        wire.close();
        throw error;
      }
    } catch (error:Dynamic) {
      for (curve in curves) curve.close();
      throw error;
    }
  }

  static function fuse(parts:Array<Part>):Part {
    var result = Part.fuseAll(parts);
    for (part in parts) part.close();
    if (!result.valid() || result.solidCount() != 1) {
      result.close();
      throw "exact oracle produced an invalid swept solid";
    }
    return result;
  }
}
