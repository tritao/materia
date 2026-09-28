package cnckit;

import cnckit.CncDiagnostic.CncSeverity;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ArcPlane;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;

/** Bounded line/arc cutter-radius offsets with explicit entry and exit moves. */
class CncCompensator {
  public static function resolve(ops:Array<ToolpathOp>):CncCompensationResult {
    var output:Array<ToolpathOp> = [], diagnostics:Array<CncDiagnostic> = [];
    var index = 0;
    while (index < ops.length) {
      switch ops[index] {
        case CutterCompStart(side, radius, plane, span):
          var end = index + 1;
          while (end < ops.length && !switch ops[end] {
            case CutterCompEnd(_): true;
            case _: false;
          }) end++;
          var after = end + 1;
          while (after < ops.length && geometry(ops[after]) == null &&
              !switch ops[after] {
                case End(_), CutterCompStart(_, _, _, _), CutterCompEnd(_): true;
                case _: false;
              }) after++;
          try {
            if (end >= ops.length) fail(span, "G41/G42 requires G40 and lead-out");
            if (after >= ops.length) fail(span, "G40 requires a lead-out line");
            var section = compensate(ops.slice(index + 1, end), ops[after],
              side, radius, plane, span);
            var insertAt = section.length - 1;
            for (cursor in (end + 1)...after)
              section.insert(insertAt++, ops[cursor]);
            for (op in section) output.push(op);
          } catch (error:CncDiagnostic) {
            diagnostics.push(error);
            for (cursor in (index + 1)...Std.int(Math.min(end, ops.length)))
              output.push(ops[cursor]);
            for (cursor in (end + 1)...Std.int(Math.min(after, ops.length)))
              output.push(ops[cursor]);
            if (after < ops.length) output.push(ops[after]);
          }
          index = Std.int(Math.min(ops.length, after + 1));
        case CutterCompEnd(span):
          diagnostics.push(new CncDiagnostic(Error, "CNC_COMP", span,
            "G40 has no active cutter compensation"));
          index++;
        case other:
          output.push(other); index++;
      }
    }
    return new CncCompensationResult(output, diagnostics);
  }

  static function compensate(section:Array<ToolpathOp>, exit:ToolpathOp, side:Int,
      radius:Float, plane:ArcPlane, span:Provenance):Array<ToolpathOp> {
    var positions:Array<Int> = [];
    for (cursor in 0...section.length)
      if (geometry(section[cursor]) != null) positions.push(cursor);
    if (positions.length < 2) fail(span,
      "cutter compensation needs a lead-in and contour");
    var entry = section[positions[0]], lead = geometry(entry);
    if (lead == null || !isLine(lead)) fail(span,
      "G41/G42 lead-in must be a line");
    if (GeometryTools.length(lead) < radius - 1e-10)
      fail(opSpan(entry), "cutter compensation lead-in is shorter than tool radius");
    var exitGeometry = geometry(exit);
    if (exitGeometry == null || !isLine(exitGeometry))
      fail(opSpan(exit), "G40 requires a linear lead-out");
    if (GeometryTools.length(exitGeometry) < 2.0 * radius - 1e-10)
      fail(opSpan(exit), "G40 lead-out is shorter than tool diameter");
    var contours:Array<ToolpathOp> = [for (cursor in 1...positions.length)
      section[positions[cursor]]];
    var shifted:Array<PathGeometry> = [];
    for (op in contours) {
      var g = geometry(op);
      if (g == null) fail(opSpan(op),
        "cutter compensation contour cannot contain a barrier");
      shifted.push(offset(g, side, radius, plane, opSpan(op)));
    }
    var output:Array<ToolpathOp> = section.slice(0, positions[0]);
    var leadStart = GeometryTools.pointAt(lead, 0.0);
    var first = GeometryTools.pointAt(shifted[0], 0.0);
    output.push(replaceGeometry(entry, PathGeometry.Line(leadStart, first)));
    for (cursor in (positions[0] + 1)...positions[1])
      output.push(section[cursor]);
    var handed = side * (plane == XZ ? -1 : 1);
    for (cursor in 0...shifted.length) {
      if (cursor + 1 < shifted.length) {
        var corner = GeometryTools.pointAt(geometry(contours[cursor]),
          GeometryTools.length(geometry(contours[cursor])));
        var current = shifted[cursor], next = shifted[cursor + 1];
        var endPoint = GeometryTools.pointAt(current,
          GeometryTools.length(current));
        var nextStart = GeometryTools.pointAt(next, 0.0);
        if (endPoint.distanceTo(nextStart) > 1e-9) {
          var turn = cross(tangent(geometry(contours[cursor]), true, plane),
            tangent(geometry(contours[cursor + 1]), false, plane));
          if (Math.abs(turn) < 1e-9) fail(opSpan(contours[cursor + 1]),
            "cutter compensation has an ambiguous tangent junction");
          if (handed * turn > 0.0) {
            var meeting = intersection(current, next, corner, plane);
            if (meeting == null) fail(opSpan(contours[cursor + 1]),
              "cutter compensation gouge at inside corner");
            shifted[cursor] = clip(current, false, meeting, plane,
              opSpan(contours[cursor]));
            shifted[cursor + 1] = clip(next, true, meeting, plane,
              opSpan(contours[cursor + 1]));
          } else {
            var join = roundJoin(corner, endPoint, nextStart, plane,
              turn < 0.0 ? -1 : 1, radius);
            output.push(replaceGeometry(contours[cursor], shifted[cursor]));
            output.push(replaceGeometry(contours[cursor], join));
            appendGap(output, section, positions, cursor + 1);
            continue;
          }
        }
      }
      output.push(replaceGeometry(contours[cursor], shifted[cursor]));
      appendGap(output, section, positions, cursor + 1);
    }
    var finalGeometry = shifted[shifted.length - 1];
    var finalPoint = GeometryTools.pointAt(finalGeometry,
      GeometryTools.length(finalGeometry));
    var exitEnd = GeometryTools.pointAt(exitGeometry,
      GeometryTools.length(exitGeometry));
    output.push(replaceGeometry(exit, PathGeometry.Line(finalPoint, exitEnd)));
    return output;
  }

  static function appendGap(output:Array<ToolpathOp>, section:Array<ToolpathOp>,
      positions:Array<Int>, motionIndex:Int):Void {
    var from = positions[motionIndex] + 1;
    var to = motionIndex + 1 < positions.length ?
      positions[motionIndex + 1] : section.length;
    for (cursor in from...to) output.push(section[cursor]);
  }

  static function offset(g:PathGeometry, side:Int, radius:Float,
      plane:ArcPlane, span:Provenance):PathGeometry {
    var handed = side * (plane == XZ ? -1 : 1);
    return switch g {
      case Line(start, end):
        var a = coords(start, plane), b = coords(end, plane);
        if (Math.abs(a[2] - b[2]) > 1e-9) fail(span,
          "cutter compensation requires motion in its active plane");
        var dx = b[0] - a[0], dy = b[1] - a[1];
        var length = Math.sqrt(dx * dx + dy * dy);
        if (length <= 1e-12) fail(span,
          "cutter compensation needs nonzero planar motion");
        var du = -handed * dy * radius / length;
        var dv = handed * dx * radius / length;
        PathGeometry.Line(point(a[0] + du, a[1] + dv, a[2], plane),
          point(b[0] + du, b[1] + dv, b[2], plane));
      case Arc(center, r, angle, sweep):
        if (plane != XY) fail(span, "cutter plane does not match arc plane");
        var shifted = r - handed * (sweep > 0.0 ? 1 : -1) * radius;
        if (shifted <= 1e-12) fail(span, "cutter compensation gouges arc radius");
        PathGeometry.Arc(center, shifted, angle, sweep);
      case Circular(center, r, angle, sweep, arcPlane, rise):
        if (arcPlane != plane || Math.abs(rise) > 1e-9)
          fail(span, "cutter compensation requires arcs in its active plane");
        var shifted = r - handed * (sweep > 0.0 ? 1 : -1) * radius;
        if (shifted <= 1e-12) fail(span, "cutter compensation gouges arc radius");
        PathGeometry.Circular(center, shifted, angle, sweep, plane, 0.0);
    };
  }

  static function intersection(a:PathGeometry, b:PathGeometry,
      corner:Point3, plane:ArcPlane):Null<Point3> {
    var candidates:Array<Point3> = [];
    var aLine = isLine(a), bLine = isLine(b);
    if (aLine && bLine) {
      var p = coords(GeometryTools.pointAt(a, 0.0), plane);
      var q = coords(GeometryTools.pointAt(b, 0.0), plane);
      var da = tangent(a, false, plane), db = tangent(b, false, plane);
      var den = cross(da, db);
      if (Math.abs(den) > 1e-12) {
        var t = cross([q[0] - p[0], q[1] - p[1]], db) / den;
        candidates.push(point(p[0] + da[0] * t, p[1] + da[1] * t, p[2], plane));
      }
    } else if (aLine != bLine) {
      var line = aLine ? a : b, arc = aLine ? b : a;
      var p = coords(GeometryTools.pointAt(line, 0.0), plane);
      var d = tangent(line, false, plane), c = arcCenter(arc, plane);
      var radius = arcRadius(arc);
      var fx = p[0] - c[0], fy = p[1] - c[1];
      var dot = fx * d[0] + fy * d[1];
      var discriminant = dot * dot - (fx * fx + fy * fy - radius * radius);
      if (discriminant >= -1e-12) {
        var root = Math.sqrt(Math.max(0.0, discriminant));
        for (t in [-dot - root, -dot + root])
          candidates.push(point(p[0] + d[0] * t,
            p[1] + d[1] * t, p[2], plane));
      }
    } else {
      var ca = arcCenter(a, plane), cb = arcCenter(b, plane);
      var dx = cb[0] - ca[0], dy = cb[1] - ca[1];
      var d = Math.sqrt(dx * dx + dy * dy), ra = arcRadius(a), rb = arcRadius(b);
      if (d > 1e-12 && d <= ra + rb + 1e-10 && d >= Math.abs(ra - rb) - 1e-10) {
        var along = (ra * ra - rb * rb + d * d) / (2.0 * d);
        var height = Math.sqrt(Math.max(0.0, ra * ra - along * along));
        var mx = ca[0] + along * dx / d, my = ca[1] + along * dy / d;
        candidates.push(point(mx - height * dy / d,
          my + height * dx / d, ca[2], plane));
        candidates.push(point(mx + height * dy / d,
          my - height * dx / d, ca[2], plane));
      }
    }
    var best:Null<Point3> = null, score = Math.POSITIVE_INFINITY;
    for (candidate in candidates) {
      if (!contains(a, candidate, plane) || !contains(b, candidate, plane)) continue;
      var distance = candidate.distanceTo(corner);
      if (distance < score) { best = candidate; score = distance; }
    }
    return best;
  }

  static function contains(g:PathGeometry, p:Point3, plane:ArcPlane):Bool {
    if (isLine(g)) {
      var start = coords(GeometryTools.pointAt(g, 0.0), plane);
      var end = coords(GeometryTools.pointAt(g, GeometryTools.length(g)), plane);
      var value = coords(p, plane), dx = end[0] - start[0], dy = end[1] - start[1];
      var length2 = dx * dx + dy * dy;
      var t = ((value[0] - start[0]) * dx + (value[1] - start[1]) * dy) / length2;
      return t >= -1e-8 && t <= 1.0 + 1e-8;
    }
    var c = arcCenter(g, plane), value = coords(p, plane);
    var angle = Math.atan2(value[1] - c[1], value[0] - c[0]);
    var start = arcStart(g), sweep = arcSweep(g);
    for (shift in -2...3) {
      var fraction = (angle + 2.0 * Math.PI * shift - start) / sweep;
      if (fraction >= -1e-8 && fraction <= 1.0 + 1e-8) return true;
    }
    return false;
  }

  static function clip(g:PathGeometry, atStart:Bool, p:Point3,
      plane:ArcPlane, span:Provenance):PathGeometry {
    if (isLine(g)) {
      var start = GeometryTools.pointAt(g, 0.0);
      var end = GeometryTools.pointAt(g, GeometryTools.length(g));
      var result = atStart ? PathGeometry.Line(p, end) : PathGeometry.Line(start, p);
      if (GeometryTools.length(result) <= 1e-10) fail(span,
        "cutter compensation gouges a line segment");
      return result;
    }
    var c = arcCenter(g, plane), value = coords(p, plane);
    var angle = Math.atan2(value[1] - c[1], value[0] - c[0]);
    var start = arcStart(g), sweep = arcSweep(g), fraction = Math.NaN;
    for (shift in -2...3) {
      var candidate = (angle + 2.0 * Math.PI * shift - start) / sweep;
      if (candidate >= -1e-8 && candidate <= 1.0 + 1e-8 &&
          (!Math.isFinite(fraction) ||
            Math.abs(candidate - (atStart ? 0.0 : 1.0)) <
            Math.abs(fraction - (atStart ? 0.0 : 1.0)))) fraction = candidate;
    }
    if (!Math.isFinite(fraction) || fraction <= 1e-8 && !atStart ||
        fraction >= 1.0 - 1e-8 && atStart)
      fail(span, "cutter compensation gouges an arc segment");
    var newStart = atStart ? start + sweep * fraction : start;
    var newSweep = atStart ? sweep * (1.0 - fraction) : sweep * fraction;
    return switch g {
      case Arc(center, radius, _, _): PathGeometry.Arc(center, radius, newStart, newSweep);
      case Circular(center, radius, _, _, arcPlane, _):
        PathGeometry.Circular(center, radius, newStart, newSweep, arcPlane, 0.0);
      case _: throw "line already handled";
    };
  }

  static function roundJoin(corner:Point3, from:Point3, to:Point3,
      plane:ArcPlane, direction:Int, radius:Float):PathGeometry {
    var c = coords(corner, plane), a = coords(from, plane), b = coords(to, plane);
    var start = Math.atan2(a[1] - c[1], a[0] - c[0]);
    var end = Math.atan2(b[1] - c[1], b[0] - c[0]);
    var sweep = end - start;
    if (direction > 0) while (sweep <= 0.0) sweep += 2.0 * Math.PI;
    else while (sweep >= 0.0) sweep -= 2.0 * Math.PI;
    return plane == XY ? PathGeometry.Arc(corner, radius, start, sweep) :
      PathGeometry.Circular(corner, radius, start, sweep, plane, 0.0);
  }

  static function tangent(g:PathGeometry, atEnd:Bool, plane:ArcPlane):Array<Float> {
    if (isLine(g)) {
      var a = coords(GeometryTools.pointAt(g, 0.0), plane);
      var b = coords(GeometryTools.pointAt(g, GeometryTools.length(g)), plane);
      var dx = b[0] - a[0], dy = b[1] - a[1];
      var length = Math.sqrt(dx * dx + dy * dy);
      return [dx / length, dy / length];
    }
    var angle = arcStart(g) + (atEnd ? arcSweep(g) : 0.0);
    var sign = arcSweep(g) > 0.0 ? 1.0 : -1.0;
    return [-Math.sin(angle) * sign, Math.cos(angle) * sign];
  }

  static function arcCenter(g:PathGeometry, plane:ArcPlane):Array<Float>
    return switch g {
      case Arc(center, _, _, _), Circular(center, _, _, _, _, _): coords(center, plane);
      case _: throw "not an arc";
    };
  static function arcRadius(g:PathGeometry):Float return switch g {
    case Arc(_, radius, _, _), Circular(_, radius, _, _, _, _): radius;
    case _: throw "not an arc";
  };
  static function arcStart(g:PathGeometry):Float return switch g {
    case Arc(_, _, start, _), Circular(_, _, start, _, _, _): start;
    case _: throw "not an arc";
  };
  static function arcSweep(g:PathGeometry):Float return switch g {
    case Arc(_, _, _, sweep), Circular(_, _, _, sweep, _, _): sweep;
    case _: throw "not an arc";
  };
  static function isLine(g:PathGeometry):Bool return switch g {
    case Line(_, _): true;
    case _: false;
  };
  static function geometry(op:ToolpathOp):Null<PathGeometry> return switch op {
    case Rapid(g, _), Feed(g, _, _, _): g;
    case _: null;
  };
  static function opSpan(op:ToolpathOp):Provenance return switch op {
    case Rapid(_, span), Feed(_, _, _, span): span;
    case CutterCompStart(_, _, _, span), CutterCompEnd(span): span;
    case Dwell(_, span), Spindle(_, _, span), Coolant(_, _, span),
        ToolChange(_, span), ToolLengthOffset(_, _, span), OptionalStop(span),
        ProgramStop(span), End(span): span;
  };
  static function replaceGeometry(op:ToolpathOp, geometry:PathGeometry):ToolpathOp
    return switch op {
      case Rapid(_, span): ToolpathOp.Rapid(geometry, span);
      case Feed(_, speed, blend, span): ToolpathOp.Feed(geometry, speed, blend, span);
      case _: throw "cutter compensation needs motion";
    };
  static function coords(p:Point3, plane:ArcPlane):Array<Float> return switch plane {
    case XY: [p.x, p.y, p.z];
    case XZ: [p.x, p.z, p.y];
    case YZ: [p.y, p.z, p.x];
  };
  static function point(u:Float, v:Float, axial:Float, plane:ArcPlane):Point3
    return switch plane {
      case XY: new Point3(u, v, axial);
      case XZ: new Point3(u, axial, v);
      case YZ: new Point3(axial, u, v);
    };
  static function cross(a:Array<Float>, b:Array<Float>):Float
    return a[0] * b[1] - a[1] * b[0];
  static function fail(span:Provenance, message:String):Void
    throw new CncDiagnostic(Error, "CNC_COMP", span, message);
}

class CncCompensationResult {
  public final ops:Array<ToolpathOp>;
  public final diagnostics:Array<CncDiagnostic>;
  public function new(ops:Array<ToolpathOp>, diagnostics:Array<CncDiagnostic>) {
    this.ops = ops;
    this.diagnostics = diagnostics;
  }
}
