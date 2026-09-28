package camkit;

import cadkit.Edge;
import cadkit.Face;
import CadKit;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SolvedSketch;
import cadkit.units.LengthUnits;
import cadkit.modeling.Vector;
import toolpathkit.path.Point3;

/** One simple closed XY boundary, in machine metres. */
class CamContour {
  public final vertices:Array<Point3>;
  public final z:Float;
  public final signedArea:Float;

  public function new(vertices:Array<Point3>) {
    if (vertices == null) throw "CAM contour needs vertices";
    var values = vertices.copy();
    if (values.length > 1 && values[0].distanceTo(values[values.length - 1]) < 1e-9)
      values.pop();
    if (values.length < 3) throw "CAM contour needs at least three distinct vertices";
    z = values[0].z;
    for (i in 0...values.length) {
      var point = values[i], next = values[(i + 1) % values.length];
      if (point == null || !Math.isFinite(point.x) || !Math.isFinite(point.y) ||
          !Math.isFinite(point.z) || Math.abs(point.z - z) > 1e-8 ||
          point.distanceTo(next) < 1e-10)
        throw "CAM contour needs finite, planar, distinct vertices";
    }
    var area = 0.0;
    for (i in 0...values.length) {
      var a = values[i], b = values[(i + 1) % values.length];
      area += a.x * b.y - b.x * a.y;
    }
    signedArea = area * 0.5;
    if (Math.abs(signedArea) < 1e-14) throw "CAM contour has zero area";
    for (i in 0...values.length) for (j in (i + 2)...values.length) {
      if ((j + 1) % values.length == i) continue;
      if (intersects(values[i], values[(i + 1) % values.length],
          values[j], values[(j + 1) % values.length]))
        throw "CAM contour self-intersects";
    }
    this.vertices = values;
  }

  /** Builds a single profile from solved cadkit lines, arcs, or one circle. */
  public static function fromSketch(authored:ConstrainedSketch,
      solved:SolvedSketch):CamContour {
    if (authored == null || solved == null) throw "CAM needs a solved sketch";
    var scale = unitScale(authored.units);
    var segments:Array<Array<Point3>> = [];
    for (entity in authored.entities()) {
      if (entity.construction) continue;
      var sampled:Array<Point3> = [];
      switch entity.kind {
        case "line":
          for (id in [entity.first, entity.second]) {
            var local = solved.point(id);
            sampled.push(sketchPoint(authored, local[0], local[1], scale));
          }
        case "arc", "circle":
          var center = solved.point(entity.first), radius = solved.radius(entity.id);
          var start = entity.kind == "circle" ? 0.0 : entity.startAngle;
          var sweep = entity.kind == "circle" ? 2.0 * Math.PI :
            entity.endAngle - entity.startAngle;
          if (entity.clockwise && sweep > 0.0) sweep -= 2.0 * Math.PI;
          if (!entity.clockwise && sweep < 0.0) sweep += 2.0 * Math.PI;
          var count = Std.int(Math.max(16, Math.ceil(Math.abs(sweep) * 32.0)));
          for (step in 0...(count + 1)) {
            var angle = start + sweep * step / count;
            sampled.push(sketchPoint(authored,
              center[0] + radius * Math.cos(angle),
              center[1] + radius * Math.sin(angle), scale));
          }
        case _: throw 'Unsupported CAM sketch entity "${entity.kind}"';
      }
      segments.push(sampled);
    }
    return connect(segments);
  }

  /** CAD edge positions are sampled adaptively to the requested chord tolerance. */
  public static function fromEdges(edges:Array<Edge>, ?unit:String = "mm",
      ?chordToleranceMetres:Float = 0.00005):CamContour {
    if (edges == null || edges.length == 0 ||
        !Math.isFinite(chordToleranceMetres) || chordToleranceMetres <= 0.0)
      throw "CAM needs edges and a positive chord tolerance";
    var scale = unitScale(unit), segments:Array<Array<Point3>> = [];
    for (edge in edges) {
      if (edge == null) throw "CAM edge must not be null";
      var shape = edge.cloneShape();
      var sampled:Array<Point3> = [];
      try {
        function sample(parameter:Float):Point3 {
          var position = shape.positionAt(parameter);
          return new Point3(position.get_x() * scale,
            position.get_y() * scale, position.get_z() * scale);
        }
        function subdivide(a:Float, b:Float, start:Point3,
            end:Point3, depth:Int):Void {
          var midpoint = sample((a + b) * 0.5);
          var chord = new Point3((start.x + end.x) * 0.5,
            (start.y + end.y) * 0.5, (start.z + end.z) * 0.5);
          if (midpoint.distanceTo(chord) > chordToleranceMetres) {
            if (depth >= 12)
              throw "CAD edge needs more samples than CAM chord budget";
            subdivide(a, (a + b) * 0.5, start, midpoint, depth + 1);
            subdivide((a + b) * 0.5, b, midpoint, end, depth + 1);
          } else sampled.push(end);
        }
        var start = sample(0.0), end = sample(1.0);
        sampled.push(start);
        subdivide(0.0, 1.0, start, end, 0);
      } catch (error:Dynamic) {
        shape.close(); throw error;
      }
      shape.close();
      segments.push(sampled);
    }
    return connect(segments);
  }

  public static function fromFace(face:Face, ?unit:String = "mm",
      ?chordToleranceMetres:Float = 0.00005):CamContour {
    var boundaries = fromFaceBoundaries(face, unit, chordToleranceMetres);
    if (boundaries.length != 1)
      throw "CAD face has multiple boundaries; select an outer or hole contour";
    return boundaries[0];
  }

  /** Returns the outer face contour first, followed by its inner boundaries. */
  public static function fromFaceBoundaries(face:Face, ?unit:String = "mm",
      ?chordToleranceMetres:Float = 0.00005):Array<CamContour> {
    if (face == null) throw "CAM needs a face";
    var shape = face.cloneShape(), result:Array<CamContour> = [];
    try {
      for (wireIndex in 0...shape.subshapeCount(CadKit.ShapeKind.Wire)) {
        var wire = shape.subshape(CadKit.ShapeKind.Wire, wireIndex);
        var edges:Array<Edge> = [];
        try {
          for (edgeIndex in 0...wire.subshapeCount(CadKit.ShapeKind.Edge))
            edges.push(new Edge(wire.subshape(CadKit.ShapeKind.Edge, edgeIndex)));
          result.push(fromEdges(edges, unit, chordToleranceMetres));
        } catch (error:Dynamic) {
          for (edge in edges) edge.close();
          wire.close();
          throw error;
        }
        for (edge in edges) edge.close();
        wire.close();
      }
      if (result.length == 0) throw "CAD face has no boundaries";
      result.sort((a, b) -> {
        var first = Math.abs(a.signedArea), second = Math.abs(b.signedArea);
        return first > second ? -1 : (first < second ? 1 : 0);
      });
      shape.close();
      return result;
    } catch (error:Dynamic) {
      shape.close();
      throw error;
    }
  }

  /** Positive inset moves toward the polygon interior; negative moves outward. */
  public function inset(distance:Float):CamContour {
    if (!Math.isFinite(distance)) throw "CAM offset must be finite";
    if (distance == 0.0) return new CamContour(vertices);
    var orientation = signedArea > 0.0 ? 1.0 : -1.0;
    for (i in 0...vertices.length) {
      var a = vertices[i], b = vertices[(i + 1) % vertices.length];
      var c = vertices[(i + 2) % vertices.length];
      var turn = cross(b.x - a.x, b.y - a.y, c.x - b.x, c.y - b.y);
      if (turn * orientation <= 1e-12)
        throw "CAM offset clearing needs a convex contour";
    }
    var shifted:Array<Point3> = [];
    for (i in 0...vertices.length) {
      var previous = vertices[(i + vertices.length - 1) % vertices.length];
      var current = vertices[i], next = vertices[(i + 1) % vertices.length];
      var ax = current.x - previous.x, ay = current.y - previous.y;
      var bx = next.x - current.x, by = next.y - current.y;
      var al = Math.sqrt(ax * ax + ay * ay), bl = Math.sqrt(bx * bx + by * by);
      var aX = previous.x - orientation * ay / al * distance;
      var aY = previous.y + orientation * ax / al * distance;
      var bX = current.x - orientation * by / bl * distance;
      var bY = current.y + orientation * bx / bl * distance;
      var denominator = cross(ax, ay, bx, by);
      var t = cross(bX - aX, bY - aY, bx, by) / denominator;
      shifted.push(new Point3(aX + ax * t, aY + ay * t, z));
    }
    var result = new CamContour(shifted);
    if (distance > 0.0 && (Math.abs(result.signedArea) >= Math.abs(signedArea) ||
        result.signedArea * signedArea <= 0.0))
      throw "CAM inset exhausted the pocket";
    if (distance > 0.0)
      for (point in shifted) for (i in 0...vertices.length) {
        var a = vertices[i], b = vertices[(i + 1) % vertices.length];
        var dx = b.x - a.x, dy = b.y - a.y;
        var inward = orientation * cross(dx, dy,
          point.x - a.x, point.y - a.y) / Math.sqrt(dx * dx + dy * dy);
        if (inward < distance - 1e-9)
          throw "CAM inset exhausted the pocket";
      }
    return result;
  }

  static function connect(segments:Array<Array<Point3>>):CamContour {
    if (segments.length == 0) throw "CAM contour has no edges";
    var remaining = segments.copy(), points = remaining.shift().copy();
    while (remaining.length > 0) {
      var found = -1, reverse = false;
      var end = points[points.length - 1];
      for (i in 0...remaining.length) {
        var candidate = remaining[i];
        if (end.distanceTo(candidate[0]) < 1e-7) {
          found = i; break;
        }
        if (end.distanceTo(candidate[candidate.length - 1]) < 1e-7) {
          found = i; reverse = true; break;
        }
      }
      if (found < 0) throw "CAD edges do not form one closed contour";
      var segment = remaining.splice(found, 1)[0];
      if (reverse) segment.reverse();
      for (i in 1...segment.length) points.push(segment[i]);
    }
    if (points[0].distanceTo(points[points.length - 1]) > 1e-7)
      throw "CAD profile is open";
    return new CamContour(points);
  }

  static function sketchPoint(authored:ConstrainedSketch, x:Float, y:Float,
      scale:Float):Point3 {
    var world = authored.plane.toWorld(new Vector(x, y, 0.0));
    return new Point3(world.x * scale, world.y * scale, world.z * scale);
  }

  static function unitScale(unit:String):Float {
    var factor = LengthUnits.factorToMillimetres(unit);
    if (factor == null) throw 'Unsupported CAM unit "$unit"';
    return factor * 0.001;
  }

  static function cross(ax:Float, ay:Float, bx:Float, by:Float):Float
    return ax * by - ay * bx;

  static function intersects(a:Point3, b:Point3,
      c:Point3, d:Point3):Bool {
    var abx = b.x - a.x, aby = b.y - a.y;
    var cdx = d.x - c.x, cdy = d.y - c.y;
    var den = cross(abx, aby, cdx, cdy);
    if (Math.abs(den) < 1e-14) return false;
    var t = cross(c.x - a.x, c.y - a.y, cdx, cdy) / den;
    var u = cross(c.x - a.x, c.y - a.y, abx, aby) / den;
    return t > 1e-9 && t < 1.0 - 1e-9 &&
      u > 1e-9 && u < 1.0 - 1e-9;
  }
}
