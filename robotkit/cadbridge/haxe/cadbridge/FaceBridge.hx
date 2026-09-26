package cadbridge;

import CadKit;
import cadkit.Face;
import cadkit.Shape;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.work.Point2;
import robotkit.work.Polygon2;
import robotkit.work.Provenance;
import robotkit.work.WorkSurface;
import robotkit.work.WorkSurfaceId;

/**
 * Converts a CadKit planar Face into a design WorkSurface: its outer wire
 * becomes the boundary, its inner wires become exclusions, all expressed
 * in a local plane frame built from the face's own center and normal
 * (`frame_T_surface`'s rotation columns are `basisU, basisV, normal`, so
 * the surface's local +Z is the face's outward normal, per WorkSurface's
 * contract). `scale` converts the shape's own linear units into meters
 * (CadKit itself is unit-agnostic; BimKit's convention is millimeters, so
 * `WallBridge` passes `scale = 0.001`).
 *
 * `cadkit.Face` (haxe/src/cadkit/Face.hx) does not expose wire/vertex
 * enumeration, but `Face.cloneShape()` already returns a full `Shape` over
 * just that face, and `Shape.subshapeCount`/`subshape` already walk any
 * `CadKit.ShapeKind`, including `Wire` and `Vertex` — no CadKit change was
 * needed to read a face's boundary loops.
 */
class FaceBridge {
  public static function toWorkSurface(face:Face, id:WorkSurfaceId, frameId:String,
      ?provenance:Provenance, ?surfaceFrameId:String, ?scale:Float = 1.0):WorkSurface {
    if (face == null) throw "Face bridge requires a face";
    if (face.surfaceKind() != CadKit.SurfaceKind.Plane) throw "Face bridge only supports planar faces";
    if (!Math.isFinite(scale) || scale <= 0.0) throw "Face bridge scale must be positive and finite";

    var center = toVec3(face.center(), scale);
    var normal = toVec3(face.normal(), 1.0).normalized();
    var up = Math.abs(normal.z) < 0.9 ? new Vec3(0.0, 0.0, 1.0) : new Vec3(1.0, 0.0, 0.0);
    var basisU = normal.cross(up).normalized();
    var basisV = normal.cross(basisU).normalized();

    var faceShape = face.cloneShape();
    var wireCount = faceShape.subshapeCount(CadKit.ShapeKind.Wire);
    if (wireCount == 0) {
      faceShape.close();
      throw "Planar face has no boundary wires";
    }

    var polygons:Array<Polygon2> = [];
    var areas:Array<Float> = [];
    for (i in 0...wireCount) {
      var wireShape = faceShape.subshape(CadKit.ShapeKind.Wire, i);
      var points = wirePoints(wireShape, center, basisU, basisV, scale);
      wireShape.close();
      var area = shoelaceArea(points);
      if (area < 0.0) {
        points.reverse();
        area = -area;
      }
      polygons.push(new Polygon2(points));
      areas.push(area);
    }
    faceShape.close();

    var outerIndex = 0;
    for (i in 1...polygons.length) if (areas[i] > areas[outerIndex]) outerIndex = i;
    var exclusions:Array<Polygon2> = [];
    for (i in 0...polygons.length) if (i != outerIndex) exclusions.push(polygons[i]);

    var rotation = Quat.fromRotationMatrix([
      basisU.x, basisU.y, basisU.z,
      basisV.x, basisV.y, basisV.z,
      normal.x, normal.y, normal.z
    ]);
    var frame_T_surface = new Transform3(center, rotation);

    return new WorkSurface(id, frameId, frame_T_surface, polygons[outerIndex], exclusions,
      0.002, "", provenance, surfaceFrameId);
  }

  static function wirePoints(wireShape:Shape, center:Vec3, basisU:Vec3, basisV:Vec3, scale:Float):Array<Point2> {
    var edgeCount = wireShape.subshapeCount(CadKit.ShapeKind.Edge);
    if (edgeCount < 3) throw "Planar face wire must contain at least three edges";
    var starts:Array<Vec3> = [];
    var ends:Array<Vec3> = [];
    for (i in 0...edgeCount) {
      var edgeShape = wireShape.subshape(CadKit.ShapeKind.Edge, i);
      try {
        if (edgeShape.subshapeCount(CadKit.ShapeKind.Vertex) != 2)
          throw "Face bridge requires two endpoints for every wire edge";
        var startShape = edgeShape.subshape(CadKit.ShapeKind.Vertex, 0);
        var endShape = edgeShape.subshape(CadKit.ShapeKind.Vertex, 1);
        starts.push(toVec3(startShape.position(), scale));
        ends.push(toVec3(endShape.position(), scale));
        startShape.close();
        endShape.close();
      } catch (error:Dynamic) {
        edgeShape.close();
        throw error;
      }
      edgeShape.close();
    }

    var tolerance = 1e-7 * Math.max(1.0, scale);
    var first = starts[0];
    var current = ends[0];
    var used = [for (_ in 0...edgeCount) false];
    used[0] = true;
    var ordered:Array<Vec3> = [first];
    for (_ in 1...edgeCount) {
      var nextIndex = -1;
      var next:Null<Vec3> = null;
      for (candidate in 0...edgeCount) if (!used[candidate]) {
        if (samePoint(current, starts[candidate], tolerance)) {
          nextIndex = candidate;
          next = ends[candidate];
          break;
        }
        if (samePoint(current, ends[candidate], tolerance)) {
          nextIndex = candidate;
          next = starts[candidate];
          break;
        }
      }
      if (nextIndex < 0 || next == null)
        throw "Face bridge could not follow a connected wire edge chain";
      ordered.push(current);
      used[nextIndex] = true;
      current = next;
    }
    if (!samePoint(current, first, tolerance))
      throw "Face bridge wire edge chain does not close";

    var points:Array<Point2> = [];
    for (position in ordered) {
      var local = position.sub(center);
      points.push(new Point2(local.dot(basisU), local.dot(basisV)));
    }
    return points;
  }

  static function samePoint(a:Vec3, b:Vec3, tolerance:Float):Bool {
    var delta = a.sub(b);
    return delta.dot(delta) <= tolerance * tolerance;
  }

  static function shoelaceArea(points:Array<Point2>):Float {
    var sum = 0.0;
    for (i in 0...points.length) {
      var a = points[i];
      var b = points[(i + 1) % points.length];
      sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
  }

  static function toVec3(v:CadKit.Vec3, scale:Float):Vec3
    return new Vec3(v.get_x() * scale, v.get_y() * scale, v.get_z() * scale);
}
