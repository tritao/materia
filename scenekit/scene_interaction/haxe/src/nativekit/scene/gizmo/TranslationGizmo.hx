package nativekit.scene.gizmo;

import nativekit.scene.gizmo.TranslationHandle;

private typedef WorldPrimitive = {final handle:TranslationHandle; final vertices:Array<GizmoVec3>;}

/** Immutable translation layout and synchronous handle picker. No scene or UI ownership. */
class TranslationGizmo {
  public final camera:GizmoCamera;
  public final basis:GizmoBasis;
  public final pivot:GizmoVec3;
  public final worldSize:Float;
  public final hitRadius:Float;
  final geometry:Array<TranslationShape> = [];
  final worldGeometry:Array<WorldPrimitive> = [];
  public function new(camera:GizmoCamera, pivot:GizmoVec3, pixels:Float = 90, hitRadius:Float = 8, basis:Null<GizmoBasis> = null) {
    if (camera == null || pivot == null || !Math.isFinite(pixels) || pixels <= 0 || pixels > 512 || !Math.isFinite(hitRadius) || hitRadius <= 0 || hitRadius > 64)
      throw "Invalid translation gizmo layout";
    this.basis = basis == null ? GizmoBasis.world() : basis;
    this.camera = camera; this.pivot = pivot; this.hitRadius = hitRadius;
    var projected = camera.project(pivot); worldSize = projected == null ? 0 : camera.worldLength(pivot, pixels);
    if (projected == null || !camera.contains(projected.x, projected.y) || !Math.isFinite(worldSize) || worldSize <= 0) return;
    var eyeDirection = camera.eye.subtract(pivot).normalized();
    for (handle in [AxisX, AxisY, AxisZ]) {
      var direction = this.basis.axis(handle);
      if (Math.abs(direction.dot(eyeDirection)) > 0.99) continue;
      var vertices = [pivot.add(direction.scaled(worldSize*0.16)), pivot.add(direction.scaled(worldSize))];
      var points = project(vertices);
      if (points == null) continue;
      var dx = points[1].x-points[0].x, dy = points[1].y-points[0].y;
      if (dx*dx+dy*dy < 144) continue;
      geometry.push(new TranslationShape(handle, label(handle), points));
      worldGeometry.push({handle:handle, vertices:vertices});
    }
    for (handle in [PlaneXY, PlaneXZ, PlaneYZ]) {
      var axes = this.basis.planeAxes(handle), normal = axes[0].cross(axes[1]);
      if (Math.abs(normal.dot(eyeDirection)) < 0.2) continue;
      var vertices = [for (pair in [[0.22,0.22],[0.40,0.22],[0.40,0.40],[0.22,0.40]])
        pivot.add(axes[0].scaled(pair[0]*worldSize)).add(axes[1].scaled(pair[1]*worldSize))];
      var points = project(vertices);
      if (points == null) continue;
      var area = 0.0;
      for (index in 0...4) { var a = points[index], b = points[(index+1)%4]; area += a.x*b.y-a.y*b.x; }
      if (Math.abs(area)*0.5 < 36) continue;
      geometry.push(new TranslationShape(handle, label(handle), points));
      worldGeometry.push({handle:handle, vertices:vertices});
    }
  }
  function project(values:Array<GizmoVec3>):Null<Array<GizmoPoint>> {
    var out:Array<GizmoPoint> = [];
    for (value in values) { var point = camera.project(value); if (point == null) return null; out.push(point); }
    return out;
  }
  public function shapes():Array<TranslationShape> return geometry.copy();
  /** Follow a proposal while retaining pointer-down size, visible handles and camera. */
  public function translatedShapes(delta:GizmoVec3):Array<TranslationShape> {
    if (delta == null) throw "Missing translation displacement";
    var result:Array<TranslationShape> = [];
    for (primitive in worldGeometry) {
      var points = project(primitive.vertices.map(vertex -> vertex.add(delta)));
      if (points != null) result.push(new TranslationShape(primitive.handle, label(primitive.handle), points));
    }
    return result;
  }
  public function pick(x:Float, y:Float):Null<TranslationHandle> {
    if (!camera.contains(x, y)) return null;
    var best:Null<TranslationHandle> = null, bestDistance = 1e300, bestDepth = 1e300;
    // Stable enum order resolves exact ties after distance and depth.
    for (shape in geometry) {
      var distance = shape.distance(x, y); if (distance == null || distance > hitRadius) continue;
      var depth = shape.depth();
      if (distance < bestDistance || (distance == bestDistance && depth < bestDepth)) {
        best = shape.handle; bestDistance = distance; bestDepth = depth;
      }
    }
    return best;
  }
  public function begin(handle:TranslationHandle, x:Float, y:Float, snap:Float = 0):TranslationDrag {
    if (handle == null || pick(x, y) != handle) throw "Translation drag must begin on a visible picked handle";
    return new TranslationDrag(camera, pivot, handle, x, y, snap, basis);
  }
  public static function axis(handle:TranslationHandle):GizmoVec3 return switch handle {
    case AxisX: new GizmoVec3(1,0,0); case AxisY: new GizmoVec3(0,1,0); case AxisZ: new GizmoVec3(0,0,1);
    default: throw "Expected translation axis";
  };
  public static function planeAxes(handle:TranslationHandle):Array<GizmoVec3> return switch handle {
    case PlaneXY: [axis(AxisX), axis(AxisY)]; case PlaneXZ: [axis(AxisX), axis(AxisZ)]; case PlaneYZ: [axis(AxisY), axis(AxisZ)];
    default: throw "Expected translation plane";
  };
  public static function label(handle:TranslationHandle):String return switch handle {
    case AxisX: "X"; case AxisY: "Y"; case AxisZ: "Z"; case PlaneXY: "XY"; case PlaneXZ: "XZ"; case PlaneYZ: "YZ";
  };
}
