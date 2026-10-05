package machinekit.welding;

import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;

/** Conservative plane-intersection uncertainty in the CAD model's length unit. */
class WeldProbeRegionBounds {
  public final halfU:Float;
  public final halfV:Float;
  public final normalTravel:Float;
  public final tiltU:Float;
  public final tiltV:Float;
  public function new(halfU:Float, halfV:Float, normalTravel:Float, tiltU:Float, tiltV:Float) {
    this.halfU = halfU; this.halfV = halfV; this.normalTravel = normalTravel;
    this.tiltU = tiltU; this.tiltV = tiltV;
  }
}

/** Parking error rotates about the chassis origin, not about the workpiece or the arm mount. */
class WeldProbeParkingBounds {
  final rootInWork:AssemblyFrame;
  final workInRoot:AssemblyFrame;
  final translation:Vector;
  final yaw:Float;

  public function new(rootInWork:AssemblyFrame, translation:Vector, yaw:Float) {
    if (rootInWork == null || translation == null || translation.x < 0 || translation.y < 0 || translation.z < 0 ||
        !Math.isFinite(yaw) || yaw < 0 || yaw >= Math.PI / 2)
      throw "Probe parking uncertainty needs a nominal chassis frame, nonnegative translation and yaw below 90 degrees";
    var values = [rootInWork.x, rootInWork.y, rootInWork.z, rootInWork.qx, rootInWork.qy, rootInWork.qz, rootInWork.qw];
    for (value in values) if (!Math.isFinite(value)) throw "Probe parking frame must be finite";
    var norm = rootInWork.qx * rootInWork.qx + rootInWork.qy * rootInWork.qy + rootInWork.qz * rootInWork.qz + rootInWork.qw * rootInWork.qw;
    if (Math.abs(norm - 1) > 1e-6) throw "Probe parking frame needs a unit rotation";
    this.rootInWork = rootInWork; workInRoot = AssemblyFrames.inverse(rootInWork);
    this.translation = translation; this.yaw = yaw;
  }

  function rootDirection(direction:Vector):Vector {
    var result = AssemblyFrames.transformVector(workInRoot, direction.x, direction.y, direction.z);
    return new Vector(result.x, result.y, result.z);
  }

  /** Exact extrema of the yaw displacement projected on an arbitrary axis. */
  function yawProjection(point:Vector, axis:Vector):Float {
    var a = axis.x * point.x + axis.y * point.y;
    var b = -axis.x * point.y + axis.y * point.x;
    function magnitude(angle:Float):Float return Math.abs(a * (Math.cos(angle) - 1) + b * Math.sin(angle));
    var bound = Math.max(magnitude(-yaw), magnitude(yaw));
    var critical = Math.atan2(b, a);
    for (turn in -2...3) {
      var angle = critical + turn * Math.PI;
      if (angle >= -yaw && angle <= yaw) bound = Math.max(bound, magnitude(angle));
    }
    return bound;
  }

  function projected(point:Vector, axis:Vector):Float
    return Math.abs(axis.x) * translation.x + Math.abs(axis.y) * translation.y + Math.abs(axis.z) * translation.z + yawProjection(point, axis);

  public function region(face:WeldProbeFace, point:Vector):WeldProbeRegionBounds {
    if (face == null || point == null) throw "Probe uncertainty needs a nominal face and point";
    var local = AssemblyFrames.transformPoint(workInRoot, point.x, point.y, point.z);
    var p = new Vector(local.x, local.y, local.z);
    var normal = rootDirection(face.normal), u = rootDirection(face.u), v = rootDirection(face.v);
    // The tilted search ray meets the nominal plane with at least cos(yaw) normal projection.
    var depth = projected(p, normal) / Math.cos(yaw);
    return new WeldProbeRegionBounds(projected(p, u) + depth * yawProjection(normal, u),
      projected(p, v) + depth * yawProjection(normal, v), depth, yawProjection(normal, u), yawProjection(normal, v));
  }

  public function fits(face:WeldProbeFace, point:Vector, inset:Float = 1):Bool {
    var bounds = region(face, point);
    return face.containsRegion(point, bounds.halfU, bounds.halfV, inset);
  }
}
