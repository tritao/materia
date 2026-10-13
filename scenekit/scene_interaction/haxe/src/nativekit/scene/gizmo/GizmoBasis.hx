package nativekit.scene.gizmo;

/** Immutable right-handed orthonormal movement axes; scale never changes handle size or snap units. */
class GizmoBasis {
  public final x:GizmoVec3;
  public final y:GizmoVec3;
  public final z:GizmoVec3;
  public function new(x:GizmoVec3, y:GizmoVec3, z:GizmoVec3) {
    if (x == null || y == null || z == null || Math.abs(x.dot(x)-1) > 0.000001 || Math.abs(y.dot(y)-1) > 0.000001 || Math.abs(z.dot(z)-1) > 0.000001 || Math.abs(x.dot(y)) > 0.000001 || Math.abs(x.dot(z)) > 0.000001 || Math.abs(y.dot(z)) > 0.000001 || x.cross(y).dot(z) < 0.999999) throw "Expected right-handed orthonormal gizmo basis";
    this.x = x; this.y = y; this.z = z;
  }
  public static function world():GizmoBasis return new GizmoBasis(new GizmoVec3(1,0,0),new GizmoVec3(0,1,0),new GizmoVec3(0,0,1));
  /** Rotation only, from a finite nonzero quaternion. Normalize without inheriting object/parent scale. */
  public static function fromQuaternion(qx:Float,qy:Float,qz:Float,qw:Float):GizmoBasis {
    if (!Math.isFinite(qx) || !Math.isFinite(qy) || !Math.isFinite(qz) || !Math.isFinite(qw)) throw "Invalid gizmo quaternion";
    var largest = Math.max(Math.max(Math.abs(qx),Math.abs(qy)),Math.max(Math.abs(qz),Math.abs(qw)));
    if (largest == 0) throw "Invalid gizmo quaternion";
    qx /= largest; qy /= largest; qz /= largest; qw /= largest;
    var length = Math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qx /= length; qy /= length; qz /= length; qw /= length;
    return new GizmoBasis(new GizmoVec3(1-2*(qy*qy+qz*qz),2*(qx*qy+qw*qz),2*(qx*qz-qw*qy)),
      new GizmoVec3(2*(qx*qy-qw*qz),1-2*(qx*qx+qz*qz),2*(qy*qz+qw*qx)),
      new GizmoVec3(2*(qx*qz+qw*qy),2*(qy*qz-qw*qx),1-2*(qx*qx+qy*qy)));
  }
  public function axis(handle:TranslationHandle):GizmoVec3 return switch handle {
    case AxisX: x; case AxisY: y; case AxisZ: z; default: throw "Expected translation axis";
  };
  public function planeAxes(handle:TranslationHandle):Array<GizmoVec3> return switch handle {
    case PlaneXY: [x,y]; case PlaneXZ: [x,z]; case PlaneYZ: [y,z]; default: throw "Expected translation plane";
  };
}
