package robotkit.spatial;

import robotkit.mobile.Pose2;

/**
 * Immutable rigid transform: translation (meters) plus a unit quaternion
 * rotation (x, y, z, w). By convention a value named `a_T_b` maps
 * coordinates expressed in frame `b` into frame `a`; composition follows
 * `world_T_sensor = world_T_link * link_T_sensor` (ARCHITECTURE.md).
 */
class Transform3 {
  public final translation:Vec3;
  public final rotation:Quat;

  public function new(translation:Vec3, rotation:Quat) {
    if (translation == null || rotation == null)
      throw "Transform3 requires a translation and rotation";
    this.translation = translation;
    this.rotation = rotation;
  }

  public static function identity():Transform3
    return new Transform3(Vec3.zero(), Quat.identity());

  /** `this.compose(local)`: if this is `a_T_b` and local is `b_T_c`, returns `a_T_c`. */
  public function compose(local:Transform3):Transform3 {
    if (local == null) throw "Transform3 composition requires a transform";
    return new Transform3(
      translation.add(rotation.rotate(local.translation)),
      rotation.multiply(local.rotation)
    );
  }

  public function inverse():Transform3 {
    var inverseRotation = rotation.conjugate();
    return new Transform3(inverseRotation.rotate(translation.negate()), inverseRotation);
  }

  /** Maps a point expressed in this transform's local frame into its parent frame. */
  public function transformPoint(point:Vec3):Vec3
    return translation.add(rotation.rotate(point));

  /** Maps a free vector (direction, no translation) into the parent frame. */
  public function transformVector(vector:Vec3):Vec3
    return rotation.rotate(vector);

  /**
   * Adjoint action: moves a twist expressed in this transform's local frame
   * into its parent frame. If this is `a_T_b`, a twist known in `b` becomes
   * the equivalent twist in `a`.
   */
  public function transformTwist(twist:Twist3):Twist3 {
    var angular = rotation.rotate(twist.angular);
    var linear = rotation.rotate(twist.linear).add(translation.cross(angular));
    return new Twist3(linear, angular);
  }

  /** Adjoint action for wrenches; same block structure as `transformTwist`. */
  public function transformWrench(wrench:Wrench3):Wrench3 {
    var force = rotation.rotate(wrench.force);
    var torque = rotation.rotate(wrench.torque).add(translation.cross(force));
    return new Wrench3(force, torque);
  }

  /**
   * First-order integration of a body-frame twist over `dt`: composes this
   * transform with the small motion the twist produces in its own frame.
   * Exact for a static twist only in the limit dt -> 0.
   */
  public function integrate(twist:Twist3, dt:Float):Transform3 {
    if (!Math.isFinite(dt) || dt < 0.0)
      throw "Transform3 integration requires a finite non-negative duration";
    var angle = twist.angular.norm() * dt;
    var deltaRotation = angle <= 1e-12 ? Quat.identity() :
      Quat.fromAxisAngle(twist.angular.normalized(), angle);
    var delta = new Transform3(twist.linear.scale(dt), deltaRotation);
    return compose(delta);
  }

  /** Bridges to the planar stack: `Pose2` becomes a Transform3 at height `z`. */
  public static function fromPose2(pose:Pose2, z:Float = 0.0):Transform3 {
    if (pose == null) throw "Transform3.fromPose2 requires a pose";
    return new Transform3(new Vec3(pose.x, pose.y, z),
      Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), pose.yaw));
  }

  /** Lossy yaw projection for driving the planar stack from a 3D pose. */
  public function toPose2():Pose2 {
    var q = rotation;
    var yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return new Pose2(translation.x, translation.y, yaw);
  }

  /** Reads the raw `position`/`rotation` (xyzw) arrays used by Joint and Frame. */
  public static function fromArrays(position:Array<Float>, rotation:Array<Float>):Transform3
    return new Transform3(Vec3.fromArray(position), Quat.fromArray(rotation));

  /** Column-major 4x4 matrix (16 values), matching ARCHITECTURE.md/SceneKit storage. */
  public function toColumnMajorArray():Array<Float> {
    var r = rotation.toRotationMatrix();
    return [
      r[0], r[1], r[2], 0.0,
      r[3], r[4], r[5], 0.0,
      r[6], r[7], r[8], 0.0,
      translation.x, translation.y, translation.z, 1.0
    ];
  }
}
