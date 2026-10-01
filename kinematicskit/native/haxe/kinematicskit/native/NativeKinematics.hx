package kinematicskit.native;

import KinematicsKitNative;
import kinematicskit.KinematicModel;

/**
 * A `KinematicModel` uploaded to the native core (packed once, see
 * `kinematicskit.h`). Owns the native handle; call `dispose` when done.
 */
class NativeKinematics {
  public final model:KinematicModel;
  final owner:Ownedkk_model_handle;
  var disposed = false;

  public function new(model:KinematicModel) {
    if (model == null) throw "Native kinematics requires a model";
    this.model = model;
    var ints = packInts(model), reals = packReals(model);
    var created = KinematicsKitNative.kk_model_create(ints, reals);
    check(created.status, "model.create");
    owner = created.out_model;
  }

  /** World poses of every body, seven floats each, for DOF values `q` (and optional root poses, seven per body). */
  public function forward(q:Array<Float>, ?rootPoses:Array<Float>):Array<Float> {
    requireLive();
    var result = KinematicsKitNative.kk_forward(owner.borrow(), q, rootPoses == null ? [] : rootPoses,
      model.bodyCount() * 7);
    check(result.status, "forward");
    return result.out_poses;
  }

  /** 6 x `columns.length` Jacobian (row-major) of a world point on `body`, over the listed DOFs. */
  public function pointJacobian(q:Array<Float>, body:Int, point:Array<Float>, columns:Array<Int>):Array<Float> {
    requireLive();
    var result = KinematicsKitNative.kk_point_jacobian(owner.borrow(), q, body, point, columns, 6 * columns.length);
    check(result.status, "point_jacobian");
    return result.out_jacobian;
  }

  /** The live native handle, for other native objects built on this model (e.g. a collision world). */
  public function handle():kk_model_handle {
    requireLive();
    return owner.borrow();
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  /** The integer half of the packed model (format 2). */
  public static function packInts(model:KinematicModel):Array<Int> {
    var ints = [2, model.bodyCount(), model.jointCount(), model.dofCount(), model.frameCount(), model.jointTermDof.length];
    for (body in 0...model.bodyCount()) ints.push(model.bodyParentJoint[body]);
    for (body in model.bodyOrder) ints.push(body);
    for (joint in 0...model.jointCount()) {
      ints.push(cast model.jointKind[joint]);
      ints.push(model.jointParent[joint]);
      ints.push(model.jointChild[joint]);
      ints.push(model.jointDof[joint]);
      ints.push(model.jointSource[joint]);
    }
    for (joint in model.jointOrder) ints.push(joint);
    for (joint in model.jointValueOrder) ints.push(joint);
    for (frame in 0...model.frameCount()) ints.push(model.frameBody[frame]);
    for (value in model.jointTermStart) ints.push(value);
    for (value in model.jointTermDof) ints.push(value);
    return ints;
  }

  /** The floating-point half of the packed model (format 2). */
  public static function packReals(model:KinematicModel):Array<Float> {
    var reals:Array<Float> = [];
    for (pose in model.bodyRootPoses) {
      reals.push(pose.x); reals.push(pose.y); reals.push(pose.z);
      reals.push(pose.qx); reals.push(pose.qy); reals.push(pose.qz); reals.push(pose.qw);
    }
    for (joint in 0...model.jointCount()) {
      for (k in 0...7) reals.push(model.jointParentTJoint[joint * 7 + k]);
      for (k in 0...7) reals.push(model.jointJointTChild[joint * 7 + k]);
      for (k in 0...3) reals.push(model.jointAxis[joint * 3 + k]);
      reals.push(model.jointRatio[joint]);
      reals.push(model.jointOffset[joint]);
      reals.push(model.jointScale[joint]);
    }
    for (value in model.frameOffset) reals.push(value);
    for (value in model.jointConstant) reals.push(value);
    for (value in model.jointTermScale) reals.push(value);
    return reals;
  }

  function requireLive():Void {
    if (disposed) throw "Native kinematics has been disposed";
  }

  static function check(status:Int, operation:String):Void {
    if (status != KinematicsKitNativeConstants.KK_OK)
      throw 'Native kinematics $operation failed with error $status';
  }
}
