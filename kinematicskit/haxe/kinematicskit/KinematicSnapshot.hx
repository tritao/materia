package kinematicskit;

/**
 * World poses of every body and frame, and every movable joint's world
 * origin and axis, for one `KinematicState`. `evaluate` reuses its arrays,
 * so one snapshot serves any number of states without allocating.
 */
class KinematicSnapshot {
  public final model:KinematicModel;
  /** Joint values after couplings, per joint. */
  final values:Array<Float>;
  /** World pose per body, seven floats each. */
  final poses:Array<Float>;
  /** World origin and unit axis per joint (valid for movable joints), three floats each. */
  final origins:Array<Float>;
  final axes:Array<Float>;
  final scratch:Array<Float> = [for (_ in 0...21) 0.0];
  var evaluated = false;

  public function new(model:KinematicModel) {
    if (model == null) throw "Kinematic snapshot requires a model";
    this.model = model;
    values = [for (_ in 0...model.jointCount()) 0.0];
    poses = [for (_ in 0...model.bodyCount() * 7) 0.0];
    origins = [for (_ in 0...model.jointCount() * 3) 0.0];
    axes = [for (_ in 0...model.jointCount() * 3) 0.0];
  }

  /** Convenience: a new snapshot evaluated at `state`. */
  public static function of(state:KinematicState):KinematicSnapshot {
    var snapshot = new KinematicSnapshot(state.model);
    snapshot.evaluate(state);
    return snapshot;
  }

  public function evaluate(state:KinematicState):Void {
    if (state == null || state.model != model) throw "Kinematic snapshot requires a state of its own model";
    evaluateValues(state.q);
    for (body in model.bodyOrder) if (model.bodyParentJoint[body] < 0) {
      var root = state.rootPose(body);
      var o = body * 7;
      poses[o] = root.x; poses[o + 1] = root.y; poses[o + 2] = root.z;
      poses[o + 3] = root.qx; poses[o + 4] = root.qy; poses[o + 5] = root.qz; poses[o + 6] = root.qw;
    }
    var parentTJoint = model.jointParentTJoint;
    var jointTChild = model.jointJointTChild;
    var jointAxis = model.jointAxis;
    for (joint in model.jointOrder) {
      var parentOffset = model.jointParent[joint] * 7;
      var childOffset = model.jointChild[joint] * 7;
      // scratch[0..6]: world joint frame; scratch[7..13]: motion; scratch[14..20]: frame after motion.
      compose(poses, parentOffset, parentTJoint, joint * 7, scratch, 0);
      var kind = model.jointKind[joint];
      if (kind == JointKind.Fixed) {
        compose(scratch, 0, jointTChild, joint * 7, poses, childOffset);
        continue;
      }
      var a = joint * 3;
      var ax = jointAxis[a], ay = jointAxis[a + 1], az = jointAxis[a + 2];
      origins[a] = scratch[0]; origins[a + 1] = scratch[1]; origins[a + 2] = scratch[2];
      rotate(scratch, 0, ax, ay, az, axes, a);
      var value = values[joint];
      if (kind == JointKind.Revolute) {
        var s = Math.sin(value * 0.5);
        scratch[7] = 0.0; scratch[8] = 0.0; scratch[9] = 0.0;
        scratch[10] = ax * s; scratch[11] = ay * s; scratch[12] = az * s; scratch[13] = Math.cos(value * 0.5);
      } else {
        scratch[7] = ax * value; scratch[8] = ay * value; scratch[9] = az * value;
        scratch[10] = 0.0; scratch[11] = 0.0; scratch[12] = 0.0; scratch[13] = 1.0;
      }
      compose(scratch, 0, scratch, 7, scratch, 14);
      compose(scratch, 14, jointTChild, joint * 7, poses, childOffset);
    }
    evaluated = true;
  }

  function evaluateValues(q:Array<Float>):Void {
    if (q.length != model.dofCount())
      throw 'Kinematic snapshot requires ${model.dofCount()} DOF values, got ${q.length}';
    for (joint in model.jointValueOrder) {
      var source = model.jointSource[joint];
      if (source >= 0) values[joint] = values[source] * model.jointRatio[joint] + model.jointOffset[joint];
      else {
        var dof = model.jointDof[joint];
        values[joint] = dof < 0 ? 0.0 : q[dof];
      }
    }
  }

  public function jointValue(joint:Int):Float {
    requireEvaluated();
    return values[joint];
  }

  public function bodyPose(body:Int):Transform {
    requireEvaluated();
    var o = body * 7;
    return new Transform(poses[o], poses[o + 1], poses[o + 2], poses[o + 3], poses[o + 4], poses[o + 5], poses[o + 6]);
  }

  public function framePose(frame:Int):Transform {
    requireEvaluated();
    compose(poses, model.frameBody[frame] * 7, model.frameOffset, frame * 7, scratch, 0);
    return new Transform(scratch[0], scratch[1], scratch[2], scratch[3], scratch[4], scratch[5], scratch[6]);
  }

  /** World origin of a movable joint's frame (its position before its own motion). */
  public function jointOrigin(joint:Int):Vector3 {
    requireEvaluated();
    var a = joint * 3;
    return new Vector3(origins[a], origins[a + 1], origins[a + 2]);
  }

  /** World direction of a movable joint's axis. */
  public function jointWorldAxis(joint:Int):Vector3 {
    requireEvaluated();
    var a = joint * 3;
    return new Vector3(axes[a], axes[a + 1], axes[a + 2]);
  }

  /**
   * Geometric Jacobian, 6 x dofCount, row-major (rows 0..2 linear velocity of
   * the frame origin, rows 3..5 angular velocity), in world coordinates.
   * Writes into `out` when given (resized as needed) and returns it.
   */
  public function frameJacobian(frame:Int, ?out:Array<Float>):Array<Float> {
    requireEvaluated();
    compose(poses, model.frameBody[frame] * 7, model.frameOffset, frame * 7, scratch, 0);
    return pointJacobian(model.frameBody[frame], scratch[0], scratch[1], scratch[2], out);
  }

  /** As `frameJacobian`, for a world point `(px, py, pz)` moving rigidly with `body`. */
  public function pointJacobian(body:Int, px:Float, py:Float, pz:Float, ?out:Array<Float>):Array<Float> {
    requireEvaluated();
    var n = model.dofCount();
    var result = out == null ? [] : out;
    var size = 6 * n;
    if (result.length > size) result.resize(size);
    for (i in 0...size) result[i] = 0.0;
    for (joint in model.bodyChain[body]) {
      var dof = model.jointDof[joint];
      var scale = model.jointScale[joint];
      var a = joint * 3;
      var ax = axes[a], ay = axes[a + 1], az = axes[a + 2];
      if (model.jointKind[joint] == JointKind.Prismatic) {
        result[dof] += scale * ax;
        result[n + dof] += scale * ay;
        result[2 * n + dof] += scale * az;
      } else {
        var rx = px - origins[a], ry = py - origins[a + 1], rz = pz - origins[a + 2];
        result[dof] += scale * (ay * rz - az * ry);
        result[n + dof] += scale * (az * rx - ax * rz);
        result[2 * n + dof] += scale * (ax * ry - ay * rx);
        result[3 * n + dof] += scale * ax;
        result[4 * n + dof] += scale * ay;
        result[5 * n + dof] += scale * az;
      }
    }
    return result;
  }

  function requireEvaluated():Void {
    if (!evaluated) throw "Kinematic snapshot has not been evaluated";
  }

  /** `out[oi..] = a[ai..] · b[bi..]`; `out` may alias either input. */
  static function compose(a:Array<Float>, ai:Int, b:Array<Float>, bi:Int, out:Array<Float>, oi:Int):Void {
    var x = a[ai], y = a[ai + 1], z = a[ai + 2];
    var qx = a[ai + 3], qy = a[ai + 4], qz = a[ai + 5], qw = a[ai + 6];
    var bx = b[bi], by = b[bi + 1], bz = b[bi + 2];
    var bqx = b[bi + 3], bqy = b[bi + 4], bqz = b[bi + 5], bqw = b[bi + 6];
    var tx = 2.0 * (qy * bz - qz * by);
    var ty = 2.0 * (qz * bx - qx * bz);
    var tz = 2.0 * (qx * by - qy * bx);
    out[oi] = bx + qw * tx + qy * tz - qz * ty + x;
    out[oi + 1] = by + qw * ty + qz * tx - qx * tz + y;
    out[oi + 2] = bz + qw * tz + qx * ty - qy * tx + z;
    out[oi + 3] = qw * bqx + qx * bqw + qy * bqz - qz * bqy;
    out[oi + 4] = qw * bqy - qx * bqz + qy * bqw + qz * bqx;
    out[oi + 5] = qw * bqz + qx * bqy - qy * bqx + qz * bqw;
    out[oi + 6] = qw * bqw - qx * bqx - qy * bqy - qz * bqz;
  }

  /** `out[oi..oi+2]` = rotation of `a[ai..]` applied to `(vx, vy, vz)`. */
  static function rotate(a:Array<Float>, ai:Int, vx:Float, vy:Float, vz:Float, out:Array<Float>, oi:Int):Void {
    var qx = a[ai + 3], qy = a[ai + 4], qz = a[ai + 5], qw = a[ai + 6];
    var tx = 2.0 * (qy * vz - qz * vy);
    var ty = 2.0 * (qz * vx - qx * vz);
    var tz = 2.0 * (qx * vy - qy * vx);
    out[oi] = vx + qw * tx + qy * tz - qz * ty;
    out[oi + 1] = vy + qw * ty + qz * tx - qx * tz;
    out[oi + 2] = vz + qw * tz + qx * ty - qy * tx;
  }
}
