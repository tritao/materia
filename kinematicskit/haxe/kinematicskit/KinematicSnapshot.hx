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
      FlatTransform.compose(poses, parentOffset, parentTJoint, joint * 7, scratch, 0);
      var kind = model.jointKind[joint];
      if (kind == JointKind.Fixed) {
        FlatTransform.compose(scratch, 0, jointTChild, joint * 7, poses, childOffset);
        continue;
      }
      var a = joint * 3;
      var ax = jointAxis[a], ay = jointAxis[a + 1], az = jointAxis[a + 2];
      origins[a] = scratch[0]; origins[a + 1] = scratch[1]; origins[a + 2] = scratch[2];
      FlatTransform.rotate(scratch, 0, ax, ay, az, axes, a);
      var value = values[joint];
      if (kind == JointKind.Revolute) {
        var s = Math.sin(value * 0.5);
        scratch[7] = 0.0; scratch[8] = 0.0; scratch[9] = 0.0;
        scratch[10] = ax * s; scratch[11] = ay * s; scratch[12] = az * s; scratch[13] = Math.cos(value * 0.5);
      } else {
        scratch[7] = ax * value; scratch[8] = ay * value; scratch[9] = az * value;
        scratch[10] = 0.0; scratch[11] = 0.0; scratch[12] = 0.0; scratch[13] = 1.0;
      }
      FlatTransform.compose(scratch, 0, scratch, 7, scratch, 14);
      FlatTransform.compose(scratch, 14, jointTChild, joint * 7, poses, childOffset);
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
    FlatTransform.compose(poses, model.frameBody[frame] * 7, model.frameOffset, frame * 7, scratch, 0);
    return new Transform(scratch[0], scratch[1], scratch[2], scratch[3], scratch[4], scratch[5], scratch[6]);
  }

  /** Writes a body's world pose (seven floats) into `out` at `offset`. */
  public function bodyPoseInto(body:Int, out:Array<Float>, offset:Int):Void {
    requireEvaluated();
    var o = body * 7;
    for (i in 0...7) out[offset + i] = poses[o + i];
  }

  /** Writes the world pose of `body · bodyTPoint` (seven floats at `pointOffset`) into `out` at `offset`. */
  public function attachedPoseInto(body:Int, bodyTPoint:Array<Float>, pointOffset:Int, out:Array<Float>,
      offset:Int):Void {
    requireEvaluated();
    FlatTransform.compose(poses, body * 7, bodyTPoint, pointOffset, out, offset);
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
    FlatTransform.compose(poses, model.frameBody[frame] * 7, model.frameOffset, frame * 7, scratch, 0);
    return pointJacobian(model.frameBody[frame], scratch[0], scratch[1], scratch[2], out);
  }

  /** As `frameJacobian`, at a body's own origin. */
  public function bodyJacobian(body:Int, ?out:Array<Float>):Array<Float> {
    requireEvaluated();
    var o = body * 7;
    return pointJacobian(body, poses[o], poses[o + 1], poses[o + 2], out);
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

  /**
   * As `pointJacobian`, but only for the DOFs in `layout`: writes 6 rows of
   * `layout.width` columns into `out` (which must hold 6 x width values).
   * Allocates nothing.
   */
  public function pointJacobianColumns(body:Int, px:Float, py:Float, pz:Float, layout:JacobianLayout,
      out:Array<Float>):Void {
    requireEvaluated();
    var w = layout.width;
    for (i in 0...6 * w) out[i] = 0.0;
    for (joint in model.bodyChain[body]) {
      var column = layout.columnOfDof[model.jointDof[joint]];
      if (column < 0) continue;
      var scale = model.jointScale[joint];
      var a = joint * 3;
      var ax = axes[a], ay = axes[a + 1], az = axes[a + 2];
      if (model.jointKind[joint] == JointKind.Prismatic) {
        out[column] += scale * ax;
        out[w + column] += scale * ay;
        out[2 * w + column] += scale * az;
      } else {
        var rx = px - origins[a], ry = py - origins[a + 1], rz = pz - origins[a + 2];
        out[column] += scale * (ay * rz - az * ry);
        out[w + column] += scale * (az * rx - ax * rz);
        out[2 * w + column] += scale * (ax * ry - ay * rx);
        out[3 * w + column] += scale * ax;
        out[4 * w + column] += scale * ay;
        out[5 * w + column] += scale * az;
      }
    }
  }

  function requireEvaluated():Void {
    if (!evaluated) throw "Kinematic snapshot has not been evaluated";
  }
}
