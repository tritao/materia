package kinematicskit;

/**
 * Moves a point frame rigidly attached to a body (`body · offset`) to a
 * world target.
 *
 * Position: with all three axes selected, three rows along world X/Y/Z; with
 * a subset (`AXIS_X | AXIS_Y | AXIS_Z`), one row per selected axis of the
 * *target* frame, so e.g. `AXIS_Z` alone only fixes depth along the target's Z.
 *
 * Orientation (`FrameOrientation`): `Full` uses the rotation vector of
 * `target · current⁻¹` in world coordinates with the angular Jacobian rows
 * (first order, exact at convergence; the exact log-map derivative was
 * measured and rejected, see KINEMATICS.md K-log "orientation Jacobian");
 * `Axis(u)` aligns the frame's local axis `u` with the target's, leaving
 * rotation about it free.
 *
 * With `relativeTo(reference, offset)` the target is expressed in a frame
 * on another body (e.g. a workpiece on a positioner): the task holds the
 * frame at that pose relative to the reference, both bodies free to move.
 * Its rows are then J_body − J_reference, the reference's taken at the
 * target point (exact for position), so the solver may move either side.
 *
 * Position and orientation errors stay separate (straight-line position
 * error, per-part masks and tolerances). A different error formulation
 * (e.g. mink's SE(3) logarithm) belongs in its own task type, not a mode.
 */
class FrameTask implements KinematicTask {
  public static inline var AXIS_X = 1;
  public static inline var AXIS_Y = 2;
  public static inline var AXIS_Z = 4;
  public static inline var ALL_AXES = 7;

  public final body:Int;
  public final positionAxes:Int;
  public final orientation:FrameOrientation;
  public var positionTolerance:Float;
  public var orientationTolerance:Float;
  public var positionWeight:Float = 1.0;
  public var orientationWeight:Float = 1.0;
  final name:String;
  final rows:Int;
  /** body_T_point, then the target, seven floats each. */
  final offsetFlat:Array<Float> = [for (_ in 0...7) 0.0];
  final targetFlat:Array<Float> = [for (_ in 0...7) 0.0];
  /** Current pose (0..6), a rotated axis (7..9), rotation vector (10..12), basis (13..18), second axis (19..21). */
  final scratch:Array<Float> = [for (_ in 0...22) 0.0];
  final pointJacobian:Array<Float> = [];
  final referenceJacobian:Array<Float> = [];
  /** Body the target is relative to (-1: the world), and reference_T_frame on it. */
  var referenceBody = -1;
  final referenceFlat:Array<Float> = [for (_ in 0...7) 0.0];
  /** The target in world coordinates for this evaluation (0..6), the reference pose (7..13). */
  final goal:Array<Float> = [for (_ in 0...14) 0.0];
  var preference:Bool = false;
  var lastPositionError = 0.0;
  var lastOrientationError = 0.0;

  public function new(model:KinematicModel, body:Int, offset:Null<Transform>, target:Transform,
      positionTolerance:Float, orientationTolerance:Float, ?positionAxes:Int = 7,
      ?orientation:FrameOrientation, ?name:String) {
    if (model == null || body < 0 || body >= model.bodyCount()) throw "Frame task requires a body of the model";
    if (positionAxes < 0 || positionAxes > ALL_AXES) throw "Frame task position axes must be a mask of AXIS_X/Y/Z";
    if (!(positionTolerance > 0.0) || !(orientationTolerance > 0.0))
      throw "Frame task tolerances must be positive";
    this.body = body;
    FlatTransform.write(offset == null ? Transform.identity() : Transform.checked(offset, "Frame task offset"), offsetFlat, 0);
    setTarget(target);
    this.positionAxes = positionAxes;
    this.orientation = orientation == null ? FrameOrientation.Full : normalizedAxis(orientation);
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.name = name == null ? model.bodyIds[body] : name;
    rows = bitCount(positionAxes) + switch this.orientation {
      case Full: 3;
      case Axis(_, _, _): 2;
      case Free: 0;
    };
  }

  /** A task on a model frame, optionally shifted further by `extraOffset` (e.g. a tool centre point). */
  public static function atFrame(model:KinematicModel, frame:Int, target:Transform, positionTolerance:Float,
      orientationTolerance:Float, ?extraOffset:Transform, ?positionAxes:Int = 7,
      ?orientation:FrameOrientation):FrameTask {
    if (frame < 0 || frame >= model.frameCount()) throw "Frame task requires a frame of the model";
    var offset = model.frameTransform(frame);
    if (extraOffset != null) offset = offset.compose(extraOffset);
    return new FrameTask(model, model.frameBody[frame], offset, target, positionTolerance, orientationTolerance,
      positionAxes, orientation, model.frameIds[frame]);
  }

  /**
   * Expresses the target relative to the frame `reference · referenceOffset`
   * instead of the world. Returns this task.
   */
  public function relativeTo(model:KinematicModel, reference:Int, ?referenceOffset:Transform):FrameTask {
    if (reference < 0 || reference >= model.bodyCount()) throw "Frame task reference must be a body of the model";
    referenceBody = reference;
    FlatTransform.write(referenceOffset == null ? Transform.identity()
      : Transform.checked(referenceOffset, "Frame task reference offset"), referenceFlat, 0);
    return this;
  }

  /** The frame's pose relative to the reference (the world pose without one) in an evaluated snapshot. */
  public function relativePose(snapshot:KinematicSnapshot):Transform {
    var pose = currentPose(snapshot);
    if (referenceBody < 0) return pose;
    snapshot.attachedPoseInto(referenceBody, referenceFlat, 0, goal, 7);
    return FlatTransform.read(goal, 7).inverse().compose(pose);
  }

  /** Moves the target, e.g. while an editor gizmo is dragged; takes effect at the next `evaluate`. */
  public function setTarget(target:Transform):Void
    FlatTransform.write(Transform.checked(target, "Frame task target"), targetFlat, 0);

  public function target():Transform return FlatTransform.read(targetFlat, 0);

  public function label():String return name;
  public function rowCount():Int return rows;
  /** Makes this task a preference, optimized after the hard tasks. */
  public function asPreference():FrameTask {
    preference = true;
    return this;
  }
  public function isSoft():Bool return preference;
  public function positionError():Float return lastPositionError;
  public function orientationError():Float return lastOrientationError;
  public function satisfied():Bool
    return preference || lastPositionError <= positionTolerance && lastOrientationError <= orientationTolerance;

  /** The frame's current world pose in an evaluated snapshot. */
  public function currentPose(snapshot:KinematicSnapshot):Transform {
    snapshot.attachedPoseInto(body, offsetFlat, 0, scratch, 0);
    return FlatTransform.read(scratch, 0);
  }

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    if (pointJacobian.length < 6 * w) pointJacobian.resize(6 * w);
    snapshot.attachedPoseInto(body, offsetFlat, 0, scratch, 0);
    snapshot.pointJacobianColumns(body, scratch[0], scratch[1], scratch[2], layout, pointJacobian);
    // The target in world coordinates; relative to a moving reference, the relative velocity's rows.
    var targetFlat = this.targetFlat;
    if (referenceBody >= 0) {
      snapshot.attachedPoseInto(referenceBody, referenceFlat, 0, goal, 7);
      FlatTransform.compose(goal, 7, this.targetFlat, 0, goal, 0);
      targetFlat = goal;
      if (referenceJacobian.length < 6 * w) referenceJacobian.resize(6 * w);
      // The residual is target − frame, so the reference moves the target point itself.
      snapshot.pointJacobianColumns(referenceBody, goal[0], goal[1], goal[2], layout, referenceJacobian);
      for (i in 0...6 * w) pointJacobian[i] -= referenceJacobian[i];
    }
    var ex = targetFlat[0] - scratch[0], ey = targetFlat[1] - scratch[1], ez = targetFlat[2] - scratch[2];
    var r = row;
    if (positionAxes == ALL_AXES) {
      lastPositionError = Math.sqrt(ex * ex + ey * ey + ez * ez);
      residual[r] = positionWeight * ex;
      residual[r + 1] = positionWeight * ey;
      residual[r + 2] = positionWeight * ez;
      for (axis in 0...3) for (c in 0...w) jacobian[(r + axis) * w + c] = positionWeight * pointJacobian[axis * w + c];
      r += 3;
    } else {
      var squared = 0.0;
      for (bit in 0...3) if (positionAxes & (1 << bit) != 0) {
        FlatTransform.rotate(targetFlat, 0, bit == 0 ? 1.0 : 0.0, bit == 1 ? 1.0 : 0.0, bit == 2 ? 1.0 : 0.0, scratch, 7);
        var ax = scratch[7], ay = scratch[8], az = scratch[9];
        var e = ax * ex + ay * ey + az * ez;
        squared += e * e;
        residual[r] = positionWeight * e;
        for (c in 0...w)
          jacobian[r * w + c] = positionWeight * (ax * pointJacobian[c] + ay * pointJacobian[w + c] + az * pointJacobian[2 * w + c]);
        r++;
      }
      lastPositionError = Math.sqrt(squared);
    }
    switch orientation {
      case Full:
        Rotations.logOfProduct(targetFlat[3], targetFlat[4], targetFlat[5], targetFlat[6], -scratch[3], -scratch[4],
          -scratch[5], scratch[6], scratch, 10);
        var wx = scratch[10], wy = scratch[11], wz = scratch[12];
        lastOrientationError = Math.sqrt(wx * wx + wy * wy + wz * wz);
        residual[r] = orientationWeight * wx;
        residual[r + 1] = orientationWeight * wy;
        residual[r + 2] = orientationWeight * wz;
        for (axis in 0...3) for (c in 0...w) jacobian[(r + axis) * w + c] = orientationWeight * pointJacobian[(3 + axis) * w + c];
      case Axis(ux, uy, uz):
        FlatTransform.rotate(scratch, 0, ux, uy, uz, scratch, 7);
        FlatTransform.rotate(targetFlat, 0, ux, uy, uz, scratch, 19);
        var ax = scratch[7], ay = scratch[8], az = scratch[9];
        var bx = scratch[19], by = scratch[20], bz = scratch[21];
        Rotations.perpendicularBasis(bx, by, bz, scratch, 13);
        lastOrientationError = Math.acos(Math.min(1.0, Math.max(-1.0, ax * bx + ay * by + az * bz)));
        for (k in 0...2) {
          var px = scratch[13 + 3 * k], py = scratch[14 + 3 * k], pz = scratch[15 + 3 * k];
          // current = a · p; d(a · p)/dq = (a × p) · ω.
          var cx = ay * pz - az * py, cy = az * px - ax * pz, cz = ax * py - ay * px;
          residual[r] = -orientationWeight * (ax * px + ay * py + az * pz);
          for (c in 0...w)
            jacobian[r * w + c] = orientationWeight * (cx * pointJacobian[3 * w + c] + cy * pointJacobian[4 * w + c] +
              cz * pointJacobian[5 * w + c]);
          r++;
        }
      case Free:
        lastOrientationError = 0.0;
    }
  }

  static function normalizedAxis(orientation:FrameOrientation):FrameOrientation return switch orientation {
    case Axis(x, y, z):
      var norm = Math.sqrt(x * x + y * y + z * z);
      if (!Math.isFinite(norm) || norm < 1e-12) throw "Frame task orientation axis must be non-zero";
      FrameOrientation.Axis(x / norm, y / norm, z / norm);
    case other: other;
  };

  static function bitCount(mask:Int):Int return (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1);
}
