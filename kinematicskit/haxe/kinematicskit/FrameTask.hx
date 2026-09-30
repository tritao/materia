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
 * (first order, exact at convergence); `Axis(u)` aligns the frame's local
 * axis `u` with the target's, leaving rotation about it free.
 */
class FrameTask implements KinematicTask {
  public static inline var AXIS_X = 1;
  public static inline var AXIS_Y = 2;
  public static inline var AXIS_Z = 4;
  public static inline var ALL_AXES = 7;

  public final body:Int;
  public final offset:Transform;
  public var target:Transform;
  public final positionAxes:Int;
  public final orientation:FrameOrientation;
  public var positionTolerance:Float;
  public var orientationTolerance:Float;
  public var positionWeight:Float = 1.0;
  public var orientationWeight:Float = 1.0;
  final name:String;
  final rows:Int;
  final pointJacobian:Array<Float> = [];
  var lastPositionError = 0.0;
  var lastOrientationError = 0.0;

  public function new(model:KinematicModel, body:Int, offset:Transform, target:Transform,
      positionTolerance:Float, orientationTolerance:Float, ?positionAxes:Int = 7,
      ?orientation:FrameOrientation, ?name:String) {
    if (model == null || body < 0 || body >= model.bodyCount()) throw "Frame task requires a body of the model";
    if (positionAxes < 0 || positionAxes > ALL_AXES) throw "Frame task position axes must be a mask of AXIS_X/Y/Z";
    if (!(positionTolerance > 0.0) || !(orientationTolerance > 0.0))
      throw "Frame task tolerances must be positive";
    this.body = body;
    this.offset = offset == null ? Transform.identity() : Transform.checked(offset, "Frame task offset");
    this.target = Transform.checked(target, "Frame task target");
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

  public function label():String return name;
  public function rowCount():Int return rows;
  public function isSoft():Bool return false;
  public function positionError():Float return lastPositionError;
  public function orientationError():Float return lastOrientationError;
  public function satisfied():Bool
    return lastPositionError <= positionTolerance && lastOrientationError <= orientationTolerance;

  /** The frame's current world pose in an evaluated snapshot. */
  public function currentPose(snapshot:KinematicSnapshot):Transform return snapshot.bodyPose(body).compose(offset);

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, residual:Array<Float>,
      jacobian:Array<Float>, row:Int):Void {
    var n = snapshot.model.dofCount();
    var pose = currentPose(snapshot);
    snapshot.pointJacobian(body, pose.x, pose.y, pose.z, pointJacobian);
    var ex = target.x - pose.x, ey = target.y - pose.y, ez = target.z - pose.z;
    var r = row;
    if (positionAxes == ALL_AXES) {
      lastPositionError = Math.sqrt(ex * ex + ey * ey + ez * ez);
      var errors = [ex, ey, ez];
      for (axis in 0...3) {
        residual[r] = positionWeight * errors[axis];
        for (c in 0...n) jacobian[r * n + c] = positionWeight * pointJacobian[axis * n + c];
        r++;
      }
    } else {
      var squared = 0.0;
      for (bit in 0...3) if (positionAxes & (1 << bit) != 0) {
        var a = target.transformVector(bit == 0 ? 1.0 : 0.0, bit == 1 ? 1.0 : 0.0, bit == 2 ? 1.0 : 0.0);
        var e = a.x * ex + a.y * ey + a.z * ez;
        squared += e * e;
        residual[r] = positionWeight * e;
        for (c in 0...n)
          jacobian[r * n + c] = positionWeight * (a.x * pointJacobian[c] + a.y * pointJacobian[n + c] +
            a.z * pointJacobian[2 * n + c]);
        r++;
      }
      lastPositionError = Math.sqrt(squared);
    }
    switch orientation {
      case Full:
        var w = Rotations.logOfProduct(target.qx, target.qy, target.qz, target.qw, -pose.qx, -pose.qy, -pose.qz, pose.qw);
        lastOrientationError = Math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
        var errors = [w.x, w.y, w.z];
        for (axis in 0...3) {
          residual[r] = orientationWeight * errors[axis];
          for (c in 0...n) jacobian[r * n + c] = orientationWeight * pointJacobian[(3 + axis) * n + c];
          r++;
        }
      case Axis(ux, uy, uz):
        var a = pose.transformVector(ux, uy, uz);
        var b = target.transformVector(ux, uy, uz);
        var basis = Rotations.perpendicularBasis(b.x, b.y, b.z);
        lastOrientationError = Math.acos(Math.min(1.0, Math.max(-1.0, a.x * b.x + a.y * b.y + a.z * b.z)));
        for (p in basis) {
          // current = a · p; d(a · p)/dq = (a × p) · ω.
          var cx = a.y * p.z - a.z * p.y, cy = a.z * p.x - a.x * p.z, cz = a.x * p.y - a.y * p.x;
          residual[r] = -orientationWeight * (a.x * p.x + a.y * p.y + a.z * p.z);
          for (c in 0...n)
            jacobian[r * n + c] = orientationWeight * (cx * pointJacobian[3 * n + c] + cy * pointJacobian[4 * n + c] +
              cz * pointJacobian[5 * n + c]);
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
