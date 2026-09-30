package kinematicskit;

/**
 * Points a local axis of a frame (`body · offset`) at a world point, e.g. a
 * camera's optical axis at a part. Two rows: the axis components across the
 * line of sight. Rotation about the line of sight and the frame's position
 * are free. The error is the angle between the axis and the line of sight.
 */
class LookAtTask implements KinematicTask {
  public final body:Int;
  public var tolerance:Float;
  public var weight:Float = 1.0;
  final ux:Float;
  final uy:Float;
  final uz:Float;
  final name:String;
  final offsetFlat:Array<Float> = [for (_ in 0...7) 0.0];
  final point:Array<Float> = [0.0, 0.0, 0.0];
  /** Pose (0..6), axis (7..9), basis across the line of sight (10..15). */
  final scratch:Array<Float> = [for (_ in 0...16) 0.0];
  final pointJacobian:Array<Float> = [];
  var lastError = 0.0;

  public function new(model:KinematicModel, body:Int, offset:Null<Transform>, axis:Vector3, target:Vector3,
      tolerance:Float, ?name:String) {
    if (model == null || body < 0 || body >= model.bodyCount()) throw "Look-at task requires a body of the model";
    if (!(tolerance > 0.0)) throw "Look-at task tolerance must be positive";
    var norm = axis == null ? 0.0 : Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (!Math.isFinite(norm) || norm < 1e-12) throw "Look-at task axis must be non-zero";
    this.body = body;
    ux = axis.x / norm; uy = axis.y / norm; uz = axis.z / norm;
    FlatTransform.write(offset == null ? Transform.identity() : Transform.checked(offset, "Look-at task offset"), offsetFlat, 0);
    setTarget(target);
    this.tolerance = tolerance;
    this.name = name == null ? model.bodyIds[body] : name;
  }

  public static function atFrame(model:KinematicModel, frame:Int, axis:Vector3, target:Vector3, tolerance:Float):LookAtTask {
    if (frame < 0 || frame >= model.frameCount()) throw "Look-at task requires a frame of the model";
    return new LookAtTask(model, model.frameBody[frame], model.frameTransform(frame), axis, target, tolerance,
      model.frameIds[frame]);
  }

  public function setTarget(target:Vector3):Void {
    if (target == null || !Math.isFinite(target.x) || !Math.isFinite(target.y) || !Math.isFinite(target.z))
      throw "Look-at task target must be finite";
    point[0] = target.x; point[1] = target.y; point[2] = target.z;
  }

  public function label():String return name;
  public function rowCount():Int return 2;
  public function isSoft():Bool return false;
  public function positionError():Float return 0.0;
  public function orientationError():Float return lastError;
  public function satisfied():Bool return lastError <= tolerance;

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    if (pointJacobian.length < 6 * w) pointJacobian.resize(6 * w);
    var s = scratch;
    snapshot.attachedPoseInto(body, offsetFlat, 0, s, 0);
    snapshot.pointJacobianColumns(body, s[0], s[1], s[2], layout, pointJacobian);
    FlatTransform.rotate(s, 0, ux, uy, uz, s, 7);
    var ax = s[7], ay = s[8], az = s[9];
    var lx = point[0] - s[0], ly = point[1] - s[1], lz = point[2] - s[2];
    var distance = Math.sqrt(lx * lx + ly * ly + lz * lz);
    if (distance < 1e-12) throw 'Look-at task "$name" target coincides with the frame';
    var dx = lx / distance, dy = ly / distance, dz = lz / distance;
    lastError = Math.acos(Math.min(1.0, Math.max(-1.0, ax * dx + ay * dy + az * dz)));
    Rotations.perpendicularBasis(dx, dy, dz, s, 10);
    for (k in 0...2) {
      var px = s[10 + 3 * k], py = s[11 + 3 * k], pz = s[12 + 3 * k];
      residual[row + k] = -weight * (ax * px + ay * py + az * pz);
      // d(a·p) ≈ (a × p)·ω + p·v / distance: the axis turns with the frame, and the
      // line of sight turns as the frame moves across it.
      var cx = ay * pz - az * py, cy = az * px - ax * pz, cz = ax * py - ay * px;
      for (c in 0...w)
        jacobian[(row + k) * w + c] = weight * (cx * pointJacobian[3 * w + c] + cy * pointJacobian[4 * w + c] +
          cz * pointJacobian[5 * w + c] + (px * pointJacobian[c] + py * pointJacobian[w + c] + pz * pointJacobian[2 * w + c]) / distance);
    }
  }
}
