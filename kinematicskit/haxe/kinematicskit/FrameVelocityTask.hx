package kinematicskit;

/**
 * Moves a point frame rigidly attached to a body (`body · offset`) at a
 * twist for one step: the residual is `(v, ω)·dt` and the rows are the
 * frame point's Jacobian, so a differential solve (`DifferentialIk`) returns
 * the joint velocities that produce that twist. With `relativeTo`, the twist
 * is relative to another body's frame (a workpiece on a positioner) and
 * expressed in it; otherwise it is in the world frame.
 *
 * A step goal, not a pose goal: it is never "satisfied", so use it in
 * differential solves, not iterative IK.
 */
class FrameVelocityTask implements KinematicTask {
  public final body:Int;
  public var weight:Float = 1.0;
  final offsetFlat:Array<Float> = [for (_ in 0...7) 0.0];
  var referenceBody = -1;
  final referenceFlat:Array<Float> = [for (_ in 0...7) 0.0];
  /** Twist in the reference frame (linear then angular), times dt. */
  final displacement:Array<Float> = [for (_ in 0...6) 0.0];
  final scratch:Array<Float> = [for (_ in 0...14) 0.0];
  final pointJacobian:Array<Float> = [];
  final referenceJacobian:Array<Float> = [];
  final name:String;

  public function new(model:KinematicModel, body:Int, ?offset:Transform, ?name:String) {
    if (model == null || body < 0 || body >= model.bodyCount()) throw "Frame velocity task requires a body of the model";
    this.body = body;
    FlatTransform.write(offset == null ? Transform.identity() : Transform.checked(offset, "Frame velocity task offset"),
      offsetFlat, 0);
    this.name = name == null ? model.bodyIds[body] + " velocity" : name;
  }

  /** A task on a model frame, optionally shifted further by `extraOffset` (e.g. a tool centre point). */
  public static function atFrame(model:KinematicModel, frame:Int, ?extraOffset:Transform):FrameVelocityTask {
    if (frame < 0 || frame >= model.frameCount()) throw "Frame velocity task requires a frame of the model";
    var offset = model.frameTransform(frame);
    if (extraOffset != null) offset = offset.compose(extraOffset);
    return new FrameVelocityTask(model, model.frameBody[frame], offset, model.frameIds[frame] + " velocity");
  }

  /** Expresses the twist relative to the frame `reference · referenceOffset`. Returns this. */
  public function relativeTo(model:KinematicModel, reference:Int, ?referenceOffset:Transform):FrameVelocityTask {
    if (reference < 0 || reference >= model.bodyCount()) throw "Frame velocity task reference must be a body of the model";
    referenceBody = reference;
    FlatTransform.write(referenceOffset == null ? Transform.identity()
      : Transform.checked(referenceOffset, "Frame velocity task reference offset"), referenceFlat, 0);
    return this;
  }

  /** The twist for the next step: linear `(vx, vy, vz)`, angular `(wx, wy, wz)`, over `dt` seconds. */
  public function setTwist(twist:Array<Float>, dt:Float):Void {
    if (twist == null || twist.length != 6) throw "A frame twist has six components";
    if (!(dt > 0.0) || !Math.isFinite(dt)) throw "A frame twist needs a positive step";
    for (i in 0...6) {
      if (!Math.isFinite(twist[i])) throw "A frame twist must be finite";
      displacement[i] = twist[i] * dt;
    }
  }

  public function label():String return name;
  public function rowCount():Int return 6;
  public function isSoft():Bool return false;
  public function positionError():Float return 0.0;
  public function orientationError():Float return 0.0;
  public function satisfied():Bool return false;

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    if (pointJacobian.length < 6 * w) pointJacobian.resize(6 * w);
    snapshot.attachedPoseInto(body, offsetFlat, 0, scratch, 0);
    snapshot.pointJacobianColumns(body, scratch[0], scratch[1], scratch[2], layout, pointJacobian);
    // Twist into world coordinates (the reference's rotation), and the reference's own motion out of the rows.
    var qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (referenceBody >= 0) {
      if (referenceJacobian.length < 6 * w) referenceJacobian.resize(6 * w);
      snapshot.attachedPoseInto(referenceBody, referenceFlat, 0, scratch, 7);
      qx = scratch[10]; qy = scratch[11]; qz = scratch[12]; qw = scratch[13];
      snapshot.pointJacobianColumns(referenceBody, scratch[0], scratch[1], scratch[2], layout, referenceJacobian);
      for (i in 0...6 * w) pointJacobian[i] -= referenceJacobian[i];
    }
    var rotation = new Transform(0.0, 0.0, 0.0, qx, qy, qz, qw);
    for (block in 0...2) {
      var world = rotation.transformVector(displacement[3 * block], displacement[3 * block + 1], displacement[3 * block + 2]);
      residual[row + 3 * block] = weight * world.x;
      residual[row + 3 * block + 1] = weight * world.y;
      residual[row + 3 * block + 2] = weight * world.z;
    }
    for (r in 0...6) for (c in 0...w) jacobian[(row + r) * w + c] = weight * pointJacobian[r * w + c];
  }
}
