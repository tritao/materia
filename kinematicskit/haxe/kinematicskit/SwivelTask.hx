package kinematicskit;

/**
 * The swivel (elbow) angle of an arm with a shoulder, an elbow and a wrist
 * point: how far the elbow is turned about the shoulder-wrist line, measured
 * from the plane that contains that line and a world `reference` direction
 * (world Z by default), positive by the right-hand rule about
 * shoulder -> wrist. On a 7-axis arm with a 6D tool target this is the one
 * redundant motion, so one row picks the arm's configuration exactly.
 *
 * The residual is exact (the wrapped angle difference). The Jacobian row is
 * ψ's gradient with respect to the three points (central differences of the
 * closed-form angle, accurate to about 1e-8) times their point Jacobians, so
 * no extra forward kinematics runs. Degenerate when the elbow lies on the
 * shoulder-wrist line or that line is parallel to `reference`.
 */
class SwivelTask implements KinematicTask {
  public var target:Float;
  public var tolerance:Float;
  public var weight:Float = 1.0;
  final soft:Bool;
  final bodies:Array<Int>;
  /** Shoulder, elbow, wrist positions in their bodies, three floats each. */
  final local:Array<Float>;
  final reference:Array<Float>;
  final name:String;
  /** World points (0..8), gradient (9..17), perturbed points (18..26). */
  final scratch:Array<Float> = [for (_ in 0...27) 0.0];
  final pointJacobian:Array<Float> = [];
  final pose:Array<Float> = [for (_ in 0...7) 0.0];
  var lastError = 0.0;

  public function new(model:KinematicModel, shoulderBody:Int, shoulder:Vector3, elbowBody:Int, elbow:Vector3,
      wristBody:Int, wrist:Vector3, target:Float, tolerance:Float, ?reference:Vector3, ?soft:Bool = false,
      ?name:String) {
    for (body in [shoulderBody, elbowBody, wristBody])
      if (model == null || body < 0 || body >= model.bodyCount()) throw "Swivel task requires bodies of the model";
    if (!Math.isFinite(target) || !(tolerance > 0.0)) throw "Swivel task needs a finite target and positive tolerance";
    var r = reference == null ? new Vector3(0, 0, 1) : reference;
    var norm = Math.sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    if (!Math.isFinite(norm) || norm < 1e-12) throw "Swivel task reference must be non-zero";
    bodies = [shoulderBody, elbowBody, wristBody];
    local = [shoulder.x, shoulder.y, shoulder.z, elbow.x, elbow.y, elbow.z, wrist.x, wrist.y, wrist.z];
    this.reference = [r.x / norm, r.y / norm, r.z / norm];
    this.target = target;
    this.tolerance = tolerance;
    this.soft = soft;
    this.name = name == null ? "swivel" : name;
  }

  public function label():String return name;
  public function rowCount():Int return 1;
  public function isSoft():Bool return soft;
  public function positionError():Float return 0.0;
  public function orientationError():Float return lastError;
  public function satisfied():Bool return soft || lastError <= tolerance;

  /** The current swivel angle in an evaluated snapshot. */
  public function angle(snapshot:KinematicSnapshot):Float {
    worldPoints(snapshot);
    return swivel(scratch, 0);
  }

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    if (pointJacobian.length < 6 * w) pointJacobian.resize(6 * w);
    worldPoints(snapshot);
    var psi = swivel(scratch, 0);
    var e = wrap(target - psi);
    lastError = Math.abs(e);
    residual[row] = weight * e;
    // ∂ψ/∂(point coordinates), by central differences of the closed form.
    var scale = 0.0;
    for (i in 0...9) scale = Math.max(scale, Math.abs(scratch[i]));
    var h = 1e-6 * Math.max(1.0, scale);
    for (i in 0...9) {
      for (k in 0...9) scratch[18 + k] = scratch[k];
      scratch[18 + i] = scratch[i] + h;
      var plus = swivel(scratch, 18);
      scratch[18 + i] = scratch[i] - h;
      var minus = swivel(scratch, 18);
      scratch[9 + i] = wrap(plus - minus) / (2.0 * h);
    }
    for (c in 0...w) jacobian[row * w + c] = 0.0;
    for (p in 0...3) {
      snapshot.pointJacobianColumns(bodies[p], scratch[3 * p], scratch[3 * p + 1], scratch[3 * p + 2], layout,
        pointJacobian);
      var gx = scratch[9 + 3 * p], gy = scratch[10 + 3 * p], gz = scratch[11 + 3 * p];
      for (c in 0...w)
        jacobian[row * w + c] += weight * (gx * pointJacobian[c] + gy * pointJacobian[w + c] + gz * pointJacobian[2 * w + c]);
    }
  }

  function worldPoints(snapshot:KinematicSnapshot):Void {
    for (p in 0...3) {
      snapshot.bodyPoseInto(bodies[p], pose, 0);
      FlatTransform.rotate(pose, 0, local[3 * p], local[3 * p + 1], local[3 * p + 2], scratch, 3 * p);
      scratch[3 * p] += pose[0];
      scratch[3 * p + 1] += pose[1];
      scratch[3 * p + 2] += pose[2];
    }
  }

  /** ψ from shoulder, elbow, wrist at `values[o..o+8]`. */
  function swivel(values:Array<Float>, o:Int):Float {
    var sx = values[o], sy = values[o + 1], sz = values[o + 2];
    var ax = values[o + 6] - sx, ay = values[o + 7] - sy, az = values[o + 8] - sz;
    var length = Math.sqrt(ax * ax + ay * ay + az * az);
    if (length < 1e-12) throw 'Swivel task "$name": shoulder and wrist coincide';
    ax /= length; ay /= length; az /= length;
    // u: the reference direction across the axis; v = a × u.
    var g = reference;
    var along = g[0] * ax + g[1] * ay + g[2] * az;
    var ux = g[0] - along * ax, uy = g[1] - along * ay, uz = g[2] - along * az;
    var un = Math.sqrt(ux * ux + uy * uy + uz * uz);
    if (un < 1e-9) throw 'Swivel task "$name": the shoulder-wrist line is parallel to the reference';
    ux /= un; uy /= un; uz /= un;
    var vx = ay * uz - az * uy, vy = az * ux - ax * uz, vz = ax * uy - ay * ux;
    var px = values[o + 3] - sx, py = values[o + 4] - sy, pz = values[o + 5] - sz;
    var x = px * ux + py * uy + pz * uz, y = px * vx + py * vy + pz * vz;
    if (x * x + y * y < 1e-18) throw 'Swivel task "$name": the elbow lies on the shoulder-wrist line';
    return Math.atan2(y, x);
  }

  static function wrap(angle:Float):Float {
    var tau = 2.0 * Math.PI;
    var wrapped = (angle + Math.PI) % tau;
    if (wrapped < 0.0) wrapped += tau;
    return wrapped - Math.PI;
  }
}
