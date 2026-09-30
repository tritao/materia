package kinematicskit;

/**
 * Holds one of the model's loop closures. Rows (each divided by its
 * tolerance, so a residual of 1 is "exactly at tolerance"):
 * - Fixed: `pB − pA` (3) and the rotation vector of `A⁻¹B` in A (3);
 * - Revolute: `pB − pA` (3) and B's axis in the plane perpendicular to A's (2);
 * - Prismatic: `pB − pA` across A's axis (2) and the axis rows (2).
 * Axis rows accept anti-parallel axes. The residual definitions match
 * CadKit's `AssemblyLoopSolver`; Jacobians are analytic (exact for the
 * position and fixed-rotation rows, first order in the transverse and axis
 * rows, exact at closure).
 */
class ClosureTask implements KinematicTask {
  public final closure:Int;
  public final positionTolerance:Float;
  public final angularTolerance:Float;
  final model:KinematicModel;
  final kind:ClosureKind;
  final rows:Int;
  final jacobianA:Array<Float> = [];
  final jacobianB:Array<Float> = [];
  var lastPositionError = 0.0;
  var lastOrientationError = 0.0;

  public function new(model:KinematicModel, closure:Int, positionTolerance:Float, angularTolerance:Float) {
    if (model == null || closure < 0 || closure >= model.closureCount()) throw "Closure task requires a closure of the model";
    if (!(positionTolerance > 0.0) || !(angularTolerance > 0.0)) throw "Closure task tolerances must be positive";
    this.model = model;
    this.closure = closure;
    this.positionTolerance = positionTolerance;
    this.angularTolerance = angularTolerance;
    kind = model.closureKind[closure];
    rows = switch kind {
      case ClosureKind.Fixed: 6;
      case ClosureKind.Revolute: 5;
      default: 4; // Prismatic
    };
  }

  public function label():String return model.closureIds[closure];
  public function rowCount():Int return rows;
  public function isSoft():Bool return false;
  public function positionError():Float return lastPositionError;
  public function orientationError():Float return lastOrientationError;
  public function satisfied():Bool
    return lastPositionError <= positionTolerance && lastOrientationError <= angularTolerance;

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, residual:Array<Float>,
      jacobian:Array<Float>, row:Int):Void {
    var n = model.dofCount();
    var frameA = model.closureFrameA[closure], frameB = model.closureFrameB[closure];
    var first = snapshot.framePose(frameA), second = snapshot.framePose(frameB);
    snapshot.frameJacobian(frameA, jacobianA);
    snapshot.frameJacobian(frameB, jacobianB);
    var ax = model.closureAxis[closure * 3], ay = model.closureAxis[closure * 3 + 1], az = model.closureAxis[closure * 3 + 2];
    var axisA = first.transformVector(ax, ay, az), axisB = second.transformVector(ax, ay, az);
    var dx = second.x - first.x, dy = second.y - first.y, dz = second.z - first.z;
    var p = 1.0 / positionTolerance, g = 1.0 / angularTolerance;
    var r = row;
    switch kind {
      case ClosureKind.Fixed:
        lastPositionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        r = positionRows(residual, jacobian, r, n, [dx, dy, dz], p);
        var phi = Rotations.relativeRotationVector(first, second);
        lastOrientationError = Math.sqrt(phi.x * phi.x + phi.y * phi.y + phi.z * phi.z);
        // dφ = J_l⁻¹(φ) · R_Aᵀ (ω_B − ω_A), one column at a time.
        var inverseA = new Transform(0.0, 0.0, 0.0, -first.qx, -first.qy, -first.qz, first.qw);
        for (c in 0...n) {
          var local = inverseA.transformVector(jacobianB[3 * n + c] - jacobianA[3 * n + c],
            jacobianB[4 * n + c] - jacobianA[4 * n + c], jacobianB[5 * n + c] - jacobianA[5 * n + c]);
          var d = Rotations.inverseLeftJacobian(phi, local);
          jacobian[r * n + c] = g * d.x;
          jacobian[(r + 1) * n + c] = g * d.y;
          jacobian[(r + 2) * n + c] = g * d.z;
        }
        residual[r] = -g * phi.x; residual[r + 1] = -g * phi.y; residual[r + 2] = -g * phi.z;
        r += 3;
      case ClosureKind.Revolute:
        lastPositionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        r = positionRows(residual, jacobian, r, n, [dx, dy, dz], p);
        lastOrientationError = axisAngle(axisA, axisB);
        axisRows(residual, jacobian, r, n, axisA, axisB, g);
      default: // Prismatic
        var along = dx * axisA.x + dy * axisA.y + dz * axisA.z;
        var basis = Rotations.perpendicularBasis(axisA.x, axisA.y, axisA.z);
        var squared = 0.0;
        for (u in basis) {
          var transverse = dx * u.x + dy * u.y + dz * u.z;
          squared += transverse * transverse;
          residual[r] = -p * transverse;
          // d(d·u) ≈ u·(v_B − v_A) − along · (a × u)·ω_A, since u stays perpendicular to a.
          var cx = axisA.y * u.z - axisA.z * u.y, cy = axisA.z * u.x - axisA.x * u.z, cz = axisA.x * u.y - axisA.y * u.x;
          for (c in 0...n) {
            var linear = u.x * (jacobianB[c] - jacobianA[c]) + u.y * (jacobianB[n + c] - jacobianA[n + c]) +
              u.z * (jacobianB[2 * n + c] - jacobianA[2 * n + c]);
            var turning = cx * jacobianA[3 * n + c] + cy * jacobianA[4 * n + c] + cz * jacobianA[5 * n + c];
            jacobian[r * n + c] = p * (linear - along * turning);
          }
          r++;
        }
        lastPositionError = Math.sqrt(squared);
        lastOrientationError = axisAngle(axisA, axisB);
        axisRows(residual, jacobian, r, n, axisA, axisB, g);
    }
  }

  /** Rows for `d = pB − pA`: residual `−d`, Jacobian `v_B − v_A`. */
  function positionRows(residual:Array<Float>, jacobian:Array<Float>, row:Int, n:Int, d:Array<Float>,
      scale:Float):Int {
    for (axis in 0...3) {
      residual[row] = -scale * d[axis];
      for (c in 0...n) jacobian[row * n + c] = scale * (jacobianB[axis * n + c] - jacobianA[axis * n + c]);
      row++;
    }
    return row;
  }

  /** B's axis components across A's axis (sign-corrected so anti-parallel also closes). */
  function axisRows(residual:Array<Float>, jacobian:Array<Float>, row:Int, n:Int, a:Vector3, b:Vector3,
      scale:Float):Void {
    var sign = a.x * b.x + a.y * b.y + a.z * b.z < 0 ? -1.0 : 1.0;
    for (u in Rotations.perpendicularBasis(a.x, a.y, a.z)) {
      residual[row] = -scale * sign * (b.x * u.x + b.y * u.y + b.z * u.z);
      // d(b·u) ≈ (b × u)·(ω_B − ω_A) near closure.
      var cx = b.y * u.z - b.z * u.y, cy = b.z * u.x - b.x * u.z, cz = b.x * u.y - b.y * u.x;
      for (c in 0...n)
        jacobian[row * n + c] = scale * sign * (cx * (jacobianB[3 * n + c] - jacobianA[3 * n + c]) +
          cy * (jacobianB[4 * n + c] - jacobianA[4 * n + c]) + cz * (jacobianB[5 * n + c] - jacobianA[5 * n + c]));
      row++;
    }
  }

  static function axisAngle(a:Vector3, b:Vector3):Float
    return Math.acos(Math.min(1.0, Math.max(-1.0, Math.abs(a.x * b.x + a.y * b.y + a.z * b.z))));
}
