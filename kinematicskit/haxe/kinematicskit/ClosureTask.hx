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
  /** Pose A (0..6), pose B (7..13), axis A (14..16), axis B (17..19), rotation vector (20..22),
      basis (23..28), per-column temporaries (29..34). */
  final scratch:Array<Float> = [for (_ in 0...35) 0.0];
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

  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    if (jacobianA.length < 6 * w) { jacobianA.resize(6 * w); jacobianB.resize(6 * w); }
    var frameA = model.closureFrameA[closure], frameB = model.closureFrameB[closure];
    var bodyA = model.frameBody[frameA], bodyB = model.frameBody[frameB];
    var s = scratch;
    snapshot.attachedPoseInto(bodyA, model.frameOffset, frameA * 7, s, 0);
    snapshot.attachedPoseInto(bodyB, model.frameOffset, frameB * 7, s, 7);
    snapshot.pointJacobianColumns(bodyA, s[0], s[1], s[2], layout, jacobianA);
    snapshot.pointJacobianColumns(bodyB, s[7], s[8], s[9], layout, jacobianB);
    var c3 = closure * 3;
    FlatTransform.rotate(s, 0, model.closureAxis[c3], model.closureAxis[c3 + 1], model.closureAxis[c3 + 2], s, 14);
    FlatTransform.rotate(s, 7, model.closureAxis[c3], model.closureAxis[c3 + 1], model.closureAxis[c3 + 2], s, 17);
    var dx = s[7] - s[0], dy = s[8] - s[1], dz = s[9] - s[2];
    var p = 1.0 / positionTolerance, g = 1.0 / angularTolerance;
    var r = row;
    switch kind {
      case ClosureKind.Fixed:
        lastPositionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        r = positionRows(residual, jacobian, r, w, dx, dy, dz, p);
        Rotations.relativeRotationVector(s, 0, s, 7, s, 20);
        var phx = s[20], phy = s[21], phz = s[22];
        lastOrientationError = Math.sqrt(phx * phx + phy * phy + phz * phz);
        // dφ = J_l⁻¹(φ) · R_Aᵀ (ω_B − ω_A), one column at a time.
        for (c in 0...w) {
          FlatTransform.rotateInverse(s, 0, jacobianB[3 * w + c] - jacobianA[3 * w + c],
            jacobianB[4 * w + c] - jacobianA[4 * w + c], jacobianB[5 * w + c] - jacobianA[5 * w + c], s, 29);
          Rotations.inverseLeftJacobian(phx, phy, phz, s[29], s[30], s[31], s, 32);
          jacobian[r * w + c] = g * s[32];
          jacobian[(r + 1) * w + c] = g * s[33];
          jacobian[(r + 2) * w + c] = g * s[34];
        }
        residual[r] = -g * phx; residual[r + 1] = -g * phy; residual[r + 2] = -g * phz;
      case ClosureKind.Revolute:
        lastPositionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        r = positionRows(residual, jacobian, r, w, dx, dy, dz, p);
        lastOrientationError = axisAngle();
        axisRows(residual, jacobian, r, w, g);
      default: // Prismatic
        var ax = s[14], ay = s[15], az = s[16];
        var along = dx * ax + dy * ay + dz * az;
        Rotations.perpendicularBasis(ax, ay, az, s, 23);
        var squared = 0.0;
        for (k in 0...2) {
          var ux = s[23 + 3 * k], uy = s[24 + 3 * k], uz = s[25 + 3 * k];
          var transverse = dx * ux + dy * uy + dz * uz;
          squared += transverse * transverse;
          residual[r] = -p * transverse;
          // d(d·u) ≈ u·(v_B − v_A) − along · (a × u)·ω_A, since u stays perpendicular to a.
          var cx = ay * uz - az * uy, cy = az * ux - ax * uz, cz = ax * uy - ay * ux;
          for (c in 0...w) {
            var linear = ux * (jacobianB[c] - jacobianA[c]) + uy * (jacobianB[w + c] - jacobianA[w + c]) +
              uz * (jacobianB[2 * w + c] - jacobianA[2 * w + c]);
            var turning = cx * jacobianA[3 * w + c] + cy * jacobianA[4 * w + c] + cz * jacobianA[5 * w + c];
            jacobian[r * w + c] = p * (linear - along * turning);
          }
          r++;
        }
        lastPositionError = Math.sqrt(squared);
        lastOrientationError = axisAngle();
        axisRows(residual, jacobian, r, w, g);
    }
  }

  /** Rows for `d = pB − pA`: residual `−d`, Jacobian `v_B − v_A`. */
  function positionRows(residual:Array<Float>, jacobian:Array<Float>, row:Int, w:Int, dx:Float, dy:Float, dz:Float,
      scale:Float):Int {
    residual[row] = -scale * dx;
    residual[row + 1] = -scale * dy;
    residual[row + 2] = -scale * dz;
    for (axis in 0...3) for (c in 0...w)
      jacobian[(row + axis) * w + c] = scale * (jacobianB[axis * w + c] - jacobianA[axis * w + c]);
    return row + 3;
  }

  /** B's axis components across A's axis (sign-corrected so anti-parallel also closes). */
  function axisRows(residual:Array<Float>, jacobian:Array<Float>, row:Int, w:Int, scale:Float):Void {
    var s = scratch;
    var ax = s[14], ay = s[15], az = s[16], bx = s[17], by = s[18], bz = s[19];
    var sign = ax * bx + ay * by + az * bz < 0 ? -1.0 : 1.0;
    Rotations.perpendicularBasis(ax, ay, az, s, 23);
    for (k in 0...2) {
      var ux = s[23 + 3 * k], uy = s[24 + 3 * k], uz = s[25 + 3 * k];
      residual[row] = -scale * sign * (bx * ux + by * uy + bz * uz);
      // d(b·u) ≈ (b × u)·(ω_B − ω_A) near closure.
      var cx = by * uz - bz * uy, cy = bz * ux - bx * uz, cz = bx * uy - by * ux;
      for (c in 0...w)
        jacobian[row * w + c] = scale * sign * (cx * (jacobianB[3 * w + c] - jacobianA[3 * w + c]) +
          cy * (jacobianB[4 * w + c] - jacobianA[4 * w + c]) + cz * (jacobianB[5 * w + c] - jacobianA[5 * w + c]));
      row++;
    }
  }

  function axisAngle():Float {
    var s = scratch;
    return Math.acos(Math.min(1.0, Math.max(-1.0, Math.abs(s[14] * s[17] + s[15] * s[18] + s[16] * s[19]))));
  }
}
