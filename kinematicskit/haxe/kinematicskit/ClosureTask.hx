package kinematicskit;

/**
 * Holds one of the model's loop closures. Rows (each divided by its
 * tolerance, so a residual of 1 is "exactly at tolerance"):
 * - Fixed: `pB − pA` (3) and the rotation vector of `A⁻¹B` in A (3);
 * - Revolute: `pB − pA` (3) and B's axis in the plane perpendicular to A's (2);
 * - Prismatic: `pB − pA` across A's axis (2), the axis rows (2) and the twist about the axis (1);
 * - Cylindrical: the Prismatic rows without the twist (4);
 * - Spherical: `pB − pA` (3);
 * - Planar: `pB − pA` along A's axis, the plane normal, minus the offset value (1), and the axis rows (2);
 * - Parallel: the axis rows (2); Perpendicular: a·b (1); Angle: a·b − cos(value) (1);
 * - Distance: |pB − pA| − value (1).
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
      case ClosureKind.Spherical, ClosureKind.Planar: 3;
      case ClosureKind.Parallel: 2;
      case ClosureKind.Perpendicular, ClosureKind.Distance, ClosureKind.Angle: 1;
      case ClosureKind.Prismatic: 5;
      default: 4; // Cylindrical
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
      case ClosureKind.Spherical:
        lastPositionError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        lastOrientationError = 0;
        positionRows(residual, jacobian, r, w, dx, dy, dz, p);
      case ClosureKind.Planar:
        // B's origin on A's plane: d·a, with d(d·a) = a·(v_B − v_A) + (a × d)·ω_A exactly.
        var ax = s[14], ay = s[15], az = s[16];
        var along = dx * ax + dy * ay + dz * az - model.closureValue[closure];
        var cx = ay * dz - az * dy, cy = az * dx - ax * dz, cz = ax * dy - ay * dx;
        residual[r] = -p * along;
        for (c in 0...w) {
          var linear = ax * (jacobianB[c] - jacobianA[c]) + ay * (jacobianB[w + c] - jacobianA[w + c]) +
            az * (jacobianB[2 * w + c] - jacobianA[2 * w + c]);
          jacobian[r * w + c] = p * (linear + cx * jacobianA[3 * w + c] + cy * jacobianA[4 * w + c] + cz * jacobianA[5 * w + c]);
        }
        r++;
        lastPositionError = Math.abs(along);
        lastOrientationError = axisAngle();
        axisRows(residual, jacobian, r, w, g);
      case ClosureKind.Parallel:
        lastPositionError = 0;
        lastOrientationError = axisAngle();
        axisRows(residual, jacobian, r, w, g);
      case ClosureKind.Perpendicular, ClosureKind.Angle:
        // a·b against 0 or cos(value); d(a·b) = (a × b)·(ω_A − ω_B) exactly.
        var ax = s[14], ay = s[15], az = s[16], bx = s[17], by = s[18], bz = s[19];
        var target = kind == ClosureKind.Angle ? Math.cos(model.closureValue[closure]) : 0.0;
        var value = ax * bx + ay * by + az * bz - target;
        var cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
        residual[r] = -g * value;
        for (c in 0...w)
          jacobian[r * w + c] = g * (cx * (jacobianA[3 * w + c] - jacobianB[3 * w + c]) +
            cy * (jacobianA[4 * w + c] - jacobianB[4 * w + c]) + cz * (jacobianA[5 * w + c] - jacobianB[5 * w + c]));
        lastPositionError = 0;
        lastOrientationError = Math.abs(value);
      case ClosureKind.Distance:
        // |d| − value; d|d| = d̂·(v_B − v_A), undefined only when the origins meet.
        var length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        var value = length - model.closureValue[closure];
        residual[r] = -p * value;
        var ux = length > 0 ? dx / length : 0.0, uy = length > 0 ? dy / length : 0.0, uz = length > 0 ? dz / length : 0.0;
        for (c in 0...w)
          jacobian[r * w + c] = p * (ux * (jacobianB[c] - jacobianA[c]) + uy * (jacobianB[w + c] - jacobianA[w + c]) +
            uz * (jacobianB[2 * w + c] - jacobianA[2 * w + c]));
        lastPositionError = Math.abs(value);
        lastOrientationError = 0;
      default: // Prismatic (with its twist row below), Cylindrical
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
        if (kind == ClosureKind.Prismatic) {
          // Twist about the axis: the relative rotation's component along it, dφ·a ≈ a·(ω_B − ω_A) near closure.
          Rotations.relativeRotationVector(s, 0, s, 7, s, 20);
          var twist = s[20] * model.closureAxis[c3] + s[21] * model.closureAxis[c3 + 1] + s[22] * model.closureAxis[c3 + 2];
          lastOrientationError = Math.max(lastOrientationError, Math.abs(twist));
          var row = r + 2;
          residual[row] = -g * twist;
          for (c in 0...w)
            jacobian[row * w + c] = g * (ax * (jacobianB[3 * w + c] - jacobianA[3 * w + c]) +
              ay * (jacobianB[4 * w + c] - jacobianA[4 * w + c]) + az * (jacobianB[5 * w + c] - jacobianA[5 * w + c]));
        }
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
