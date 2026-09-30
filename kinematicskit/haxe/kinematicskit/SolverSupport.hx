package kinematicskit;

/** Shared bookkeeping for the solver policies. */
class SolverSupport {
  /**
   * Applies `factor` x `delta` (one entry per layout column) to `state`: DOFs
   * add directly; a moving root composes the rotation Exp(ω) on the left
   * (about its origin) and adds v to its position. Allocates one pose per
   * moving root.
   */
  public static function applyStep(problem:KinematicProblem, state:KinematicState, delta:Array<Float>,
      factor:Float):Void {
    var layout = problem.layout();
    for (i in 0...layout.dofs.length) state.q[layout.dofs[i]] += factor * delta[i];
    for (block in 0...layout.rootBodies.length) {
      var body = layout.rootBodies[block], c = layout.rootColumns[block];
      var pose = state.rootPose(body);
      var vx = factor * delta[c], vy = factor * delta[c + 1], vz = 0.0;
      var wx = 0.0, wy = 0.0, wz:Float;
      if (layout.rootModes[block] == RootMotion.Planar) wz = factor * delta[c + 2];
      else {
        vz = factor * delta[c + 2];
        wx = factor * delta[c + 3]; wy = factor * delta[c + 4]; wz = factor * delta[c + 5];
      }
      var angle = Math.sqrt(wx * wx + wy * wy + wz * wz);
      var rx = 0.0, ry = 0.0, rz = 0.0, rw = 1.0;
      if (angle > 1e-15) {
        var s = Math.sin(angle * 0.5) / angle;
        rx = wx * s; ry = wy * s; rz = wz * s; rw = Math.cos(angle * 0.5);
      }
      // Exp(ω) · q, renormalized against drift.
      var qx = rw * pose.qx + rx * pose.qw + ry * pose.qz - rz * pose.qy;
      var qy = rw * pose.qy - rx * pose.qz + ry * pose.qw + rz * pose.qx;
      var qz = rw * pose.qz + rx * pose.qy - ry * pose.qx + rz * pose.qw;
      var qw = rw * pose.qw - rx * pose.qx - ry * pose.qy - rz * pose.qz;
      var norm = Math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      state.setRootPose(body, new Transform(pose.x + vx, pose.y + vy, pose.z + vz, qx / norm, qy / norm, qz / norm, qw / norm));
    }
  }

  /** The poses of the problem's moving roots, for rolling a step back with `restoreRoots`. */
  public static function saveRoots(problem:KinematicProblem, state:KinematicState, into:Array<Transform>):Void {
    var layout = problem.layout();
    for (block in 0...layout.rootBodies.length) into[block] = state.rootPose(layout.rootBodies[block]);
  }

  public static function restoreRoots(problem:KinematicProblem, state:KinematicState, from:Array<Transform>):Void {
    var layout = problem.layout();
    for (block in 0...layout.rootBodies.length) state.setRootPose(layout.rootBodies[block], from[block]);
  }

  /**
   * Re-evaluates the problem at `state` and packages the diagnostics
   * (allocates; once per solve). `scales` multiplies each Jacobian column
   * for the rank test, as the solver saw it.
   */
  public static function finish(problem:KinematicProblem, state:KinematicState, workspace:SolverWorkspace,
      status:KinematicStatus, iterations:Int, rankTolerance:Float, useScales:Bool):KinematicSolution {
    var snapshot = workspace.snapshot;
    problem.evaluate(state, snapshot, workspace.residual, workspace.jacobian);
    var rows = problem.rowCount();
    var width = problem.layout().width;
    var hard = problem.hardRows();
    var matrix:Array<Float> = [];
    for (row in hard) for (c in 0...width)
      matrix.push(workspace.jacobian[row * width + c] * (useScales ? workspace.scales[c] : 1.0));
    var rank = LinearAlgebra.rank(matrix, hard.length, width, rankTolerance);
    return new KinematicSolution(status, state, iterations, LinearAlgebra.norm(workspace.residual, rows),
      [for (task in problem.tasks) new TaskResult(task)], rank, width - rank, problem.limitHits(state.q));
  }
}
