package kinematicskit;

/**
 * Tasks to meet, the DOFs a solver may move (all by default; the others stay
 * at the seed), and the DOF limits (the model's by default). Solvers read a
 * problem and never modify it.
 */
class KinematicProblem {
  public final model:KinematicModel;
  public final tasks:Array<KinematicTask> = [];
  public final lower:Array<Float>;
  public final upper:Array<Float>;
  var active:Array<Int>;
  var columns:JacobianLayout;

  public function new(model:KinematicModel) {
    if (model == null) throw "Kinematic problem requires a model";
    this.model = model;
    lower = model.dofLower.copy();
    upper = model.dofUpper.copy();
    active = [for (dof in 0...model.dofCount()) dof];
    columns = new JacobianLayout(model, active);
  }

  public function add(task:KinematicTask):KinematicProblem {
    if (task == null) throw "Kinematic problem task is required";
    tasks.push(task);
    return this;
  }

  /** Restricts the solve to these DOFs, in this order (the order solvers use for their columns). */
  public function setActiveDofs(dofs:Array<Int>):KinematicProblem {
    if (dofs == null || dofs.length == 0) throw "Kinematic problem needs at least one active DOF";
    var seen = new Map<Int, Bool>();
    for (dof in dofs) {
      if (dof < 0 || dof >= model.dofCount()) throw 'Kinematic problem DOF $dof is not part of the model';
      if (seen.exists(dof)) throw 'Kinematic problem lists DOF "${model.dofId(dof)}" more than once';
      seen.set(dof, true);
    }
    active = dofs.copy();
    columns = new JacobianLayout(model, active);
    return this;
  }

  /** `setActiveDofs` by joint ID; each must be a joint that drives its own DOF (not fixed, not coupled). */
  public function setActiveJoints(jointIds:Array<String>):KinematicProblem {
    if (jointIds == null) throw "Kinematic problem needs joint IDs";
    return setActiveDofs([for (id in jointIds) {
      var dof = model.dofIndex(id);
      if (dof < 0) throw 'Joint "$id" does not drive a DOF of its own';
      dof;
    }]);
  }

  /** The Jacobian columns of a solve: one per active DOF, in active order. */
  public function layout():JacobianLayout return columns;

  public function activeDofs():Array<Int> return active.copy();

  /** Overrides one DOF's range; infinite bounds mean unlimited. */
  public function setLimits(dof:Int, lowerBound:Float, upperBound:Float):KinematicProblem {
    if (dof < 0 || dof >= model.dofCount()) throw 'Kinematic problem DOF $dof is not part of the model';
    if (Math.isNaN(lowerBound) || Math.isNaN(upperBound) || lowerBound > upperBound)
      throw 'Kinematic problem limits for "${model.dofId(dof)}" are invalid';
    lower[dof] = lowerBound;
    upper[dof] = upperBound;
    return this;
  }

  public function rowCount():Int {
    var rows = 0;
    for (i in 0...tasks.length) rows += tasks[i].rowCount();
    return rows;
  }

  /**
   * Evaluates `snapshot` at `state`, then every task into `residual` and the
   * row-major `jacobian` (`layout().width` columns).
   */
  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, residual:Array<Float>,
      jacobian:Array<Float>):Void {
    snapshot.evaluate(state);
    var row = 0;
    for (i in 0...tasks.length) {
      var task = tasks[i];
      task.evaluate(state, snapshot, columns, residual, jacobian, row);
      row += task.rowCount();
    }
  }

  /** Whether every hard task met its tolerances at the last `evaluate`. */
  public function satisfied():Bool {
    for (i in 0...tasks.length) if (!tasks[i].isSoft() && !tasks[i].satisfied()) return false;
    return true;
  }

  /** Rows belonging to hard tasks, in order. */
  public function hardRows():Array<Int> {
    var rows:Array<Int> = [];
    var row = 0;
    for (task in tasks) {
      if (!task.isSoft()) for (i in 0...task.rowCount()) rows.push(row + i);
      row += task.rowCount();
    }
    return rows;
  }

  /** Clamps the active DOFs of `q` into their limits. */
  public function clamp(q:Array<Float>):Void {
    for (i in 0...active.length) {
      var dof = active[i];
      if (q[dof] < lower[dof]) q[dof] = lower[dof];
      if (q[dof] > upper[dof]) q[dof] = upper[dof];
    }
  }

  public function limitHits(q:Array<Float>):Array<Int>
    return [for (dof in active) if (q[dof] <= lower[dof] || q[dof] >= upper[dof]) dof];
}
