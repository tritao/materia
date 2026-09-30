package kinematicskit;

/** What a solver returns; the problem and seed are never modified. */
class KinematicSolution {
  public final status:KinematicStatus;
  public final state:KinematicState;
  public final iterations:Int;
  /** Norm of all weighted residual rows at `state`. */
  public final residualNorm:Float;
  public final tasks:Array<TaskResult>;
  /** Rank of the hard-task Jacobian over the active DOFs at `state`. */
  public final rank:Int;
  /** Active DOFs left unconstrained by the hard tasks: active count − rank. */
  public final freeDofs:Int;
  /** Active DOFs sitting on a limit at `state`. */
  public final limitHits:Array<Int>;

  public function new(status:KinematicStatus, state:KinematicState, iterations:Int, residualNorm:Float,
      tasks:Array<TaskResult>, rank:Int, freeDofs:Int, limitHits:Array<Int>) {
    this.status = status;
    this.state = state;
    this.iterations = iterations;
    this.residualNorm = residualNorm;
    this.tasks = tasks;
    this.rank = rank;
    this.freeDofs = freeDofs;
    this.limitHits = limitHits;
  }

  public function converged():Bool return status == KinematicStatus.Converged;

  /** Labels of the hard tasks that missed their tolerances. */
  public function unsatisfied():Array<String> return [for (task in tasks) if (!task.soft && !task.satisfied) task.label];
}
