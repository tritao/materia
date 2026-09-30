package kinematicskit;

/**
 * A soft cost on moving DOFs: one row `weights[dof] · Δq` per weighted DOF,
 * with zero target. Unlike a `PostureTask` it pulls towards nothing, so a
 * solve still meets its hard tasks exactly; it only makes weighted DOFs
 * take a smaller share of each step. A robot's external axes (a rail, a
 * positioner) use it to move only for what the arm cannot do alone.
 * The joint-space analogue of `RootDampingTask`.
 */
class DofDampingTask implements KinematicTask {
  /** Per-DOF weights (0 leaves a DOF undamped), indexed by model DOF. */
  public final weights:Array<Float>;
  final dofCount:Int;

  public function new(model:KinematicModel, weights:Array<Float>) {
    if (model == null || weights == null || weights.length != model.dofCount())
      throw 'DOF damping task requires ${model == null ? 0 : model.dofCount()} weights';
    for (w in weights) if (!Math.isFinite(w) || w < 0.0) throw "DOF damping weights must be finite and non-negative";
    this.weights = weights.copy();
    dofCount = model.dofCount();
  }

  public function label():String return "dof damping";
  public function rowCount():Int return dofCount;
  public function isSoft():Bool return true;
  public function positionError():Float return 0.0;
  public function orientationError():Float return 0.0;
  public function satisfied():Bool return true;

  /** Rows for DOFs outside the solve are zero. */
  public function evaluate(state:KinematicState, snapshot:KinematicSnapshot, layout:JacobianLayout,
      residual:Array<Float>, jacobian:Array<Float>, row:Int):Void {
    var w = layout.width;
    for (i in 0...dofCount) {
      var column = layout.columnOfDof[i];
      residual[row + i] = 0.0;
      for (c in 0...w) jacobian[(row + i) * w + c] = c == column ? weights[i] : 0.0;
    }
  }
}
